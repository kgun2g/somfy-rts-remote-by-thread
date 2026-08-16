#include "cc1101.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"   /* esp_rom_delay_us */
#include <string.h>

static const char *TAG = "CC1101";

/* ─── TI 권장 CC1101 수동 파워업 리셋 (SWRS061 19.1.2) ──────────────
 *  SPI 드라이버에 핀을 넘기기 *전에* GPIO 비트뱅으로 수행.
 *  핵심: CSn LOW 후 SO(MISO)가 LOW 로 떨어질 때까지 대기(XOSC 안정).
 *  - 단순 "SRES + 10ms" 만으로는 XOSC 미안정 시 레지스터가 0x00 으로
 *    읽힌다(교체 모듈에서도 0x00 → 이 핸드셰이크 누락이 원인 후보).
 *  - 동시에 결정적 하드웨어 진단: CSn LOW 인데도 SO 가 끝내 LOW 가
 *    안 되면 전원 미인가/배선 단선/모듈 불량으로 확정(타임아웃 false).
 *  반환: true=리셋+XOSC 준비됨, false=SO 무반응(하드웨어 문제). */
static bool _cc1101_manual_reset(void)
{
    const gpio_num_t SCK  = CC1101_PIN_SCK;
    const gpio_num_t MOSI = CC1101_PIN_MOSI;
    const gpio_num_t MISO = CC1101_PIN_MISO;
    const gpio_num_t CS   = CC1101_PIN_CS;

    gpio_config_t out = {
        .pin_bit_mask = (1ULL << SCK) | (1ULL << MOSI) | (1ULL << CS),
        .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << MISO), .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);

    /* idle: SCK=1(mode0 idle low 지만 TI reset 절차는 SCK 안정 요구),
     *  TI 절차상 SCLK=1, SI=0 후 CSn 토글 */
    gpio_set_level(SCK, 0);
    gpio_set_level(MOSI, 0);
    gpio_set_level(CS, 1);
    esp_rom_delay_us(50);

    /* CSn 토글: low→high, ≥40us 대기 */
    gpio_set_level(CS, 0);
    esp_rom_delay_us(10);
    gpio_set_level(CS, 1);
    esp_rom_delay_us(60);

    /* CSn low, SO(MISO) 가 low 될 때까지 대기 (XOSC/chip ready) */
    gpio_set_level(CS, 0);
    int timeout_us = 5000;
    while (gpio_get_level(MISO) && timeout_us > 0) {
        esp_rom_delay_us(10);
        timeout_us -= 10;
    }
    if (timeout_us <= 0) {
        gpio_set_level(CS, 1);
        ESP_LOGE(TAG, "CSn LOW 후 SO(MISO/IO%d) 가 LOW 로 안 떨어짐 — "
                      "CC1101 전원(3.3V)/GND/배선 확인 필요", MISO);
        return false;
    }

    /* SRES(0x30) 스트로브 비트뱅 (MSB first, mode0: 상승엣지 샘플) */
    uint8_t cmd = 0x30;
    for (int i = 7; i >= 0; i--) {
        gpio_set_level(MOSI, (cmd >> i) & 1);
        esp_rom_delay_us(1);
        gpio_set_level(SCK, 1);
        esp_rom_delay_us(1);
        gpio_set_level(SCK, 0);
    }

    /* SRES 후 SO 가 다시 low 될 때까지 대기 (reset 완료) */
    timeout_us = 10000;
    while (gpio_get_level(MISO) && timeout_us > 0) {
        esp_rom_delay_us(10);
        timeout_us -= 10;
    }
    if (timeout_us <= 0) {
        ESP_LOGW(TAG, "SRES 후 SO 가 LOW 복귀 안 함 — 계속 진행(약식)");
    }

    /* ─── 결정적 진단: GPIO 비트뱅으로 PARTNUM(0x30)/VERSION(0x31) 직접 읽기.
     *  이건 SCK(IO%d)·MOSI 경로까지 실제로 사용한다(헤더 송신+데이터 수신).
     *  CSn 은 이미 LOW 유지중. mode0: MOSI 는 SCK LOW 일 때 세팅,
     *  MISO 는 SCK HIGH(상승엣지 후) 에 샘플. */
    uint8_t bb[2] = {0x30 | 0xC0, 0x31 | 0xC0};   /* PARTNUM, VERSION (burst) */
    uint8_t bbval[2] = {0, 0};
    for (int r = 0; r < 2; r++) {
        gpio_set_level(SCK, 0);
        esp_rom_delay_us(2);
        /* 헤더 8비트 송신 */
        for (int i = 7; i >= 0; i--) {
            gpio_set_level(MOSI, (bb[r] >> i) & 1);
            esp_rom_delay_us(2);
            gpio_set_level(SCK, 1);
            esp_rom_delay_us(2);
            gpio_set_level(SCK, 0);
            esp_rom_delay_us(2);
        }
        /* 데이터 8비트 수신 (MISO) */
        uint8_t v = 0;
        for (int i = 7; i >= 0; i--) {
            gpio_set_level(SCK, 1);
            esp_rom_delay_us(2);
            v |= (gpio_get_level(MISO) & 1) << i;
            gpio_set_level(SCK, 0);
            esp_rom_delay_us(2);
        }
        bbval[r] = v;
    }
    gpio_set_level(CS, 1);
    ESP_LOGW(TAG, "비트뱅 진단: PARTNUM=0x%02X VERSION=0x%02X "
                  "(정상 CC1101: PARTNUM=0x00, VERSION=0x14/0x04/0x17)",
             bbval[0], bbval[1]);
    if (bbval[1] == 0x00 || bbval[1] == 0xFF) {
        ESP_LOGE(TAG, "비트뱅 읽기도 실패 → SCK(IO%d)/MOSI(IO%d) 배선·납땜 "
                      "의심 (CSn/MISO 는 정상 확인됨)",
                 CC1101_PIN_SCK, CC1101_PIN_MOSI);
    } else {
        ESP_LOGW(TAG, "비트뱅 읽기 성공! → 칩·배선 모두 정상, ESP HW SPI "
                      "드라이버 설정이 문제 (코드로 해결 가능)");
    }
    ESP_LOGI(TAG, "CC1101 수동 파워업 리셋 OK (SO ready)");
    return true;
}

/* ─── SPI 접근 헬퍼 ──────────────────────────── */

static void spi_write(cc1101_t *dev, uint8_t addr, const uint8_t *data, size_t len)
{
    spi_transaction_t t = {
        .length    = (1 + len) * 8,
        .tx_buffer = NULL,
    };

    uint8_t tx_buf[1 + len];
    tx_buf[0] = (len > 1) ? (addr | 0x40) : addr;  // burst bit
    memcpy(&tx_buf[1], data, len);
    t.tx_buffer = tx_buf;
    t.length    = (1 + len) * 8;

    spi_device_polling_transmit(dev->spi, &t);
}

static void spi_read(cc1101_t *dev, uint8_t addr, uint8_t *data, size_t len)
{
    uint8_t tx_buf[1 + len];
    uint8_t rx_buf[1 + len];
    memset(tx_buf, 0xFF, sizeof(tx_buf));
    tx_buf[0] = 0x80 | ((len > 1) ? (addr | 0x40) : addr);  // read + burst

    spi_transaction_t t = {
        .length    = (1 + len) * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    spi_device_polling_transmit(dev->spi, &t);
    memcpy(data, &rx_buf[1], len);
}

static uint8_t spi_strobe(cc1101_t *dev, uint8_t cmd)
{
    uint8_t status = 0;
    spi_transaction_t t = {
        .length    = 8,
        .tx_buffer = &cmd,
        .rx_buffer = &status,
    };
    spi_device_polling_transmit(dev->spi, &t);
    return status;
}

/* ─── Somfy RTS용 CC1101 기본 레지스터 값 ───── */
static const uint8_t cc1101_init_regs[][2] = {
    /* IOCFG2  */ { 0x00, 0x2E },
    /* IOCFG1  */ { 0x01, 0x2E },
    /* IOCFG0  */ { 0x02, 0x2E },   /* 3-state: async serial TX 시 GDO0 를
                                    *  ESP RMT 가 데이터 입력으로 구동하도록
                                    *  CC1101 출력 드라이버를 끈다(충돌 방지).
                                    *  (이전 0x2D 는 GDO0 출력 구동 가능 →
                                    *   RMT 와 충돌해 변조 데이터 미입력 의심) */
    /* FIFOTHR */ { 0x03, 0x47 },
    /* SYNC1   */ { 0x04, 0xD3 },
    /* SYNC0   */ { 0x05, 0x91 },
    /* PKTLEN  */ { 0x06, 0xFF },
    /* PKTCTRL1*/ { 0x07, 0x04 },
    /* PKTCTRL0*/ { 0x08, 0x32 },
    /* ADDR    */ { 0x09, 0x00 },
    /* CHANNR  */ { 0x0A, 0x00 },
    /* FSCTRL1 */ { 0x0B, 0x06 },
    /* FSCTRL0 */ { 0x0C, 0x03 },   /* ★ freq offset +4761Hz — 0x04(mean+150 과도→UP/DOWN byte8 mark 스파이크) 완화.
                                    *  목표 mean~-183(정품). PROG(byte8=0)는 0x04서도 P2=24 였으나 UP/DOWN(byte8≠0)은 스파이크로 깸 → 0x03 절충. */
    /* FREQ2/1/0 는 이 표에서 설정하지 않는다 — cc1101_init 이 _set_freq_regs() 로
     *  보드 기본 주파수를 직접 써 넣는다(옛 하드코딩 447.269MHz 사표값 제거). */
    /* MDMCFG4 */ { 0x10, 0x8A },   /* ★★ DRATE 38.4k baud (E:10). 153.6k 는 FM 변조를 죽여(max 1713→11)
                                    *  역효과 → 38.4k 복귀. P2 는 DRATE 무관(DEVIATN/inter-frame 으로만 변함). (구주석↓)
                                    *  SDR 실측: DRATE 2400 일 때 변조기 데이터
                                    *  레이트 필터가 너무 좁아 640µs Somfy 펄스
                                    *  의 FSK 천이가 ~300µs 에 걸쳐 느리게 일어남
                                    *  → 1T 펄스 듀티 왜곡(HIGH780/LOW490) →
                                    *  일부 1T 가 ~1000µs 로 늘어 디코더·모터가
                                    *  2T 로 오인 → 프레임 비트 밀림 → 깨짐.
                                    *  DRATE 를 올리면 필터가 넓어져 천이가
                                    *  날카로워진다(async 모드는 비트 타이밍이
                                    *  GD0 에서 오므로 DRATE 는 변조 필터폭만
                                    *  결정 — 높을수록 충실도↑). CHANBW(상위
                                    *  니블 0x8)는 RX 전용이라 유지. */
    /* MDMCFG3 */ { 0x11, 0x83 },   /* DRATE_M=131 (E=10 과 합쳐 38384 baud) */
    /* MDMCFG2 */ { 0x12, 0x00 },   /* ★★ MOD_FORMAT 011(ASK/OOK) → 000
                                    *  (2-FSK). IQ 분석으로 확정: 정품 한국
                                    *  베네치아 Somfy 는 OOK 가 아니라 2-FSK
                                    *  (carrier 항상 ON, 주파수 ±2.5kHz 변조).
                                    *  MANCHESTER_EN=0, SYNC_MODE=000 유지. */
    /* MDMCFG1 */ { 0x13, 0x22 },
    /* MDMCFG0 */ { 0x14, 0xF8 },
    /* DEVIATN */ { 0x15, 0x12 },   /* ★ FSK deviation ±4kHz(E=1,M=2) — 1번 동작 검증값으로 복구.
                                    *  (0x22 는 FM range 정품화했으나 스파이크(-11256~17966)로 1번도 깸 + P2 여전히 112
                                    *   → P2=24 는 deviation 무관 확정, 천이속도(DRATE) 별도 접근.) */
    /* MCSM2   */ { 0x16, 0x07 },
    /* MCSM1   */ { 0x17, 0x30 },
    /* MCSM0   */ { 0x18, 0x18 },
    /* FOCCFG  */ { 0x19, 0x16 },
    /* BSCFG   */ { 0x1A, 0x6C },
    /* AGCCTRL2*/ { 0x1B, 0x03 },
    /* AGCCTRL1*/ { 0x1C, 0x40 },
    /* AGCCTRL0*/ { 0x1D, 0x91 },
    /* WOREVT1 */ { 0x1E, 0x87 },
    /* WOREVT0 */ { 0x1F, 0x6B },
    /* WORCTRL */ { 0x20, 0xFB },
    /* FREND1  */ { 0x21, 0x56 },
    /* FREND0  */ { 0x22, 0x10 },   /* ★ PA_POWER 1→0: 2-FSK 는 carrier 가
                                    *  항상 ON 이라 PA_TABLE[0] 한 entry 만
                                    *  사용(OOK 의 off/on 2-entry 토글과 다름). */
    /* FSCAL3  */ { 0x23, 0xE9 },
    /* FSCAL2  */ { 0x24, 0x2A },
    /* FSCAL1  */ { 0x25, 0x00 },
    /* FSCAL0  */ { 0x26, 0x1F },
    /* TEST2   */ { 0x2C, 0x81 },
    /* TEST1   */ { 0x2D, 0x35 },
    /* TEST0   */ { 0x2E, 0x09 },
};

#define CC1101_XTAL_FREQ 26000000UL

static void _set_freq_regs(cc1101_t *dev, float freq_mhz)
{
    uint32_t reg_val = (uint32_t)((double)freq_mhz * 1e6 * 65536.0 / (double)CC1101_XTAL_FREQ);
    uint8_t freq2 = (reg_val >> 16) & 0xFF;
    uint8_t freq1 = (reg_val >>  8) & 0xFF;
    uint8_t freq0 = (reg_val      ) & 0xFF;

    cc1101_write_reg(dev, CC1101_FREQ2, freq2);
    cc1101_write_reg(dev, CC1101_FREQ1, freq1);
    cc1101_write_reg(dev, CC1101_FREQ0, freq0);

    ESP_LOGI(TAG, "Freq: %.2f MHz → FREQ=0x%02X%02X%02X", freq_mhz, freq2, freq1, freq0);
}

/* ─── 공개 API 구현 ───────────────────────────── */

bool cc1101_init(cc1101_t *dev)
{
    /* ★ SPI 드라이버 인계 전에 TI 수동 파워업 리셋 + 하드웨어 진단.
     *  SO(MISO) 무반응이면 전원/배선 문제로 확정 → 조기 false. */
    if (!_cc1101_manual_reset()) {
        ESP_LOGE(TAG, "CC1101 미응답 (수동 리셋 단계) — RF 비활성, 페어링은 계속");
        return false;
    }

    /* SPI 버스 초기화 */
    spi_bus_config_t buscfg = {
        .miso_io_num     = CC1101_PIN_MISO,
        .mosi_io_num     = CC1101_PIN_MOSI,
        .sclk_io_num     = CC1101_PIN_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 64,   /* DMA 미사용 → FIFO 한도(64B). 실제 최대 9B */
    };
    /* ★★★2026-08-16 SPI_DMA_CH_AUTO → SPI_DMA_DISABLED. **패닉의 직접 원인이었다.**
     *
     *  coredump 로 확인한 크래시 스택(H2, 배터리 구동 중):
     *      #0 memcpy
     *      #1 uninstall_priv_desc   spi_master.c:1181
     *      #2 setup_priv_desc       spi_master.c:1260
     *      #3 spi_device_polling_start
     *
     *  기전: 위 spi_write/spi_read 는 **스택 버퍼**(VLA)를 넘긴다. 스택은 DMA 불가라
     *  DMA 가 켜져 있으면 IDF 가 **전송할 때마다 바운스 버퍼를 heap 에 할당**한다
     *  (setup_dma_priv_buffer). heap 이 모자라 그 할당이 실패하면 `goto clean_up` 로
     *  가는데, 거기서 부르는 uninstall_priv_desc 가
     *      memcpy(orig_rx_buffer, trans_buf->buffer_to_rcv, ...)
     *  를 하는데 **buffer_to_rcv 는 성공 경로에서만 대입된다** → 초기화 안 된
     *  쓰레기 포인터로 memcpy → 패닉. (IDF 버그: OOM 을 크래시로 바꾼다.)
     *
     *  CC1101 은 최대 전송이 PATABLE 8B(+주소 1B)=9B 뿐이라 DMA 가 애초에 무의미하다.
     *  끄면 FIFO 직접 전송(64B 한도)이 되어 **할당이 아예 없어지고**, 크래시 경로가
     *  사라지는 동시에 전송마다 나던 malloc/free 도 없어져 heap 압박도 준다. */
    esp_err_t e = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize 실패: %s — RF 비활성", esp_err_to_name(e));
        return false;
    }

    /* ★ PCB v3.0: SCK=IO6(FSPICLK)/MOSI=IO7(FSPID)/MISO=IO2(FSPIQ) →
     *  esp-idf SPI 드라이버가 GPIO-matrix 우회 없이 IO-MUX 직결 경로를
     *  자동 선택. 따라서 input_delay_ns 보정/1MHz 강등 모두 불필요.
     *  CC1101 SPI 최대 6.5MHz(burst) → 안정 마진 위해 4MHz 사용. */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz   = 4 * 1000 * 1000,   /* IO-MUX 직결: 4MHz 안정 */
        .mode             = 0,                  /* CC1101 = SPI mode 0 */
        .spics_io_num     = CC1101_PIN_CS,
        .queue_size       = 7,
        .pre_cb           = NULL,
    };
    if (spi_bus_add_device(SPI2_HOST, &devcfg, &dev->spi) != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device 실패 — RF 비활성");
        return false;
    }

    /* GD0 핀 GPIO 설정 (출력, Async TX에서 MCU가 드라이브) */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CC1101_PIN_GD0),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(CC1101_PIN_GD0, 0);

    /* CC1101 리셋 */
    cc1101_strobe(dev, CC1101_SRES);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 칩 파트넘버 확인.
     * ★ CC1101 프로토콜: 상태 레지스터(0x30~0x3D, PARTNUM=0x30 VERSION=0x31)
     *  는 헤더 바이트에 BURST 비트(0x40)도 함께 세워야 한다(=0xC0|addr).
     *  R/W(0x80)만 세우면(0x80|addr) 칩이 그 영역을 *커맨드 스트로브* 로
     *  해석해 레지스터 값을 절대 못 읽는다(→ 항상 0xFF/0x00 처럼 보임).
     *  0xC0|addr = R/W=1, BURST=1 → 올바른 status register read. */
    uint8_t partnum = cc1101_read_reg(dev, 0x30 | 0xC0);
    uint8_t version = cc1101_read_reg(dev, 0x31 | 0xC0);
    ESP_LOGI(TAG, "CC1101 PARTNUM=0x%02X VERSION=0x%02X", partnum, version);
    /* ★ CC1101 부재/무응답 감지 (SPI all-1s=0xFF 또는 all-0s=0x00) →
     *   graceful false. 정상 CC1101 VERSION 은 0x14/0x04 등 비-0xFF/0x00. */
    if ((version == 0xFF && partnum == 0xFF) || (version == 0x00 && partnum == 0x00)) {
        ESP_LOGE(TAG, "CC1101 미응답 (배선/하드웨어 없음) — RF 비활성, 페어링은 계속");
        return false;
    }

    /* 레지스터 일괄 설정 */
    for (int i = 0; i < sizeof(cc1101_init_regs) / sizeof(cc1101_init_regs[0]); i++) {
        cc1101_write_reg(dev, cc1101_init_regs[i][0], cc1101_init_regs[i][1]);
    }

    /* ★★ PA 출력 +10dBm → 0dBm 감소. SDR FSK 천이 실측: +10dBm 일 때
     *  우리 신호에 위상잡음·오버슈트·글리치(240~300µs 스파이크)가 있어
     *  펄스폭이 왜곡되고 모터가 frame 비트를 오독한다. PA 출력이 높을수록
     *  강한 RF 가 VCO 를 끌어당겨(VCO pulling) 주파수가 흔들린다.
     *  0dBm 으로 낮추면 pulling 이 크게 줄어 FSK 천이가 깨끗해진다.
     *  모터와 10~30cm 면 0dBm 으로 충분(테스트는 초근접 → 여유 충분). */
    cc1101_set_pa_table(dev, CC1101_TX_POWER_0DBM);

    dev->freq_mhz = 447.72f;   /* SDR 실측 정품 447.678 + crystal 오차 보정 */
    _set_freq_regs(dev, dev->freq_mhz);

    cc1101_strobe(dev, CC1101_SIDLE);

    ESP_LOGI(TAG, "CC1101 초기화 완료");
    return true;
}

void cc1101_set_frequency(cc1101_t *dev, float freq_mhz)
{
    dev->freq_mhz = freq_mhz;
    cc1101_strobe(dev, CC1101_SIDLE);
    _set_freq_regs(dev, freq_mhz);
    cc1101_strobe(dev, CC1101_SCAL);
    /* ★2026-08-14 vTaskDelay → esp_rom_delay_us.
     *  pdMS_TO_TICKS(2) 는 tick 100Hz 에서 0 tick → 대기 소멸 → SCAL(주파수
     *  캘리브레이션, ~720us) 완료 전에 다음 동작이 나간다. tick 무관으로 고정. */
    esp_rom_delay_us(2000);
}

float cc1101_get_frequency(const cc1101_t *dev)
{
    return dev->freq_mhz;
}

/* MARCSTATE 가 목표값이 될 때까지 polling (esp_rom_delay_us, 상한 max_us). */
static uint8_t _wait_marcstate(cc1101_t *dev, uint8_t want, int max_us)
{
    uint8_t m = 0;
    for (int t = 0; t <= max_us; t += 200) {
        spi_read(dev, 0x35 | 0xC0, &m, 1);   /* MARCSTATE burst read */
        if ((m & 0x1F) == want) return m & 0x1F;
        esp_rom_delay_us(200);
    }
    return m & 0x1F;
}

void cc1101_enter_tx_mode(cc1101_t *dev)
{
    /* ★★ 근본 수정 (PROG/UP/DOWN 무반응의 칩-레벨 원인 — 실측 확정):
     *  부팅 후 '첫' STX 만 TX 진입 성공하고, 이후 모든 STX 는 IDLE/STARTCAL
     *  에 갇혀 RF 가 안 나갔다. 원인: cc1101_enter_tx_mode 가 SIDLE strobe
     *  '직후' 곧바로 STX 를 쏨 — SIDLE→IDLE 천이가 끝나기 전(직전 송신·
     *  calibration 잔류 상태)에 STX 가 들어가면 무시된다.
     *  → SIDLE 후 MARCSTATE 가 IDLE(0x01) 로 안정된 것을 확인하고 STX,
     *    다시 TX(0x13) 안정을 확인한다. 실패 시 1회 재시도. */
    for (int attempt = 0; attempt < 2; attempt++) {
        cc1101_strobe(dev, CC1101_SIDLE);
        uint8_t ms_idle = _wait_marcstate(dev, 0x01, 8000);   /* IDLE 대기 8ms */
        cc1101_strobe(dev, CC1101_STX);
        uint8_t ms_tx = _wait_marcstate(dev, 0x13, 8000);     /* TX 대기 8ms */
        if (ms_tx == 0x13) return;                            /* TX 진입 성공 */
        ESP_LOGW(TAG, "TX 진입 재시도 %d: IDLE후=0x%02X STX후=0x%02X",
                 attempt, ms_idle, ms_tx);
    }
    ESP_LOGW(TAG, "TX 진입 최종 실패 — RF 미방사");
}

void cc1101_idle(cc1101_t *dev)
{
    cc1101_strobe(dev, CC1101_SIDLE);
    gpio_set_level(CC1101_PIN_GD0, 0);
}

void cc1101_write_reg(cc1101_t *dev, uint8_t addr, uint8_t val)
{
    spi_write(dev, addr, &val, 1);
}

uint8_t cc1101_read_reg(cc1101_t *dev, uint8_t addr)
{
    uint8_t val = 0;
    spi_read(dev, addr, &val, 1);
    return val;
}

void cc1101_strobe(cc1101_t *dev, uint8_t cmd)
{
    spi_strobe(dev, cmd);
}

uint8_t cc1101_get_status(cc1101_t *dev)
{
    return spi_strobe(dev, CC1101_SNOP);
}

void cc1101_set_pa_table(cc1101_t *dev, uint8_t pa_value)
{
    /* ★ 2-FSK 용: carrier 가 항상 ON 이므로 PA_TABLE[0] 에 출력 파워를
     *  넣는다(FREND0.PA_POWER=0 → index 0 사용). OOK 시절엔 [0]=0(off),
     *  [1]=파워 였으나 FSK 는 진폭 일정 — 모든 entry 를 파워로 채운다. */
    uint8_t table[8] = { pa_value, pa_value, pa_value, pa_value,
                         pa_value, pa_value, pa_value, pa_value };
    spi_write(dev, 0x3E | 0x40, table, 8);
}
