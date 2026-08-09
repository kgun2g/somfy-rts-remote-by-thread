/*
 * oled_only_test.c — OLED 단독 부팅 테스트 (2026-07-22)
 * ─────────────────────────────────────────────────────────────────────
 * 목적: OLED 를 제외한 모든 기능(Matter/Thread/BLE/CC1101 RF/PCF8574 버튼/
 *   배터리 ADC/진동/절전)을 **전혀 초기화하지 않고**, OLED I2C(22/23)만 단독
 *   구동한다. app_main 최상단에서 이 함수로 단락 → 다른 어떤 코드도 실행되지
 *   않으므로 OLED 외 나머지 핀 입력은 자동으로 무시된다.
 *
 * 활성화: build.ps1 -OledTest  → -D OLED_ONLY_TEST=1 (CMakeLists 가 이 소스 추가).
 *
 * 동작:
 *   1) I2CDIAG   : 22/23 핀 pull 특성(float/내부PD/내부PU) → 외부풀업/stuck 판별
 *   2) bit-bang  : IDF 드라이버 우회 순수 GPIO 로 0x08~0x77 스캔 → 응답주소
 *   3) FORCE     : HW I2C 로 0x3C 에 직접 write → ACK/timeout
 *   4) 패널 init + "OLED TEST" 표시
 *   5) 무한 루프 : frame 카운터/반전바 갱신 + 매 5초 write 결과 로그
 *      → OLED 정상이면 화면이 갱신되고 write=ESP_OK, 고장이면 무반응+INVALID_STATE
 * ───────────────────────────────────────────────────────────────────── */
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "ssd1306.h"
#include "board_select.h"      /* BOARD_PIN_OLED_SDA/SCL, BOARD_OLED_WIDTH/HEIGHT */

#define TAG      "OLED_ONLY"
#define OT_SDA   ((gpio_num_t)BOARD_PIN_OLED_SDA)   /* GPIO22 (D4) */
#define OT_SCL   ((gpio_num_t)BOARD_PIN_OLED_SCL)   /* GPIO23 (D5) */

/* open-drain 에뮬: v=1 릴리즈(입력, 외부풀업 HIGH) / v=0 LOW 구동 */
static inline void _bb(gpio_num_t p, int v) {
    if (v) gpio_set_direction(p, GPIO_MODE_INPUT);
    else { gpio_set_level(p, 0); gpio_set_direction(p, GPIO_MODE_OUTPUT); }
}
/* 순수 GPIO bit-bang write-probe (IDF I2C 완전 우회). true=ACK.
 *  ★2026-07-23: 핀을 인자로 받아 **정방향/뒤바뀜 배선 둘 다** 시험할 수 있게 함.
 *  회로 시뮬레이션(sim/tools/oled_i2c_fault_sim.py) 결과, COM7 실측
 *  (SDA/SCL 모두 1/1/1 + 전주소 무응답)과 일치하는 모드는
 *    ① 모듈 미연결  ② 모듈 SDA↔SCL 뒤바뀜  ③ 칩 사망
 *  뿐이고, 사용자가 "모듈 정상 연결" 확인 → ②를 펌웨어로 직접 검증한다.
 *  (시중 0.96" 모듈은 GND-VCC-SCL-SDA 와 GND-VCC-SDA-SCL 두 변종이 흔함) */
static bool _bb_probe_pins(gpio_num_t sda, gpio_num_t scl, uint8_t addr7) {
    gpio_reset_pin(sda); gpio_reset_pin(scl);
    gpio_set_level(sda, 0); gpio_set_level(scl, 0);
    _bb(sda, 1); _bb(scl, 1); esp_rom_delay_us(10);
    _bb(sda, 0); esp_rom_delay_us(5);            /* START */
    _bb(scl, 0); esp_rom_delay_us(5);
    uint8_t byte = (uint8_t)(addr7 << 1);        /* write */
    for (int i = 0; i < 8; i++) {
        _bb(sda, (byte & 0x80) ? 1 : 0); byte <<= 1; esp_rom_delay_us(3);
        _bb(scl, 1); esp_rom_delay_us(5);
        _bb(scl, 0); esp_rom_delay_us(3);
    }
    _bb(sda, 1); esp_rom_delay_us(3);            /* ACK 읽기 */
    _bb(scl, 1); esp_rom_delay_us(5);
    int ack = gpio_get_level(sda);
    _bb(scl, 0); esp_rom_delay_us(5);
    _bb(sda, 0); esp_rom_delay_us(3);            /* STOP */
    _bb(scl, 1); esp_rom_delay_us(5);
    _bb(sda, 1); esp_rom_delay_us(5);
    return ack == 0;
}
static bool _bb_probe(uint8_t addr7) { return _bb_probe_pins(OT_SDA, OT_SCL, addr7); }

/* 한 배선방향으로 0x08~0x77 전체 스캔 → 응답 주소 문자열. 반환: 응답 개수 */
static int _bb_scan_pins(gpio_num_t sda, gpio_num_t scl, char *out, size_t outsz) {
    int n = 0, cnt = 0; out[0] = '\0';
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (_bb_probe_pins(sda, scl, a)) {
            cnt++;
            if (n < (int)outsz - 6) n += snprintf(out + n, outsz - n, "0x%02X ", a);
        }
    }
    if (!out[0]) snprintf(out, outsz, "(없음)");
    return cnt;
}

/* ★2026-07-23 상승시간(RC) 측정 — "모듈에 전원이 실제로 들어오는가"를 정량 판별.
 *  배경: h4(COM7)는 PCB 에 4.7K 풀업(R8/R9)이 있어, 모듈이 무전원/미연결이어도
 *  버스가 HIGH 로 보인다(디지털 읽기로는 구분 불가). 그러나 **모듈이 살아 있으면
 *  모듈 온보드 풀업(≈4.7K)이 병렬로 붙어 합성 2.35K → 상승시간이 절반**이 된다.
 *  라인을 LOW 로 방전시킨 뒤 릴리즈하고 HIGH 로 읽힐 때까지의 폴링 횟수를 센다.
 *  v4(COM4)는 PCB 풀업이 없어 **모듈 풀업 단독** → 두 보드 수치를 비교하면
 *  COM7 모듈의 풀업 기여 여부(=전원 인가 여부)를 알 수 있다.
 *  (절대값은 보드/온도마다 다르므로 SDA/SCL 및 보드 간 **상대비교**로 해석) */
/* 부팅 혼잡 구간 로그 유실 대비 — 진단 결과를 저장해 루프에서 재출력 */
static int s_xconn_base_sda = -1, s_xconn_drv_sda = -1;
static int s_xconn_base_scl = -1, s_xconn_drv_scl = -1, s_xconn_shorted = -1;
static int s_recover_released = -1, s_recover_sda_after = -1;

static uint32_t _rise_ticks(gpio_num_t pin)
{
    uint32_t best = 0xFFFFFFFFu;
    for (int rep = 0; rep < 8; rep++) {
        gpio_reset_pin(pin);
        gpio_set_level(pin, 0);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);   /* 강제 LOW 로 방전 */
        esp_rom_delay_us(30);
        gpio_set_direction(pin, GPIO_MODE_INPUT);    /* 릴리즈 → 풀업이 끌어올림 */
        uint32_t n = 0;
        while (n < 20000 && gpio_get_level(pin) == 0) n++;
        if (n < best) best = n;                      /* 최소값 = 인터럽트 방해 없는 측정 */
    }
    return best;
}

void oled_only_test(void)
{
    ESP_LOGW(TAG, "================ OLED ONLY TEST ================");
    ESP_LOGW(TAG, "OLED 단독 부팅 — Matter/Thread/BLE/RF/버튼/배터리 전부 비활성. SDA=IO%d SCL=IO%d",
             (int)OT_SDA, (int)OT_SCL);

    /* 1) 핀 pull 특성 (외부풀업 존재 / GND stuck 판별) */
    for (int i = 0; i < 2; i++) {
        gpio_num_t pin = i ? OT_SCL : OT_SDA;
        const char *nm = i ? "SCL(23)" : "SDA(22)";
        gpio_reset_pin(pin); gpio_set_direction(pin, GPIO_MODE_INPUT);
        gpio_set_pull_mode(pin, GPIO_FLOATING);      vTaskDelay(pdMS_TO_TICKS(2)); int lf = gpio_get_level(pin);
        gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY); vTaskDelay(pdMS_TO_TICKS(2)); int pd = gpio_get_level(pin);
        gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);   vTaskDelay(pdMS_TO_TICKS(2)); int pu = gpio_get_level(pin);
        gpio_set_pull_mode(pin, GPIO_FLOATING);
        ESP_LOGW(TAG, "[I2CDIAG] %s float=%d int-PD=%d(HIGH=외부풀업존재) int-PU=%d(LOW=GND단락)",
                 nm, lf, pd, pu);
    }

    /* ★1b) 교차연결(스위치 위치) 검사 — 2026-07-23 회로도 분석으로 발견.
     *  h4(신 PCB)에는 I2C 라인에 SPDT 슬라이드 스위치(SDA1/SCL1)가 있어
     *    C=I2C(GPIO22/23) ─ 스위치 ─ B=PCF8575  또는  A=LP_I2C(GPIO6/7)
     *  스위치가 A 위치면 GPIO22↔GPIO6, GPIO23↔GPIO7 이 물리적으로 단락된다.
     *  → 정상 펌웨어의 LP 비트뱅(GPIO6/7 구동)이 OLED 버스를 직접 깨뜨림.
     *  검사법: LP 핀을 LOW 로 구동하고 OLED 핀 레벨을 읽는다. 따라가면 단락. */
    {
        const gpio_num_t LP_SDA = (gpio_num_t)6, LP_SCL = (gpio_num_t)7;
        gpio_reset_pin(OT_SDA); gpio_set_direction(OT_SDA, GPIO_MODE_INPUT);
        gpio_set_pull_mode(OT_SDA, GPIO_PULLUP_ONLY);
        gpio_reset_pin(OT_SCL); gpio_set_direction(OT_SCL, GPIO_MODE_INPUT);
        gpio_set_pull_mode(OT_SCL, GPIO_PULLUP_ONLY);
        gpio_reset_pin(LP_SDA); gpio_reset_pin(LP_SCL);
        /* LP 를 입력(방치) 상태에서 기준 레벨 */
        gpio_set_direction(LP_SDA, GPIO_MODE_INPUT); gpio_set_direction(LP_SCL, GPIO_MODE_INPUT);
        vTaskDelay(pdMS_TO_TICKS(2));
        int base_sda = gpio_get_level(OT_SDA), base_scl = gpio_get_level(OT_SCL);
        /* LP 를 LOW 로 구동 */
        gpio_set_level(LP_SDA, 0); gpio_set_direction(LP_SDA, GPIO_MODE_OUTPUT);
        gpio_set_level(LP_SCL, 0); gpio_set_direction(LP_SCL, GPIO_MODE_OUTPUT);
        vTaskDelay(pdMS_TO_TICKS(2));
        int drv_sda = gpio_get_level(OT_SDA), drv_scl = gpio_get_level(OT_SCL);
        gpio_set_direction(LP_SDA, GPIO_MODE_INPUT); gpio_set_direction(LP_SCL, GPIO_MODE_INPUT);
        gpio_set_pull_mode(OT_SDA, GPIO_FLOATING); gpio_set_pull_mode(OT_SCL, GPIO_FLOATING);
        int shorted = (base_sda == 1 && drv_sda == 0) || (base_scl == 1 && drv_scl == 0);
        /* ★부팅 혼잡 구간엔 USB-JTAG 이 로그를 버린다(실측). 결과를 저장해 아래 루프에서 재출력. */
        s_xconn_base_sda = base_sda; s_xconn_drv_sda = drv_sda;
        s_xconn_base_scl = base_scl; s_xconn_drv_scl = drv_scl; s_xconn_shorted = shorted;
        ESP_LOGW(TAG, "[XCONN] LP(6/7) LOW 구동 시 OLED핀: SDA %d→%d, SCL %d→%d  ⇒ %s",
                 base_sda, drv_sda, base_scl, drv_scl,
                 shorted ? "★단락됨(스위치 A위치) — LP 비트뱅이 OLED 버스를 깸!"
                         : "독립(정상) — 스위치 B/무관, LP 간섭 없음");
    }

    /* ★1c) I2C 버스 복구 — SDA 가 LOW 로 붙잡혀 있으면(slave 가 전송 중 리셋된 상태)
     *  SCL 9클럭 + STOP 으로 풀어준다. I2C 표준 복구 절차. SDA 가 이미 HIGH 면 건너뜀
     *  (정상 버스에 쓰면 오히려 해로움). */
    {
        gpio_reset_pin(OT_SDA); gpio_set_direction(OT_SDA, GPIO_MODE_INPUT);
        gpio_set_pull_mode(OT_SDA, GPIO_PULLUP_ONLY); vTaskDelay(pdMS_TO_TICKS(2));
        /* ★2026-07-23 변경: SDA HIGH 여도 복구를 시도한다.
         *  실측(COM7 프리즈): 버스는 idle(1/1)인데 모듈만 무응답 = 슬레이브 내부 상태 고착.
         *  이 경우 SDA 는 HIGH 라 기존 조건(SDA==0)에 안 걸려 복구가 건너뛰어졌다.
         *  표준 9클럭+STOP 은 idle 버스에서도 무해하므로 항상 시도하고 결과를 비교한다. */
        {
            ESP_LOGW(TAG, "[RECOVER] SDA=%d — 9클럭+STOP 복구 무조건 시도(고착 슬레이브 해제 목적)",
                     gpio_get_level(OT_SDA));
            gpio_reset_pin(OT_SCL); gpio_set_level(OT_SCL, 1);
            int released = 0;
            for (int i = 0; i < 9; i++) {
                _bb(OT_SCL, 0); esp_rom_delay_us(6);
                _bb(OT_SCL, 1); esp_rom_delay_us(6);
                if (gpio_get_level(OT_SDA)) { released = 1; break; }
            }
            _bb(OT_SDA, 0); esp_rom_delay_us(6);   /* STOP: SCL HIGH 중 SDA LOW→HIGH */
            _bb(OT_SCL, 1); esp_rom_delay_us(6);
            _bb(OT_SDA, 1); esp_rom_delay_us(6);
            gpio_set_pull_mode(OT_SDA, GPIO_PULLUP_ONLY); vTaskDelay(pdMS_TO_TICKS(2));
            s_recover_released = released; s_recover_sda_after = gpio_get_level(OT_SDA);
            ESP_LOGW(TAG, "[RECOVER] 결과: SDA=%d released=%d (%s)",
                     gpio_get_level(OT_SDA), released,
                     gpio_get_level(OT_SDA) ? "라인 정상(HIGH) — 아래 스캔으로 모듈 응답 확인"
                                            : "여전히 LOW — 물리 단락(복구 불가)");
        }
        gpio_set_pull_mode(OT_SDA, GPIO_FLOATING);
    }

    /* ★1d) 상승시간 측정 → 모듈 풀업 존재(=모듈 전원 인가) 여부 정량 판별.
     *  판독법:
     *   · COM4(v4, PCB풀업 없음): 이 수치 = **모듈 풀업 단독**. 기준값(reference).
     *   · COM7(h4, PCB풀업 4.7K 있음):
     *       COM4 와 비슷 → 풀업 1개뿐 = **모듈이 풀업을 안 걸고 있음 = 모듈 무전원/미연결**
     *       COM4 의 약 1/2 → 풀업 2개 병렬 = **모듈 전원 정상** (→ 칩 무응답 쪽) */
    {
        uint32_t t_sda = _rise_ticks(OT_SDA);
        uint32_t t_scl = _rise_ticks(OT_SCL);
        ESP_LOGW(TAG, "[RISE] 상승 폴링수: SDA=%u SCL=%u  (작을수록 풀업 강함=병렬 개수 많음)",
                 (unsigned)t_sda, (unsigned)t_scl);
        ESP_LOGW(TAG, "[RISE] 해석: COM4(PCB풀업X)=모듈풀업 단독 기준 / "
                      "COM7(PCB풀업O)이 COM4와 비슷하면 모듈 무전원, 절반이면 모듈 전원정상");
    }

    /* 2) bit-bang 전체 스캔 (IDF 드라이버 우회) — 정방향 배선 */
    char buf[100];
    _bb_scan_pins(OT_SDA, OT_SCL, buf, sizeof(buf));
    if (0) {   /* (구 루프 보존 — 위 헬퍼가 동일 동작) */
        int n = 0;
        for (uint8_t a = 0x08; a <= 0x77; a++) {
            if (_bb_probe(a)) {
                n += snprintf(buf + n, sizeof(buf) - n, "0x%02X ", a);
                if (n >= (int)sizeof(buf) - 6) break;
            }
        }
    }
    ESP_LOGW(TAG, "[BITBANG] 응답주소(0x08~0x77) = %s", buf[0] ? buf : "(없음)");

    /* ★2b) 배선 뒤바뀜(SDA↔SCL) 자동 검증 — 정방향 스캔이 비었을 때만.
     *  시뮬레이션이 지목한 유력 후보. 모듈이 GND-VCC-SDA-SCL 순서 변종이면
     *  SCL/SDA 가 교차 연결되어 "버스는 정상인데 영원히 무응답" 이 된다.
     *  여기서 응답이 나오면 → 펌웨어에서 핀만 맞바꾸면 즉시 해결. */
    if (buf[0] == '\0' || buf[0] == '(') {
        char sw[100];
        int cnt = _bb_scan_pins(OT_SCL, OT_SDA, sw, sizeof(sw));   /* SDA/SCL 교환 */
        ESP_LOGW(TAG, "[SWAP] 뒤바뀜 배선(SDA=IO%d,SCL=IO%d) 스캔 = %s",
                 (int)OT_SCL, (int)OT_SDA, sw);
        if (cnt == 1) {
            ESP_LOGE(TAG, "[SWAP] ★★해결책 발견: 모듈 SDA/SCL 이 뒤바뀌어 연결됨!");
            ESP_LOGE(TAG, "[SWAP] ★★boards/xiao-c6.h 에서 BOARD_PIN_OLED_SDA=%d, "
                          "BOARD_PIN_OLED_SCL=%d 로 맞바꾸면 동작합니다.",
                     (int)OT_SCL, (int)OT_SDA);
        } else if (cnt > 1) {
            ESP_LOGW(TAG, "[SWAP] 다수 응답(%d) = 교환배선에서 SDA 고착 — 유효한 결과 아님", cnt);
        } else {
            ESP_LOGW(TAG, "[SWAP] 뒤바뀜도 무응답 → 배선방향 문제 아님(모듈 사망/미연결 쪽)");
        }
    }

    /* 3) HW I2C init + 0x3C 강제 write */
    SSD1306_t dev;
    memset(&dev, 0, sizeof(dev));
    i2c_master_init(&dev, BOARD_PIN_OLED_SDA, BOARD_PIN_OLED_SCL, -1);
    uint8_t cmd_on[2]  = { 0x00, 0xAF };   /* command stream + display ON */
    esp_err_t r = i2c_master_transmit(dev._i2c_dev_handle, cmd_on, 2, 100);
    ESP_LOGW(TAG, "[FORCE] 0x3C 직접 write = %s  (ESP_OK=모듈 ACK/살아있음, 그 외=무응답)",
             esp_err_to_name(r));

    /* ★부팅 혼잡 구간에 유실되는 진단 결과를 여기서 한 번 더(양쪽 분기 공통) 출력.
     *  실측: 부팅 100ms 근처 로그는 USB-JTAG 이 버린다. 200ms 지연 후 재출력하면 확실히 나간다. */
    /* ★긴 한글/기호 줄은 USB-JTAG 에서 계속 유실됐다(실측). 짧은 ASCII 한 줄로 고정. */
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGE(TAG, "XCONN sda=%d>%d scl=%d>%d SHORT=%d",
             s_xconn_base_sda, s_xconn_drv_sda,
             s_xconn_base_scl, s_xconn_drv_scl, s_xconn_shorted);
    ESP_LOGE(TAG, "RECOVER rel=%d sda=%d", s_recover_released, s_recover_sda_after);

    /* 4) 응답 여부로 분기 */
    if (r == ESP_OK) {
        /* ── 모듈 살아있음 → 패널 init + 화면 테스트 루프(애니메이션 표시) ── */
        ssd1306_init(&dev, BOARD_OLED_WIDTH, BOARD_OLED_HEIGHT);
        ssd1306_contrast(&dev, 0xFF);
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 0, " OLED TEST", 10, false);
        ssd1306_display_text(&dev, 2, " standalone", 11, false);
        int frame = 0;
        while (1) {
            char line[20];
            snprintf(line, sizeof(line), " frame %d", frame);
            ssd1306_display_text(&dev, 4, line, (int)strlen(line), false);
            if (frame & 1) ssd1306_display_text(&dev, 6, "##########", 10, true);
            else           ssd1306_clear_line(&dev, 6, false);
            ESP_LOGW(TAG, "[LOOP] frame=%d — 화면 갱신중 (OLED 정상 동작)", frame);
            frame++;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } else {
        /* ── 모듈 무응답 → SSD1306 라이브러리 호출은 죽은 버스에서 매 write 100ms
         *  타임아웃+에러도배라 무의미. 경량 bit-bang 스캔을 2초마다 반복해 깔끔하게
         *  상태 재확인(정상 OLED 로 연결/교체하면 이 루프가 응답 감지). ── */
        ESP_LOGW(TAG, "0x3C 무응답 → 패널 init 스킵(라이브러리 에러도배 방지). bit-bang 재스캔 반복.");
        int frame = 0;
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            char sb[100]; int m = 0; sb[0] = '\0';
            for (uint8_t a = 0x08; a <= 0x77; a++) {
                if (_bb_probe(a)) {
                    m += snprintf(sb + m, sizeof(sb) - m, "0x%02X ", a);
                    if (m >= (int)sizeof(sb) - 6) break;
                }
            }
            gpio_reset_pin(OT_SDA); gpio_set_direction(OT_SDA, GPIO_MODE_INPUT);
            gpio_set_pull_mode(OT_SDA, GPIO_FLOATING); vTaskDelay(pdMS_TO_TICKS(2));
            int sda = gpio_get_level(OT_SDA);
            ESP_LOGW(TAG, "[LOOP] frame=%d 무응답 (bit-bang=%s, SDA idle=%d) — 정상 OLED 연결/교체 시 화면 시작",
                     frame, sb[0] ? sb : "(없음)", sda);
            /* ★부팅 때 유실된 진단 결과를 여기서 재출력(혼잡 구간 지난 뒤라 확실히 나감) */
            if (frame < 3) {
                ESP_LOGE(TAG, "XCONN sda=%d>%d scl=%d>%d SHORT=%d",
                         s_xconn_base_sda, s_xconn_drv_sda,
                         s_xconn_base_scl, s_xconn_drv_scl, s_xconn_shorted);
            }
            frame++;
        }
    }
}
