#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "boards/board_select.h"   // BOARD_PIN_CC1101_* (보드별 핀맵)

#ifdef __cplusplus
extern "C" {
#endif

/* ─── 핀 설정 (PCB v3.0 — FSPI IO-MUX 네이티브) ────────────────
 *  ★ GNPE ESP32-C6-0.42 스키매틱 + ESP32-C6 datasheet 교차분석 결과:
 *    이전 (SCK=IO7, MOSI=IO6, MISO=IO5) 배선은 GPSPI2(FSPI)의
 *    IO-MUX 고정핀과 클럭/데이터가 X자로 교차되어 SPI 가 전부
 *    GPIO-matrix 우회로 라우팅 → input_delay_ns 땜질 + 1MHz 강등이
 *    필요했고, "SCK/MOSI 신호경로 결함"으로 보였던 정체가 사실상
 *    이 매트릭스 우회였음. 예전 펌웨어-단독 스왑 테스트가 깨진 것은
 *    모듈 배선을 함께 옮기지 않아 물리적으로 클럭/데이터가 뒤바뀐 탓.
 *
 *  ESP32-C6 GPSPI2 IO-MUX 고정핀:
 *    FSPICLK = GPIO6 , FSPID(MOSI) = GPIO7 , FSPIQ(MISO) = GPIO2
 *
 *  ★ 하드웨어 점퍼 이동 필요(P1 헤더 내, 펌웨어와 동시 적용):
 *    SCK  : IO7 → IO6  (P1-6 → P1-5)   [FSPICLK 네이티브]
 *    MOSI : IO6 → IO7  (P1-5 → P1-6)   [FSPID  네이티브]  ※SCK과 맞바꿈
 *    MISO : IO5 → IO2  (P1-4 → P1-1)   [FSPIQ  네이티브]
 *    CSN  : IO4 (유지, P1-3)  — CS 는 클럭X, 매트릭스 무방
 *    GD0  : IO8 (유지, P1-7, RMT async TX 데이터)
 *  → 세 핀 모두 IO-MUX 직결 → input_delay_ns 불필요, 클럭 상향 안정.
 * ─────────────────────────────────────────────── */
/* 핀 번호는 boards/<board>.h 의 BOARD_PIN_CC1101_* 에서 가져온다. */
#define CC1101_PIN_SCK      ((gpio_num_t)BOARD_PIN_CC1101_SCK)   // FSPICLK
#define CC1101_PIN_MISO     ((gpio_num_t)BOARD_PIN_CC1101_MISO)  // FSPIQ
#define CC1101_PIN_MOSI     ((gpio_num_t)BOARD_PIN_CC1101_MOSI)  // FSPID
#define CC1101_PIN_CS       ((gpio_num_t)BOARD_PIN_CC1101_CS)
#define CC1101_PIN_GD0      ((gpio_num_t)BOARD_PIN_CC1101_GD0)   // Async TX 데이터 입력 핀

/* ─── CC1101 레지스터 주소 ────────────────────── */
#define CC1101_IOCFG2       0x00
#define CC1101_IOCFG1       0x01
#define CC1101_IOCFG0       0x02
#define CC1101_FIFOTHR      0x03
#define CC1101_SYNC1        0x04
#define CC1101_SYNC0        0x05
#define CC1101_PKTLEN       0x06
#define CC1101_PKTCTRL1     0x07
#define CC1101_PKTCTRL0     0x08
#define CC1101_ADDR         0x09
#define CC1101_CHANNR       0x0A
#define CC1101_FSCTRL1      0x0B
#define CC1101_FSCTRL0      0x0C
#define CC1101_FREQ2        0x0D
#define CC1101_FREQ1        0x0E
#define CC1101_FREQ0        0x0F
#define CC1101_MDMCFG4      0x10
#define CC1101_MDMCFG3      0x11
#define CC1101_MDMCFG2      0x12
#define CC1101_MDMCFG1      0x13
#define CC1101_MDMCFG0      0x14
#define CC1101_DEVIATN      0x15
#define CC1101_MCSM2        0x16
#define CC1101_MCSM1        0x17
#define CC1101_MCSM0        0x18
#define CC1101_FOCCFG       0x19
#define CC1101_BSCFG        0x1A
#define CC1101_AGCCTRL2     0x1B
#define CC1101_AGCCTRL1     0x1C
#define CC1101_AGCCTRL0     0x1D
#define CC1101_WOREVT1      0x1E
#define CC1101_WOREVT0      0x1F
#define CC1101_WORCTRL      0x20
#define CC1101_FREND1       0x21
#define CC1101_FREND0       0x22
#define CC1101_FSCAL3       0x23
#define CC1101_FSCAL2       0x24
#define CC1101_FSCAL1       0x25
#define CC1101_FSCAL0       0x26
#define CC1101_RCCTRL1      0x27
#define CC1101_RCCTRL0      0x28
#define CC1101_FSTEST       0x29
#define CC1101_PTEST        0x2A
#define CC1101_AGCTEST      0x2B
#define CC1101_TEST2        0x2C
#define CC1101_TEST1        0x2D
#define CC1101_TEST0        0x2E

/* ─── CC1101 커맨드 스트로브 ──────────────────── */
#define CC1101_SRES         0x30   // 리셋
#define CC1101_SFSTXON      0x31   // PLL 켜기
#define CC1101_SXOFF        0x32   // 크리스탈 끄기
#define CC1101_SCAL         0x33   // PLL 캘리브레이션
#define CC1101_SRX          0x34   // RX 모드
#define CC1101_STX          0x35   // TX 모드
#define CC1101_SIDLE        0x36   // IDLE 모드
#define CC1101_SWOR         0x38   // Wake-on-Radio
#define CC1101_SPWD         0x39   // 파워다운
#define CC1101_SFRX         0x3A   // RX FIFO flush
#define CC1101_SFTX         0x3B   // TX FIFO flush
#define CC1101_SWORRST      0x3C   // WOR 타이머 리셋
#define CC1101_SNOP         0x3D   // NOP

/* ─── CC1101 PATABLE ────────────────────────── */
#define CC1101_PATABLE      0x3E   // TX 전력 테이블

/* ─── CC1101 상태 ────────────────────────────── */
#define CC1101_STATUS_IDLE      0x00
#define CC1101_STATUS_RX        0x10
#define CC1101_STATUS_TX        0x20
#define CC1101_STATUS_FSTXON    0x30
#define CC1101_STATUS_CALIBRATE 0x40
#define CC1101_STATUS_SETTLING  0x50
#define CC1101_STATUS_RXFIFO_OVF 0x60
#define CC1101_STATUS_TXFIFO_UVF 0x70

/* ─── 주파수 계산 ─────────────────────────────
   Fcarrier = Fxosc × (FREQ / 2^16)
   Fxosc = 26 MHz
   FREQ = Fcarrier × 2^16 / Fxosc
   예) 447.20 MHz → FREQ = 447200000 * 65536 / 26000000 = 1127141
────────────────────────────────────────────── */
#define CC1101_XTAL_FREQ    26000000UL
#define CC1101_FREQ_MHZ_TO_REG(mhz) \
    ((uint32_t)((double)(mhz) * 1e6 * 65536.0 / CC1101_XTAL_FREQ))

/* ─── TX 전력 설정 (433/447MHz PA 테이블 값) ───
 *  ★ VCO pulling 완화: OOK 송신 시 carrier on/off 마다 PA 전류가 급변해
 *   VCO 주파수가 끌리고(SDR 워터폴 가로 번짐), 안테나 매칭 부정합 시
 *   load pulling 까지 겹친다. PA 출력을 낮추면 PA 전류·load pulling 이
 *   줄어 스펙트럼이 좁고 깨끗해진다(모터가 10~30cm 면 0dBm 도 충분). */
#define CC1101_TX_POWER_10DBM   0xC0   /* +10 dBm */
#define CC1101_TX_POWER_7DBM    0xCC   /* +7  dBm */
#define CC1101_TX_POWER_0DBM    0x84   /* 0   dBm — VCO pulling 최소 */
#define CC1101_TX_POWER_NEG6DBM 0x60   /* -6  dBm */

/* ─── 드라이버 구조체 ────────────────────────── */
typedef struct {
    spi_device_handle_t spi;
    gpio_num_t          gd0_pin;
    float               freq_mhz;   // 현재 주파수 (MHz)
    /* ★★★2026-09-01 SPWD(파워다운) 상태 플래그 — 절전.
     *  이 기기는 **송신만** 한다(수신·rfscan 없음). 그런데 지금까지 송신이 끝나면
     *  `SIDLE` 로만 두어 크리스탈이 계속 돌았다 = 데이터시트 기준 **상시 ~1.7mA**.
     *  실측 총 소비가 7.3mA 이므로 그 중 23% 를 아무 일도 안 하면서 쓰고 있었다.
     *  → 송신이 끝나면 `SPWD`, SPI 접근이 생기면 자동으로 깨운다(_wake_if_pd).
     *  ※CC1101 은 SLEEP 상태에서 **레지스터 값을 보존**하므로 재설정이 필요 없다. */
    bool                powered_down;
} cc1101_t;

/* ─── API ────────────────────────────────────── */

/**
 * @brief CC1101 초기화 (SPI + GPIO 설정 포함)
 * @return true: 성공, false: 실패 (칩 ID 불일치 등)
 */
bool cc1101_init(cc1101_t *dev);

/**
 * @brief 주파수 설정 (MHz 단위, 예: 447.20)
 */
void cc1101_set_frequency(cc1101_t *dev, float freq_mhz);

/**
 * @brief 현재 주파수 반환 (MHz)
 */
float cc1101_get_frequency(const cc1101_t *dev);

/**
 * @brief Async TX 모드 진입 (Somfy RTS 전송 전 호출)
 *        GD0 핀이 데이터 입력으로 동작
 */
void cc1101_enter_tx_mode(cc1101_t *dev);

/**
 * @brief IDLE 모드로 복귀
 */
void cc1101_idle(cc1101_t *dev);

/**
 * @brief 단일 레지스터 쓰기
 */
void cc1101_write_reg(cc1101_t *dev, uint8_t addr, uint8_t val);

/**
 * @brief 단일 레지스터 읽기
 */
uint8_t cc1101_read_reg(cc1101_t *dev, uint8_t addr);

/**
 * @brief 커맨드 스트로브 전송
 */
void cc1101_strobe(cc1101_t *dev, uint8_t cmd);

/**
 * @brief 칩 상태 반환
 */
uint8_t cc1101_get_status(cc1101_t *dev);

/**
 * @brief 파티 테이블(TX 전력) 설정
 */
void cc1101_set_pa_table(cc1101_t *dev, uint8_t pa_value);

#ifdef __cplusplus
}
#endif
