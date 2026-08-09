#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "somfy_rts.h"
#include "boards/board_select.h"   // BOARD_PIN_OLED_*
#include "driver/i2c_master.h"     // i2c_master_bus_handle_t (공유 I2C 버스 노출)

#ifdef __cplusplus
extern "C" {
#endif

/* ─── OLED 규격 — 보드별 (boards/<board>.h 의 BOARD_OLED_*) ───────────
 *  보드 헤더가 BOARD_OLED_* 를 정의하지 않으면 GNPE 0.42"(72×40) 기본값.
 *
 *  ▶ "물리 패널"  = OLED_PANEL_W × OLED_PANEL_H (예: GNPE 72×40, XIAO 128×64)
 *     - 프레임버퍼 크기 / 라이브러리 init / flush / 회전 / 패널 보정에 사용.
 *  ▶ "논리 캔버스" = OLED_WIDTH × OLED_HEIGHT (= 72×40, UI 레이아웃 설계 기준)
 *     - 모든 화면 그리기 코드는 이 좌표계로 그린다. 물리 패널이 더 크면
 *       oled_ui.c 의 origin 으로 자동 중앙 정렬된다(전체화면 레이아웃은 추후).
 * ──────────────────────────────────────────────────────────────────── */
/* BOARD_OLED_* 는 boards/<board>.h 에 정의(누락 시 board_select.h 가 GNPE 기본값). */
/* 물리 패널 — 라이브러리 init / flush 출력 차원. 항상 보드 원본 W×H. */
#define OLED_PHYS_W            BOARD_OLED_WIDTH
#define OLED_PHYS_H            BOARD_OLED_HEIGHT
#define OLED_ROTATE_90         BOARD_OLED_ROTATE_90   /* 0 / 90(시계) / 270(반시계) */
/* 렌더러 프레임버퍼 차원(OLED_PANEL_W/H) — 90/270 회전이면 물리와 가로↔세로 swap.
 *  렌더러는 이 차원으로 그리고, _fb_flush 가 물리 패널로 90° 재배치한다.
 *  (회전 0/180 이면 물리와 동일 → 기존 동작 그대로.) */
#if (OLED_ROTATE_90 == 90) || (OLED_ROTATE_90 == 270)
#  define OLED_PANEL_W         BOARD_OLED_HEIGHT   /* FB 폭 = 물리 높이 */
#  define OLED_PANEL_H         BOARD_OLED_WIDTH    /* FB 높이 = 물리 폭 */
#else
#  define OLED_PANEL_W         BOARD_OLED_WIDTH    /* 물리 패널 폭 */
#  define OLED_PANEL_H         BOARD_OLED_HEIGHT   /* 물리 패널 높이 */
#endif
#define OLED_ROTATE_180        BOARD_OLED_ROTATE_180
#define OLED_FLIP_X            BOARD_OLED_FLIP_X   /* 좌우(가로) 반전 — 회전과 독립 */
#define OLED_PANEL_FIXUP_72X40 BOARD_OLED_FIXUP_72X40
#define OLED_I2C_ADDR          BOARD_OLED_ADDR

/* 논리 UI 캔버스(레이아웃 설계 기준). 물리 패널이 더 크면 중앙 배치. */
#define OLED_WIDTH   72
#define OLED_HEIGHT  40

/* ─── 렌더러 선택 — OLED "해상도"별 (보드 무관) ───────────────────────
 *  렌더러는 보드가 아니라 **물리 패널 해상도**(OLED_PANEL_W×H)로 자동
 *  선택된다. 따라서 어느 보드든(GNPE 포함) 보드 헤더의 BOARD_OLED_* 만
 *  바꾸면 그 해상도에 맞는 렌더러가 컴파일타임에 골라진다.
 *
 *    128×64 (가로)  → OLED_RENDER_128X64 : 풀스크린 네이티브(가로 레이아웃)
 *    64×128 (세로)  → OLED_RENDER_64X128 : 풀스크린 네이티브(세로/포트레이트)
 *    그 외          → (둘 다 0)          : 기본 72×40 논리 캔버스 렌더러
 *                                          (어떤 패널에도 중앙 배치/블록 스케일)
 *
 *  ▶ 네이티브 두 해상도는 같은 프리미티브(고딕 6×9 폰트 + 7세그 시계)를
 *    공유한다 → OLED_RENDER_NATIVE. 레이아웃만 가로/세로로 분기.
 *  ▶ 새 해상도 전용 렌더러를 추가하려면:
 *     1) 여기에 `#define OLED_RENDER_XXX (OLED_PANEL_W==.. && OLED_PANEL_H==..)`
 *     2) (네이티브면) OLED_RENDER_NATIVE 에 `|| OLED_RENDER_XXX` 추가
 *     3) oled_ui.c 의 각 _render_* 함수에 `#elif OLED_RENDER_XXX` 분기 추가.
 *  ▶ 예) GNPE 에 0.96" 128×64 패널을 달면 boards/gnpe-c6.h 의 BOARD_OLED_*
 *        를 128×64 로 바꾸기만 하면 자동으로 네이티브 렌더러가 선택된다.
 * ──────────────────────────────────────────────────────────────────── */
#define OLED_RENDER_128X64  (OLED_PANEL_W == 128 && OLED_PANEL_H == 64)
#define OLED_RENDER_64X128  (OLED_PANEL_W == 64  && OLED_PANEL_H == 128)
/* 네이티브(물리 1:1) 렌더 프리미티브를 쓰는 해상도 묶음 (가로 128×64 + 세로 64×128) */
#define OLED_RENDER_NATIVE  (OLED_RENDER_128X64 || OLED_RENDER_64X128)

/* ─── OLED I2C 핀 (ESP32-C6-0.42 내부 배선, 변경 불가) ────────
 *  ESP32-C6-Zero / ESP32-C6-LCD-0.42 보드의 0.42" SSD1306 OLED는
 *  실측 silkscreen 라벨 "SDA-1" / "SCL-0" 으로 표시되어 있고
 *  실제 GPIO 번호는 다음과 같다:
 *
 *      IO1  = OLED SDA   (silkscreen "SDA-1")
 *      IO0  = OLED SCL   (silkscreen "SCL-0")
 *
 *  ESP32-C6의 GPIO0/GPIO1은 strapping 핀이 아니므로 일반 GPIO처럼
 *  자유롭게 사용 가능. Arduino 포럼 스레드 1407271 참조.
 * ─────────────────────────────────────────────── */
#define OLED_PIN_SDA    ((gpio_num_t)BOARD_PIN_OLED_SDA)
#define OLED_PIN_SCL    ((gpio_num_t)BOARD_PIN_OLED_SCL)

/* ─── 화면 상태 ──────────────────────────────── */
typedef enum {
    OLED_STATE_NORMAL          = 0,  // 일반 동작 화면
    OLED_STATE_ACTION          = 1,  // 버튼 동작 중 애니메이션
    OLED_STATE_SCREENSAVER     = 2,  // 화면 보호기
    OLED_STATE_FREQ_EDIT       = 3,  // 주파수 편집 화면
    OLED_STATE_PAIRING         = 4,  // Matter 페어링 화면 (BLE 광고 + PIN 표시)
    OLED_STATE_THREAD_PROV     = 5,  // Thread 네트워크 부착 대기/안내
    OLED_STATE_CHARGING        = 6,  // 충전 다이내믹 애니메이션 (USB 연결)
    OLED_STATE_SETUP_MENU      = 7,  // 설정 메뉴 화면 (v3.1+, 5개 항목)
    OLED_STATE_THREAD_RESET    = 8,  // Thread 리셋 확인 (SETUP 2s 길게 → 실행)
    OLED_STATE_TIME_EDIT       = 9,  // 날짜/시간 수동 설정 화면 (v3.9+)
    /* OLED_STATE_RF_SCAN = 10 — 제거됨 (Manchester 극성 정정 후 기본 freq 검증
     *   완료, 보드별 calibration 불요). 향후 다른 보드 지원 시 재도입 가능. */
    OLED_STATE_FW_UPDATE       = 11, // 펌웨어 업데이트(Matter OTA) 화면
} oled_state_t;

/* v2.x 호환 별칭 (삭제 예정) */
#define OLED_STATE_WIFI_PROV OLED_STATE_THREAD_PROV

/* ─── Matter 연결/페어링 상태 (메인 화면 상단 표시) ───
 *  UNPAIRED : fabric 없음, 페어링 대기 (BLE 광고만)        → 'X' 아이콘
 *  PAIRING  : commissioner 가 페어링 트랜잭션 진행 중      → "P" 점멸
 *  CONNECTED: 커미셔닝 완료/Matter 연결됨            → 신호막대만(RSSI 가변) */
typedef enum {
    OLED_MT_UNPAIRED  = 0,
    OLED_MT_PAIRING   = 1,
    OLED_MT_CONNECTED = 2,
} oled_matter_state_t;

#define OLED_RSSI_INVALID  127

/* ─── 페어링 화면(OLED_STATE_PAIRING) 세부 단계 ───
 *  WAIT   : 페어링 대기 — BLE 광고, 코드 표시 + "WAITING" 점멸
 *  ACTIVE : 페어링 진행 중(fail-safe armed) — "PAIRING" + 회전 스피너
 *  FAIL   : 실패 — "FAILED" + 오류코드 점멸, STOP 복귀 전까지 유지
 *  DONE   : 완료 — "SUCCESS" 표시 후 5초 뒤 메인 화면 복귀(안테나 갱신)
 *  READY  : 사용자가 SETUP 으로 페어링 준비 확정 — "READY" 점멸 */
typedef enum {
    OLED_PAIR_WAIT   = 0,
    OLED_PAIR_ACTIVE = 1,
    OLED_PAIR_FAIL   = 2,
    OLED_PAIR_DONE   = 3,
    OLED_PAIR_READY  = 4,
} oled_pair_phase_t;

/* ─── 동작 표시 타입 ─────────────────────────── */
typedef enum {
    OLED_ACTION_NONE    = 0,
    OLED_ACTION_UP      = 1,
    OLED_ACTION_DOWN    = 2,
    OLED_ACTION_STOP    = 3,
    OLED_ACTION_TILT_UP = 4,
    OLED_ACTION_TILT_DN = 5,
    OLED_ACTION_PROG    = 6,
    OLED_ACTION_ROT_CW  = 7,   // 로터리 시계 방향 회전 모션
    OLED_ACTION_ROT_CCW = 8,   // 로터리 반시계 방향 회전 모션
    OLED_ACTION_UP_DOWN = 9,   // 동시작동 UP+DOWN (limit 설정/등록)
    OLED_ACTION_MY_UP   = 10,  // 동시작동 MY+UP
    OLED_ACTION_MY_DOWN = 11,  // 동시작동 MY+DOWN
} oled_action_t;

/* ─── UI 컨텍스트 ────────────────────────────── */
typedef struct {
    oled_state_t   state;
    oled_action_t  action;
    bool           action_active;        // 버튼 누름 중
    uint8_t        selected_blind;       // 0..N-1=개별, N(=BLIND_MAX_COUNT)=ALL
    uint8_t        action_blind_mask;    // 모션 중 대상 블라인드 비트마스크
                                         // bit0..N-1=채널, (1<<BLIND_MAX_COUNT)-1=ALL,
                                         // 0=selected_blind 로 폴백
    float          freq_mhz;
    char           time_str[6];          // "HH:MM"
    uint8_t        anim_frame;           // 애니메이션 프레임 카운터
    uint32_t       last_activity_ms;     // 마지막 조작 시각
    uint32_t       action_start_ms;      // 버튼 누름 시작 시각
    uint32_t       screensaver_start_ms; // 화면 보호기 시작 시각
    char           pair_code[24];        // Matter manual pairing code (11~21자리)
    bool           thread_connected;     // Thread 네트워크 부착 여부 (구 wifi_connected)
    oled_matter_state_t matter_state;    // UNPAIRED / PAIRING / CONNECTED (상단 표시)
    int8_t         parent_rssi;          // Thread 부모 RSSI dBm (CONNECTED 시 신호막대), 127=INVALID
    oled_pair_phase_t pair_phase;        // 페어링 화면 세부 단계 (WAIT/ACTIVE/FAIL/DONE)
    char           pair_err[16];         // 페어링 실패 진단 코드 (FAIL 단계 표시용, "" 가능)
    char           blind_names[BLIND_MAX_COUNT + 1][16];   // 블라인드 이름 (0..N-1 + "ALL")
    /* Thread 프로비저닝 표시용 */
    char           thread_prov_name[17]; // Thread network name 또는 "Thread"
    char           thread_prov_qr[12];   // Matter 페어링 코드 (commissioner 가 자격증명 주입)
    char           qr_payload[40];       // Matter QR 내용("MT:...") — 큰 OLED 페어링 화면 QR 렌더용 ("" = PIN 폴백)
    /* 설정 메뉴 (v3.1+) */
    uint8_t        setup_cursor;         // 0..N (메뉴 항목 인덱스)
    bool           freq_edit_dirty;      // 편집 중 freq 가 스냅샷과 다른지 (UI 힌트)
    /* 날짜/시간 수동 설정 (v3.9+) — [0]=년 [1]=월 [2]=일 [3]=시 [4]=분 */
    int            time_edit_val[5];
    uint8_t        time_edit_field;      // 0..4 (현재 편집 자리)
    /* 충전 애니메이션 */
    uint8_t        chg_percent;          // 0~100 (외부에서 estimate 후 설정)
    bool           usb_pwr;              // GP3(VBUS) ADC 감지: USB 연결됨
    bool           bat_low;              // GP12 풀업 1임계: 배터리 저전압(<~3V)
    uint32_t       chg_anim_start_ms;    // 충전 애니메이션 시작 시각
    oled_state_t   chg_resume_state;     // 애니메이션 종료 후 복귀할 상태
    /* 펌웨어 업데이트(Matter OTA) 화면 — somfy_app.c 가 매 tick 갱신 */
    char           fw_version[12];       // 현재 펌웨어 버전 문자열 (예 "3.5")
    uint8_t        fw_ota_state;         // matter_ota_state_t (0=Idle..5=Unknown)
    uint8_t        fw_ota_progress;      // 다운로드 진행 % (DOWNLOADING 시)
#if BOARD_HAS_LR_BUTTONS
    uint8_t        freq_edit_cursor;     // 주파수 편집 디지트 커서: 0=0.1자리, 1=0.01자리 (PCF8575 좌/우)
#endif
} oled_ui_ctx_t;

/* ─── 타임아웃 설정 ──────────────────────────── */
#define OLED_SCREENSAVER_IDLE_MS    300000   // 5분 후 화면 보호기
                                             // (1분 시 사용자 시계 사용 중 불편)
#define OLED_SCREENSAVER_ANIM_MS    60000    // 화면 보호기 1분 후 애니메이션
#define OLED_ACTION_DISPLAY_MS      2500     // 액션 애니메이션 2.5초 후 NORMAL 복귀
#define OLED_TASK_INTERVAL_MS       50       // 화면 업데이트 주기 (50ms = 20fps)
#define OLED_CHG_ANIM_DISPLAY_MS    6000     // 충전 애니메이션 1회 재생 길이 6초

/* ─── API ────────────────────────────────────── */

/**
 * @brief OLED UI 초기화 (I2C + SSD1306 드라이버 초기화)
 * @param ctx  UI 컨텍스트 (호출 전 초기값 설정)
 */
void oled_ui_init(oled_ui_ctx_t *ctx);

/**
 * @brief OLED 가 생성한 하드웨어 I2C 마스터 버스 핸들 반환.
 *        BOARD_I2C_SHARED 보드(ESP32-H2 등)에서 PCF8574 를 같은 버스에
 *        device 로 붙일 때 사용. oled_ui_init() 호출 이후에만 유효(그 전엔 NULL).
 * @return i2c_master_bus_handle_t (실패/미초기화 시 NULL)
 */
i2c_master_bus_handle_t oled_ui_get_i2c_bus(void);

/* 공유 HW I2C 버스 직렬화(OLED flush ↔ PCF8574 read). oled_ui_init() 전엔 무동작.
 * btn_handler 가 PCF8574 read 를 이 lock 으로 감싸 동시 접근 nack 을 막는다. */
void oled_ui_i2c_lock(void);
void oled_ui_i2c_unlock(void);

/* 2026-07-17 추가: 유한 대기 lock. timeout_ms 안에 못 잡으면 false 를 돌려주고
 * 호출자는 이번 주기를 건너뛴다. oled_ui_i2c_lock() 은 portMAX_DELAY 라
 * flush 가 IDF I2C NACK 무한 스핀에 걸리면 **영원히** 막혀 워치독이 난다.
 * → 급하지 않은 호출자(5초 주기 BAT ADC 측정)는 반드시 이 쪽을 쓸 것.
 * 성공(true) 시 해제는 oled_ui_i2c_unlock() 으로 한다. */
bool oled_ui_i2c_trylock(uint32_t timeout_ms);

/**
 * @brief UI 업데이트 태스크 시작 (FreeRTOS 태스크)
 * @param ctx  UI 컨텍스트 포인터
 */
void oled_ui_start_task(oled_ui_ctx_t *ctx);

/**
 * @brief 버튼 동작 알림 (버튼 누름 시작)
 * @param ctx     UI 컨텍스트
 * @param action  어떤 버튼인지
 */
/**
 * @brief 모션 화면 하단에 표시할 대상 블라인드 설정.
 * @param mask bit0..N-1 = 채널. (1<<BLIND_MAX_COUNT)-1 = ALL. 0 = selected_blind 폴백.
 *   notify_action_start 직전에 호출.
 */
void oled_ui_set_action_blinds(oled_ui_ctx_t *ctx, uint8_t mask);

void oled_ui_notify_action_start(oled_ui_ctx_t *ctx, oled_action_t action);

/**
 * @brief 버튼 동작 알림 (버튼 떼기)
 * @param ctx  UI 컨텍스트
 */
void oled_ui_notify_action_end(oled_ui_ctx_t *ctx);

/**
 * @brief 선택된 블라인드 변경
 * @param ctx   UI 컨텍스트
 * @param idx   0..BLIND_MAX_COUNT-1: 개별 블라인드, BLIND_MAX_COUNT: ALL
 */
void oled_ui_set_blind(oled_ui_ctx_t *ctx, uint8_t idx);

/**
 * @brief 주파수 업데이트
 */
void oled_ui_set_freq(oled_ui_ctx_t *ctx, float freq_mhz);

/**
 * @brief 현재 시간 문자열 업데이트 ("HH:MM")
 */
void oled_ui_set_time(oled_ui_ctx_t *ctx, const char *time_str);

/**
 * @brief Matter 페어링 화면 표시
 */
void oled_ui_show_pairing(oled_ui_ctx_t *ctx, const char *pair_code);

/* 페어링 화면 QR 코드 내용("MT:...") 설정. ""/NULL = QR 끄고 PIN 표시.
 *  큰 패널(128×64/64×128)에서만 QR 렌더(작은 패널은 무시하고 PIN). */
void oled_ui_set_qr(oled_ui_ctx_t *ctx, const char *qr_payload);

/**
 * @brief Thread 네트워크 부착 상태 업데이트 (구 oled_ui_set_wifi).
 */
void oled_ui_set_thread(oled_ui_ctx_t *ctx, bool attached);

/**
 * @brief 메인 화면 상단 Matter 상태 표시 갱신.
 * @param st    UNPAIRED / PAIRING / CONNECTED
 * @param rssi  CONNECTED 일 때 Thread 부모 RSSI(dBm). 그 외/조회불가 시
 *              OLED_RSSI_INVALID(127) 전달 → 풀바 또는 무관.
 */
void oled_ui_set_matter_status(oled_ui_ctx_t *ctx,
                               oled_matter_state_t st, int8_t rssi);

/**
 * @brief 페어링 화면 세부 단계 갱신 (WAIT/ACTIVE/FAIL/DONE).
 *        OLED_STATE_PAIRING 상태에서만 의미 있음.
 */
void oled_ui_set_pair_phase(oled_ui_ctx_t *ctx, oled_pair_phase_t ph);

/**
 * @brief 페어링 화면을 1회만 렌더 (연속 태스크 없이 단발 I2C).
 *        커미셔닝 중 802.15.4/SRP 타이밍 보호를 위해 연속 OLED 태스크는
 *        커미셔닝 완료 후에만 시작하므로, 그 전에 코드를 보여주는 용도.
 */
void oled_ui_render_pairing_once(oled_ui_ctx_t *ctx, const char *pair_code,
                                 oled_pair_phase_t ph);

/**
 * @brief 메인(NORMAL) 화면을 1회만 렌더 (연속 태스크 없이 단발 I2C).
 *        부팅 직후 페어링 화면 대신 메인을 즉시 보여주되, 무거운 연속
 *        OLED/버튼 태스크는 커미셔닝 타이밍 보호를 위해 지연하는 용도.
 */
void oled_ui_render_main_once(oled_ui_ctx_t *ctx);

/* v2.x 호환 별칭 */
static inline void oled_ui_set_wifi(oled_ui_ctx_t *ctx, bool connected) {
    oled_ui_set_thread(ctx, connected);
}

/**
 * @brief Thread 커미셔닝 화면 표시 (Matter pair-code 안내).
 *        BLE commissioner 가 자격증명을 주입하므로 별도 SoftAP/QR 불필요.
 * @param ctx       UI 컨텍스트
 * @param net_name  현재 Thread network name (없으면 "Thread")
 * @param pair_code Matter 페어링 코드
 */
void oled_ui_show_thread_prov(oled_ui_ctx_t *ctx,
                               const char *net_name, const char *pair_code);
/* v2.x 호환 별칭 (인자 의미 변경 — ssid→net_name, ip_str→pair_code) */
static inline void oled_ui_show_wifi_prov(oled_ui_ctx_t *ctx,
                                          const char *a, const char *b) {
    oled_ui_show_thread_prov(ctx, a, b);
}

/**
 * @brief 블라인드 선택 변경 알림 (SEL 버튼)
 * @param ctx  UI 컨텍스트
 * @param idx  0~count-1: 개별 블라인드, count: ALL
 */
void oled_ui_notify_blind_select(oled_ui_ctx_t *ctx, uint8_t idx);

/**
 * @brief 즉시 화면 복귀 (화면 보호기 해제)
 */
void oled_ui_wake(oled_ui_ctx_t *ctx);

/* ★2026-07-23 RF 송신 구간 알림 — true 동안 OLED I2C 접근을 전면 중단한다.
 *  CC1101 447MHz 송신 노이즈가 I2C 를 깨뜨려 SSD1306 이 고착되는 것을 막는다
 *  (고착 시 모듈 전원차단 전엔 복구 불가). somfy_app.c 의 _do_rf_send 가 감싼다. */
void oled_ui_set_rf_tx(bool active);

/**
 * @brief 설정 메뉴 화면 진입 (v3.1+).
 * @param ctx     UI 컨텍스트
 * @param cursor  초기 커서 위치 (0..3)
 */
void oled_ui_show_setup_menu(oled_ui_ctx_t *ctx, uint8_t cursor);

/**
 * @brief 설정 메뉴 커서 위치 변경 (즉시 redraw).
 */
void oled_ui_set_setup_cursor(oled_ui_ctx_t *ctx, uint8_t cursor);

/**
 * @brief Thread 리셋 확인 화면 표시.
 */
void oled_ui_show_thread_reset(oled_ui_ctx_t *ctx);

/**
 * @brief 펌웨어 업데이트(Matter OTA) 화면 표시.
 *        현재 버전 + OTA 상태/진행률 표시. 진입 후 somfy_app 가 매 tick
 *        fw_* 필드를 갱신하면 화면이 따라간다.
 */
void oled_ui_show_fw_update(oled_ui_ctx_t *ctx);

/**
 * @brief 날짜/시간 수동 설정 화면 진입.
 * @param v      [0]=년 [1]=월(1~12) [2]=일 [3]=시(0~23) [4]=분(0~59)
 * @param field  최초 편집 자리 (0~4)
 */
void oled_ui_show_time_edit(oled_ui_ctx_t *ctx, const int v[5], uint8_t field);

/**
 * @brief 날짜/시간 편집 값/자리 갱신 (화면 상태 변경 없이 즉시 redraw).
 */
void oled_ui_set_time_edit(oled_ui_ctx_t *ctx, const int v[5], uint8_t field);

/**
 * @brief 충전 애니메이션 트리거 — OLED_CHG_ANIM_DISPLAY_MS 동안 다이내믹
 *        배터리 충전 영상을 재생한 뒤 이전 상태로 자동 복귀.
 *        (USB 케이블 연결 후 1분 마다 호출하여 사용)
 * @param ctx       UI 컨텍스트
 * @param percent   0~100 (예상 충전량, ADC 미가용 시 추정값)
 */
void oled_ui_show_charging(oled_ui_ctx_t *ctx, uint8_t percent);

/**
 * @brief OLED 디스플레이 전원 제어 (SSD1315 명령 0xAE/0xAF).
 *        light sleep 진입 시 화면 OFF, wake 시 ON 으로 사용.
 *        UI 렌더 task 자체는 그대로 동작하지만 픽셀 데이터가 표시 안 됨.
 * @param on  true: 디스플레이 ON, false: 디스플레이 OFF (저전력)
 */
void oled_ui_set_display_on(bool on);

#ifdef __cplusplus
}
#endif
