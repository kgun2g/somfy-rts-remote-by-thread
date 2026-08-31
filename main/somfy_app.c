/*
 * Somfy RTS Wood Blind Controller - Main Application
 *
 * ESP32-C6-0.42 + CC1101 + SmartThings Matter
 * ─────────────────────────────────────────────
 * 주요 흐름:
 *   1. NVS + 블라인드 설정 로드
 *   2. CC1101 SPI 초기화
 *   3. Somfy RTS 엔진 초기화
 *   4. OLED UI 초기화 + 태스크 시작
 *   5. 버튼 핸들러 초기화 + 태스크 시작
 *   6. WiFi 프로비저닝 (저장값 있으면 바로 연결, 없으면 SoftAP)
 *   7. Matter 커미셔닝 (BLE + WiFi)
 *   8. SNTP 시간 동기화
 *   9. 메인 이벤트 루프
 */

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "esp_sntp.h"
#include "esp_system.h"       /* esp_reset_reason — boot 시 panic 감지 */
#include "esp_heap_caps.h"    /* heap_caps_get_largest_free_block — heap 진단 */
#include "esp_task_wdt.h"    /* Task WDT — 메인 루프 hang 자동 리부트 */
#include "wake_diag.h"        /* ★2026-08-20 깨어남 출처 계측 */
#include "esp_attr.h"         /* RTC_NOINIT_ATTR — 리부트 살아남는 메모리 */
#include "thread_provision.h"

#include "boot_diag.h"        /* 부팅 단계 NVS 기록 — 배터리 부팅 멈춤 진단 */
#include "esp_timer.h"
/* WiFi 헤더 제거 — v3.0 Thread 전환.
 * esp_wifi 가 sdkconfig 에서 disabled 이므로 컴파일 단계에서 제외.
 * 모뎀 절전은 OpenThread RCP 가 자동 관리. */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "blind_manager.h"
#include "button_handler.h"
#include "cc1101.h"
#include "matter_blinds.h"
#include "oled_ui.h"
#include "somfy_config.h"
/* CHIP PacketBuffer 풀 크기 매크로 — 순수 #define 이라 C 에서도 안전 */
#include "CHIPProjectConfig.h"
#include "app_log.h"
#ifdef SOMFY_SELFTEST
#include "somfy_selftest.h"
#endif
#ifdef SOMFY_ONAIR_TEST
#include "somfy_onair_test.h"
#endif
#ifdef SOMFY_STRESS_TEST
#include "somfy_stress_test.h"
#endif
#ifdef SOMFY_RXDECODE_TEST
#include "somfy_rxdecode_test.h"
#endif
#ifdef SOMFY_TXPROBE_TEST
#include "somfy_txprobe_test.h"
#endif
#ifdef SOMFY_RXBYTE_TEST
#include "somfy_rxbyte_test.h"
#endif
#ifdef SOMFY_TXDECODE_TEST
#include "somfy_txdecode_test.h"
#endif
#ifdef SOMFY_CWTEST_TEST
#include "somfy_cwtest_test.h"
#endif
#include "somfy_rts.h"
#include "build_epoch.h"   /* ★ CMake 가 매 빌드 생성: BUILD_EPOCH_UNIX (UTC) */

/* ★2026-08-16 안테나 진단 이벤트 코드 — 배터리 가드 **밖**에 둔다.
 *  BOARD_HAS_BAT_ADC=0 보드에서도 호출부가 참조한다(위 #else 스텁이 받는다). */
#define BLEV_NOSIG  10   /* '-' 등록 유지 + 권외(detach) */
#define BLEV_UNPAIR 11   /* 'X' 미등록(FabricCount==0) — 이게 찍히면 예상 밖이다 */

#if BOARD_HAS_BAT_ADC
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_rom_sys.h"   /* esp_rom_delay_us — tick 무관 busy-wait */
#include "esp_adc/adc_cali_scheme.h"

/* ★★2026-08-22 g_wake_iter 정의는 **가드 밖**에 둔다.
 *  처음엔 batlog 블록(`#if BOARD_BATLOG_ENABLE`) 안에 뒀는데, H2 는 그 값이 0 이라
 *  정의가 통째로 빠져 링크가 깨졌다:
 *      undefined reference to `g_wake_iter`  (somfy_app.c / oled_ui.c 여러 곳)
 *  카운터를 올리는 쪽(button_handler·oled_ui·메인 루프·hold_repeat)은 보드 설정과
 *  무관하게 항상 컴파일되므로 정의도 무조건 있어야 한다.
 *  ※NVS 로 남기는 쪽만 batlog 가드 안에 있으면 된다(배터리 없는 보드는 저장 안 함). */
volatile uint32_t g_wake_iter[WI_COUNT];

#endif
/* WiFi → Thread 전환. wifi_provision.h 제거, thread_provision.h 사용. */

static const char *TAG = "MAIN";

/* ═══════════════════════════════════════════════
   전역 객체
   ──────────────────────────────────────────────
   cc1101/somfy/blind_manager 는 app_main.cpp(Matter 코어)가 단일 소유한다.
   여기서는 extern 으로 같은 인스턴스를 공유하고, s_* 명칭은 #define 으로
   g_* 에 매핑한다. s_ui(OLED 컨텍스트)는 본 모듈 로컬. */
extern cc1101_t        g_cc1101;
extern somfy_rts_t     g_somfy;
extern blind_manager_t g_mgr;
#define s_cc1101 g_cc1101
#define s_somfy  g_somfy
#define s_mgr    g_mgr
static oled_ui_ctx_t s_ui = {0};

/* ═══════════════════════════════════════════════
   설정 메뉴 상태머신 (v3.1+)
   ──────────────────────────────────────────────
   메인 → SETUP 짧게 → SETUP_MENU
   (★ SETUP 15초 이상 hold → 화면 무관 강제 재부팅 — button_handler.c SETUP_REBOOT_HOLD_MS, 기기 멈춤 대비)
   ★★2026-08-12 STOP(MY) ↔ SETUP 기능 교환 (사용자 요청) — 설정 화면 **전체**에 적용.
      규칙: STOP(MY) = 확인/저장/실행,  SETUP = 취소/뒤로.
      길이 문턱이 서로 다르다: SETUP_LONG=1초(button_handler.c),
      ROT_CLICK long_press=2초(CFG_BTN_LONG_PRESS_MS). 교환하면 "길게" 의 실제
      시간도 같이 바뀐다(예: Thread 리셋 실행 = SETUP 1초 → MY 2초).
   SETUP_MENU :
     UP/DOWN/tilt    : 커서 이동
     STOP (any)      : 선택 항목 진입   (교환 전: 메인 복귀)
     SETUP (any)     : 메인 복귀        (교환 전: 선택 항목 진입)
     PROG/SELECT     : 무시
   SETUP_FREQ_EDIT :
     tilt / UP / DOWN: freq ±0.01
     STOP (any)      : 저장 후 메인 복귀 (좌측 상단 freq 갱신)
     SETUP (any)     : 변경 폐기 후 SETUP_MENU 복귀
   SETUP_TIME_EDIT :
     STOP (any)      : 저장
     SETUP (any)     : 적용 안 함 → SETUP_MENU 복귀
   SETUP_MATTER_PAIR :
     진입 시 자동 commissioning window 오픈
     STOP 짧게       : 대기(WAITING) → 준비(READY) 확정
     STOP 길게       : 무시
     SETUP 짧게      : SETUP_MENU 복귀
     SETUP 길게      : 메인 복귀
   SETUP_THREAD_RESET :
     STOP 길게(2s)   : thread_prov_erase + matter restart → THREAD_PROV
     STOP 짧게       : SETUP_MENU 복귀 (취소)
     SETUP 짧게      : SETUP_MENU 복귀
     SETUP 길게      : 메인 복귀
   SETUP_FW_UPDATE :
     STOP 짧게       : 수동 업데이트 확인 (QueryImage)
     SETUP 짧게      : SETUP_MENU 복귀
     SETUP 길게      : 메인 복귀
═══════════════════════════════════════════════ */
typedef enum {
  SETUP_NONE = 0,        // 메인 화면 (설정 아님)
  SETUP_MENU_SCR,        // 설정 메뉴 화면
  SETUP_FREQ_EDIT,       // 주파수 편집
  SETUP_TIME_EDIT,       // 날짜/시간 수동 설정 (v3.9+)
  SETUP_MATTER_PAIR,     // Matter 페어링 (commissioning window 진행 중)
  SETUP_THREAD_RESET,    // Thread 리셋 확인 다이얼로그
  SETUP_FW_UPDATE,       // 펌웨어 업데이트 (Matter OTA over Thread)
  /* SETUP_RF_SCAN 제거 — 기본 freq(447.72) 가 보드 검증됨, calibration 불필요 */
} setup_screen_t;

static setup_screen_t s_setup_screen = SETUP_NONE;
/* 페어링 화면에서 사용자가 SETUP 으로 '준비(READY)' 를 확정했는지.
 *  SETUP_MATTER_PAIR 진입/이탈 시 false 로 리셋, 페어링 트랜잭션
 *  시작(is_pairing_in_progress) 시에도 클리어한다. */
static bool           s_pair_ready  = false;
static uint8_t        s_setup_cursor = 0;          // 0..SETUP_CURSOR_MAX-1
/* 설정 메뉴 cursor → 동작 매핑. 조건부 항목(Time/OTA)은 컴파일 시 제외되어
 * 커서 인덱스가 자동으로 당겨진다 (oled_ui.c SETUP_MENU_ITEMS 와 동일 순서·동기). */
enum { MA_CANCEL, MA_FREQ, MA_TIME, MA_PAIR, MA_THREAD, MA_FWUP, MA_REBOOT };
static const uint8_t kMenuAction[] = {
  MA_CANCEL,
  MA_FREQ,
#if !BOARD_DISABLE_TIME
  MA_TIME,
#endif
  MA_PAIR,
  MA_THREAD,
#if !BOARD_DISABLE_OTA
  MA_FWUP,
#endif
  MA_REBOOT,
};
#define SETUP_CURSOR_MAX ((int)(sizeof(kMenuAction) / sizeof(kMenuAction[0])))

/* 메뉴 항목(커서) → 진입 (전방 선언; 정의는 시간편집 헬퍼 근처) */
static void _time_edit_enter(void);

static float s_freq_edit_orig = 0.0f; // 편집 진입 시 freq 스냅샷

/* ───────────── 설정 모드 헬퍼 ───────────── */

static inline float _freq_edit_current(void) {
  uint8_t sel = s_mgr.selected;
  if (sel < s_mgr.count) return s_mgr.blinds[sel].freq_mhz;
  if (sel == BLIND_SEL_ALL && s_mgr.block_count > 0)
    return s_mgr.all_blocks[0].freq_mhz;   /* ALL — 블록 ALL 주소의 주파수 */
  return s_ui.freq_mhz;
}

static inline bool _in_setup_mode(void) {
  return s_setup_screen != SETUP_NONE;
}

/* 모든 화면에서 공통 — 메인 복귀 */
static void _setup_exit_to_main(const char *reason) {
  s_setup_screen = SETUP_NONE;
  s_ui.state = OLED_STATE_NORMAL;
  /* 좌측 상단 freq 가 현재 선택된 블라인드의 NVS 값으로 표시되도록 */
  oled_ui_set_freq(&s_ui, _freq_edit_current());
  s_ui.freq_edit_dirty = false;
  ESP_LOGI(TAG, "[SETUP] 메인 복귀 (%s)", reason ? reason : "");
}

/* 페어링(커미셔닝) 보호 — Settings>Matter Pair 윈도우 동안에만.
 *  부팅 후 한참 뒤 수동 페어링 시 SNTP/DFS 가 이미 가동 중이라
 *  operational mDNS/SRP·CASE 를 굶겨 39-517 이 나는 것을 막는다.
 *  (구현은 SNTP/PM 코드 근처에 정의 — 여기서는 전방 선언만) */
static void _pairing_protect_begin(void);
static void _pairing_protect_end(void);

/* 설정 메뉴 진입 (메인 → 메뉴) */
static void _setup_enter_menu_from_main(void) {
  s_setup_screen = SETUP_MENU_SCR;
  s_setup_cursor = 0;
  oled_ui_show_setup_menu(&s_ui, s_setup_cursor);
  ESP_LOGI(TAG, "[SETUP] 메뉴 진입");
}

/* 메뉴 → 항목 진입 */
static void _setup_activate_menu_item(void) {
  uint8_t idx = s_setup_cursor;
  /* 동작은 kMenuAction[idx] 로 결정 (조건부 항목 제외 시 인덱스 자동 정렬). */
  if (idx >= SETUP_CURSOR_MAX) { _setup_exit_to_main("(invalid menu)"); return; }
  switch (kMenuAction[idx]) {
    case MA_CANCEL:  /* Cancel — 즉시 메인 복귀 */
      _setup_exit_to_main("Cancel 메뉴 선택");
      break;
    case MA_FREQ: { /* Freq Edit */
      s_setup_screen = SETUP_FREQ_EDIT;
      s_freq_edit_orig = _freq_edit_current();
#if BOARD_HAS_LR_BUTTONS
      s_ui.freq_edit_cursor = 1;   /* 진입 시 0.01 자리(가장 미세) */
#endif
      s_ui.freq_edit_dirty = false;
      oled_ui_set_freq(&s_ui, s_freq_edit_orig);
      s_ui.state = OLED_STATE_FREQ_EDIT;
      ESP_LOGI(TAG, "[SETUP] 주파수 편집 진입 (orig=%.2f MHz)", s_freq_edit_orig);
      break;
    }
#if !BOARD_DISABLE_TIME
    case MA_TIME:  /* Time Set — 날짜/시간 수동 설정 */
      _time_edit_enter();
      break;
#endif
    case MA_PAIR: { /* Matter Pair — BLE commissioning window 재오픈 */
      s_setup_screen = SETUP_MATTER_PAIR;
      s_pair_ready   = false;          /* 진입 시 항상 WAITING 부터 */
      /* ★ 커미셔닝 보호 시작 — open 직전에 SNTP off + CPU max 고정
       *  (수분 뒤 수동 페어링 시 39-517 방지). */
      _pairing_protect_begin();
      const char *code = matter_blinds_open_commissioning_window();
      oled_ui_set_qr(&s_ui, matter_blinds_get_qr_payload());
      oled_ui_show_pairing(&s_ui, code);
      ESP_LOGI(TAG, "[SETUP] Matter 페어링 진입 (PIN=%s) — BLE 재광고 시작", code);
      break;
    }
    case MA_THREAD: { /* Thread Rst */
      s_setup_screen = SETUP_THREAD_RESET;
      oled_ui_show_thread_reset(&s_ui);
      ESP_LOGI(TAG, "[SETUP] Thread 리셋 확인 화면 (SETUP 2s 길게 = 실행)");
      break;
    }
#if !BOARD_DISABLE_OTA
    case MA_FWUP: { /* FW Update — Matter OTA over Thread */
      s_setup_screen = SETUP_FW_UPDATE;
      snprintf(s_ui.fw_version, sizeof(s_ui.fw_version), "%s",
               matter_ota_version_str());
      oled_ui_show_fw_update(&s_ui);
      ESP_LOGI(TAG, "[SETUP] FW Update 화면 (v%s, SET=업데이트 확인)",
               s_ui.fw_version);
      break;
    }
#endif
    case MA_REBOOT: { /* Reboot — 시스템 재시작 (CC1101 등 HW 변경/재검출 반영) */
      ESP_LOGW(TAG, "[SETUP] 사용자 요청 재부팅 — esp_restart()");
      vTaskDelay(pdMS_TO_TICKS(300));   /* 로그 flush + 마지막 화면 표시 여유 */
      esp_restart();                    /* 복귀하지 않음 */
      break;
    }
    default:
      _setup_exit_to_main("(invalid menu)");
      break;
  }
}

/* 메뉴 화면으로 복귀 */
static void _setup_back_to_menu(const char *reason) {
  s_setup_screen = SETUP_MENU_SCR;
  oled_ui_show_setup_menu(&s_ui, s_setup_cursor);
  ESP_LOGI(TAG, "[SETUP] 메뉴 복귀 (%s)", reason ? reason : "");
}

/* 메뉴 커서 이동 (UP/DOWN/tilt 공용) */
static void _setup_menu_cursor_move(int delta) {
  int n = (int)s_setup_cursor + delta;
  if (n < 0) n = SETUP_CURSOR_MAX - 1;
  if (n >= SETUP_CURSOR_MAX) n = 0;
  s_setup_cursor = (uint8_t)n;
  oled_ui_set_setup_cursor(&s_ui, s_setup_cursor);
  ESP_LOGI(TAG, "[SETUP] 커서 → %d", s_setup_cursor);
}

/* 주파수 편집 — freq ±0.01 (UP/CW = +, DOWN/CCW = -) */
static void _freq_edit_step(int dir) {
  uint8_t sel = s_mgr.selected;
  float cur = _freq_edit_current();
  float step = CFG_FREQ_STEP_MHZ;
#if BOARD_HAS_LR_BUTTONS
  step = (s_ui.freq_edit_cursor == 0) ? 0.1f : 0.01f;   /* 커서 자리값 (좌/우로 선택) */
#endif
  float nf = roundf((cur + step * dir) * 100.0f) / 100.0f;
  if (nf > CFG_FREQ_MAX_MHZ) nf = CFG_FREQ_MAX_MHZ;
  if (nf < CFG_FREQ_MIN_MHZ) nf = CFG_FREQ_MIN_MHZ;
  if (sel < s_mgr.count || sel == BLIND_SEL_ALL) {
    /* ALL 도 적용해야 함 — set_freq 가 all_blocks[] 를 처리한다. 빠지면 다음 step 의
     *  기준값(cur=_freq_edit_current=all_blocks[0].freq)이 안 바뀌어 ±1칸만 움직인다. */
    blind_manager_set_freq(&s_mgr, sel, nf);
  }
  oled_ui_set_freq(&s_ui, nf);
  s_ui.freq_edit_dirty = (fabsf(nf - s_freq_edit_orig) > 0.005f);
  ESP_LOGI(TAG, "[FREQ_EDIT] %s → %.2f MHz (orig=%.2f)",
           dir > 0 ? "+" : "-", nf, s_freq_edit_orig);
}

/* 주파수 편집 — 저장 후 메뉴 복귀 (v3.9: 메인 아닌 메뉴로)
 *  ★ "주파수 초기화 기능 제거" 요청에 따라 _freq_edit_reset_to_snapshot 삭제. */
static void _freq_edit_save(void) {
  blind_manager_save(&s_mgr);
  uint8_t sel = s_mgr.selected;
  float saved = (sel < s_mgr.count) ? s_mgr.blinds[sel].freq_mhz : s_ui.freq_mhz;
  s_ui.freq_edit_dirty = false;
  ESP_LOGI(TAG, "[FREQ_EDIT] 저장 완료 → %.2f MHz, 메뉴 복귀", saved);
  _setup_back_to_menu("freq 저장");
}

/* 주파수 편집 — 메뉴로 복귀 (변경 폐기) */
static void _freq_edit_back_to_menu_discard(void) {
  uint8_t sel = s_mgr.selected;
  if (sel < s_mgr.count) {
    blind_manager_set_freq(&s_mgr, sel, s_freq_edit_orig);
  }
  oled_ui_set_freq(&s_ui, s_freq_edit_orig);
  s_ui.freq_edit_dirty = false;
  _setup_back_to_menu("freq 편집 변경 폐기");
}

/* Thread 리셋 실행 */
static void _setup_execute_thread_reset(void) {
  ESP_LOGW(TAG, "[SETUP] Thread+페어링 리셋 실행 — Thread dataset + Matter fabric 삭제 후 재부팅");
  thread_prov_erase();                  /* OpenThread dataset/네트워크 자격증명 삭제 */
  matter_blinds_remove_all_fabrics();   /* Matter fabric(SmartThings 페어링) 삭제 (CHIP 스레드) */
  s_setup_screen = SETUP_NONE;
  /* fabric 삭제(ScheduleWork) + NVS 커밋 여유 후 재부팅 → 미커미셔닝 상태로 부팅하며
   *  자동으로 BLE 커미셔닝 광고가 열린다(완전 초기화). 주파수 등 다른 NVS 설정은 유지. */
  vTaskDelay(pdMS_TO_TICKS(1500));
  esp_restart();
}

/* ═══════════════════════════════════════════════
   컴파일 시각으로 RTC 초기 설정
   ──────────────────────────────────────────────
   WiFi/SNTP 미연결 시에도 화면 보호기에 의미 있는 날짜/시간이 표시되도록
   빌드 시점(=PC 시각)을 초기값으로 설정한다. SNTP가 연결되면 자동으로
   정확한 NTP 시각으로 덮어씌워짐. __DATE__/__TIME__ 매크로는 컴파일 단위
   별로 갱신되므로 매 빌드(=매 플래시)마다 최신 PC 시각이 반영된다.
═══════════════════════════════════════════════ */
/* ═══════════════════════════════════════════════
   시간 NVS 영속화 (v3.6+) — 리셋 후 시간 복원
   ──────────────────────────────────────────────
   5분마다 현재 epoch 를 NVS 에 저장. 부팅 시 NVS 값이 빌드 시간보다
   미래면 NVS 값으로 복원. 정전/리셋에도 시계 연속성 보장.

   NVS write wear: 5min/write × 24h = 288/day = ~105k/year (한계 ~100k).
   안전 마진을 위해 timer 가 호출되더라도 epoch 가 직전 값과 다를 때만
   기록 (sleep 중에는 RTC 가 멈춤 → 값 동일 → 기록 skip).
═══════════════════════════════════════════════ */
#define TIME_NVS_NAMESPACE "rtc_save"
#define TIME_NVS_KEY       "last_epoch"
#define TIME_SAVE_INTERVAL_MS  (5 * 60 * 1000)  /* 5분 */

static int64_t s_last_saved_epoch = 0;

static esp_err_t _time_persist_save(void) {
  time_t now = time(NULL);
  if (now <= 0) return ESP_FAIL;
  if ((int64_t)now == s_last_saved_epoch) return ESP_OK;  /* 변경 없음 */

  nvs_handle_t h;
  esp_err_t err = nvs_open(TIME_NVS_NAMESPACE, NVS_READWRITE, &h);
  if (err != ESP_OK) return err;
  err = nvs_set_i64(h, TIME_NVS_KEY, (int64_t)now);
  if (err == ESP_OK) {
    err = nvs_commit(h);
    s_last_saved_epoch = now;
  }
  nvs_close(h);
  return err;
}

static time_t _time_persist_load(void) {
  nvs_handle_t h;
  esp_err_t err = nvs_open(TIME_NVS_NAMESPACE, NVS_READONLY, &h);
  if (err != ESP_OK) return 0;
  int64_t saved = 0;
  err = nvs_get_i64(h, TIME_NVS_KEY, &saved);
  nvs_close(h);
  return (err == ESP_OK) ? (time_t)saved : 0;
}

/* 주기적 시간 저장 태스크 */
/* ★2026-08-20 `time_persist` 태스크 제거 — 메인 루프의 _time_persist_sched()
 *  로 흡수했다(_time_tick 위 주석 참조). 스택 3,072B 회수. */

static void _init_rtc_from_build_time(void) {
  /* ★ v3.8 근본수정: __DATE__/__TIME__ (somfy_app.c 재컴파일 시에만 갱신
   *  → 증분빌드에서 시드 묵음) 대신, CMake 가 매 빌드 새로 쓰는
   *  BUILD_EPOCH_UNIX(UTC epoch) 를 직접 사용. TZ 파싱/mktime 불필요,
   *  어떤 파일만 고친 빌드든 항상 최신 빌드 시각이 시드된다. */
  setenv("TZ", CFG_TIMEZONE, 1);   // "KST-9" — localtime 표시용
  tzset();

  time_t t = (time_t)BUILD_EPOCH_UNIX;   /* 이미 UTC epoch */
  if (t <= 0) return;

  /* ★ 컴파일 → 플래시 → 부팅 사이에 흐른 시간을 보정.
   *   빌드 1분 + 플래시 30초 + 부팅 ~5초 ≈ 95초
   *   사용자가 실시간 시계 비교 시 더 정확. SNTP 동기화되면 자동 보정. */
  const time_t BUILD_TO_BOOT_OFFSET_SEC = 95;
  t += BUILD_TO_BOOT_OFFSET_SEC;

  /* v3.6+: NVS 에 저장된 epoch 가 빌드시간보다 미래면 그 값 사용 (리셋 복원).
   * NVS 저장 시점부터 리셋까지의 시간은 잃지만 빌드 시간보다는 정확. */
  time_t nvs_t = _time_persist_load();
  const char *src = "BUILD";
  if (nvs_t > t) {
    /* 부팅 시간을 NVS save 이후 누락된 시간으로 보정 — 평균 2.5분(절반)
     * 추가. NVS 저장은 5분 간격이므로 평균 절반 만큼 미래에 가까움. */
    t = nvs_t + 150;  /* +2.5분 */
    src = "NVS";
  }

  struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
  settimeofday(&tv, NULL);
  s_last_saved_epoch = t;  /* 다음 _time_persist_save() 시 skip 방지 */

  struct tm local;
  localtime_r(&t, &local);
  ESP_LOGI(TAG, "RTC 초기화 [%s] (%s, 빌드+%lds 보정): %04d-%02d-%02d %02d:%02d:%02d",
           src, CFG_TIMEZONE, (long)BUILD_TO_BOOT_OFFSET_SEC,
           local.tm_year + 1900, local.tm_mon + 1, local.tm_mday,
           local.tm_hour, local.tm_min, local.tm_sec);
}

/* ═══════════════════════════════════════════════
   날짜/시간 수동 설정 (v3.9+) — SmartThings 미연동 환경 대비
   ──────────────────────────────────────────────
   UP/DOWN     : 편집 자리 이동 (년→월→일→시→분 순환)
   틸트UP/틸트DN: 현재 자리 값 ± (래핑/일수 clamp)
   SET 짧게    : 저장(settimeofday+NVS) 후 메뉴 복귀
   STOP 짧게   : 취소 후 메뉴 복귀
═══════════════════════════════════════════════ */
static struct tm s_te_tm;     /* 작업 중 날짜/시간 스냅샷 */
static uint8_t   s_te_field;  /* 0=년 1=월 2=일 3=시 4=분 */

static int _te_days_in_month(int year, int mon0 /*0..11*/) {
  static const int dm[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (mon0 < 0 || mon0 > 11) return 31;
  if (mon0 == 1) {
    bool leap = ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
    return leap ? 29 : 28;
  }
  return dm[mon0];
}

static void _te_push_ui(void) {
  int v[5] = { s_te_tm.tm_year + 1900, s_te_tm.tm_mon + 1, s_te_tm.tm_mday,
               s_te_tm.tm_hour, s_te_tm.tm_min };
  oled_ui_set_time_edit(&s_ui, v, s_te_field);
}

static void _time_edit_enter(void) {
  time_t now = 0;
  time(&now);
  localtime_r(&now, &s_te_tm);
  s_te_tm.tm_sec = 0;
  s_te_field = 0;
  s_setup_screen = SETUP_TIME_EDIT;
  int v[5] = { s_te_tm.tm_year + 1900, s_te_tm.tm_mon + 1, s_te_tm.tm_mday,
               s_te_tm.tm_hour, s_te_tm.tm_min };
  oled_ui_show_time_edit(&s_ui, v, s_te_field);
  ESP_LOGI(TAG, "[TIME_EDIT] 진입 %04d-%02d-%02d %02d:%02d",
           v[0], v[1], v[2], v[3], v[4]);
}

static void _time_edit_field_move(int dir) {
  s_te_field = (uint8_t)(((int)s_te_field + dir + 5) % 5);
  _te_push_ui();
}

/* lo..hi 범위 래핑 */
static int _wrap_range(int v, int lo, int hi) {
  if (v < lo) return hi;
  if (v > hi) return lo;
  return v;
}

static void _time_edit_value_step(int dir) {
  switch (s_te_field) {
    case 0: {
      int y = s_te_tm.tm_year + 1900 + dir;
      y = _wrap_range(y, 2024, 2099);
      s_te_tm.tm_year = y - 1900;
      break;
    }
    case 1:
      s_te_tm.tm_mon = _wrap_range(s_te_tm.tm_mon + dir, 0, 11);
      break;
    case 2: {
      int dim = _te_days_in_month(s_te_tm.tm_year + 1900, s_te_tm.tm_mon);
      s_te_tm.tm_mday = _wrap_range(s_te_tm.tm_mday + dir, 1, dim);
      break;
    }
    case 3:
      s_te_tm.tm_hour = _wrap_range(s_te_tm.tm_hour + dir, 0, 23);
      break;
    default:
      s_te_tm.tm_min = _wrap_range(s_te_tm.tm_min + dir, 0, 59);
      break;
  }
  /* 년/월 변경으로 말일 초과 시 일자 clamp */
  {
    int dim = _te_days_in_month(s_te_tm.tm_year + 1900, s_te_tm.tm_mon);
    if (s_te_tm.tm_mday > dim) s_te_tm.tm_mday = dim;
    if (s_te_tm.tm_mday < 1)   s_te_tm.tm_mday = 1;
  }
  _te_push_ui();
}

static void _time_edit_save(void) {
  s_te_tm.tm_sec  = 0;
  s_te_tm.tm_isdst = -1;
  setenv("TZ", CFG_TIMEZONE, 1);   /* KST-9 — local→epoch 변환 기준 */
  tzset();
  time_t t = mktime(&s_te_tm);
  if (t > 0) {
    struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    s_last_saved_epoch = 0;        /* skip 가드 해제 → 강제 NVS 저장 */
    _time_persist_save();
    struct tm lt;
    localtime_r(&t, &lt);
    ESP_LOGW(TAG, "[TIME_EDIT] 저장: %04d-%02d-%02d %02d:%02d (epoch=%lld) — NVS",
             lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
             lt.tm_hour, lt.tm_min, (long long)t);
  } else {
    ESP_LOGE(TAG, "[TIME_EDIT] mktime 실패 — 저장 안 함");
  }
  _setup_back_to_menu("time 저장");
}

/* ═══════════════════════════════════════════════
   절전 모드 (1분 무입력 → light sleep)
   ──────────────────────────────────────────────
   진동 디바운스: VS1 (PCF8574 P4)에서 진동 감지 시 5초 동안
   sleep 진입 차단. 사용자가 디바이스를 계속 흔들거나 이동시키는
   동안 sleep/wake 사이클을 반복하지 않도록 함.
═══════════════════════════════════════════════ */
/* v3.2 idle 정책 ─────────────────────────────────────────────────────
 *  USB 연결 (충전 중 또는 최근 충전됨) :
 *    3분 무입력 → 화면 보호기 (OLED off, CPU 정상, Matter 수신 정상)
 *    절전(light sleep) 진입 안 함.
 *  배터리 전용 (USB 미연결) :
 *    1분 무입력 → 절전 모드 (OLED off + light sleep)
 *    버튼/진동/충전시작 시 wake. Matter 수신은 wake 후에 가능.
 *
 *  USB 연결 판정 — CHG_STAT 가 최근 LOW 였으면 USB 연결로 간주.
 *  STAT 가 HIGH 로 돌아오는 경우는 (a) 충전 완료 (b) USB 분리.
 *  구분 불가하므로 USB_DETECT_HOLD_MS 내에 충전 활동이 있었으면
 *  여전히 USB 연결 상태로 본다. (실용적 휴리스틱) */
/* ★2026-08-27 충전 애니메이션 **제거** — 2026-07-17 사용자 요구로 끈 뒤
 *  (CFG_CHG_ANIM_ENABLE=0) 한 번도 호출되지 않던 죽은 코드다. 당시 주석은
 *  "삭제 금지, 1 로 되돌리면 복구" 였으나 사용자 지시로 정리한다.
 *  제거 대상: OLED_STATE_CHARGING / _render_charging / oled_ui_show_charging /
 *             chg_anim_start_ms / OLED_CHG_ANIM_DISPLAY_MS / CFG_CHG_ANIM_ENABLE
 *  ★유지: `chg_percent` 는 **배터리 잔량 % 표시**로 살아 있는 기능이다(이름만 chg). */
/* ★2026-07-23 화면 정책: 화면보호기(중간 애니메이션) **코드 삭제**.
 *  유휴 시간이 지나면 곧바로 패널 OFF(USB=CFG_SCREEN_OFF_USB_SEC,
 *  배터리=CFG_SCREEN_OFF_SEC — somfy_config.h). 버튼/진동으로 즉시 복귀. */
#define USB_DETECT_HOLD_MS    60000      /* 마지막 충전 감지 후 1분간 USB 모드 유지 */
#define VIBRATION_HOLD_MS         5000   /* 진동 후 5초 sleep 차단 */
static volatile int64_t s_last_activity_us = 0;
static volatile int64_t s_last_chg_active_us = 0;  /* CHG_STAT LOW 였던 마지막 시각 */
static volatile bool s_is_sleeping = false;
static volatile bool s_screensaver_active = false; /* OLED off 상태 (USB 모드) */

static inline void _mark_activity(void) {
  s_last_activity_us = esp_timer_get_time();
}

/* ★2026-08-11 배터리 모드 시뮬레이션 — USB 를 물리적으로 뽑지 않고 로직만 검증한다.
 *  PC 의 USB 포트는 소프트웨어로 5V 를 끊을 수 없어(장치를 비활성화해도 VBUS 는 살아
 *  있다) 실제 방전 시험은 물리 분리가 필요하다. 하지만 **화면 10초 OFF·무선 게이팅·
 *  BATLOG 주기·NVS 기록** 같은 동작은 전원과 무관하므로, 이 플래그로 "USB 없음"인 척
 *  하면 콘솔을 살려둔 채 전 과정을 실시간으로 확인할 수 있다.
 *  콘솔: `usbsim off`(배터리인 척) / `usbsim on`(해제).
 *  ※실제 전류·전압 하강은 시뮬로 재현되지 않는다 — 그건 물리 분리로만 잴 수 있다. */
static volatile bool s_usb_sim_off = false;

/* USB 모드 여부 — 충전 중이거나 최근(USB_DETECT_HOLD_MS) 충전 감지됨 */
static inline bool _is_usb_powered(void) {
#if BOARD_ALWAYS_USB_POWERED
  /* ★2026-08-16 배터리·충전회로가 없는 보드(테스트 보드). CHG_STAT 이 플로팅이라
   *  읽어봐야 의미가 없고, 배터리로 오판하면 유휴 10초에 화면이 꺼져
   *  "화면이 안 나온다"가 된다(실측). 전원은 항상 USB 뿐이므로 참으로 고정. */
  return true;
#else
  if (s_usb_sim_off) return false;          /* 시뮬: 배터리 모드로 강제 */
  if (btn_handler_is_charging()) return true;
#if BOARD_CHG_STAT_ACTIVE_HIGH
  /* ★★2026-08-11 이 보드(xiao-c6/h2)는 **hold 를 쓰지 않는다**.
   *
   *  hold(60초)는 GNPE 용이다. 거기선 CHG_STAT 이 MCP73831 의 STAT 핀 **직결**이라
   *  충전이 끝나면 핀이 토글해 USB/배터리 모드가 깜빡거린다 → 여운이 필요했다.
   *
   *  그런데 여기선 CHG_STAT 이 **VBUS 분압**이다(GPIO17 ← 5V 를 100k/150k).
   *  꽂히면 HIGH, 빠지면 LOW 로 명확하고 흔들리지 않으므로 여운이 해롭기만 하다:
   *    · 화면 OFF 가 10초 → 60초로 늦어짐 (사용자 신고: "분리해도 한참 켜져 있다")
   *    · PM 절전(DFS/light sleep) 진입이 60초 지연 → 그동안 전속으로 소모
   *    · [BATLOG] 방전 세션 기준점이 최대 60초 늦게 잡힘
   *  → VBUS 를 그대로 믿는다. 분리 즉시 배터리 모드로 전환된다. */
  return false;
#else
  if (s_last_chg_active_us == 0) return false;
  int64_t ago = esp_timer_get_time() - s_last_chg_active_us;
  return (ago < (int64_t)USB_DETECT_HOLD_MS * 1000);
#endif
#endif /* BOARD_ALWAYS_USB_POWERED */
}

/* ═══════════════════════════════════════════════
   hold_ms 클램핑 헬퍼 (0.1초 ~ 15초)
═══════════════════════════════════════════════ */
static inline uint32_t _clamp_hold(uint32_t hold_ms) {
  if (hold_ms < CFG_BTN_MIN_HOLD_MS && hold_ms > 0)
    return CFG_BTN_MIN_HOLD_MS;
  if (hold_ms > CFG_BTN_MAX_HOLD_MS)
    return CFG_BTN_MAX_HOLD_MS;
  return hold_ms;
}

/* ═══════════════════════════════════════════════
   버튼 누름 → 송신, 뗌 → 정지 (정품 리모컨과 동일)
   ──────────────────────────────────────────────
   UP/DOWN/PROG 는 PRESS 에서 abortable 송신을 시작한다. somfy_rts_send 가
   버튼을 누르고 있는 동안 동일 frame 을 계속 반복 송신하고, RELEASE 에서
   somfy_rts_abort 를 세우면 한 frame(~150ms) 안에 멈춘다. 별도의 반복
   태스크나 누름시간 임계값은 없다 — 누른 만큼만 송신, 떼면 바로 정지. */

/* ═══════════════════════════════════════════════
   Somfy RTS 전송 — 큐 + worker 태스크 (논블로킹)
   ──────────────────────────────────────────────
   v1.0 까지 _send_command()는 _btn_event_cb()에서 직접 호출되어
   somfy_rts_send() (1~1.5s blocking) 가 끝날 때까지 버튼 폴링 태스크가
   stall. 그 사이 사용자가 다른 버튼을 누르면 raw_state 전환이 누락되어
   버튼이 "먹통"되는 현상이 발생. v1.1에서 RF 송신을 전용 worker 태스크로
   분리해 콜백은 큐에만 push 후 즉시 반환. UI는 setting state도 즉시
   반영되어 화면이 곧바로 바뀐다.
═══════════════════════════════════════════════ */

typedef struct {
  somfy_command_t cmd;
  uint32_t        hold_ms;
  uint8_t         endpoint_idx;   // matter callback 전용 (255 = btn-triggered)
  bool            abortable;      // true=hold 반복(버튼 뗌 시 중단 가능)
  uint8_t         step_count;     // ★ tilt 슬라이더 10% 당 1 step (기본 1)
  bool            keep_rolling;   // true=롤링코드 증가 안 함(hold 반복 — 정품처럼 같은 코드 유지)
} rf_job_t;

/* ★★2026-07-24 임시 진단 빌드 (사용자 요청):
 *  "충전률 측정 / 버튼 RF 송신 부분을 바꾸면서 OLED 가 망가진 것 같다" →
 *  두 기능을 임시로 빼고 화면 멈춤이 사라지는지 확인한다. 원인 격리용이며,
 *  확인이 끝나면 두 값을 0 으로 되돌릴 것(코드는 삭제하지 않고 #if 로만 차단).
 *    TEMP_NO_CHARGE : 배터리 ADC init + 충전 감지 비활성
 *    TEMP_NO_BTN_RF : 버튼으로 발생하는 RF 송신 job 투입 차단(Matter 경로는 유지) */
/* 2026-07-24 격리 2단계: 둘 다 끄면 멈춤 없음 확인됨 → RF 만 되살려 범인 판별.
 *   RF 살린 뒤 멈추면 → 버튼 RF 송신이 원인
 *   RF 살려도 안 멈추면 → 충전률 측정이 원인 */
/* ★★2026-08-11 **충전측정 재활성**(TEMP_NO_CHARGE 1 → 0).
 *  차단해 뒀던 진짜 이유는 "ADC 가 OLED 를 깬다"가 아니라 **직렬화 구멍**이었다:
 *    · `_read_bat_mv()` 는 oled_ui_i2c_trylock() 으로 `_fb_flush` 하고만 직렬화됐다.
 *    · 그런데 OLED 전송 경로는 flush 말고도 두 곳이 더 있고 **락이 없었다** —
 *        `_oled_send_cmds()`(화면 자동 OFF/ON, 10초마다) / `_bbo_probe()`(5초 재검출).
 *    · somfy_app(prio 4) 이 oled_ui(prio 3) 를 선점하므로, 이 무보호 경로가
 *      flush 중인 비트뱅 전송을 중간에 끊어 SSD1306 을 고착시켰다.
 *  → oled_ui.c 에서 락 범위를 **`_bbo_write()` 전송 함수 자체**로 옮기고 재귀
 *    뮤텍스로 바꿔 구멍을 막았다(그쪽 주석 참조). 그래서 이제 되살린다.
 *  검증: sim/tools/adc_oled_mutex_sim.py — 수정 전 8회x10분 중 6회 고착(최빠른 80초),
 *        수정 후 손상 0 / 고착 0, 배터리 측정 952/952 성공(기아 없음).
 *  ※측정 주기(5초)와 표본 수(8회)는 **그대로 둔다**. 줄이면 `_nobat_track` 의
 *    "5분 창 / 반쪽당 30표본" 노이즈 상쇄 가정이 깨져 배터리 미연결 오판이 난다.
 *  ※되돌리려면 TEMP_NO_CHARGE 를 1 로. 관찰 지표는 [OLEDMON] 의 실패/락타임아웃. */
#define TEMP_NO_CHARGE   0   /* ★충전측정: 되살림(직렬화 구멍 수정 후) */
#define TEMP_NO_BTN_RF   0   /* ★버튼 RF 송신: 되살림 */

#define RF_JOB_FROM_BTN     0xFF
#define RF_QUEUE_DEPTH      8

static QueueHandle_t s_rf_queue = NULL;

/* app_main.cpp 가 RF init 성공 시에만 true 로 set (CC1101 하드웨어
 * 미응답이면 false). false 일 때 somfy_rts_send 를 호출하면
 * g_somfy.cc1101==NULL → cc1101_set_frequency(NULL) 크래시(재부팅).
 * 따라서 RF 미준비 시 송신을 건너뛴다(버튼/메뉴/OLED 는 정상 동작). */
extern bool g_rf_ready;

/* 동시작동(combo): s_held_* 는 어느 버튼이 '논리적으로' 눌려있는지의 이벤트 기반 플래그
 *  (PRESS set, RELEASE clear). _hold_repeat_task 가 이 조합으로 combo(UP+DOWN 등)를 판정해
 *  연속 재송신한다. 이벤트 기반이라 btn_is_pressed 캐시 글리치보다 강하다. */
static volatile bool s_held_up = false, s_held_down = false, s_held_rot = false;
static volatile bool s_held_prog = false;          /* PROG 홀드(RF 발생 버튼) */
static volatile int64_t s_rf_btn_until_us = 0;     /* RF 발생 버튼의 보호구간 종료시각 */

/* ★2026-07-24 사용자 요청 — 조합/RF 중 채널변경 버튼(SELECT·좌·우) 무시.
 *  왜: 상/하, 상/정지, 하/정지 를 **동시에** 누를 때 손가락이 스치며 좌/우 가 같이
 *  눌려 채널이 바뀌어 버렸다. 또 상/하/정지/PROG 로 RF 를 쏘는 중에 채널이 바뀌면
 *  엉뚱한 블라인드로 명령이 나간다.
 *  동시 누름은 완전히 같은 순간이 아니라 **약간의 텀**이 있으므로, RF 발생 버튼이
 *  눌린 뒤 CFG_CH_LOCK_MS 동안은 채널 변경을 막는다(떼어도 여운 유지).
 *  ※정지(MY)=로터리 버튼(ROT), 채널변경=SELECT/좌/우 임을 사용자 확인. */
#ifndef CFG_CH_LOCK_MS
#define CFG_CH_LOCK_MS  700    /* RF 버튼 눌림 후 채널변경 차단 유지시간(ms) */
#endif
static inline void _ch_lock_touch(void) {          /* RF 발생 버튼 눌릴 때 갱신 */
  s_rf_btn_until_us = esp_timer_get_time() + (int64_t)CFG_CH_LOCK_MS * 1000;
}

/* ★★2026-08-11 보완 — "애니메이션 재생 중인데 좌/우가 먹는다"는 재신고 수정.
 *
 *  기존 잠금이 실사용에서 뚫린 이유 2가지 (sim/tools/ch_lock_sim.py 로 확정):
 *
 *   (1) `s_ui.state == OLED_STATE_ACTION` 판정이 긴 누름에서 무력하다.
 *       oled_ui 는 `now - action_start_ms >= OLED_ACTION_DISPLAY_MS(2.5초)` 로 상태를
 *       푸는데, `action_start_ms` 는 **누른 시각**이다. 블라인드를 올리려고 2.5초보다
 *       길게 누르면(=정상 사용) **떼는 순간 이미 2.5초가 지나 있어** 상태가 즉시
 *       NORMAL 로 떨어지고 잠금도 같이 풀린다.
 *       → 뗄 때 `_ch_lock_release()` 로 **'뗀 시각' 기준** 여운을 다시 건다.
 *
 *   (2) 실제 RF 송신 진행 여부를 아예 안 봤다. 송신은 버튼을 뗀 뒤에도 마지막
 *       job(1~1.5초)이 남고, hold 반복으로 큐에 쌓여 있으면 더 길다. 이 구간에
 *       채널이 바뀌면 **엉뚱한 블라인드로 명령이 나간다**(원래 요청의 핵심 위험).
 *       → 진행 중(`s_rf_tx_busy`) + 큐 대기분까지 잠금에 포함한다.
 *
 *  실측 시뮬: 누름 3초일 때 기존은 3.0~7.2초(4.2초간) 채널변경이 열려 있었다. */
static volatile bool s_rf_tx_busy = false;   /* _do_rf_send 진행 중 */
static volatile somfy_command_t s_rf_tx_cmd = 0;  /* 진행 중 job 의 cmd (0=없음) */

/* 같은 cmd 가 **이미 송신 중**인가.
 *  ★2026-08-13 필수 안전장치. PRESS 송신이 abortable+hold_ms=15s 로 바뀌면서,
 *  연타(탭→탭) 때 이전 job 이 아직 살아 있는 상태에서 새 PRESS 가
 *  `somfy_rts_abort=false` 로 중단요청을 해제한다. 여기서 job 을 하나 더 넣으면
 *  이전 job 이 abort 를 못 만나 **max_loops(15.3초)까지 폭주**하고 새 job 은 그
 *  뒤에 줄서서 뗀 뒤에 나간다. 같은 cmd 면 새로 넣지 않고 진행 중인 job 을
 *  그대로 이어 쓰는 것이 맞다(사용자 입장에선 계속 눌린 것과 동일).
 *  sim/tools/hold_gap_sim.py 케이스 ⑦(연타 3회)에서 확인. */
static inline bool _rf_same_cmd_inflight(somfy_command_t cmd) {
  return s_rf_tx_busy && s_rf_tx_cmd == cmd;
}

static inline void _ch_lock_release(void) {  /* RF 발생 버튼 뗄 때 — 뗀 시각 기준 연장 */
  int64_t until = esp_timer_get_time() + (int64_t)OLED_ACTION_DISPLAY_MS * 1000;
  if (until > s_rf_btn_until_us) s_rf_btn_until_us = until;
}

/* 지금 채널 변경을 막아야 하는가 */
static inline bool _ch_locked(void) {
  if (s_held_up || s_held_down || s_held_rot || s_held_prog) return true;  /* 홀드 중 */
  /* ★2026-07-24 사용자 요청 — **애니메이션이 끝날 때까지** 채널변경 금지.
   *  버튼을 떼도 동작 애니메이션(OLED_STATE_ACTION)이 재생되는 동안에는 아직
   *  그 명령이 진행 중인 것으로 보이므로, 이 구간에 채널이 바뀌면 사용자가
   *  "방금 조작한 채널"과 화면이 어긋난다. 애니메이션 종료까지 잠금을 연장한다. */
  if (s_ui.state == OLED_STATE_ACTION || s_ui.action_active) return true;
  /* ★송신이 실제로 진행 중이거나 큐에 남아 있으면 무조건 잠금 (위 (2)) */
  if (s_rf_tx_busy) return true;
  if (s_rf_queue && uxQueueMessagesWaiting(s_rf_queue) > 0) return true;
  return esp_timer_get_time() < s_rf_btn_until_us;                          /* 여운 */
}

/* ═══ 버튼 지속 누름 → 신호 연속 발생 (과거 _hold_repeat_task 복원) ══════════════════
 *  UP/DOWN/STOP 를 누르고 있는 동안 CFG_BTN_HOLD_REPEAT_MS(500ms)마다 재송신한다.
 *  s_action_press_us : 세션 첫 누름 시각(전부 뗌=0). 탭(HOLD_REPEAT_START_MS 미만)은
 *  PRESS 의 1회 송신만 나가고, 그 이상 유지하면 _hold_repeat_task 가 연속 재송신한다. */
#define HOLD_REPEAT_START_MS 500                 /* 이 시간 이상 눌러야 반복 시작(탭=1회) */
static volatile int64_t s_action_press_us = 0;   /* 세션 첫 누름 시각(0=전부 뗌) */
static volatile somfy_command_t s_last_sent_cmd = 0;  /* 직전 송신 cmd — 같으면 롤링코드 고정(정품 매칭) */

/* 실제 RF 송신 (worker task 내부에서만 호출) */
static void _do_rf_send_inner(const rf_job_t *job);

/* ★2026-07-23 RF 송신 전 구간 OLED I2C 차단 래퍼.
 *  왜: CC1101 447MHz 송신(1~1.5초) 노이즈가 I2C 트랜잭션을 깨뜨려 SSD1306 이
 *  전송 중간 상태로 **고착**되는 것이 실사용에서 확인됐다(좌/우=RF없음 정상,
 *  상/하=RF송신 후 느려지다 멈춤). 고착되면 모듈 전원차단 전엔 복구 불가
 *  (RES 핀 없는 4핀 모듈)이므로, 가장 위험한 구간엔 버스를 아예 안 건드린다.
 *  _do_rf_send 는 return 경로가 여러 개라 래퍼로 감싸 해제 누락을 원천 차단. */
static void _do_rf_send(const rf_job_t *job)
{
  oled_ui_set_rf_tx(true);
  s_rf_tx_busy = true;          /* ★채널변경 잠금용 — _ch_locked() 가 참조 */
  s_rf_tx_cmd  = job->cmd;      /* ★중복 job 방지용 — _rf_same_cmd_inflight() 참조 */
  _do_rf_send_inner(job);
  s_rf_tx_cmd  = 0;
  s_rf_tx_busy = false;
  oled_ui_set_rf_tx(false);
}

static void _do_rf_send_inner(const rf_job_t *job)
{
  if (!g_rf_ready) {
    ESP_LOGW(TAG, "RF 미준비(CC1101 하드웨어 미응답) — 송신 skip (cmd=%d)",
             job->cmd);
    return;
  }
  uint32_t hold_ms = _clamp_hold(job->hold_ms);
  somfy_rts_abortable = job->abortable;   /* hold 반복 job 만 중단 허용 */

  if (job->endpoint_idx != RF_JOB_FROM_BTN) {
    /* Matter 콜백 경로: 특정 endpoint만.
     *  step_count > 1 → Tilt 슬라이더 다단 step. somfy_rts_send_steps 가
     *  CC1101 TX 모드를 한 번만 진입한 채 N step (각 2 frames) 을 연속 송신
     *  → step 간 ~170ms 단축 → 슬랫이 끊어짐 없이 회전.
     *  step_count == 1 → 일반 단일 송신 (기존 somfy_rts_send 경로). */
    if (job->endpoint_idx >= s_mgr.count) return;
    uint8_t steps = (job->step_count > 0) ? job->step_count : 1;
    if (steps > 1) {
      somfy_rts_send_steps(&s_somfy, &s_mgr.blinds[job->endpoint_idx],
                            job->cmd, steps);
    } else {
      somfy_rts_keep_rolling = false;   /* Matter 단일 송신 — 매번 새 롤링코드 */
      somfy_rts_send(&s_somfy, &s_mgr.blinds[job->endpoint_idx],
                     job->cmd, hold_ms);
    }
    blind_manager_save_rolling(&s_mgr, job->endpoint_idx);
    return;
  }

  somfy_blind_t *targets[BLIND_MAX_COUNT];
  uint8_t count = 0;
  blind_manager_get_targets(&s_mgr, targets, &count);

  if (count == 0) {
    ESP_LOGW(TAG, "선택된 블라인드 없음 (cmd=%d)", job->cmd);
    return;
  }

  /* combo 는 _btn_event_cb/_hold_repeat_task 가 s_held_* 조합으로 이미 판정해 cmd 에 실어 보낸다.
   *  worker 는 받은 cmd 를 그대로 송신한다(150ms combo window 제거 → 연속 송신 지연 없음). */
  somfy_command_t eff_cmd = job->cmd;

  /* 위치값은 cmd 로 1회 결정(채널·ALL 공통) */
  uint8_t pos = 50;
  switch (eff_cmd) {
    case SOMFY_CMD_UP:   pos = 100; break;
    case SOMFY_CMD_DOWN: pos = 0;   break;
    case SOMFY_CMD_MY:   pos = 50;  break;
    default: break;
  }
  for (int i = 0; i < count; i++) {
    somfy_rts_keep_rolling = job->keep_rolling;   /* hold 반복이면 같은 롤링코드 재사용(정품 매칭) */
    somfy_rts_send(&s_somfy, targets[i], eff_cmd, hold_ms);

    /* targets[i] 가 채널(blinds[]) 인지 ALL 블록(all_blocks[]) 인지 판별 */
    int  off   = (int)(targets[i] - s_mgr.blinds);
    bool is_ch = (off >= 0 && off < BLIND_MAX_COUNT);
    blind_manager_save_rolling(&s_mgr, is_ch ? (uint8_t)off : 0);  /* save_rolling 은 idx 무시(전체 저장) */

    if (is_ch) {
      matter_blinds_update_position((uint8_t)off, pos);
    } else {
      /* ALL 블록 g → 그 블록이 덮는 채널 endpoint 들을 한꺼번에 갱신 */
      int g = (int)(targets[i] - s_mgr.all_blocks);
      int s = g * BLINDS_PER_BLOCK, e = s + BLINDS_PER_BLOCK;
      if (e > BLIND_MAX_COUNT) e = BLIND_MAX_COUNT;
      for (int k = s; k < e; k++)
        matter_blinds_update_position((uint8_t)k, pos);
    }
  }

  ESP_LOGI(TAG, "RF 송신: cmd=%d hold=%" PRIu32 "ms 대상=%d개",
           job->cmd, hold_ms, count);
}

/* RF worker 태스크 — 큐에서 job 꺼내 송신 (직렬화) */
/* ★ 틸트(로터리 CW/CCW) 코얼레스 플래그 — 연속 회전 시 큐 적체로 신호가
 *  수 초씩 지연되던 문제 수정. 틸트 송신이 진행/대기 중이면 새 detent 를
 *  흡수해 큐를 쌓지 않는다 → 회전을 멈추면 송신도 ≤한 burst 안에 멈춘다. */
static volatile bool s_tilt_inflight = false;

static void _rf_worker_task(void *pvParam)
{
  rf_job_t job;
  while (1) {
    if (xQueueReceive(s_rf_queue, &job, portMAX_DELAY) == pdTRUE) {
      _do_rf_send(&job);
      s_tilt_inflight = false;   /* 작업 완료 — 다음 틸트 detent 허용 */
    }
  }
}

/* 콜백 호출 측에서 사용 — 큐에 push만 하고 즉시 반환.
 * 큐가 가득 차면 가장 오래된 job을 버리고 새 job 삽입 (실시간 응답 우선). */
static void _send_command_ex(somfy_command_t cmd, uint32_t hold_ms,
                             bool abortable, bool keep_rolling) {
  if (s_rf_queue == NULL) return;
#if TEMP_NO_BTN_RF
  /* ★임시: 버튼발 RF 송신 차단(진단용). 화면/메뉴 동작은 그대로, 전파만 안 나간다. */
  ESP_LOGW(TAG, "[TEMP] 버튼 RF 송신 차단됨 (cmd=%d) — 진단 빌드", cmd);
  (void)hold_ms; (void)abortable; (void)keep_rolling;
  return;
#endif

  rf_job_t job = { .cmd = cmd, .hold_ms = hold_ms,
                   .endpoint_idx = RF_JOB_FROM_BTN, .abortable = abortable,
                   .keep_rolling = keep_rolling };

  /* 가득 차면 가장 오래된 항목을 폐기하고 새 항목으로 교체 */
  if (xQueueSend(s_rf_queue, &job, 0) != pdTRUE) {
    rf_job_t drop;
    xQueueReceive(s_rf_queue, &drop, 0);
    xQueueSend(s_rf_queue, &job, 0);
    ESP_LOGW(TAG, "RF 큐 overflow — 오래된 cmd=%d 폐기", drop.cmd);
  }
}

/* 일반 누름 — 끝까지 송신(중단 불가). */
static void _send_command(somfy_command_t cmd, uint32_t hold_ms) {
  _send_command_ex(cmd, hold_ms, false, false);
}

/* 틸트(로터리 CW/CCW) 전용 — 코얼레스. 틸트 송신이 진행/대기 중이면
 *  새 detent 는 흡수한다(큐에 쌓지 않음). 연속 회전해도 송신이 큐 적체로
 *  수 초 뒤처지지 않고, 회전을 멈추면 ≤한 burst(~0.45s) 안에 멈춘다. */
static void _send_command_tilt(somfy_command_t cmd) {
  if (s_tilt_inflight) return;          /* 진행/대기 중 — 이번 detent 흡수 */
  s_tilt_inflight = true;
  _send_command_ex(cmd, 0, false, false);   /* 3-frame 단발 burst */
}

/* 버튼 PRESS 전용 — 정품처럼 **뗄 때까지 연속 송신**한다.
 *  hold_ms=CFG_BTN_MAX_HOLD_MS + abortable → 뗌(somfy_rts_abort)에서 종료.
 *  같은 cmd 가 이미 송신 중이면 새 job 을 만들지 않는다(연타 시 폭주/유출 방지 —
 *  _rf_same_cmd_inflight 주석 참조). */
static void _send_command_press(somfy_command_t cmd) {
  if (_rf_same_cmd_inflight(cmd)) return;    /* 진행 중 job 을 그대로 이어 쓴다 */
  _send_command_ex(cmd, CFG_BTN_MAX_HOLD_MS, true, false);
}

/* 현재 눌린 조합 → 커맨드 */
static inline somfy_command_t _held_combo_cmd(void) {
  if (s_held_up && s_held_down)  return SOMFY_CMD_UP_DOWN;
  if (s_held_up && s_held_rot)   return SOMFY_CMD_MY_UP;
  if (s_held_down && s_held_rot) return SOMFY_CMD_MY_DOWN;
  if (s_held_up)                 return SOMFY_CMD_UP;
  if (s_held_down)               return SOMFY_CMD_DOWN;
  if (s_held_rot)                return SOMFY_CMD_MY;
  return (somfy_command_t)0;
}

/* ═══ 버튼 지속 누름 → 신호 **연속** 발생 태스크 ═════════════════════════════════
 *
 *  ★★2026-08-13 전면 개편 — "첫 신호 뒤 0.5초 텀" 제거 (사용자 신고).
 *
 *  [실측 근거] 정품 SDR 캡처 전수 분석
 *      데이터 D:\RTL_SDR\sdrsharp-x64\somfy_rts_447 (755개), 분석 scratchpad/hold_pattern.py
 *      ON 길이가 330ms(짧은 탭) → 400 → 700 → 900 → **1360ms** 로 **끊김 없이 한 덩어리**.
 *      간격이 보이는 캡처(340[1390]340)는 뗐다 다시 누른 **별개 누름**이다.
 *      ⇒ 정품은 누르고 있는 동안 계속 쏜다. 조각내지 않는다.
 *
 *  [기존 구조의 결함] PRESS 가 hold_ms=0(=3프레임 429ms, 중단불가)만 쏘고 멈춘 뒤,
 *      이 태스크가 HOLD_REPEAT_START_MS(500ms) 게이트를 지나서야 500ms 주기로 재개했다.
 *      게다가 이 태스크는 누름과 **위상이 맞지 않는 자유주행** 500ms 태스크다.
 *      sim/tools/hold_gap_sim.py 위상 0~499ms 스윕 결과 → 최악 공백 **561ms**.
 *      사용자가 말한 "0.5초 텀"이 바로 이것.
 *
 *  [새 구조]
 *    (1) PRESS 가 abortable + hold_ms=CFG_BTN_MAX_HOLD_MS 로 **뗄 때까지 스스로 연속 송신**.
 *        뗌 = somfy_rts_abort=true → 프레임 경계에서 종료. (정품과 동일)
 *    (2) 이 태스크에 남은 역할은 **조합 변경(UP+DOWN 진입/이탈 등)뿐**이다.
 *        같은 조합이 유지되는 동안은 아무것도 하지 않는다 — 주기 재송신을 없앴다.
 *        (주기 재송신을 남기면 같은 명령 job 이 이중으로 쌓이고, RF worker 가 직렬이라
 *         뗀 뒤에 뒤늦게 나가는 유출이 된다.)
 *
 *  [★함정 — 반드시 지킬 것] somfy_rts.c 의 abort 검사는 **프레임 경계에서 1번뿐**이고
 *      프레임은 ≈143ms 다. 그래서 `abort=true; delay(20); abort=false;` 같은 **짧은 펄스는
 *      그냥 놓친다** → 이전 job 이 안 죽고 max_loops(15.3초)까지 살아버린다.
 *      그러므로 abort 는 **레벨로 계속 세워둔 채**, 진행 중 job 이 **실제로 끝난 것을
 *      확인한 뒤에야**(s_rf_tx_busy=false && 큐 빔) 해제하고 새 조합을 넣는다.
 *      대기 중에만 COMBO_SWITCH_POLL_MS 로 촘촘히 보고, 평상시 주기는 그대로 둔다
 *      (유휴 wakeup 빈도=소비전류를 건드리지 않기 위해).
 *
 *  검증: sim/tools/hold_gap_sim.py — 케이스 ①~⑩ + 위상 스윕 전부 공백 0. */
#define COMBO_SWITCH_POLL_MS 20   /* 조합 전환 **대기 중에만** 쓰는 촘촘한 주기 */
static volatile somfy_command_t s_combo_pending = 0;  /* 전환 대기 중(0=없음) */

static void _hold_repeat_task(void *pvParam) {
  while (1) {
    uint32_t period = CFG_BTN_HOLD_REPEAT_MS;   /* 평상시 주기 — 변경하지 않음 */
    const int64_t press_us = s_action_press_us;
    const bool held = (s_held_up || s_held_down || s_held_rot);


    if (s_combo_pending != 0) {
      /* ── 조합 전환 대기: 진행 중 송신이 끝나기를 기다린다 ── */
      if (!held || press_us == 0) {
        s_combo_pending = 0;        /* 기다리는 사이 다 뗌 → 새 송신 금지(유출 방지) */
      } else if (!s_rf_tx_busy &&
                 (s_rf_queue == NULL || uxQueueMessagesWaiting(s_rf_queue) == 0)) {
        somfy_command_t cmd = _held_combo_cmd();   /* 대기 중 또 바뀌었을 수 있다 */
        s_combo_pending = 0;
        if (cmd != 0) {
          s_last_sent_cmd = cmd;
          somfy_rts_abort = false;                 /* 이제서야 해제 — 이전 job 은 죽었다 */
          _send_command_ex(cmd, CFG_BTN_MAX_HOLD_MS, true, false);
        }
      } else {
        period = COMBO_SWITCH_POLL_MS;             /* 아직 송신 중 — 곧 다시 확인 */
      }
    } else if (press_us != 0 && held
               && s_ui.state == OLED_STATE_ACTION && s_ui.action_active
               && (esp_timer_get_time() - press_us) / 1000 >= HOLD_REPEAT_START_MS) {
      /* ── 조합 변경 감지 ── */
      somfy_command_t cmd = _held_combo_cmd();
      if (cmd != 0 && cmd != s_last_sent_cmd) {
        /* 조합이 바뀌었다 → 진행 중 송신을 끊고 **새 롤링코드**로 다시 시작.
         *  정품도 조합이 바뀌면 새 코드다(같은 조합 유지 중엔 같은 코드 반복). */
        s_last_sent_cmd = cmd;
        somfy_rts_abort = true;     /* ★레벨 유지 — 프레임 경계에서 잡힐 때까지 */
        s_combo_pending = cmd;
        period = COMBO_SWITCH_POLL_MS;
      }
    }
    g_wake_iter[WI_HOLD]++;            /* ★깨어남 출처 계측 */
    vTaskDelay(pdMS_TO_TICKS(period));
  }
}

/* ── 시리얼 콘솔(app_main.cpp 의 tx/sel 명령) 진입점 — 자동 테스트 하네스.
 *  버튼과 동일한 RF 큐 경로로 송신하되 session-gate 를 거치지 않고 cmd 를
 *  그대로 쏜다(combo 직접 지정 가능). PC 가 "tx updown" 으로 무인 송신 →
 *  somfy_cli 캡처 → 분석 루프. */
void somfy_app_console_tx(int cmd, uint32_t hold_ms) {
  somfy_rts_abort = false;
  _send_command_ex((somfy_command_t)cmd, hold_ms, false, false);
}

/* ★2026-08-15 byte8 캘리브레이션 — `tx8 <raw hex> [cmd] [hold_ms]`.
 *  raw byte8 을 강제로 지정해 송신한다. SDR 로 rtl_433 표시값을 읽어
 *  **raw ↔ 표시 대응표**를 만드는 것이 목적이다(추측 금지 — somfy_rts.c 주석 참조).
 *  정품 hold 는 표시 0xC4 이므로, 그걸 만드는 raw 값을 찾으면 PROG 긴 누름을
 *  정품과 같게 만들 수 있다. 송신이 끝나면 override 를 반드시 해제한다. */
void somfy_app_console_tx_byte8(int raw_b8, int cmd, uint32_t hold_ms) {
  somfy_rts_byte8_override = raw_b8;         /* <0 이면 해제 */
  somfy_rts_abort = false;
  _send_command_ex((somfy_command_t)cmd, hold_ms, false, false);
  /* 큐가 직렬이라 이 job 이 끝난 뒤 풀어야 한다 — worker 가 실제로 비워질 때까지 대기 */
  for (int i = 0; i < 200; i++) {
    if (!s_rf_tx_busy &&
        (s_rf_queue == NULL || uxQueueMessagesWaiting(s_rf_queue) == 0)) break;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  somfy_rts_byte8_override = -1;
  ESP_LOGW(TAG, "[TX8] raw byte8=0x%02X cmd=%d hold=%lums 송신 완료 — override 해제",
           raw_b8 & 0xFF, cmd, (unsigned long)hold_ms);
}
void somfy_app_console_select(int n) {
  blind_manager_select(&s_mgr, (uint8_t)n);
  oled_ui_set_blind(&s_ui, (uint8_t)n);
}
/* 시리얼 콘솔 freq — 주파수 클램프(재부팅 보존) 무인 검증용.
 *  setfreq: blind idx 주파수 지정(447.20~447.79 클램프 + NVS 저장). print: 전체 출력. */
void somfy_app_console_setfreq(int idx, float mhz) {
  blind_manager_set_freq(&s_mgr, (uint8_t)idx, mhz);
  ESP_LOGI(TAG, "[FREQ] blind %d <- %.2f MHz (NVS 저장)", idx, (double)mhz);
}
void somfy_app_console_printfreq(void) {
  for (int i = 0; i < s_mgr.count; i++)
    ESP_LOGI(TAG, "[FREQ] blind %d: %.2f MHz", i, (double)s_mgr.blinds[i].freq_mhz);
}

/* Matter 콜백 전용 (특정 endpoint) — 큐에 push.
 *  step_count : 같은 cmd 를 N 번 송신할지 (Tilt 슬라이더 다단 step). 기본 1. */
static void _send_command_endpoint(uint8_t ep, somfy_command_t cmd,
                                    uint8_t step_count) {
  if (s_rf_queue == NULL) return;
  rf_job_t job = { .cmd = cmd, .hold_ms = 0, .endpoint_idx = ep,
                   .abortable = false,
                   .step_count = (step_count > 0) ? step_count : 1 };
  if (xQueueSend(s_rf_queue, &job, 0) != pdTRUE) {
    /* 큐 가득 — 가장 오래된 job 폐기 후 새 job 삽입. 디바이스 로터리 tilt 와
     *  SmartThings tilt 슬라이더가 동시에 들어올 때 발생 가능. drop 사실을
     *  로깅해 진단 가능하게 한다(_send_command_ex 와 동일한 정책). */
    rf_job_t drop;
    if (xQueueReceive(s_rf_queue, &drop, 0) == pdTRUE) {
      ESP_LOGW(TAG, "RF 큐 가득(EP=%u 깊이=%d) — 오래된 job(cmd=%d ep=%u step=%u) 폐기",
               (unsigned)ep, RF_QUEUE_DEPTH,
               drop.cmd, (unsigned)drop.endpoint_idx, (unsigned)drop.step_count);
    }
    xQueueSend(s_rf_queue, &job, 0);
  }
}

/* ═══════════════════════════════════════════════
   버튼 이벤트 콜백
═══════════════════════════════════════════════ */
/* 전방 선언 — 아래에 정의 */
static void _enter_screensaver(void);
static void _exit_screensaver(const char *reason);
static void _exit_sleep(const char *reason);

/* ── 블라인드 선택 순환 (dir=-1 이전, +1 다음) — SELECT(다음)의 좌우 버전.
 *  PCF8575 LEFT/RIGHT 버튼과 시리얼 콘솔 cyc 명령이 공용(하드웨어 무인 검증용).
 *  등록된 블라인드 + ALL 만 순환(미등록 슬롯 건너뜀). */
static void _blind_cycle(int dir) {
  uint8_t sel = s_mgr.selected, n;
  /* ALL 센티넬은 BLIND_SEL_ALL(=BLIND_MAX_COUNT). H2=3·C6=8 로 보드마다 다르므로
   *  절대 상수(옛 5)를 쓰면 안 된다 — 5 는 select 가 거부해 ALL 로 못 넘어간다.
   *  순환 규칙은 SELECT(다음)와 완전히 동일하게 두고 dir<0 만 역순으로. */
  if (dir > 0) {
    if (sel >= BLIND_SEL_ALL)                  n = 0;                       /* ALL → 첫 블라인드 */
    else if ((uint8_t)(sel + 1) < s_mgr.count) n = (uint8_t)(sel + 1);      /* 다음 등록 블라인드 */
    else                                       n = BLIND_SEL_ALL;          /* 마지막 → ALL */
  } else {
    if (sel == 0)                              n = BLIND_SEL_ALL;          /* 첫 → ALL */
    else if (sel >= BLIND_SEL_ALL)             n = (s_mgr.count > 0) ? (uint8_t)(s_mgr.count - 1) : BLIND_SEL_ALL;  /* ALL → 마지막 등록 */
    else                                       n = (uint8_t)(sel - 1);     /* 이전 등록 블라인드 */
  }
  blind_manager_select(&s_mgr, n);
  oled_ui_set_blind(&s_ui, n);
  if (n < s_mgr.count)                                  oled_ui_set_freq(&s_ui, s_mgr.blinds[n].freq_mhz);
  else if (n == BLIND_SEL_ALL && s_mgr.block_count > 0) oled_ui_set_freq(&s_ui, s_mgr.all_blocks[0].freq_mhz);
  ESP_LOGI(TAG, "블라인드 선택(%s): slot %d", dir > 0 ? "다음" : "이전", n);
}
/* 시리얼 콘솔 cyc <-1|1> — 물리 LEFT/RIGHT 없이 _blind_cycle 순환을 무인 검증. */
void somfy_app_console_cycle(int dir) { _blind_cycle(dir > 0 ? 1 : -1); }

#if BOARD_HAS_LR_BUTTONS
/* 주파수 편집 디지트 커서 이동 (0=0.1자리 '447.[X]', 1=0.01자리 '447.X[X]').
 *  LEFT=왼쪽(0.1자리), RIGHT=오른쪽(0.01자리). 값은 UP/DOWN 이 커서 자리값만큼
 *  증감(_freq_edit_step). 연속 렌더 태스크(20fps)가 다음 틱에 커서 밑줄을 그린다. */
static void _freq_cursor_move(int dir) {
  int c = (int)s_ui.freq_edit_cursor + dir;
  s_ui.freq_edit_cursor = (uint8_t)(c < 0 ? 0 : (c > 1 ? 1 : c));
  ESP_LOGI(TAG, "[FREQ_EDIT] 커서 → %s 자리", s_ui.freq_edit_cursor == 0 ? "0.1" : "0.01");
}
#endif

/* 마지막으로 **실제 적용된** PM 상태 코드 — 모니터 로그용.
 *   -1 = 미설정 / bit0 = DFS(min 80MHz) / bit1 = light sleep
 *   즉 0=전속·절전없음(USB), 1=DFS만, 3=DFS+light sleep(배터리·등록완료) */
static volatile int g_pm_state_applied = -1;

static int      s_bat_last_sm_mv = 0;   /* 최신 평활 전압(표시와 동일 값) */
static int      s_bat_last_raw_mv = 0;  /* 최신 원본 전압(평활 전) — 진단용 */
/* ★2026-08-12 (A) 한 번의 측정 안에서 8표본이 얼마나 흩어졌는가(ADC 카운트).
 *  8표본은 수십 us 안에 끝나 **같은 순간**을 재므로, 이 값은 순수 ADC 잡음이다.
 *  → 측정 **간** 31mV 격차의 정체를 가르는 판별자:
 *      좁음(2~4카운트)  = 그 순간 ADC 는 조용 → 격차는 **진짜 전압차**(부하/셀 임피던스)
 *      넓음(15카운트≈31mV) = **ADC 교란**(ADC1 채널 간섭이 남아 있다)
 *  배터리 구동 중엔 시리얼이 없으므로 NVS 방전기록에도 같이 넣는다. */
static uint8_t  s_bat_last_spread = 0;
/* ★2026-08-16 부모 RSSI 캐시 — 방전 로그(bat_sample_t.rssi)가 읽는다.
 *  갱신은 7초 Matter 상태 블록과 60초 [PWR] 두 곳(둘 다 이미 OT 락을 잡는다). */
static volatile int8_t s_last_rssi_cache = 127;
static int64_t  s_bat_last_us = 0;      /* 마지막 측정 시각 — 진단용 */
static int64_t  s_dis_t0_us      = 0;   /* 방전 세션 시작(USB 분리) 시각. 0=세션 없음 */
static int      s_dis_mv0        = 0;   /* 세션 시작 전압 */
static uint8_t  s_dis_pct0       = 0;   /* 세션 시작 % */
static int64_t  s_dis_prev_us    = 0;   /* 직전 로그 시각(구간 계산용) */
static int      s_dis_prev_mv    = 0;
static uint8_t  s_dis_prev_pct   = 0;

void somfy_app_batlog_button(int ev);   /* 아래 정의 — 방전 중 버튼 이벤트 NVS 기록 */

/* 버튼 이벤트 → 방전 기록용 코드. 좌/우가 이번 조사 대상이라 우선 구분한다. */
static int _btn_evt_blev(int t) {
  switch (t) {
#if BOARD_HAS_LR_BUTTONS
    case BTN_EVT_LEFT_PRESS:   return 1;   /* BLEV_LEFT */
    case BTN_EVT_RIGHT_PRESS:  return 2;   /* BLEV_RIGHT */
#endif
    case BTN_EVT_SELECT_PRESS: return 3;
    case BTN_EVT_UP_PRESS:     return 4;
    case BTN_EVT_DOWN_PRESS:   return 5;
    case BTN_EVT_ROT_PRESS:    return 6;
    case BTN_EVT_PROG_PRESS:   return 7;
    default:                   return 0;   /* release 등은 기록 안 함 */
  }
}

static const char *_btn_evt_name(int t) {
  switch (t) {
    case BTN_EVT_UP_PRESS:      return "UP↓";
    case BTN_EVT_UP_RELEASE:    return "UP↑";
    case BTN_EVT_DOWN_PRESS:    return "DN↓";
    case BTN_EVT_DOWN_RELEASE:  return "DN↑";
    case BTN_EVT_SELECT_PRESS:  return "SEL↓";
    case BTN_EVT_PROG_PRESS:    return "PROG↓";
    case BTN_EVT_PROG_RELEASE:  return "PROG↑";
    case BTN_EVT_ROT_PRESS:     return "ROT↓";
    case BTN_EVT_ROT_CLICK:     return "ROT클릭";
    case BTN_EVT_ROT_CW:        return "ROT→";
    case BTN_EVT_ROT_CCW:       return "ROT←";
#if BOARD_HAS_LR_BUTTONS
    case BTN_EVT_LEFT_PRESS:    return "LEFT↓";
    case BTN_EVT_LEFT_RELEASE:  return "LEFT↑";
    case BTN_EVT_RIGHT_PRESS:   return "RIGHT↓";
    case BTN_EVT_RIGHT_RELEASE: return "RIGHT↑";
#endif
    default:                    return "기타";
  }
}

static void _btn_event_cb(btn_event_data_t *evt, void *user_data) {
  /* ★2026-08-12 진단 로그 — "버튼을 누르면 배터리 %가 올라간다"의 원인 추적용.
   *  누른 버튼·그 시점의 전압(원본/평활)·표시%·화면상태·경과를 한 줄로 남긴다.
   *  ※여기서 ADC 를 직접 읽지 않는다: 이 콜백은 btn_task 컨텍스트라 _read_bat_mv 가
   *    쓰는 버튼 뮤텍스를 이미 쥐고 있을 수 있어 데드락/실패가 난다.
   *    대신 **마지막 주기 측정값**과 그 경과시간을 찍어, 5초 주기 로그와 시간순으로
   *    맞춰 보면 "누름 → 다음 측정에서 전압이 어떻게 변했나"가 그대로 드러난다.
   *  진단이 끝나면 이 블록을 지우거나 로그레벨을 낮출 것. */
  {
    const int64_t _n = esp_timer_get_time();
    ESP_LOGW(TAG, "[BTNDBG] %-7s hold=%lums | 원본%dmV 평활%dmV 표시%d%% (측정후 %lldms) "
                  "| 화면=%s 유휴%llds 무선=%s PM=%d | up=%llds",
             _btn_evt_name((int)evt->type), (unsigned long)evt->hold_ms,
             s_bat_last_raw_mv, s_bat_last_sm_mv,
             (int)s_ui.chg_percent,
             s_bat_last_us ? (_n - s_bat_last_us) / 1000 : -1,
             oled_ui_is_panel_on() ? "ON" : "OFF",
             (long long)((_n - s_last_activity_us) / 1000000),
             matter_blinds_get_radio_enabled() ? "ON" : "OFF",
             g_pm_state_applied, (long long)(_n / 1000000));
    /* ★USB 분리 상태(방전 세션)면 NVS 에도 남긴다 — 그때는 시리얼을 받을 호스트가
     *  없어 로그가 허공으로 나가기 때문. 누름(press) 만 기록해 링을 아끼고,
     *  실제 플래시 쓰기는 _batlog_flush_if_due 가 2초 단위로 합친다. */
    const int _blev = _btn_evt_blev((int)evt->type);
    if (_blev) somfy_app_batlog_button(_blev);
  }

  /* 모든 버튼 이벤트는 활동으로 기록 → 절전/화면보호기 타이머 리셋 */
  _mark_activity();

  /* 화면 보호기 / 절전 모드에서 첫 입력 시 즉시 복귀 (OLED panel ON).
   *  ★ stuck 복구: flag 가 false 여도 OLED state 가 SCREENSAVER 면 직전 wake
   *  가 실패한 것으로 보고 강제 재시도. */
  if (s_screensaver_active || s_ui.state == OLED_STATE_SCREENSAVER) {
    _exit_screensaver("button input");
  }
  if (s_is_sleeping) {
    _exit_sleep("button input");
  }

  /* 기기 조작 모션은 '기기에서 선택된' 블라인드 1개(또는 ALL)만 표시.
   *  s_mgr.selected: 0..N-1=개별, N(=BLIND_SEL_ALL)=ALL(전체 채널 비트). */
  oled_ui_set_action_blinds(&s_ui,
      (s_mgr.selected >= BLIND_SEL_ALL) ? (uint8_t)((1u << BLIND_MAX_COUNT) - 1)
                                        : (uint8_t)(1u << s_mgr.selected));

  switch (evt->type) {

#if BOARD_HAS_LR_BUTTONS
  /* ── 좌/우 버튼 (PCF8575 P10/P11) ──────────────────────────────
   *   메인 화면   : 블라인드 선택 이전/다음 (SELECT 의 좌우 버전)
   *   날짜 편집   : 편집 자리 이동 (년·월·일·시·분)
   *   주파수 편집 : 굵은 자리 ±0.1 (UP/DOWN 미세 ±0.01 보조) */
  case BTN_EVT_LEFT_PRESS:
    if (_ch_locked()) { ESP_LOGI(TAG, "[CHLOCK] RF 버튼 조작 중 — 채널변경 무시(LEFT)"); break; }
    if (s_setup_screen == SETUP_TIME_EDIT)      _time_edit_field_move(-1);
    else if (s_setup_screen == SETUP_FREQ_EDIT) _freq_cursor_move(-1);
    else if (!_in_setup_mode())                 _blind_cycle(-1);
    break;
  case BTN_EVT_RIGHT_PRESS:
    if (_ch_locked()) { ESP_LOGI(TAG, "[CHLOCK] RF 버튼 조작 중 — 채널변경 무시(RIGHT)"); break; }
    if (s_setup_screen == SETUP_TIME_EDIT)      _time_edit_field_move(+1);
    else if (s_setup_screen == SETUP_FREQ_EDIT) _freq_cursor_move(+1);
    else if (!_in_setup_mode())                 _blind_cycle(+1);
    break;
  case BTN_EVT_LEFT_RELEASE:
  case BTN_EVT_RIGHT_RELEASE:
    break;
#endif

  /* ── UP 버튼 ─────────────────────────────── */
  case BTN_EVT_UP_PRESS:
    _ch_lock_touch();
    if (_in_setup_mode()) {
      /* 메뉴: 커서 위로 / 주파수 편집: freq + / 그 외: 무시 */
      if (s_setup_screen == SETUP_MENU_SCR) {
        _setup_menu_cursor_move(-1);
      } else if (s_setup_screen == SETUP_FREQ_EDIT) {
        _freq_edit_step(+1);
      } else if (s_setup_screen == SETUP_TIME_EDIT) {
        _time_edit_field_move(-1);   /* 이전 자리 */
      } else {
        ESP_LOGI(TAG, "[SETUP] UP 무시 (screen=%d)", s_setup_screen);
      }
      break;
    }
    {
      /* ★버튼 지속 누름 → 연속 송신. 세션 첫 누름만 즉시 1회 송신하고 시각을 기록,
       *  이후 연속은 _hold_repeat_task 가 담당. 콤보(다른 버튼도 눌림)면 초기 1회는
       *  생략하고 플래그만 세운다 — 태스크가 s_held_* 조합으로 combo 를 재송신한다. */
      bool first = !(s_held_up || s_held_down || s_held_rot);
      s_held_up = true;
      if      (s_held_down) oled_ui_notify_action_start(&s_ui, OLED_ACTION_UP_DOWN);
      else if (s_held_rot)  oled_ui_notify_action_start(&s_ui, OLED_ACTION_MY_UP);
      else                  oled_ui_notify_action_start(&s_ui, OLED_ACTION_UP);
      somfy_rts_abort = false;                  /* 새 누름 — 이전 중단요청 해제 */
      if (first) {
        s_action_press_us = esp_timer_get_time();
        s_last_sent_cmd = SOMFY_CMD_UP;
        _send_command_press(SOMFY_CMD_UP);    /* ★뗄 때까지 연속(정품 매칭) */
      }
    }
    break;

  case BTN_EVT_UP_RELEASE:
    s_held_up = false;                          /* 이 버튼 놓음(combo 조합 갱신) */
    _ch_lock_release();                         /* ★뗀 시각 기준으로 채널변경 잠금 연장 */
    if (!(s_held_down || s_held_rot)) {         /* 전부 뗌 → 반복 게이트 닫고 진행 burst 중단 */
      s_action_press_us = 0;
      s_last_sent_cmd = 0;
      s_combo_pending = 0;                      /* ★조합 전환 대기 취소(유출 방지) */
      somfy_rts_abort = true;
    }
    if (_in_setup_mode()) break;
    if (!(s_held_down || s_held_rot)) oled_ui_notify_action_end(&s_ui);
    if (evt->hold_ms > CFG_BTN_MIN_HOLD_MS) {
      ESP_LOGI(TAG, "UP 버튼 뗌 (총 hold=%" PRIu32 "ms)", evt->hold_ms);
    }
    break;

  /* ── DOWN 버튼 ───────────────────────────── */
  case BTN_EVT_DOWN_PRESS:
    _ch_lock_touch();
    if (_in_setup_mode()) {
      if (s_setup_screen == SETUP_MENU_SCR) {
        _setup_menu_cursor_move(+1);
      } else if (s_setup_screen == SETUP_FREQ_EDIT) {
        _freq_edit_step(-1);
      } else if (s_setup_screen == SETUP_TIME_EDIT) {
        _time_edit_field_move(+1);   /* 다음 자리 */
      } else {
        ESP_LOGI(TAG, "[SETUP] DOWN 무시 (screen=%d)", s_setup_screen);
      }
      break;
    }
    {
      /* ★버튼 지속 누름 → 연속 송신(위 UP 과 동일 구조). */
      bool first = !(s_held_up || s_held_down || s_held_rot);
      s_held_down = true;
      if      (s_held_up)  oled_ui_notify_action_start(&s_ui, OLED_ACTION_UP_DOWN);
      else if (s_held_rot) oled_ui_notify_action_start(&s_ui, OLED_ACTION_MY_DOWN);
      else                 oled_ui_notify_action_start(&s_ui, OLED_ACTION_DOWN);
      somfy_rts_abort = false;                  /* 새 누름 — 이전 중단요청 해제 */
      if (first) {
        s_action_press_us = esp_timer_get_time();
        s_last_sent_cmd = SOMFY_CMD_DOWN;
        _send_command_press(SOMFY_CMD_DOWN);  /* ★뗄 때까지 연속(정품 매칭) */
      }
    }
    break;

  case BTN_EVT_DOWN_RELEASE:
    s_held_down = false;                        /* 이 버튼 놓음(combo 조합 갱신) */
    _ch_lock_release();                         /* ★뗀 시각 기준으로 채널변경 잠금 연장 */
    if (!(s_held_up || s_held_rot)) {           /* 전부 뗌 → 반복 게이트 닫고 진행 burst 중단 */
      s_action_press_us = 0;
      s_last_sent_cmd = 0;
      s_combo_pending = 0;                      /* ★조합 전환 대기 취소(유출 방지) */
      somfy_rts_abort = true;
    }
    if (_in_setup_mode()) break;
    if (!(s_held_up || s_held_rot)) oled_ui_notify_action_end(&s_ui);
    if (evt->hold_ms > CFG_BTN_MIN_HOLD_MS) {
      ESP_LOGI(TAG, "DOWN 버튼 뗌 (총 hold=%" PRIu32 "ms)", evt->hold_ms);
    }
    break;

  /* ── SW3 SELECT 버튼: 블라인드 선택 순환 ──
   *   설정 모드 (메뉴/하위 화면 모두) 에서는 무시 (v3.1+)
   *   v3.6: 버튼 모션 (OLED_STATE_ACTION) 진행 중일 때는 모션 중단 후 메인
   *         화면 표시 (블라인드 순환은 안 함) */
  case BTN_EVT_SELECT_PRESS:
    if (_ch_locked()) { ESP_LOGI(TAG, "[CHLOCK] RF 버튼 조작 중 — 채널변경 무시(SELECT)"); break; }
    if (_in_setup_mode()) {
      ESP_LOGI(TAG, "[SETUP] SELECT 무시");
      break;
    }
    if (s_ui.state == OLED_STATE_ACTION) {
      /* 모션 중단 → 메인 화면. 이후 블라인드 순환 로직 fall-through. */
      s_ui.state = OLED_STATE_NORMAL;
      s_ui.action = OLED_ACTION_NONE;
      s_ui.action_active = false;
      s_ui.anim_frame++;
      ESP_LOGI(TAG, "[SELECT] 버튼 모션 중단 → 블라인드 순환 진행");
      /* break 없음 — 아래 순환 로직 계속 실행 */
    }
    {
      /* ★ 등록된 블라인드 + ALL 만 순환한다(미등록 빈 슬롯은 건너뜀).
       *  이전엔 모든 슬롯 순환 가능 → 미등록 슬롯을 선택하면 명령
       *  전송 대상이 없어(get_targets=0) 버튼을 눌러도 무반응이었다.
       *  count=1 이면 0↔ALL, count=3 이면 0→1→2→ALL→0 식으로 순환. */
      uint8_t next;
      if (s_mgr.selected >= BLIND_SEL_ALL) {
        next = 0;                              /* ALL → 첫 블라인드 */
      } else if ((uint8_t)(s_mgr.selected + 1) < s_mgr.count) {
        next = s_mgr.selected + 1;             /* 다음 등록 블라인드 */
      } else {
        next = BLIND_SEL_ALL;                  /* 마지막 블라인드 → ALL */
      }

      blind_manager_select(&s_mgr, next);
      oled_ui_set_blind(&s_ui, next);
      if (next < s_mgr.count) {
        oled_ui_set_freq(&s_ui, s_mgr.blinds[next].freq_mhz);
      } else if (next == BLIND_SEL_ALL && s_mgr.block_count > 0) {
        oled_ui_set_freq(&s_ui, s_mgr.all_blocks[0].freq_mhz);   /* ALL — 블록 ALL 주파수 표시 */
      }
      ESP_LOGI(TAG, "블라인드 선택: %s (slot %d)",
               (next < s_mgr.count) ? s_mgr.blinds[next].name :
               (next == BLIND_SEL_ALL ? "ALL" : "(미등록)"),
               next);
    }
    break;

  case BTN_EVT_SELECT_RELEASE:
    if (_in_setup_mode()) break;
    /* SW3 누름은 단발성 동작이므로 별도 release 처리 불필요 */
    if (evt->hold_ms > CFG_BTN_MIN_HOLD_MS) {
      ESP_LOGI(TAG, "SELECT 버튼 뗌 (총 hold=%" PRIu32 "ms)", evt->hold_ms);
    }
    break;

  /* ── PROG 버튼 ─────────────────────────────
   *   v3.1+: Matter 페어링 / Thread 리셋 기능 제거 (설정 메뉴로 이전).
   *          PROG 는 단순히 Somfy PROG 커맨드만 송신. */
  case BTN_EVT_PROG_PRESS:
    _ch_lock_touch(); s_held_prog = true;
    if (_in_setup_mode()) {
      ESP_LOGI(TAG, "[SETUP] PROG 무시");
      break;
    }
    /* ALL 선택 시 PROG 금지 — PROG(신규등록/한계설정 등)는 단일 블라인드
     *  대상 절차이므로 ALL(전체) 에서는 동작하지 않는다. */
    if (s_mgr.selected >= BLIND_SEL_ALL) {
      /* ★2026-08-31 화면에도 알린다 — 로그만 남기면 사용자 눈에는 "버튼 고장"과
       *  구별되지 않는다(PROGGUARD 에서 실제로 그렇게 오해됐다). 이제 이것이
       *  PROG 가 조용히 무시되는 **유일하게 남은 경로**다. */
      ESP_LOGW(TAG, "PROG 무시 — ALL 선택 상태 (단일 블라인드만 가능)");
      oled_ui_show_notice(&s_ui, "ALL: no PROG", OLED_ACTION_DISPLAY_MS);
      break;
    }
    if (evt->hold_ms == 0) {
      /* ★★★2026-08-15 PROG 를 **정품 구조**로 재현 (연속 송신 제거).
       *  사용자 신고: "PROG 를 2초 이상 누르면 작동하지 않는다(정품은 됨)."
       *
       *  【정품 실측 — somfy_rts_447/remote_x1 의 cs16 버스트 길이】
       *    긴 누름 연속 버스트(1.3~2.0초)는 **Up/Down 구간에만** 존재하고
       *    PROG 구간(ch1 00:16:42~55 / ch2 00:20:25~37 / ch4 00:22:36~58)은
       *    **전부 178~318ms** 다.
       *    ⇒ 정품은 PROG 를 길게 눌러도 **연속 송신하지 않는다**. 짧은 버스트
       *      하나를 hold 코드(표시 C4/19)로 보내고 끝낸다.
       *
       *  ★2026-08-15 정정: 여기서 "짧은 버스트 + hold 코드" 로 나눴던 건 오독에
       *  근거한 것이라 되돌렸다. 정품의 C4 는 hold 코드가 아니라 **재전송 프레임의
       *  byte7(196+rep*4)** 이 디스크램블되어 그렇게 보인 것뿐이다.
       *  → 누르는 동안 연속 송신하고, byte7 은 프레임마다 증가한다.
       *
       *  ※되돌리기(연속 송신):
       *      _send_command_ex(SOMFY_CMD_PROG, CFG_BTN_MAX_HOLD_MS, true, false);
       *    로 바꾸고 _hold_repeat_task 의 PROG 블록을 지우면 된다. */
      /* ★★★2026-08-31 PROG 횟수 가드(PROGGUARD) **제거** — 사용자 지정.
       *
       *  있던 것: 60초에 N회를 넘으면 고장으로 보고 RF 송신을 막고, 조용해질 때까지
       *  풀지 않는 "두 번째 안전망"(2026-08-17 도입, 5회/5분 → 10회/2분으로 완화).
       *
       *  왜 없앴나 — **막으려던 사고를 이 방식으로는 잡을 수 없다.**
       *   · 원래 사고: LP 코어가 멈춰 PCF 값이 얼어붙어 기기가 하루 종일 PROG 를 쐈다.
       *     그런데 값이 얼어붙으면 '눌린 채 유지'라 **누름 이벤트는 1회**만 난다.
       *     즉 횟수 세기로는 원리상 검출되지 않는다.
       *   · 그 사고의 진짜 방어선은 button_handler.c 의 **LP 신선도 검사**(poll_cnt 고정
       *     감지 → 공유 RAM 값 폐기 + HP 폴링 전환)이고, 그건 그대로 살아 있다.
       *   · 반면 정상 조작은 계속 막혔다. 실측(COM8, 2026-08-31): PROG 43회를 눌렀는데
       *     43회 전부 정상 검출됐고(hold 42~203ms, 사람 손의 자연스러운 편차),
       *     10회만 송신되고 33회가 차단됐다 — 사용자 신고 "가끔만 눌린다"의 정체.
       *
       *  되돌리려면 이 커밋의 직전 버전을 보라(PROG_GUARD_MAX/WINDOW_MS/CALM_MS +
       *  s_pg_* 정적 변수 블록). 다만 되돌리기 전에 위 첫 번째 이유를 먼저 볼 것. */

      oled_ui_notify_action_start(&s_ui, OLED_ACTION_PROG);
      somfy_rts_abort = false;
      /* ★2026-08-15 연속 송신 — 정품과 동일. 누르는 동안 계속 쏘고, 프레임마다
       *  byte7 이 재전송 인덱스로 증가한다(somfy_rts.c _encode80_byte7).
       *  ※짧은 버스트로 끊었더니 "계속 눌러도 짧게 한 번"이 되어 되돌렸다. */
      _send_command_ex(SOMFY_CMD_PROG, CFG_BTN_MAX_HOLD_MS, true, false);
    }
    break;

  case BTN_EVT_PROG_RELEASE:
    s_held_prog = false;
    _ch_lock_release();                         /* ★뗀 시각 기준으로 채널변경 잠금 연장 */
    somfy_rts_abort = true;                     /* 뗌 → PROG 송신 즉시 종료 */
    oled_ui_notify_action_end(&s_ui);
    if (evt->hold_ms > CFG_BTN_MIN_HOLD_MS) {
      ESP_LOGI(TAG, "PROG 버튼 뗌 (총 hold=%" PRIu32 "ms)", evt->hold_ms);
    }
    break;

  /* ── 로터리 CW (틸트 다운) ───────────────────
   *   메인 화면       : 틸트 다운 RF 송신 (큐 push, 논블로킹)
   *   설정 메뉴 화면  : 커서 아래로 이동 (틸트 교환 일관)
   *   주파수 편집     : freq -0.01
   *   기타 화면        : 무시 */
  case BTN_EVT_ROT_CW:
    if (s_setup_screen == SETUP_MENU_SCR) {
      _setup_menu_cursor_move(+1);
    } else if (s_setup_screen == SETUP_FREQ_EDIT) {
      _freq_edit_step(-1);
    } else if (s_setup_screen == SETUP_TIME_EDIT) {
      _time_edit_value_step(-1);   /* CW = 값 - (교환됨) */
    } else if (_in_setup_mode()) {
      ESP_LOGI(TAG, "[SETUP] CW 무시 (screen=%d)", s_setup_screen);
    } else {
      /* 로터리 CW = 틸트 다운. 정품 Tilt 커맨드(cmd nibble 0xB) 송신 — rtl_433
       *  등 디코더가 UP/DOWN 이 아닌 Tilt 로 식별. 코얼레스로 큐 적체 방지. */
      oled_ui_notify_action_start(&s_ui, OLED_ACTION_TILT_DN);
      _send_command_tilt(SOMFY_CMD_TILT_DOWN);
      oled_ui_notify_action_end(&s_ui);
    }
    break;

  /* ── 로터리 CCW (틸트 업) ──────────────────
   *   설정 메뉴 화면  : 커서 위로 이동 (틸트 교환 일관)
   *   주파수 편집     : freq +0.01 */
  case BTN_EVT_ROT_CCW:
    if (s_setup_screen == SETUP_MENU_SCR) {
      _setup_menu_cursor_move(-1);
    } else if (s_setup_screen == SETUP_FREQ_EDIT) {
      _freq_edit_step(+1);
    } else if (s_setup_screen == SETUP_TIME_EDIT) {
      _time_edit_value_step(+1);   /* CCW = 값 + (교환됨) */
    } else if (_in_setup_mode()) {
      ESP_LOGI(TAG, "[SETUP] CCW 무시 (screen=%d)", s_setup_screen);
    } else {
      /* 로터리 CCW = 틸트 업. 정품 Tilt 커맨드(cmd nibble 0xB) 송신 — rtl_433
       *  등 디코더가 UP/DOWN 이 아닌 Tilt 로 식별. 코얼레스로 큐 적체 방지. */
      oled_ui_notify_action_start(&s_ui, OLED_ACTION_TILT_UP);
      _send_command_tilt(SOMFY_CMD_TILT_UP);
      oled_ui_notify_action_end(&s_ui);
    }
    break;

  /* ── 로터리 클릭 (정지) ───────────────────────
   *   메인 화면           : STOP/MY 송신
   *   설정 메뉴 화면      : 선택 항목 진입 (★2026-08-12 SETUP 과 교환)
   *   주파수 편집        : 짧게 → 스냅샷 리셋, 길게 (≥2s) → 메인 복귀
   *   Matter pair / Thread reset: 짧게 → 메뉴, 길게 → 메인 */
  /* ── 로터리 버튼(STOP/MY) 누름 ─────────────────
   *   메인 화면: 정품 리모컨처럼 누르고 있는 동안 STOP 송신.
   *   설정 모드: 무시 (메뉴/취소는 release=ROT_CLICK 가 처리). */
  case BTN_EVT_ROT_PRESS:
    _ch_lock_touch();
    if (_in_setup_mode()) break;
    {
      /* ★버튼 지속 누름 → 연속 송신(위 UP/DOWN 과 동일 구조). */
      bool first = !(s_held_up || s_held_down || s_held_rot);
      s_held_rot = true;
      if      (s_held_up)   oled_ui_notify_action_start(&s_ui, OLED_ACTION_MY_UP);
      else if (s_held_down) oled_ui_notify_action_start(&s_ui, OLED_ACTION_MY_DOWN);
      else                  oled_ui_notify_action_start(&s_ui, OLED_ACTION_STOP);
      somfy_rts_abort = false;                  /* 새 누름 — 이전 중단요청 해제 */
      if (first) {
        s_action_press_us = esp_timer_get_time();
        s_last_sent_cmd = SOMFY_CMD_MY;
        _send_command_press(SOMFY_CMD_MY);    /* ★뗄 때까지 연속(정품 매칭) */
      }
    }
    break;

  case BTN_EVT_ROT_CLICK: {
    bool long_press = (evt->hold_ms >= CFG_BTN_LONG_PRESS_MS);

    s_held_rot = false;                         /* 이 버튼 놓음(combo 조합 갱신) */
    _ch_lock_release();                         /* ★뗀 시각 기준으로 채널변경 잠금 연장 */
    if (!_in_setup_mode()) {
      if (!(s_held_up || s_held_down)) {        /* 전부 뗌 → 반복 정지·burst 종료 */
        s_action_press_us = 0;
        s_last_sent_cmd = 0;
        s_combo_pending = 0;                    /* ★조합 전환 대기 취소(유출 방지) */
        somfy_rts_abort = true;
        oled_ui_notify_action_end(&s_ui);
      }
      break;
    }

    /* ★2026-08-12 설정 메뉴 화면에서 STOP(MY) ↔ SETUP 기능 교환 (사용자 요청).
     *  STOP (어느 길이든) → 선택 항목 진입  (이전: 메인 복귀)
     *  ※하위 화면(편집/페어링/리셋)의 STOP 은 그대로 취소/복귀다 — 교환은
     *    **메뉴 화면에서만** 이다. */
    if (s_setup_screen == SETUP_MENU_SCR) {
      _setup_activate_menu_item();
      break;
    }

    /* ★2026-08-12 하위 화면도 STOP(MY) ↔ SETUP 기능 교환 (사용자 요청).
     *  STOP(MY) 이 **확인/저장/실행** 을 맡는다 — 아래는 전부 예전 SETUP 의 동작이다.
     *  ※길이 문턱이 서로 다르다: SETUP_LONG=1초, ROT_CLICK long_press=2초
     *    (CFG_BTN_LONG_PRESS_MS). 즉 Thread 리셋 실행은 "SETUP 1초"에서
     *    "MY 2초"로 바뀐다 — 화면 문구 "2s=execute" 와는 오히려 맞아떨어진다. */
    switch (s_setup_screen) {
      case SETUP_FREQ_EDIT:            /* 예전 SETUP 짧게/길게 = 저장 */
        _freq_edit_save();
        break;
      case SETUP_TIME_EDIT:            /* 예전 SETUP 짧게/길게 = 저장 */
        _time_edit_save();
        break;
      case SETUP_MATTER_PAIR:
        /* 예전 SETUP 짧게 = "대기(WAITING)" → "준비(READY)" 확정.
         * 예전 SETUP 길게는 무시였으므로 여기서도 길게는 무시한다. */
        if (!long_press) {
          s_pair_ready = true;
          ESP_LOGI(TAG, "[SETUP] 페어링 준비(READY) 확정 (MY)");
        }
        break;
      case SETUP_THREAD_RESET:
        if (long_press) _setup_execute_thread_reset();   /* 예전 SETUP 길게 = 실행 */
        else            _setup_back_to_menu("MY 짧게");  /* 예전 SETUP 짧게 = 메뉴 복귀 */
        break;
      case SETUP_FW_UPDATE:
        /* 예전 SETUP 짧게 = 수동 업데이트 확인. 길게는 무시였다. */
        if (!long_press) {
          if (matter_ota_trigger_check()) ESP_LOGI(TAG, "[FW] 업데이트 확인 트리거 (MY)");
          else                            ESP_LOGW(TAG, "[FW] 업데이트 확인 트리거 실패");
        }
        break;
      default:
        break;
    }
    break;
  }

  /* ── SW6 SETUP 짧은 누름 ──────────────────────
   *   메인 화면              : 설정 메뉴 진입
   *   설정 메뉴 화면         : 메인 복귀 (★2026-08-12 STOP 과 교환)
   *   하위 화면 (편집/페어링/리셋) : 메뉴 복귀 (변경 폐기) */
  case BTN_EVT_SETUP_SHORT:
    switch (s_setup_screen) {
      case SETUP_NONE:
        _setup_enter_menu_from_main();
        break;
      case SETUP_MENU_SCR:
        /* ★2026-08-12 STOP(MY) 와 기능 교환 (사용자 요청) — 이전: 선택 항목 진입 */
        _setup_exit_to_main("SETUP @ menu");
        break;
      /* ★2026-08-12 이하 전부 예전 STOP(MY) **짧게** 의 동작이다 (기능 교환).
       *  SETUP 은 이제 **취소/뒤로** 전담이다. */
      case SETUP_FREQ_EDIT:        /* 예전 STOP = 변경 폐기 후 메뉴 */
        _freq_edit_back_to_menu_discard();
        break;
      case SETUP_TIME_EDIT:        /* 예전 STOP = 적용 안 함 → 메뉴 */
        _setup_back_to_menu("time edit 취소");
        break;
      case SETUP_MATTER_PAIR:
      case SETUP_THREAD_RESET:
      case SETUP_FW_UPDATE:        /* 예전 STOP 짧게 = 메뉴 복귀 */
        _setup_back_to_menu("SETUP 짧게");
        break;
    }
    break;

  /* ── SW6 SETUP 롱프레스(1초) ──────────────────
   *  ★2026-08-12 기능 교환 — 이하 전부 예전 STOP(MY) **길게** 의 동작이다.
   *   편집 화면(freq/time) : 예전 STOP 은 길이 무관 취소였으므로 길게도 취소
   *   그 외 하위 화면      : 전체 취소 → 메인 복귀
   *   메인 화면            : 무시 (메뉴 진입은 SETUP 짧게 그대로) */
  case BTN_EVT_SETUP_LONG:
    switch (s_setup_screen) {
      case SETUP_FREQ_EDIT:
        _freq_edit_back_to_menu_discard();
        break;
      case SETUP_TIME_EDIT:
        _setup_back_to_menu("time edit 취소");
        break;
      case SETUP_MENU_SCR:
      case SETUP_MATTER_PAIR:
      case SETUP_THREAD_RESET:
      case SETUP_FW_UPDATE:
        _setup_exit_to_main("SETUP 길게 — 전체 취소");
        break;
      case SETUP_NONE:
      default:
        ESP_LOGI(TAG, "[SETUP] SETUP_LONG @ screen=%d — 무시", s_setup_screen);
        break;
    }
    break;

  default:
    break;
  }
}

/* ═══════════════════════════════════════════════
   Matter → Somfy RTS 콜백
═══════════════════════════════════════════════ */
static void _matter_action_cb(uint8_t endpoint_idx, somfy_command_t cmd,
                              uint8_t position_pct, uint8_t oled_action,
                              uint8_t step_count, void *user_data) {
  /* SmartThings 명령 수신 → 항상 wake 우선 처리 (endpoint 유효성 검증 전).
   * 미등록 블라인드 슬롯에 대한 명령도 절전/화면보호기 해제 트리거. */
  _mark_activity();
  if (s_screensaver_active) {
    _exit_screensaver("Matter command");
  }
  if (s_is_sleeping) {
    _exit_sleep("Matter command");
  }

  ESP_LOGI(TAG, "SmartThings → 블라인드[%d] cmd=%d pos=%d%% oled=%d (state was %s)",
           endpoint_idx, cmd, position_pct, oled_action,
           s_is_sleeping ? "sleep" : (s_screensaver_active ? "saver" : "active"));

  /* ── 다중 블라인드 동시 조작 표시 ──────────────────────────────
   *  SmartThings 는 엔드포인트별로 개별 명령을 보낸다(그룹/씬이면 짧은
   *  시간 내 여러 개). 3초 윈도우로 누적해 마스크 구성:
   *   - 단일      → 그 블라인드 1개
   *   - 여러 개   → 명령 전달된 번호 모두
   *   - 전체  → 모든 채널 비트 (OLED 가 "ALL" 표시) */
  if (endpoint_idx < BLIND_MAX_COUNT) {
    static uint8_t  s_st_mask = 0;
    static int64_t  s_st_t    = 0;
    int64_t nowu = esp_timer_get_time();
    if (nowu - s_st_t > 3000000) s_st_mask = 0;   /* 3s 윈도우 리셋 */
    s_st_mask |= (uint8_t)(1u << endpoint_idx);
    s_st_t = nowu;
    oled_ui_set_action_blinds(&s_ui, s_st_mask);
  }

  /* OLED 모션: delegate 가 산출한 oled_action 사용 (lift/tilt + 방향).
   *  슬라이더 중간값도 방향대로 up/down, 틸트는 tilt-up/down. */
  oled_action_t ui_action = (oled_action_t)oled_action;
  if (ui_action == OLED_ACTION_NONE) {
    /* 폴백: cmd 기반 (PROG 등) */
    switch (cmd) {
    case SOMFY_CMD_UP:   ui_action = OLED_ACTION_UP;   break;
    case SOMFY_CMD_DOWN: ui_action = OLED_ACTION_DOWN; break;
    case SOMFY_CMD_PROG: ui_action = OLED_ACTION_PROG; break;
    default:             ui_action = OLED_ACTION_STOP; break;
    }
  }

  /* 항상 모션 시작 — 설정 모드/페어링 화면 중이 아니라면. */
  if (!_in_setup_mode()) {
    oled_ui_notify_action_start(&s_ui, ui_action);
    s_ui.anim_frame++;  /* OLED task 가 다음 tick 에서 즉시 redraw 하도록 */
  }

  /* RF 송신 — endpoint 유효 시만 (논블로킹).
   *  step_count: Tilt 슬라이더 다단 step (1=일반, N=Tilt drag N×10%). */
  if (endpoint_idx < s_mgr.count) {
    _send_command_endpoint(endpoint_idx, cmd, step_count);
  } else {
    ESP_LOGW(TAG, "  → endpoint %d 미등록 블라인드 (등록=%d개) — RF 송신 skip",
             endpoint_idx, s_mgr.count);
  }

  if (!_in_setup_mode()) {
    oled_ui_notify_action_end(&s_ui);  /* 2.5s 타임아웃 시작 */
  }
}

/* ═══════════════════════════════════════════════
   Thread 부착 후 처리 — SNTP 시작 등
   (Thread role 변경 이벤트는 matter_blinds.cpp 의 app_event_cb 에서 감지)
═══════════════════════════════════════════════ */
/* SNTP 시간 동기화 이벤트 콜백 — sync 직후 NVS 즉시 저장 */
static void _sntp_sync_cb(struct timeval *tv) {
  ESP_LOGI(TAG, "[TIME] SNTP 동기화 → epoch=%lld", (long long)tv->tv_sec);
  _time_persist_save();
}

/* ═══════════════════════════════════════════════
   PM 설정 (idempotent) — DFS 는 항상, light sleep 은 **등록 완료 후에만**
   ──────────────────────────────────────────────
   ★ v3.6: Thread 가 RX_ON_WHEN_IDLE (항상 수신) 로 동작하므로 light sleep
     으로 라디오를 끄면 Matter 명령 수신이 끊긴다. 따라서 light_sleep_enable
     = false 로 두고 CPU 주파수 스케일링(DFS)만 사용 — 라디오는 항상 ON
     유지하면서 idle 시 CPU clock 만 낮춰 소폭 절전.
   ★ 커미셔닝 완료 후 호출 (그 전엔 주파수 변동도 보수적으로 피함).
   peripheral init 가 모두 끝난 뒤여야 tickless idle 데드락이 없다.
═══════════════════════════════════════════════ */

/* time_update 태스크 핸들 — 화면이 켜질 때 즉시 깨우기 위해 보관(위 5분 대기 참조). */
/* ★2026-08-20 `time_update` 태스크 제거 — 메인 루프의 _time_tick_sched() 로
 *  흡수했다. 통지로 즉시 깨우던 목적(화면이 켜지면 낡은 시각을 안 보이게)은
 *  메인 루프가 어차피 100ms 마다 도는 것으로 대체된다 — 최대 100ms 지연.
 *  호출부(_exit_sleep/_exit_screensaver)는 그대로 두고 여기서 무해화한다. */
static inline void _wake_time_task(void) {
}

static void _enable_pm_light_sleep(void) {
#if CONFIG_PM_ENABLE
  /* ★★2026-08-11 재작성 — light sleep 을 **커미셔닝 완료 후에만** 켠다.
   *
   *  왜 나눠야 하나(과거 실패 이력): Thread SED + auto light sleep 을 커미셔닝 도중에
   *  켜면 Thread operational 단계(SRP 등록 + CASE 핸드셰이크)가 굶어 SmartThings
   *  페어링이 **마지막에 실패**한다. 그래서 DFS(주파수 스케일링)는 항상 켜되
   *  light sleep 만 등록 후로 미룬다.
   *
   *  또 FTD(rx-on) 구성에서는 라디오를 못 재우므로 light sleep 이 의미가 없다 →
   *  MTD + ICD(Sleepy End Device) 일 때만 켠다.
   *
   *  ※이 함수는 메인 루프가 주기적으로 부른다. 상태가 바뀔 때만 재설정하고
   *    로그는 **실제 설정값**을 찍는다(예전엔 문자열이 하드코딩돼 있어 light sleep 이
   *    켜졌는지 로그로 알 수 없었다). */
  const bool paired = matter_blinds_is_commissioning_complete();
#if CONFIG_OPENTHREAD_MTD && CONFIG_ENABLE_ICD_SERVER
  /* ★★2026-08-12 조건 완화 — 실측에서 절전 효과가 0 이었던 원인.
   *  등록 완료만 조건으로 두니 **미등록 기기는 CPU 가 영영 안 자서** 무선을 꺼도
   *  소모가 그대로였다(4.5시간에 100%→43%, 평균 89mA — 개선 전 84mA 와 동일).
   *  커미셔닝 게이트를 넣은 이유는 "Thread operational 단계를 굶기면 페어링이
   *  실패한다"였는데, **무선이 꺼져 있으면 굶길 대상 자체가 없다**.
   *  → 등록 완료 **또는** 무선 OFF 이면 light sleep 을 켠다. */
  const bool want_ls = paired || !matter_blinds_get_radio_enabled();
#else
  const bool want_ls = false;   /* rx-on Thread(FTD): 라디오 상시 ON 이라 무의미 */
#endif
  /* 현재 적용 상태. 모니터 로그가 읽어 실제 값을 보여준다(하드코딩 금지). */
  /* ★★2026-08-11 USB 연결 중에는 DFS 를 끈다(min=max=160).
   *
   *  왜: DFS 가 80MHz 로 내리면 **USB-Serial-JTAG 가 죽는다**. 우리는 USB-JTAG 를
   *  보조 ROM 콘솔로만 쓰고 드라이버를 설치하지 않아 PM 락을 잡지 않기 때문이다.
   *  실제로 DFS 를 켠 직후 COM 포트가 목록에서 사라져 플래시조차 못 했다.
   *  절전은 어차피 배터리 구동에서만 의미가 있으므로, USB 일 땐 전속으로 돌려
   *  디버깅·플래시를 지키고 배터리일 때만 DFS·light sleep 을 건다. */
  /* ★PM 만은 **물리 VBUS** 를 본다(시뮬 무시). 시뮬 중에 DFS 가 켜지면 USB-JTAG 가
   *  죽어 관찰 자체가 불가능해지기 때문이다(실제로 겪은 문제). 시뮬은 "배터리 모드
   *  로직"을 보려는 것이지 전력 자체를 재현하려는 게 아니다. */
  /* ★★★2026-08-16 BOARD_ALWAYS_USB_POWERED 를 여기서도 본다.
   *  PM 은 일부러 _is_usb_powered() 를 우회해 물리 CHG_STAT 을 직접 읽는다
   *  (usbsim 에 속지 않으려고). 그런데 **충전 감지 회로가 없는 보드**는 그 핀이
   *  플로팅이라 항상 '배터리'로 읽혀, USB 에 꽂힌 채 DFS+light sleep 이 돌았다.
   *  실측(테스트 보드, 2026-08-16): `[PWR] light_sleep=ON (PM상태 3) VBUS=0` 상태로
   *  45분/3541페이지를 정상 전송하다 I2C 페리페럴이 고착
   *    (라인 idle 1/1 인데 검출실패 → 9클럭 복구 → i2c_del_master_bus 가
   *     ESP_ERR_INVALID_STATE). 같은 공유 I2C 인 H2 는 VBUS=1 이라 PM상태 0
   *    (light sleep OFF)이고 멀쩡했다 — 공유 방식이 아니라 **절전이 차이**였다.
   *  → 배터리가 아예 없는 보드는 전원이 USB 뿐이므로 참으로 고정한다. */
  const bool on_usb = BOARD_ALWAYS_USB_POWERED ? true : btn_handler_is_charging();
  /* ★★★2026-08-16 BOARD_DISABLE_LIGHT_SLEEP — **패닉 임시 차단**.
   *  H2 에서 절전 빌드(①MTD ②tick ③PM)를 넣은 뒤 배터리 구동 중 패닉이 반복됐다.
   *  부팅진단 근거: 리셋사유가 PM 적용 전 `USB리셋` → 적용 후 `★패닉/예외` 로 바뀌고
   *  실패 누적이 29 → 39 (13번 부팅에 10번). 방전 로그상 크래시 직전은 `pm=3`
   *  (light sleep 적용) 상태의 버튼 조작 구간이었다.
   *  H2 는 OLED 와 PCF 가 I2C 를 공유하므로 light sleep 의 주변장치 복귀와 그 경로가
   *  충돌하는 것이 1순위 의심이지만, **아직 backtrace 를 못 봤다**(배터리 구동 중엔
   *  콘솔이 없다). 원인 확정 전까지 light sleep 만 끈다 —
   *  ①(FTD→MTD/SED, 가장 큰 절감)과 ②③의 DFS 는 그대로 유지된다. */
  const bool ls      = want_ls && !on_usb && !BOARD_DISABLE_LIGHT_SLEEP;

  /* ★★★2026-08-16 max 160MHz 하드코딩 제거 — **H2 에서 PM 이 통째로 실패했다**.
   *
   *  ESP32-H2 의 CPU 최대는 96MHz 다(rtc_clk_cpu_freq_mhz_to_config: 96=PLL,
   *  그 외는 XTAL 32MHz 의 정수 분주만 허용). 그래서 max=160 을 넘기면
   *  esp_pm_configure 가 ESP_ERR_INVALID_ARG 를 돌려주고 **DFS 도 light sleep 도
   *  전혀 걸리지 않았다** — 실측 `PM 설정: 80MHz~160MHz ... → ESP_ERR_INVALID_ARG`
   *  이고 방전 로그의 sleep 체류가 0.0% 였던 이유다.
   *  → 보드 설정값을 그대로 쓴다(C6=160 / H2=96).
   *
   *  min 도 보드마다 유효값이 다르므로 후보를 순서대로 시도해 첫 성공을 채택한다:
   *    ① 절반(DFS 효과 최대)  ② XTAL  ③ DFS 없음(min=max — light sleep 만이라도)
   *  C6 는 ①이 80MHz 라 지금까지와 **완전히 동일**하다(측정 연속성 유지). */
  const int max_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
  const int cand[3] = {
      on_usb ? max_mhz : max_mhz / 2,
      on_usb ? max_mhz : CONFIG_XTAL_FREQ,
      max_mhz,
  };

  static int s_pm_state = -1;   /* 적용된 상태 코드(아래 want 와 동일 규칙) */
  const int want = (ls ? 2 : 0) + (on_usb ? 0 : 1);
  if (s_pm_state == want) return;

  esp_err_t pm_err = ESP_ERR_INVALID_ARG;
  int min_mhz = max_mhz;
  for (int i = 0; i < 3 && pm_err != ESP_OK; i++) {
    if (cand[i] <= 0 || cand[i] > max_mhz) continue;
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz       = max_mhz,
        .min_freq_mhz       = cand[i],
        .light_sleep_enable = ls,
    };
    pm_err  = esp_pm_configure(&pm_cfg);
    min_mhz = cand[i];
  }
  if (pm_err == ESP_OK) { s_pm_state = want; g_pm_state_applied = want; }
  ESP_LOGW(TAG, "PM 설정: %dMHz~%dMHz, light_sleep=%s (전원=%s, 등록=%d) → %s",
           min_mhz, max_mhz, ls ? "ON" : "OFF", on_usb ? "USB" : "배터리",
           paired ? 1 : 0, esp_err_to_name(pm_err));
#else
  static bool s_pm_warned = false;
  if (!s_pm_warned) {
    s_pm_warned = true;
    ESP_LOGW(TAG, "CONFIG_PM_ENABLE not set — DFS 비활성 (1회 안내)");
  }
#endif
}

/* SNTP 1회 시작 (idempotent). ★ v3.6: 반드시 커미셔닝 완료 후 호출.
 *  커미셔닝 중(Thread attach 시점)에 SNTP(UDP 소켓/lwIP)를 띄우면 CHIP
 *  operational mDNS/SRP 와 갓 올라온 Thread netif IPv6 자원이 경합 →
 *  "Failed to advertise records: CHIP_ERROR_INVALID_ADDRESS(46)" → 페어링
 *  영구 실패(39-517). 따라서 커미셔닝이 끝난 뒤에만 SNTP 를 시작한다. */
static void _sntp_start_once(void) {
  static bool s_sntp_started = false;
  if (s_sntp_started || esp_sntp_enabled()) {
    esp_sntp_restart();
    return;
  }
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, CFG_SNTP_SERVER);
  esp_sntp_set_sync_interval(60 * 60 * 1000);          /* 1시간 (ms) */
  esp_sntp_set_time_sync_notification_cb(_sntp_sync_cb);
  esp_sntp_init();
  setenv("TZ", CFG_TIMEZONE, 1);
  tzset();
  s_sntp_started = true;
  ESP_LOGI(TAG, "[TIME] SNTP 시작 (server=%s, 1시간 주기)", CFG_SNTP_SERVER);
}

/* ═══════════════════════════════════════════════
   커미셔닝 보호 (Settings>Matter Pair 윈도우 한정)
   ──────────────────────────────────────────────
   문서화된 39-517 근본 원인 재적용(수동 페어링 경로):
    1) SNTP 일시중지 — operational mDNS/SRP 와 UDP/lwIP 경합
       (CHIP_ERROR_INVALID_ADDRESS(46)) 방지
    2) CPU 최대 주파수 고정 — CASE/SRP 핸드셰이크 중 DFS 변동 차단
   페어링 화면 이탈(성공/취소 무관) 시 원상 복구.
═══════════════════════════════════════════════ */
#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t s_pair_cpu_lock = NULL;
#endif
static bool s_pair_protect_on  = false;
static bool s_pair_sntp_was_on = false;

static void _pairing_protect_begin(void) {
  if (s_pair_protect_on) return;
  s_pair_protect_on = true;
  s_pair_sntp_was_on = esp_sntp_enabled();
  if (s_pair_sntp_was_on) {
    esp_sntp_stop();
    ESP_LOGI(TAG, "[PAIR] SNTP 일시중지 (커미셔닝 mDNS/SRP 보호)");
  }
#if CONFIG_PM_ENABLE
  if (!s_pair_cpu_lock) {
    esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "pair", &s_pair_cpu_lock);
  }
  if (s_pair_cpu_lock) esp_pm_lock_acquire(s_pair_cpu_lock);
#endif
  ESP_LOGI(TAG, "[PAIR] 커미셔닝 보호 ON (SNTP off + CPU max)");
}

static void _pairing_protect_end(void) {
  if (!s_pair_protect_on) return;
  s_pair_protect_on = false;
#if CONFIG_PM_ENABLE
  if (s_pair_cpu_lock) esp_pm_lock_release(s_pair_cpu_lock);
#endif
  if (s_pair_sntp_was_on) {
    esp_sntp_restart();          /* 평상시 시계 동작 복귀 */
    ESP_LOGI(TAG, "[PAIR] SNTP 재개");
  }
  ESP_LOGI(TAG, "[PAIR] 커미셔닝 보호 OFF (CPU 정상)");
}

/* RF 스캔 기능 제거됨 (2026-05-25) — Manchester 극성 정정 후 기본 freq 447.72 가
 *  보드 검증됨. 다른 보드/주파수 지원 필요 시 향후 재도입 가능. */

static void _on_thread_attached(void) {
  ESP_LOGI(TAG, "✅ Thread 네트워크 부착 완료 — Matter 트랜스포트 활성화");
  oled_ui_set_thread(&s_ui, true);
  /* Thread 부착이 사용자 화면(ACTION/메뉴 등)을 지우면 안 된다 —
   *  state 강제 변경하지 않는다. */
  /* ★ 커미셔닝 완료 전에는 SNTP 시작 금지 (operational mDNS/SRP 보호).
   *   미완료면 _deferred_task_starter 가 완료 후 _sntp_start_once() 호출. */
  if (matter_blinds_is_commissioning_complete()) {
    _sntp_start_once();
  } else {
    ESP_LOGI(TAG, "[TIME] SNTP 지연 — 커미셔닝 완료 후 시작 (operational 보호)");
  }
}

/* ─── (제거됨) 버튼 누름유지 반복 태스크 ───────────────
 *  '누르는 동안 송신, 떼면 정지'는 이제 somfy_rts_send 가 직접 처리한다
 *  (PRESS 에서 abortable 송신 시작, RELEASE 에서 somfy_rts_abort). 별도
 *  반복 태스크/누름시간 임계값 불필요 — 정품 리모컨과 동일 동작. */

/* ═══════════════════════════════════════════════
   시간 업데이트 태스크 — 1초 폴링, 분 단위 변화 즉시 감지/로깅
   _render_normal()이 매 50ms 마다 time(NULL)을 직접 읽으므로
   화면 갱신은 본 태스크와 독립적이지만, ctx->time_str 사용자(screensaver 외)와
   진단 로그를 위해 1초 주기로 폴링.
═══════════════════════════════════════════════ */
/* ═══════════════════════════════════════════════
   애플리케이션 태스크 스타터
   ──────────────────────────────────────────────
   ★ 요구사항: 이 기기는 SmartThings 없이 단독(standalone) 으로 사용
     가능해야 하며 Matter 페어링은 옵션이다. 따라서 앱 태스크(OLED/버튼/
     RF/시계)는 페어링 여부와 무관하게 동작한다.
   ★ 단, *실제 커미셔닝 핸드셰이크가 진행 중* 일 때만 고빈도 bit-bang
     태스크(OLED 20fps/버튼 폴링)가 802.15.4 operational/SRP/CASE 타이밍을
     굶기지 않도록 잠시 지연한다(_deferred_task_starter). 활성 페어링이
     없으면 부팅 후 짧은 유예 뒤 단독 동작을 시작한다.
═══════════════════════════════════════════════ */
/* _time_tick_sched / _time_persist_sched 는 아래에서 정의 — 전방 선언 */
static void _time_tick_sched(int64_t now_us);
static void _time_persist_sched(int64_t now_us);

/* 앱 태스크가 시작되었는지(=단독/페어링 무관 본격 동작 중). 메인 루프
 * 게이트가 이 플래그를 사용한다. */
static volatile bool s_app_started = false;

/* 모든 애플리케이션 태스크 시작. idempotent — 1회만 생성. */
static void _start_app_tasks_once(void) {
  if (s_app_started) return;
  s_app_started = true;

  /* PCF/OLED 공유 HW I2C: SCL 은 양쪽 400kHz 로 일치하지만, 보드 풀업이 약하면
   *  (ssd1306 라이브러리가 내부 풀업만 켬) 고빈도 OLED flush 직후 PCF read 가
   *  간헐적 nack(=INVALID_STATE)을 본다. 우리 재시도로 매번 복구되어 기능 영향은
   *  없으나, 드라이버의 ERROR 3줄 스팸이 115200 UART 를 포화시켜 Thread/BLE
   *  커미셔닝 타이밍을 방해할 수 있다. 드라이버 로그만 끄고(BTN 로그로 상태는
   *  계속 추적) 페어링 타이밍을 확보한다. */
  esp_log_level_set("i2c.master", ESP_LOG_NONE);

  if (s_rf_queue == NULL) {
    s_rf_queue = xQueueCreate(RF_QUEUE_DEPTH, sizeof(rf_job_t));
  }
  /* ui(콜론 anim)·btn(폴링) 을 앞에 두고 시작. (composed 로 free 확보돼 스택 축소 불필요.) */
  oled_ui_start_task(&s_ui);                                            /* _ui_task 4096 (콜론 anim) */
  btn_handler_start_task();                                             /* 3072 (버튼 폴링) */
  xTaskCreate(_rf_worker_task,    "rf_worker",   3072, NULL, 9, NULL);  /* composed 로 free 확보 → 원래 3072 복원 */
  xTaskCreate(_hold_repeat_task,  "hold_repeat", 2048, NULL, 8, NULL);  /* 버튼 지속 누름 → 신호 연속 발생 */
  /* 시계 폴링 + 시간 NVS 영속. BOARD_DISABLE_TIME(H2)=1 이면 BLE 커미셔닝 heap
   *  확보를 위해 time 태스크(스택 ~5KB)·SNTP 를 만들지 않는다. */
#if !BOARD_DISABLE_TIME
  /* ★2026-08-20 time_persist(3072B)·time_update(2048B) 생성을 없앴다 —
   *  둘 다 메인 루프 틱으로 흡수(_time_tick 위 주석). RAM 5,120B 회수. */
  _sntp_start_once();
#endif

  /* 단독(미페어링) 시작: 부팅 시 그린 페어링 화면(OLED_STATE_PAIRING)에서
   *  빠져나와 일반 화면으로 전환한다. 그렇지 않으면 화면이 페어링에 묶여
   *  버튼/SELECT 동작이 화면에 안 보여 "안 되는 것처럼" 보인다.
   *  (커미셔닝된 부팅 경로는 이미 NORMAL 로 설정했으므로 영향 없음.) */
  if (!matter_blinds_is_commissioning_complete()) {
    s_ui.state = OLED_STATE_NORMAL;
    oled_ui_set_matter_status(&s_ui, OLED_MT_UNPAIRED, OLED_RSSI_INVALID);
    s_ui.anim_frame++;
  }
  ESP_LOGI(TAG, "[APP] 애플리케이션 태스크 시작 (OLED/버튼/RF/시계/SNTP)");
  /* [HEAP] 부팅 직후 heap 현황 — 블라인드 개수/태스크 메모리 점검(1회). */
  ESP_LOGW(TAG, "[HEAP] free=%uB  min_ever=%uB  largest_block=%uB  (블라인드 %d개, ep %d)",
           (unsigned)esp_get_free_heap_size(),
           (unsigned)esp_get_minimum_free_heap_size(),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT),
           BLIND_MAX_COUNT, BLIND_MAX_COUNT + 1);
  /* ★★★2026-08-17 CHIP PacketBuffer 풀이 **정적으로 얼마를 먹는지** 같이 찍는다.
   *  이 풀은 .bss 라 heap 에는 안 잡히지만, DRAM 총량을 나눠 쓰므로 늘리면
   *  그만큼 heap(특히 **연속 블록**)이 줄어든다. 실측에서 NOC 검증이
   *  `VerifyCredentials: b`(NO_MEMORY)로 실패할 때 free 4.9KB 인데
   *  largest_block 은 3.4KB 뿐이었다 — 총량이 아니라 조각 크기가 벽이다.
   *  두 값을 나란히 봐야 "풀을 키울지 줄일지" 를 근거로 정할 수 있다.
   *  ※per-buffer 크기는 CHIP SystemConfig.h 의 PACKETBUFFER_CAPACITY_MAX(1583)
   *    기준이다. CHIP 헤더를 C 파일에서 직접 include 하지 않으므로 여기 상수로
   *    둔다 — CHIP 쪽 값이 바뀌면 이 숫자도 같이 고칠 것. */
#ifndef PBUF_ENTRY_BYTES
#define PBUF_ENTRY_BYTES 1583
#endif
  ESP_LOGW(TAG, "[HEAP] CHIP PacketBuffer 풀: %d개 × %dB = %dB (.bss, heap 아님)",
           CHIP_SYSTEM_CONFIG_PACKETBUFFER_POOL_SIZE, PBUF_ENTRY_BYTES,
           CHIP_SYSTEM_CONFIG_PACKETBUFFER_POOL_SIZE * PBUF_ENTRY_BYTES);
}

/* ★2026-08-17 페어링 중 heap 추적 — 실패 순간의 **연속 블록**을 잡기 위함.
 *  60초 주기 [OLEDMON] 으로는 5초 만에 끝나는 NOC 검증 실패를 놓친다.
 *  페어링 화면일 때만 2초 간격으로 찍는다(평상시 부담 없음). */
static void _heap_watch_tick(int64_t now_us, bool pairing_screen)
{
  static int64_t last_us = 0;
  static uint32_t min_largest = 0xFFFFFFFFu;
  if (!pairing_screen) { last_us = 0; min_largest = 0xFFFFFFFFu; return; }
  if (last_us && (now_us - last_us) < 2LL * 1000000LL) return;
  last_us = now_us;
  const uint32_t lb = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
  if (lb < min_largest) min_largest = lb;
  ESP_LOGW(TAG, "[HEAPW] free=%uB  largest=%uB  (페어링 중 최저 largest=%uB)",
           (unsigned)esp_get_free_heap_size(), (unsigned)lb, (unsigned)min_largest);
}

/* 커미셔닝이 완료되면 즉시, 아니면 '활성 페어링이 없는 상태가 유예시간
 * 동안 지속' 되면 단독 모드로 앱을 시작한다.
 *  - 커미셔너가 실제 핸드셰이크 중(fail-safe armed)이면 그 동안은 지연
 *    하고 유예 타이머를 리셋(39-517 타이밍 보호).
 *  - 아무도 페어링하지 않으면 부팅 후 APP_START_GRACE_MS 뒤 단독 시작
 *    → SmartThings 없이도 버튼/OLED/RF 정상 동작. */
#define APP_BOOT_PRE_DELAY_MS 5000    /* 부팅 후 백그라운드 페어링 감시 시작 지연 */
#define APP_PAIR_WATCH_MS     12000   /* 페어링 감시 최대 시간(아무것도 없으면 종료) */
#define APP_BOOT_TOTAL_MS     (APP_BOOT_PRE_DELAY_MS + APP_PAIR_WATCH_MS)

/* ★2026-08-12 부팅 진행 바 (사용자 요청) ────────────────────────────────────
 *  이 대기 구간(약 17초) 동안 예전에는 **메인 화면을 1회 그려놓고 방치**했다.
 *  화면은 멀쩡한데 OLED 갱신·버튼 폴링 태스크가 아직 없어 아무것도 안 먹으니
 *  "부팅 후 메인화면이 나타나고 한참 멈춰 있다" 로 보였다(실측 16.5초).
 *  → 끝날 때까지 로고+진행 바를 계속 그려 **부팅 중임을 분명히** 한다.
 *  ※진행률은 단조 증가만 시킨다: 활성 페어링이 감지되면 watch 가 12초로
 *    되감기는데(아래 루프), 그대로 쓰면 바가 뒤로 가서 고장처럼 보인다.
 *  ※95% 에서 멈춘다 — 실제 완료는 _start_app_tasks_once() 시점이다. 페어링이
 *    끼어들어 감시가 연장되면 여기서 대기하는 게 정직하다. */
static uint8_t s_boot_pct = 0;

static void _boot_progress(int elapsed_ms) {
  int p = (elapsed_ms * 95) / APP_BOOT_TOTAL_MS;
  if (p < 0)  p = 0;
  if (p > 95) p = 95;
  if ((uint8_t)p < s_boot_pct) p = s_boot_pct;   /* 뒤로 가지 않게 */
  s_boot_pct = (uint8_t)p;
  oled_ui_show_booting(&s_ui, s_boot_pct);
}

static void _deferred_task_starter(void *pv) {
  (void)pv;

  /* 이미 커미셔닝된 기기 — 즉시 앱 시작(메인 경로에서 처리되지만 방어). */
  if (matter_blinds_is_commissioning_complete()) {
    ESP_LOGI(TAG, "[DEFER] 이미 커미셔닝됨 — 즉시 앱 시작");
    _start_app_tasks_once();
    vTaskDelete(NULL);
    return;
  }

  /* 1) 부팅 후 5초 대기. 무거운 연속 태스크는 아직 띄우지 않는다
   *    (802.15.4/SRP 타이밍 보호). 화면은 부팅 진행 바를 계속 갱신한다. */
  int pre = APP_BOOT_PRE_DELAY_MS;
  while (pre > 0 && !matter_blinds_is_commissioning_complete()) {
    _boot_progress(APP_BOOT_PRE_DELAY_MS - pre);
    vTaskDelay(pdMS_TO_TICKS(500));
    pre -= 500;
  }

  /* 2) 백그라운드 페어링 감시(최대 12초). '페어링됨' 가정 — 완료가
   *    확인되면 즉시 종료하고 상단 안테나/Matter 상태를 갱신한다.
   *    활성 페어링 트랜잭션 진행 중이면 감시를 연장(끊지 않음).
   *    아무것도 없으면 12초 뒤 감시를 끝내고 단독 모드로 진행. */
  if (!matter_blinds_is_commissioning_complete())
    ESP_LOGI(TAG, "[DEFER] 백그라운드 페어링 감시 시작 (최대 %dms)",
             APP_PAIR_WATCH_MS);
  int watch = APP_PAIR_WATCH_MS;
  while (!matter_blinds_is_commissioning_complete()) {
    _boot_progress(APP_BOOT_PRE_DELAY_MS + (APP_PAIR_WATCH_MS - watch));
    if (matter_blinds_is_pairing_in_progress()) {
      watch = APP_PAIR_WATCH_MS;            /* 활성 페어링 — 감시 유지 */
    } else {
      watch -= 500;
      if (watch <= 0) {
        ESP_LOGI(TAG, "[DEFER] 페어링 없음(12s) — 백그라운드 감시 종료, "
                       "단독(standalone) 모드");
        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  /* 페어링됨 확인 → 즉시 상단 안테나/Matter 상태 갱신(CONNECTED). */
  if (matter_blinds_is_commissioning_complete()) {
    ESP_LOGI(TAG, "[DEFER] 페어링됨 확인 — 감시 종료, 안테나 정보 갱신");
    oled_ui_set_matter_status(&s_ui, OLED_MT_CONNECTED,
                              thread_prov_get_parent_rssi());
  }
  /* 바를 100% 로 채워 "부팅 완료" 를 한 프레임 보여준 뒤 메인으로 넘어간다.
   * (95% 에서 멈춰 있다가 화면이 바뀌면 중간에 끊긴 것처럼 보인다) */
  s_boot_pct = 100;
  oled_ui_show_booting(&s_ui, 100);
  vTaskDelay(pdMS_TO_TICKS(150));
  _start_app_tasks_once();
  vTaskDelete(NULL);
}

/* ★★2026-08-20 태스크 통합 — `time_update`(2048B)·`time_persist`(3072B) 를
 *  메인 루프로 흡수한다(사용자 요청: "task 가 너무 많지 않어?").
 *
 *  왜 이 둘인가: 메인 루프가 이미 100ms 로 돌고 있어 **추가 깨어남이 0** 이다.
 *    · time_update  — 화면 ON 1초 / OFF 300초. 100ms 루프 안의 경과 검사로 동일.
 *      통지(xTaskNotifyGive)로 즉시 깨우던 것도 불필요해진다 — 메인 루프가
 *      어차피 100ms 마다 도니 화면이 켜지면 최대 100ms 안에 시각이 갱신된다.
 *    · time_persist — 5분마다 NVS 저장뿐. 경과 검사 한 줄이면 된다.
 *  회수: **RAM 5,120B**(H2 는 free 13KB 라 재페어링 여유에 직접 도움), 깨어남 ~1/초.
 *
 *  ★합치지 **않은** 것들과 이유(조사 결과):
 *    · hold_repeat → rf_worker : **불가**. hold_repeat 은 조합이 바뀌면
 *      `somfy_rts_abort = true` 로 **진행 중인 송신을 프레임 경계에서 끊는다**.
 *      _do_rf_send 는 somfy_rts.c:332 의 vTaskDelay 로 블로킹하므로, 지금은
 *      prio 8 인 hold_repeat 이 그 틈에 끼어들어 abort 를 세울 수 있다.
 *      rf_worker(prio 9) 안으로 합치면 송신이 끝나야 검사하므로 **조합 전환이
 *      송신 종료(최대 CFG_BTN_MAX_HOLD_MS)까지 먹통**이 된다.
 *    · oled_ui → 어디든  : **불가**. prio 3 격리가 버그 수정으로 확정된 값이고
 *      (somfy_app(4) 이 선점할 수 있어야 한다), 주기도 50ms/500ms 로 안 맞는다. */
static void _time_tick(void) {
  static int last_min = -1;
  static time_t prev = 0;
  {
    time_t now = 0;
    struct tm ti = {0};
    time(&now);

    /* ★ 외부 권위 시각 동기 감지: 본 루프는 1초 주기인데 시각이 크게
     *  앞으로 점프하면 SmartThings Matter SetUTCTime 또는 SNTP 가 시계를
     *  보정한 것. 즉시 NVS 에 저장해 다음 부팅에도 정확히 유지(드리프트
     *  방지) + 로그로 동기 발생을 가시화. */
    if (prev > 0 && (now - prev) > 120) {
      ESP_LOGW(TAG,
               "[TIME] 외부 시각 동기 감지: +%llds 점프 (epoch=%lld) — NVS 즉시 저장",
               (long long)(now - prev), (long long)now);
      _time_persist_save();
    }
    prev = now;

    localtime_r(&now, &ti);

    char ts[6];
    strftime(ts, sizeof(ts), "%H:%M", &ti);
    oled_ui_set_time(&s_ui, ts);

    /* 분 단위 변화 감지 → 로그 + UI 강제 invalidate (재 render 트리거) */
    int cur_min = ti.tm_hour * 60 + ti.tm_min;
    if (cur_min != last_min) {
      ESP_LOGI(TAG, "[CLOCK] %04d-%02d-%02d %02d:%02d:%02d (epoch=%lld)",
               ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
               ti.tm_hour, ti.tm_min, ti.tm_sec, (long long)now);
      last_min = cur_min;
      /* UI 강제 invalidate — anim_frame 증가시켜 즉시 재 render 트리거 */
      s_ui.anim_frame++;
    }

    /* ★2026-08-12 화면 OFF 시 5분 (사용자 지정) — 시계가 안 보이는데 1초마다
     *  깨울 이유가 없다. 단순히 길게 자면 화면이 켜졌을 때 최대 5분 낡은 시각이
     *  보이므로, **알림(notify)으로 즉시 깨어나게** 한다(_exit_sleep/_exit_screensaver
     *  가 화면을 켜면서 통지). 그래서 5분을 자도 표시는 늦지 않는다. */
  }
}

/* 메인 루프가 100ms 마다 부른다. 화면 ON 1초 / OFF 300초 주기를 여기서 만든다
 *  (원래 태스크의 ulTaskNotifyTake 타임아웃과 같은 값). */
static void _time_tick_sched(int64_t now_us) {
  static int64_t last_us = 0;
  const int64_t iv = (oled_ui_is_panel_on() ? 1000 : 300000) * 1000LL;
  if (last_us != 0 && (now_us - last_us) < iv) return;
  last_us = now_us;
  _time_tick();
}

/* 5분마다 NVS 저장 — 원래 `time_persist` 태스크가 하던 일. */
static void _time_persist_sched(int64_t now_us) {
  static int64_t last_us = 0;
  if (last_us == 0) { last_us = now_us; return; }   /* 부팅 직후엔 저장 안 함 */
  if ((now_us - last_us) < (int64_t)TIME_SAVE_INTERVAL_MS * 1000) return;
  last_us = now_us;
  esp_err_t err = _time_persist_save();
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "[TIME] NVS 저장: epoch=%lld", (long long)time(NULL));
  } else if (err != ESP_FAIL) {
    ESP_LOGW(TAG, "[TIME] NVS 저장 실패: %s", esp_err_to_name(err));
  }
}

/* ═══════════════════════════════════════════════
   배터리 충전량 추정 (단순 heuristic)
   ──────────────────────────────────────────────
   하드웨어 ADC 분압 회로가 없으므로 정확한 측정 불가.
   대신 충전 시작 후 경과 시간 기반으로 5%→100% 선형 추정.

   완충시간을 보드별 (용량 ÷ 충전전류) 로 파생한다(고정값 아님):
     CHG_FULL_DURATION_MS = (BOARD_BATT_MAH / BOARD_CHG_MA) h × 5/4(CV 오버헤드)
   BOARD_BATT_MAH / BOARD_CHG_MA 는 boards/<board>.h (미정의 시 board_select.h
   기본 600/300)에서 온다. 예:
     GNPE 600mAh / 300mA → 2.0h × 1.25 = 2.5h  (기존 고정값과 동일)
     XIAO 500mAh / 120mA → 4.17h × 1.25 ≈ 5.2h (느린 SGM40567 충전 반영)
   ※ 이 %는 OLED 충전 애니메이션 표시 전용 — 안전/컷오프(4.2V)는 충전 IC 가
     하드웨어로 처리하므로 추정 오차가 안전에 영향 없음.
   향후 ADC GPIO 가용 시 실제 BAT+ 전압 측정으로 교체 권장.
═══════════════════════════════════════════════ */
/* CC 이론시간(용량÷전류)에 CV 단계 보정 ×5/4(=×1.25). 컴파일타임 상수. */
#define CHG_FULL_DURATION_MS \
    ((int64_t)BOARD_BATT_MAH * 3600000LL / (BOARD_CHG_MA) * 5 / 4)
static int64_t s_chg_start_us = 0;

#if BOARD_HAS_BAT_ADC
/* ─── 배터리 전압 ADC (실측 %) ── A(VBUS USB감지) + B(BAT 분압 ADC) 조합의 B ───
 *   BAT 분압을 ADC 로 읽어 OCV-SoC 곡선으로 % 산출. LED-only STAT 라 충전상태를
 *   못 읽는 XIAO/H2 에서 시간기반 추정 대신 실측값을 쓴다. */
static adc_oneshot_unit_handle_t s_bat_adc  = NULL;
static adc_cali_handle_t         s_bat_cali = NULL;
static adc_channel_t             s_bat_ch;
static bool                      s_bat_adc_ok = false;

static void _bat_adc_init(void) {
  adc_unit_t unit;
  if (adc_oneshot_io_to_channel(BOARD_PIN_BAT_ADC, &unit, &s_bat_ch) != ESP_OK) {
    ESP_LOGW(TAG, "[BAT] GPIO%d 는 ADC 핀 아님 — 시간기반으로 폴백", BOARD_PIN_BAT_ADC);
    return;
  }
  adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = unit };
  if (adc_oneshot_new_unit(&ucfg, &s_bat_adc) != ESP_OK) return;
  adc_oneshot_chan_cfg_t ccfg = { .atten = ADC_ATTEN_DB_12,
                                  .bitwidth = ADC_BITWIDTH_DEFAULT };
  adc_oneshot_config_channel(s_bat_adc, s_bat_ch, &ccfg);
  adc_cali_curve_fitting_config_t calcfg = {
      .unit_id = unit, .chan = s_bat_ch,
      .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
  if (adc_cali_create_scheme_curve_fitting(&calcfg, &s_bat_cali) != ESP_OK)
    s_bat_cali = NULL;   /* 캘리 미지원 시 raw 근사 */
  s_bat_adc_ok = true;
  ESP_LOGI(TAG, "[BAT] ADC 초기화 GPIO%d (분압 %dk/%dk → Vbat=Vadc×%d/%d)",
           BOARD_PIN_BAT_ADC, BOARD_BAT_DIV_TOP, BOARD_BAT_DIV_BOT,
           BOARD_BAT_DIV_TOP + BOARD_BAT_DIV_BOT, BOARD_BAT_DIV_BOT);
}

/* ★2026-08-12 진단 모드: 배터리 로그 주기(측정 몇 회마다 1줄).
 *  1 = 매 측정(5초). 버튼 연타 구간의 전압 추이를 보려면 촘촘해야 한다.
 *  ★2026-08-20 **120(=10분)으로 되돌린다**(사용자 지시).
 *    측정 주기가 5초이므로 120회 = 600초. 촘촘한 진단이 다시 필요하면 1 로.
 *    ※절전 기여는 작다(초당 0.01% 수준). 주 목적은 로그를 읽을 수 있게 하는 것이고,
 *      **전압 추이 자체는 NVS batlog 가 120초마다 계속 남기므로 손실이 없다.** */
#ifndef BAT_DBG_EVERY
#define BAT_DBG_EVERY 120
#endif

/* BAT 단자 전압(mV). 실패 시 -1 (8회 평균). */
static int _read_bat_mv(void) {
  if (!s_bat_adc_ok) return -1;
  int raw, sum = 0, n = 0;
  /* ★★ADC 읽기 직렬화 — **진짜 근본원인**(2026-07-17 A/B 실측으로 확정).
   *
   *  `adc_oneshot_read()` 는 IDF(esp_adc/adc_oneshot.c:190) 에서 변환 전체를
   *      portENTER_CRITICAL(&rtc_spinlock);   ... adc_oneshot_hal_convert() ...
   *      portEXIT_CRITICAL(&rtc_spinlock);
   *  로 감싸 **인터럽트를 끈 채** 폴링한다. 이걸 8회 연속 돌리는 동안 **I2C 인터럽트가
   *  막혀** 진행 중이던 OLED 전송이 깨진다 → NACK → 화면 정지/깨짐.
   *  ★A/B 실측(120초): ADC 읽기 함=SSD1306 에러 505~827 / ADC 읽기 차단(init 만)=1
   *    / ADC 완전정지=1 → **읽기 호출 자체가 범인**(init 은 무해).
   *  → 그래서 OLED flush(oled_ui_i2c_lock) **와** PCF 비트뱅(btn 뮤텍스) **양쪽 모두**
   *    와 직렬화해야 한다. btn 뮤텍스만 잡으면 워치독(IDLE 굶음)은 잡히지만
   *    **화면 깨짐은 그대로**다(그 실수로 오래 헤맴).
   *
   *  [부수] ADC1 핀 겹침도 있다: BAT_ADC(GP1)=ADC1_CH1 이고 PCF 비트뱅 핀
   *   (GP6)=ADC1_CH6 이라 **같은 ADC1 유닛**이다. 둘이 동시에 접근하면 GP6 의 디지털
   *   토글이 GP1 변환을 교란해 btn_handler 가 `_i2c_read_bit` 에서 못 빠져나오고
   *   CPU 를 점유 → **IDLE 태스크가 굶어 Task Watchdog 폭주**(실측 380건,
   *   RA=_sda_read/_i2c_read_bit, 대상=IDLE, 첫 발동=첫 ADC 읽기 시점 21초).
   *   ※BOARD_BAT_SWAPPED=0(=% 실측)으로 ADC 를 읽기 시작하면 반드시 재발한다.
   *   → PCF 비트뱅과 **같은 뮤텍스**로 감싸 ADC1 동시접근을 없앤다.
   *     (scratchpad/sim_adc_i2c_serialize.py 로 겹침 0·기아 0·데드락 0 검증) */
  /*  ※2026-07-17 2차 수정 — **무한 대기 금지**.
   *   처음엔 oled_ui_i2c_lock()(portMAX_DELAY) 을 썼는데, flush 가 NACK 1회로
   *   IDF I2C 무한 스핀(i2c_master.c: `while (i2c_ll_is_bus_busy()) nop;` — 타임아웃
   *   없음)에 걸려 **뮤텍스를 쥔 채** CPU 를 놓지 않자 somfy_app 이 영원히 막혔다.
   *   실측: 120초에 BAT 읽기 **2회뿐**(정상 ~24회) + IDLE 워치독 17회(35.7초~120초
   *   전 구간, 5초 간격). → 배터리 측정은 5초 주기 비긴급 작업이므로
   *   **못 잡으면 이번 주기를 건너뛴다**(다음 주기에 읽으면 그만).
   *   lock 순서도 통일: btn → oled 로만 잡고, 실패 시 즉시 되돌려 데드락을 없앤다. */
  SemaphoreHandle_t _i2c_mtx = btn_handler_get_i2c_mutex();
  /* ★타임아웃 20ms → 300ms. 버튼 태스크가 10ms 마다 이 뮤텍스를 쥐므로 20ms 로는
   *  거의 매번 실패한다. 아래에서 실패 시 측정을 건너뛰도록 바꾸자 **영영 측정이 안 돼
   *  화면에 "--%" 만 나왔다**(실사용 신고). 배터리 측정은 5초 주기라 300ms 를 기다려도
   *  아무 문제 없다 — 넉넉히 기다려 확실히 잡는다. */
  bool _locked = (_i2c_mtx && xSemaphoreTake(_i2c_mtx, pdMS_TO_TICKS(300)) == pdTRUE);
  /* ★★2026-08-12 **버튼 뮤텍스를 못 잡으면 이번 주기를 건너뛴다.**
   *
   *  버그였다: 예전엔 _locked 가 false 여도 그대로 ADC 를 읽었다(OLED 락만 검사).
   *  그런데 BAT_ADC(GPIO1)=ADC1_CH1 이고 PCF 비트뱅(GPIO6)=ADC1_CH6 으로 **같은 ADC1
   *  유닛**이라, 버튼 비트뱅이 GP6 을 토글하는 동안 GP1 변환이 교란된다.
   *  → 좌/우 버튼을 연타하면 버튼 태스크가 뮤텍스를 계속 쥐어 20ms 타임아웃이 나고,
   *    오염된 값이 그대로 표시된다. 실측 신고: 연타 중 38→39→**40%** 로 **올라갔고**
   *    (부하가 걸리면 내려가야 하므로 물리적으로 불가능), 재부팅 후 44→45→**37%** 로
   *    널뛰었다. 교란은 방향성이 없어 위아래 아무 쪽으로나 튄다.
   *
   *  배터리 측정은 5초 주기 비긴급 작업이므로 **건너뛰는 게 옳다**(다음 주기에 읽으면
   *  그만). 이 직렬화가 필요하다는 건 위 주석에 이미 있었는데 실패 경로가 빠져 있었다. */
  if (_i2c_mtx && !_locked) {
    return -1;                          /* 버튼 비트뱅 진행 중 → ADC1 교란 회피 */
  }
  if (!oled_ui_i2c_trylock(30)) {      /* ★OLED flush 와도 직렬화(위 주석의 핵심 이유) */
    if (_locked) xSemaphoreGive(_i2c_mtx);
    return -1;                          /* 전송 중 → 이번 주기 건너뜀(무한 대기 X) */
  }
  int _s[8];
  int _rmin = 4096, _rmax = -1;             /* ★A: 같은 측정 안의 표본 산포 */
  for (int i = 0; i < 8; i++)
    if (adc_oneshot_read(s_bat_adc, s_bat_ch, &raw) == ESP_OK) {
      _s[n++] = raw; sum += raw;
      if (raw < _rmin) _rmin = raw;
      if (raw > _rmax) _rmax = raw;
    }
  oled_ui_i2c_unlock();
  if (_locked) xSemaphoreGive(_i2c_mtx);
  if (n == 0) return -1;
  /* 산포 진단값은 **전체 8표본**의 min/max 를 쓴다 — 절사한 값으로 재면 잡음
   * 크기 자체를 못 본다(그걸 보려고 넣은 진단이다). */
  {
    const int _d = _rmax - _rmin;
    s_bat_last_spread = (uint8_t)(_d > 255 ? 255 : _d);
  }
  /* ★★2026-08-12 평균 → **25% 절사평균**(정렬 후 가운데 절반만 평균).
   *
   *  근거(실측): 8표본은 수십 us 안에 끝나 **같은 순간**을 재는데, 배터리 구동에서
   *  그 안의 산포가 중앙값 36카운트(54mV), 64%가 30카운트 이상, 최대 115(174mV)였다
   *  (USB 는 3~14카운트). 배터리가 수십 us 에 54mV 를 움직일 수는 없으므로 이건
   *  **ADC 교란**이다. 그런데 단순 평균은 이상치 하나에 통째로 끌려간다 —
   *  조용한 7표본 + 115카운트 튐 1개면 평균이 21mV(=3%p) 밀린다.
   *  → 정렬해 양 끝을 버리고 가운데만 평균내면 그 밀림이 0 이 된다.
   *  검증: sim/tools/bat_adc_trim_sim.py — 실측 산포 분포에 맞춘 모델에서
   *        참값 대비 RMS 오차 7.2mV → 3.4mV (53% 감소).
   *  ※표본 수 8 은 **그대로** 둔다 — _nobat_track 의 5분 창 통계 가정 유지.
   *  ※이건 표시 안정화일 뿐 잡음을 없애지 못한다. 근본 해법은 HW:
   *    BAT_ADC 분압(100k/100k = 소스 임피던스 50kΩ) 하단에 100nF. */
  for (int i = 1; i < n; i++) {            /* 삽입정렬(최대 8개) */
    int v = _s[i], j = i - 1;
    while (j >= 0 && _s[j] > v) { _s[j + 1] = _s[j]; j--; }
    _s[j + 1] = v;
  }
  if (n >= 4) {
    const int lo = n / 4, hi = n - n / 4;  /* 가운데 절반 [lo, hi) */
    sum = 0;
    for (int i = lo; i < hi; i++) sum += _s[i];
    n = hi - lo;
  }
  raw = sum / n;
  int mv;
  if (s_bat_cali) {
    if (adc_cali_raw_to_voltage(s_bat_cali, raw, &mv) != ESP_OK) return -1;
  } else {
    mv = raw * 3100 / 4095;   /* 12dB 근사(캘리 없을 때) */
  }
  int _vbat = mv * (BOARD_BAT_DIV_TOP + BOARD_BAT_DIV_BOT) / BOARD_BAT_DIV_BOT;
  static int _bdbg = 0;
  if ((_bdbg++ % BAT_DBG_EVERY) == 0) {  /* ★진단 중: 매 측정(5초) */
    /* 산포를 배터리 전압 기준 mV 로 환산. 나눗셈 순서 주의 — `*(TOP+BOT)` 를 먼저
     * 하면 int32 가 넘친다(4095*3100*200 = 2.5e9). BOT 로 먼저 나눈다. */
    const int _sp_mv = (int)s_bat_last_spread * 3100 / BOARD_BAT_DIV_BOT
                       * (BOARD_BAT_DIV_TOP + BOARD_BAT_DIV_BOT) / 4095;
    ESP_LOGW(TAG, "[BAT?] raw=%d vadc=%dmV -> vbat=%dmV  표본산포=%d카운트(%dmV) n=%d",
             raw, mv, _vbat, (int)s_bat_last_spread, _sp_mv, n);
  }
  return _vbat;
}

/* ── 만충 전압 (이 값 이상 = 100%) ────────────────────────────────────────
 *  ★2026-08-11 실기 실측 기준. 이론값 4,200 mV 보다 낮은 이유는 분압 저항 오차 +
 *  ADC 캘리브레이션 오차(약 2%)다.
 *
 *  ★★기준을 4,128 → 4,108 로 정정(2026-08-11 2차): 처음에 쓴 4,128~4,132 mV 는
 *  **충전이 진행 중일 때(CV 구간)** 측정한 값이었다. 그때는 충전 전류가 내부저항을
 *  지나며 단자 전압을 들뜨게 한다. 몇 시간 충전해 **전류가 끊긴 뒤** 안정된 실제
 *  무부하 만충 전압은 **4,108 mV** 였고, OCV 곡선은 무부하 전압을 전제로 하므로
 *  이 값이 맞다. (4,128 로 두면 만충인데도 97% 에서 멈춰 보인다 — 실제 신고 증상.)
 *
 *  ★★★2026-08-13 **4,108 로 원복**(사용자 지시). 경위: 2026-08-12 에 4,158 →
 *    4,128 로 올렸다가 되돌렸다. 올린 근거는 "BAT_ADC 에 10µF 필터캡을 단 뒤
 *    충전 상태 실측이 4,152~4,160 으로 안정되니 그 위가 100% 로 뭉개진다" 였다.
 *
 *  ★★함정 기록 — **이 상수는 "USB 꽂자마자 100% 로 뛰는" 증상의 원인이 아니다.**
 *    실사용 신고: 배터리 구동 90%(4,044mV) → USB 연결 즉시 100%(4,152mV).
 *    그런데 두 곡선 모두 같은 결과다:
 *        4,044mV → 4108곡선 90% / 4128곡선 90%
 *        4,152mV → 4108곡선 100% / 4128곡선 100%
 *    진짜 원인은 **충전 전류가 내부저항을 지나며 단자전압을 108mV 들뜨게 하는 것**
 *    이고, 지금 코드가 그 전압을 그대로 % 로 환산하기 때문이다. BAT_FULL_MV 를
 *    어떤 값으로 잡아도 4,152mV 는 만충 문턱을 넘는다.
 *    → 고치려면 **충전 중 상승률 제한**(방전 중 하한 s_pct_floor 의 대칭)이 필요하다.
 *      전압 상수를 건드리는 걸로는 해결되지 않는다.
 *  ※XIAO 는 충전 IC 의 STAT 이 온보드 LED 전용이라 어떤 패드에도 안 나온다 →
 *    "충전 완료 신호로 100% 표시" 같은 정석 방법을 쓸 수 없어 전압 기준으로 잡는다.
 *
 *  왜 곡선 전체를 스케일하지 않고 **상단만 압축**하나:
 *    전체를 곱하면(=게인 보정) 중·저 구간이 통째로 움직여, 오래 검증한 방전 곡선과
 *    무배터리 float 판정(3,940~4,010 mV 창, 3,970 mV = 78%)까지 흔들린다.
 *    상단 3개 앵커만 원곡선 **모양을 유지한 채** 압축하면 만충 표시만 고쳐지고
 *    나머지는 그대로다.
 *
 *  기기·배터리마다 다르면 이 값만 바꾸면 된다(보드 헤더에서 먼저 정의해도 됨). */
#ifndef BAT_FULL_MV
#define BAT_FULL_MV 4108
#endif
/* 원곡선 상단: 3980=80%, 4080=90%, 4150=96%, 4200=100%
 * → [3980..4200] 구간을 [3980..BAT_FULL_MV] 로 선형 압축(모양 보존). */
#define BAT_TOP_BASE 3980
#define BAT_TOP_SCALE(v) (BAT_TOP_BASE + ((v) - BAT_TOP_BASE) *                           (BAT_FULL_MV - BAT_TOP_BASE) / (4200 - BAT_TOP_BASE))

/* 단셀 Li-ion OCV-SoC 곡선(battery_charge_sim 과 동일 anchors) → % */
static uint8_t _bat_mv_to_pct(int mv) {
  static const int V[] = {3200,3450,3580,3680,3750,3850, BAT_TOP_BASE,
                          BAT_TOP_SCALE(4080), BAT_TOP_SCALE(4150), BAT_FULL_MV};
  static const int P[] = {   0,   5,  10,  20,  40,  60,  80,  90,  96, 100};
  if (mv <= V[0]) return 0;
  for (size_t i = 1; i < sizeof(V) / sizeof(V[0]); i++)
    if (mv < V[i])
      return (uint8_t)(P[i-1] + (P[i]-P[i-1]) * (mv - V[i-1]) / (V[i]-V[i-1]));
  return 100;
}

/* ─── 표시용 배터리 전압 평활 (2026-08-11) ──────────────────────────────────
 *  증상: 배터리 구동으로 바꾸자 표시가 81→82→**74**→84→81→83→81→82 % 로 튀었다.
 *
 *  원인: USB 전원일 땐 레일이 단단해 전압이 거의 안 변했다. 배터리로 바꾸면 BLE 광고·
 *  RF 송신 순간 **실제로** 전압이 떨어진다. 5초 주기 측정이 그 순간에 걸리면 한 점이
 *  크게 낮게 찍힌다. OCV 곡선상 이 구간은 1% ≈ 10mV 라 100mV 만 흔들려도 10%p 가 움직인다.
 *  `_read_bat_mv()` 의 8회 평균은 수십 us 안에 끝나 **같은 순간을 8번 재는 것**이라
 *  ADC 노이즈만 줄일 뿐 부하 변동은 전혀 못 거른다(게다가 평균은 이상치에 끌려간다).
 *
 *  대책(표시 경로에만 적용):
 *    1) **중앙값 5주기** — 송신 순간에 걸린 점을 통째로 버린다(평균과 달리 안 끌려감)
 *    2) **EMA(α=1/4)** — 남은 흔들림을 시간축으로 눌러 표시가 안 튀게 한다
 *
 *  ★`_nobat_track` 에는 **원본 값**을 준다 — 그쪽은 5분 창 30표본 통계라 평활하면
 *    "전압이 오르는가" 판정 가정이 깨진다. 측정 주기·표본 수도 그대로 둔다.
 *
 *  ★★EMA 를 **1/16 mV 단위**로 누적하는 이유: C 의 정수 나눗셈은 0 쪽으로 절단한다.
 *    1mV 단위로 `ema += (med-ema)/4` 를 쓰면 차이가 1~3mV 일 때 몫이 0 이 되어
 *    **EMA 가 영영 안 움직인다**(고착). 16배 해상도면 1mV 차이도 4/16mV 씩 수렴한다.
 *    (sim/tools/bat_pct_smooth_sim.py 가 이 버그를 잡아냈다 — 파이썬 // 는 내림이라
 *     그냥 옮겼으면 못 봤을 것이다.)
 *
 *  검증: 진폭 19%p→9%p, 주기간 변동 4.1→0.38%p, 송신 강하 이상치 미노출, 방전 추종 지연 2%p */
/* ★2026-08-11 만충 보정으로 곡선 상단이 압축되며 1% ≈ 10mV → 7.4mV 로 **가팔라졌다**.
 *  같은 전압 흔들림이 더 큰 %p 로 보이므로 창을 5 → 9 로 늘린다.
 *  실측 비교(sim): 창5 진폭 11%p·변동 0.44%p / 창9 진폭 6%p·변동 0.20%p,
 *  추종 지연은 둘 다 1%p 로 동일 — 늘려도 손해가 없다. */
/* ── ★2026-08-11 USB 분리 후 방전 기록 (사용자 요청) ────────────────────────
 *  이 보드에는 **전류 센서(션트/게이지 IC)가 없다** — 잴 수 있는 건 배터리 전압뿐이다.
 *  그래서 전류는 **전압 하강에서 역산**한다:
 *      평균전류(mA) = 소모된 용량(mAh) / 경과시간(h)
 *      소모된 용량   = (시작% - 현재%) × 배터리용량 / 100
 *  OCV 곡선이 비선형이라 짧은 구간은 오차가 크지만, 세션 전체 평균으로 보면
 *  "몇 mA 를 쓰고 있나 / 몇 시간 버티나"를 판단하기에 충분하다.
 *  ※구간(직전 1분) 값도 같이 남긴다 — 화면 ON/OFF, 무선 ON/OFF 처럼 부하가 바뀌는
 *    순간을 잡으려면 누적 평균만으로는 안 보이기 때문이다. */
#ifndef BAT_CAPACITY_MAH
#define BAT_CAPACITY_MAH 700     /* EASYLANDER 402560 3.7V 700mAh */
#endif
/* ── ★2026-08-11 방전 기록 **NVS 영속 저장** ────────────────────────────────
 *  왜 NVS 인가: USB 를 뽑은 동안에는 시리얼 로그를 받을 호스트가 없어 전부 허공으로
 *  나간다(실제로 첫 시도에서 기록이 통째로 사라졌다). 전원이 끊겨도 남는 플래시에
 *  써야 나중에 USB 를 다시 꽂아 꺼내볼 수 있다.
 *
 *  기록 주기(사용자 지정, 임의로 바꾸지 말 것):
 *    · USB 분리 직후 2분간 : 5초마다
 *    · 그 이후            : 2분마다
 *
 *  구조: RAM 링버퍼에 쌓고 **매 샘플마다 통째로 NVS blob 1회 쓰기**.
 *  샘플 6바이트 × 300 = 1.8KB — 2분 간격이면 약 10시간, 초반 5초 24건 포함.
 *  꽉 차면 오래된 것부터 덮어써 최근 구간을 남긴다.
 *  조회: 콘솔 `bl` (덤프) / `bl clear` (삭제). */
#define BATLOG_MAX      BOARD_BATLOG_MAX   /* 보드별(board_select.h) — H2 는 128 */
#define BATLOG_FAST_S   120     /* 이 시간까지는 빠른 주기 */
#define BATLOG_FAST_IV  5       /* 빠른 주기(초) */
#define BATLOG_SLOW_IV  120     /* 이후 주기(초) */
/* 버튼 이벤트 코드(flags 상위 4비트). 0 은 주기측정이므로 1부터. */
#define BLEV_NONE   0
#define BLEV_LEFT   1
#define BLEV_RIGHT  2
#define BLEV_SELECT 3
#define BLEV_UP     4
#define BLEV_DOWN   5
#define BLEV_ROT    6
#define BLEV_PROG   7
#define BLEV_OTHER  8
/* ★★★2026-08-16 세션 경계 표식.
 *  예전엔 새 방전 세션이 시작될 때 `_batlog_reset()` 으로 링을 **통째로 비웠다**.
 *  그래서 세션 판정이 한 번만 틀려도 직전 기록이 전부 사라졌다(실제로 여러 번
 *  날아갔다). 이제는 비우지 않고 이 표식을 한 건 넣어 경계만 남긴다 —
 *  오래된 기록은 링이 돌면서 자연히 밀려날 뿐, **지워지지 않는다**. */
#define BLEV_SESS   9
/* ★2026-08-16 (①진단) 안테나 표시가 CONNECTED 를 벗어난 순간을 기록한다.
 *  사용자 신고가 배터리 구동 중(콘솔 없음) 상황이라 시리얼 로그로는 못 잡는다.
 *  → 방전 로그에 남겨 NVS 덤프로 나중에 읽는다(sim/tools/batlog_decode.py). */
/* ★NVS 쓰기 합치기: 버튼 연타 중 누름마다 1.8KB blob 을 쓰면 플래시 마모가 크고
 *  쓰기(10~20ms)가 버튼 태스크를 붙잡는다. RAM 링에는 **즉시** 넣고, NVS 는
 *  BATLOG_FLUSH_MS 마다 한 번만 쓴다. 전원이 갑자기 끊기면 최대 그만큼 잃지만,
 *  이 진단의 실패 모드는 정전이 아니라 "USB 재연결 후 조회" 라 문제없다. */
#define BATLOG_FLUSH_MS 2000

typedef struct __attribute__((packed)) {
  uint32_t t_s;      /* 세션 시작 후 경과 초.
                      *  ★★★2026-08-17 uint16 → uint32. 16비트는 **18.2시간
                      *  (65535초)에서 포화**한다. 실제로 37.8시간짜리 배터리
                      *  세션에서 300건 전부 t_s=65535 로 뭉개져 "언제 무슨 일이
                      *  있었는지" 시간축이 통째로 사라졌다(PROG 281건이 언제부터
                      *  시작됐는지 못 봄). 레코드가 9→11B 로 늘지만 C6 는
                      *  300건×11B=3.3KB 로 여유가 충분하다. */
  uint16_t mv;       /* 평활 전압 */
  uint8_t  pct;      /* 표시 % */
  uint8_t  flags;    /* bit0=무선ON bit1=화면ON bit2~3=PM상태 bit4~7=이벤트코드
                      *  이벤트코드 0=주기측정, 1~15=버튼(아래 BLEV_*) */
  uint8_t  sp;       /* ★2026-08-12 (A) 직전 측정의 8표본 산포(ADC 카운트).
                      *  배터리 구동 중엔 시리얼이 없으니 여기 실어 나른다. */
  uint8_t  ls;       /* ★★★2026-08-14 직전 샘플 이후 **light sleep 체류 비율**(0~100%).
                      *
                      *  왜 여기에 넣는가: 이 보드는 **포트를 여는 순간 리셋된다**
                      *  (부팅진단 `리셋사유=USB리셋(플래시/포트열기)`). 그래서
                      *  esp_pm_dump_locks 의 RAM 통계는 **읽으러 가는 행위 자체가
                      *  지워버린다** — 실제로 배터리 구간 통계를 그렇게 날렸다.
                      *  → 배터리 로그와 같은 NVS 경로로 실어 리셋에 견디게 한다.
                      *  값은 esp_pm_light_sleep_register_cbs 의 exit 콜백이 누적한
                      *  실제 sleep_time_us 로 계산한다(추정 아님). */
  int8_t   rssi;     /* ★★★2026-08-16 부모 RSSI(dBm). 127=측정불가.
                      *  왜 필요한가: RSSI 는 [PWR]/[MTDIAG] **시리얼로만** 나가서
                      *  배터리 구동 중엔 통째로 사라졌다. 사용자가 "특정 위치의
                      *  신호세기 이력"을 물었을 때 줄 게 없었다(권외 이벤트만 남음).
                      *  → 방전 로그에 같이 실어 NVS 덤프로 위치·시간별 세기를 본다.
                      *  값은 7초 Matter 갱신/60초 [PWR] 이 갱신하는 캐시를 쓴다
                      *  (여기서 OT 락을 잡으면 버튼 이벤트마다 락이 걸린다). */
} bat_sample_t;
/* ※구조체가 6→7바이트로 바뀌었다. _batlog_load() 가 blob 길이를 검사하므로
 *  예전 형식으로 저장된 기록은 자동 폐기된다(오해석 없음). */

#if BOARD_BATLOG_ENABLE
static bat_sample_t s_bl_buf[BATLOG_MAX];
static uint16_t     s_bl_n = 0;     /* 저장된 개수(최대 BATLOG_MAX) */
static uint16_t     s_bl_head = 0;  /* 다음 쓸 위치(링) */
static uint32_t     s_bl_sess = 0;  /* 세션 번호(부팅/분리마다 증가) */
/* NVS 에서 읽어온 직전 세션 상태 — _batlog_try_resume 이 사용 */
static uint8_t  s_bl_saved_on   = 0;
static uint32_t s_bl_saved_el   = 0;
static int      s_bl_saved_mv0  = 0;
static uint8_t  s_bl_saved_pct0 = 0;

/* ★★★2026-08-16 light sleep **진입 횟수**를 세션 단위로 누적해 NVS 에 남긴다.
 *
 *  왜: `rt`(태스크별 CPU) 실측에서 전 태스크 작업량 합이 **1.02%** 로 나왔다
 *  (IDLE 98.98%, somfy_app 0.70% = 회당 703µs). 그런데 배터리에서 sleep 체류는
 *  85% 다 — 남는 14%p 는 **일이 아니라** 깨어남 1회당 고정비용(진입/복귀)이거나
 *  락 보유 구간이라는 뜻이다. 그 둘을 가르려면 **진입 횟수**가 있어야 한다:
 *      회당 오버헤드 = (구간 - sleep시간)/진입횟수 - 703µs
 *  이 값이 십수 ms 면 메인 루프 주기가 유효한 지렛대이고, 1ms 수준이면 아니다.
 *
 *  왜 NVS 인가: 배터리 구동 중엔 콘솔이 없고, 포트를 열면 리셋되어 RAM 통계가
 *  날아간다. 방전 로그와 같은 경로로 실어 **NVS 덤프로 리셋 없이** 읽는다
 *  (sim/tools/batlog_decode.py).
 *
 *  세션 누적 = 이전 부팅들의 누적(base) + 이번 부팅에서 세션 시작 이후 증가분. */
static uint32_t s_dis_ls_cnt0     = 0;   /* 세션 시작 시점의 s_ls_count */
static int64_t  s_dis_ls_us0      = 0;   /* 세션 시작 시점의 s_ls_total_us */
static uint32_t s_dis_ls_base_cnt = 0;   /* 이전 부팅들에서 누적된 진입 횟수 */
static int64_t  s_dis_ls_base_us  = 0;   /* 〃 sleep 시간 */
static uint32_t s_bl_saved_lscnt  = 0;   /* NVS 에서 읽어온 값 */
static uint64_t s_bl_saved_lsus   = 0;
/* 세션 기준점 — 실제 정의는 아래 g_wake_iter 옆. C 의 tentative definition
 *  이라 두 번 나와도 같은 객체다(이 함수가 정의보다 앞서 나오기 때문). */
static uint32_t s_wi_sess0[WI_COUNT];
static void _batlog_ls_session_begin(bool resume);   /* 정의는 아래(_batlog_new_session 뒤) */

/* ═══ light sleep 실측 누적 — NVS 로 남기기 위한 계측기 ═══════════════════════
 *  esp_pm_light_sleep_register_cbs 의 exit 콜백이 **실제 잔 시간**(sleep_time_us)
 *  을 준다. IDLE 태스크 컨텍스트라 블로킹 금지 → 누적만 한다.
 *  이 값이 있어야 "절전이 됐나"를 배터리에서 직접 알 수 있다. USB 에서는
 *  설계상 light sleep 을 끄므로(USB-JTAG 보호) 0 이 정상이다. */
static volatile int64_t  s_ls_total_us = 0;   /* 부팅 후 누적 light sleep 시간 */
static volatile uint32_t s_ls_count    = 0;   /* 진입 횟수 */

/* ★★★2026-08-20 **수면 길이 히스토그램** — 깨어남이 어디서 오는지 가르는 계측.
 *
 *  문제: 배터리 세션 실측이 **93.8회/초**인데, 태스크 주기를 전수 조사해 더하면
 *      btn_handler 25ms(LP=ON) 40 + oled_ui(화면OFF) 500ms 2 +
 *      hold_repeat 500ms 2 + somfy_app 메인루프 100ms 10  =  54회/초
 *    뿐이다(coalescing 은 줄이기만 하므로 초과분을 설명 못 한다).
 *    약 40회/초가 우리 태스크가 아닌 곳에서 온다 — CHIP/OpenThread 타이머,
 *    또는 **LP 코어가 배터리 구간에 멈춰 btn 이 10ms 로 되돌아간 것**이 후보다.
 *    (LP=ON 은 USB 구동일 때 확인한 값이고, 배터리 구간은 확인된 적이 없다.)
 *
 *  추정을 그만두고 잰다: 잠든 길이의 분포를 보면 **지배 주기가 그대로 보인다**.
 *    ~10ms 봉우리 → btn 이 10ms(=LP 정지). ~25ms 봉우리 → LP 정상.
 *    그 어느 쪽도 아닌 짧은 봉우리 → 우리 태스크가 아닌 타이머.
 *  IDLE 컨텍스트에서 도는 콜백이라 배열 증가 한 번만 한다(블로킹·나눗셈 없음). */
#define LS_HIST_N 8
/* 경계(us): <2ms, 2~5, 5~8, 8~12, 12~20, 20~30, 30~60, ≥60 */
static const int32_t s_ls_hist_edge[LS_HIST_N - 1] = {
    2000, 5000, 8000, 12000, 20000, 30000, 60000 };
static volatile uint32_t s_ls_hist[LS_HIST_N];
/* ★2026-08-20 **재부팅 누적용 base** — 이게 없으면 부팅마다 0 에서 시작한 배열이
 *  저장본을 덮어써 기록이 통째로 날아간다. 실제로 첫 시도에서 그랬다:
 *  배터리 22분(깨어남 118,710회) 뒤 USB 를 꽂자 리셋이 걸렸고, 갓 부팅한
 *  히스토그램(40표본)이 저장본을 덮어써 분포를 못 봤다.
 *  (batlog 본체는 2026-08-15 에 같은 이유로 이미 이어받기를 넣어 뒀다.) */
static uint32_t s_ls_hist_base[LS_HIST_N];

/* ★2026-08-20 태스크별 실제 반복 횟수 (wake_diag.h 주석 참조) */
/* ★2026-08-20 **세션 기준점** — 이게 없으면 비교가 불가능하다.
 *  실제로 -Os 회차에서 틀렸다: 카운터는 부팅 후 누적인데 세션(596초)은 리셋돼
 *  회/초가 부풀려졌고, "남의 몫 -9%" 라는 불가능한 값이 나왔다.
 *  light sleep 계측(s_dis_ls_cnt0)이 이미 쓰는 방식을 그대로 따른다. */
static uint32_t s_wi_sess0[WI_COUNT];

/* ★2026-08-20 깨어난 원인 분포 — esp_sleep_get_wakeup_cause() 를 그대로 센다.
 *  IDF 상수는 값이 흩어져 있어 표를 두고 선형 탐색한다(8개, IDLE 에서 무해). */
#define WC_N 8
static const uint8_t s_wc_id[WC_N] = {
    ESP_SLEEP_WAKEUP_TIMER, ESP_SLEEP_WAKEUP_GPIO, ESP_SLEEP_WAKEUP_ULP,
    ESP_SLEEP_WAKEUP_UART,  ESP_SLEEP_WAKEUP_BT,   ESP_SLEEP_WAKEUP_WIFI,
    ESP_SLEEP_WAKEUP_EXT1,  ESP_SLEEP_WAKEUP_UNDEFINED };
static volatile uint32_t s_wc_hist[WC_N + 1];   /* 마지막 칸 = 표에 없는 값 */
static uint32_t s_wc_base[WC_N + 1];
static int64_t s_ls_mark_us   = 0;            /* 직전 배터리 샘플 시점의 누적값 */
static int64_t s_ls_mark_wall = 0;            /* 그때의 벽시계 */

#if CONFIG_PM_ENABLE && CONFIG_FREERTOS_USE_TICKLESS_IDLE
static esp_err_t _ls_exit_cb(int64_t sleep_time_us, void *arg) {
  (void)arg;
  if (sleep_time_us > 0) {
    s_ls_total_us += sleep_time_us; s_ls_count++;
    int b = 0;
    while (b < LS_HIST_N - 1 && sleep_time_us >= s_ls_hist_edge[b]) b++;
    s_ls_hist[b]++;
    /* ★2026-08-20 깨어난 원인도 함께 센다 */
    {
      const esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
      int i = 0;
      while (i < WC_N && s_wc_id[i] != (uint8_t)wc) i++;
      s_wc_hist[i]++;
      /* ★★★2026-08-23 ULP 깨움이면 **버튼 태스크를 즉시 깨운다**.
       *  LP 코어가 PCF 변화를 보고 ulp_lp_core_wakeup_main_processor() 로 HP CPU 를
       *  꺼냈다는 뜻이다. 그런데 그것만으로는 vTaskDelay 로 블록된 버튼 태스크가
       *  풀리지 않아, 폴 주기를 늘리면 그만큼 반응이 늦어진다(25→100ms 시도가
       *  "너무 느리다"로 되돌아간 이유). IDF 에 LP→HP 인터럽트 API 가 없어
       *  이 콜백이 유일한 통로다. 자세한 배경은 btn_handler_notify_from_lp 주석. */
      if (wc == ESP_SLEEP_WAKEUP_ULP) btn_handler_notify_from_lp();
    }
  }
  return ESP_OK;
}
static void _ls_stats_init(void) {
  esp_pm_sleep_cbs_register_config_t cfg = {
      .enter_cb = NULL, .exit_cb = _ls_exit_cb,
      .enter_cb_user_arg = NULL, .exit_cb_user_arg = NULL,
      .enter_cb_prior = 0, .exit_cb_prior = 0 };
  esp_err_t e = esp_pm_light_sleep_register_cbs(&cfg);
  ESP_LOGW(TAG, "[LS] light sleep 계측 등록: %s", esp_err_to_name(e));
}
#else
static void _ls_stats_init(void) { ESP_LOGW(TAG, "[LS] PM/tickless 미사용 — 계측 없음"); }
#endif

/* ★★★2026-08-24 **PM 락 보유자 덤프를 NVS 로** — H2 가 안 자는 이유를 밝히려고.
 *
 *  실측: H2 는 배터리 13.9시간 동안 light sleep 이 **0.0%** 였다. 그런데 batlog 의
 *  pm 열은 **3**(= light sleep ON + 배터리) — 즉 **펌웨어 설정은 정상**인데 실제로
 *  안 잔다. 누군가 ESP_PM_NO_LIGHT_SLEEP 락을 계속 쥐고 있다는 뜻이다.
 *  (같은 조건에서 C6 는 96.2% 잔다. USJ_NO_AUTO_LS_ON_CONNECTION 을 껐지만
 *   변화가 없었으므로 그 가설은 틀렸다.)
 *
 *  `esp_pm_dump_locks()` 가 답을 갖고 있는데 **stdout 으로만** 출력한다. H2 는
 *  배터리 구동 중 USB 콘솔이 죽고 write 도 불안정해(기록된 지뢰) 콘솔로는 못 받는다.
 *  → open_memstream 으로 메모리에 받아 NVS 에 남긴다. 배터리에서 잡아 USB 로
 *    돌아온 뒤 esptool 로 읽으면 된다.
 *  ※CONFIG_PM_PROFILING 이 켜져 있으면 각 락의 보유 시간까지 나온다. H2 는 지금
 *    절전이 0 이라 프로파일링 오버헤드로 잃을 것이 없다. */
#if CONFIG_PM_ENABLE
#define PMDUMP_MAX 1024
static void _pmdump_save(void) {
  char  *buf = NULL;
  size_t len = 0;
  FILE  *ms  = open_memstream(&buf, &len);
  if (!ms) return;
  esp_pm_dump_locks(ms);
  fclose(ms);                       /* 여기서 buf/len 이 확정된다 */
  if (buf) {
    nvs_handle_t h;
    if (nvs_open("pmdump", NVS_READWRITE, &h) == ESP_OK) {
      if (len > PMDUMP_MAX - 1) len = PMDUMP_MAX - 1;
      buf[len] = 0;
      nvs_set_str(h, "locks", buf);
      nvs_commit(h);
      nvs_close(h);
    }
    free(buf);
  }
}
#else
static void _pmdump_save(void) {}
#endif

/* 직전 샘플 이후 light sleep 비율(0~100). 호출 시 기준점을 갱신한다. */
static uint8_t _ls_take_pct(int64_t now_us) {
  const int64_t d_sleep = s_ls_total_us - s_ls_mark_us;
  const int64_t d_wall  = now_us - s_ls_mark_wall;
  s_ls_mark_us   = s_ls_total_us;
  s_ls_mark_wall = now_us;
  if (d_wall <= 0) return 0;
  int64_t pct = (d_sleep * 100) / d_wall;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
}

/* ★★★2026-08-15 **세션 상태도 함께 NVS 에 남긴다** — 재부팅에도 기록이 이어지게.
 *
 *  사용자 신고: "6시간 배터리 구동했는데 기록이 25.9분뿐이다."
 *  원인: 전환 감지의 `static bool was_usb_pwr = true;` 가 **부팅 시 항상 USB 로
 *  가정**해서, 배터리 구동 중 재부팅(배터리 글리치/브라운아웃)이 나면 첫 판독에서
 *  "USB→배터리 전환"으로 오인하고 `_batlog_reset()` 이 **직전 기록을 통째로 지웠다**.
 *  실제로 부팅진단에 `261회차 bat=3990mV 리셋사유=전원투입` 이 남아 있었다.
 *
 *  → 세션 경과시간·기준값·활성여부를 로그와 같이 저장하고, 부팅 시 배터리 상태면
 *    **같은 세션을 이어받는다**(_batlog_try_resume). 그러면 재부팅이 나도 기록이 남는다. */
static void _batlog_save(void) {
  nvs_handle_t h;
  if (nvs_open("batlog", NVS_READWRITE, &h) != ESP_OK) return;
  nvs_set_u16(h, "n",    s_bl_n);
  nvs_set_u16(h, "head", s_bl_head);
  nvs_set_u32(h, "sess", s_bl_sess);
  nvs_set_blob(h, "buf", s_bl_buf, sizeof(bat_sample_t) * BATLOG_MAX);
  /* ★세션 상태 — 재부팅 후 이어받기용.
   *  경과시간은 esp_timer 가 리셋되므로 **초 단위 절대 경과**로 저장한다. */
  const uint32_t el_s = s_dis_t0_us
      ? (uint32_t)((esp_timer_get_time() - s_dis_t0_us) / 1000000) : 0;
  nvs_set_u8 (h, "dis_on",  s_dis_t0_us ? 1 : 0);
  nvs_set_u32(h, "dis_el",  el_s);
  nvs_set_u32(h, "dis_mv0", (uint32_t)s_dis_mv0);
  nvs_set_u8 (h, "dis_pc0", s_dis_pct0);
  /* ★light sleep 세션 누적 — 위 s_dis_ls_* 주석 참조 */
  nvs_set_u32(h, "ls_cnt", s_dis_ls_base_cnt + (s_ls_count - s_dis_ls_cnt0));
  nvs_set_u64(h, "ls_us",  (uint64_t)(s_dis_ls_base_us +
                                      (s_ls_total_us - s_dis_ls_us0)));
  /* ★2026-08-20 수면 길이 히스토그램 + LP 상태 — 깨어남 출처 판별용.
   *  부팅 후 누적(세션 기준 차감 안 함) — 분포의 **모양**만 보면 되므로 충분하다.
   *  blob 한 덩어리로 저장해 NVS 엔트리 낭비를 줄인다(8×u32 = 32B). */
  {
    uint32_t hist[LS_HIST_N];
    for (int i = 0; i < LS_HIST_N; i++)
      hist[i] = s_ls_hist_base[i] + s_ls_hist[i];   /* ★재부팅 누적 */
    nvs_set_blob(h, "ls_hist", hist, sizeof(hist));
    {
      uint32_t wc[WC_N + 1], it[WI_COUNT];
      for (int i = 0; i <= WC_N; i++) wc[i] = s_wc_base[i] + s_wc_hist[i];
      /* ★세션 시작 이후분만 — 위 s_wi_sess0 주석 참조 */
      for (int i = 0; i < WI_COUNT; i++) it[i] = g_wake_iter[i] - s_wi_sess0[i];
      nvs_set_blob(h, "wc_hist", wc, sizeof(wc));
      nvs_set_blob(h, "wi_cnt",  it, sizeof(it));
    }
    nvs_set_u8  (h, "lp_on", btn_handler_lp_active() ? 1 : 0);
    nvs_set_u16 (h, "poll_ms", (uint16_t)btn_handler_poll_ms());
  }
  nvs_commit(h);
  nvs_close(h);
}

static void _batlog_load(void) {
  nvs_handle_t h;
  if (nvs_open("batlog", NVS_READONLY, &h) != ESP_OK) return;
  size_t len = sizeof(s_bl_buf);
  if (nvs_get_blob(h, "buf", s_bl_buf, &len) != ESP_OK || len != sizeof(s_bl_buf)) {
    memset(s_bl_buf, 0, sizeof(s_bl_buf)); s_bl_n = 0; s_bl_head = 0;
  } else {
    nvs_get_u16(h, "n", &s_bl_n);
    nvs_get_u16(h, "head", &s_bl_head);
    nvs_get_u32(h, "sess", &s_bl_sess);
    /* 세션 상태(있으면) — 실제 이어받기는 _batlog_try_resume 이 판단한다. */
    nvs_get_u8 (h, "dis_on",  &s_bl_saved_on);
    nvs_get_u32(h, "dis_el",  &s_bl_saved_el);
    uint32_t _mv = 0; nvs_get_u32(h, "dis_mv0", &_mv); s_bl_saved_mv0 = (int)_mv;
    nvs_get_u8 (h, "dis_pc0", &s_bl_saved_pct0);
    nvs_get_u32(h, "ls_cnt", &s_bl_saved_lscnt);
    nvs_get_u64(h, "ls_us",  &s_bl_saved_lsus);
    /* ★2026-08-20 수면 히스토그램 이어받기 — 선언부 주석의 덮어쓰기 사고 대응 */
    {
      size_t hl = sizeof(s_ls_hist_base);
      if (nvs_get_blob(h, "ls_hist", s_ls_hist_base, &hl) != ESP_OK ||
          hl != sizeof(s_ls_hist_base)) {
        for (int i = 0; i < LS_HIST_N; i++) s_ls_hist_base[i] = 0;
      }
      size_t wl = sizeof(s_wc_base);
      if (nvs_get_blob(h, "wc_hist", s_wc_base, &wl) != ESP_OK ||
          wl != sizeof(s_wc_base)) {
        for (int i = 0; i <= WC_N; i++) s_wc_base[i] = 0;
      }
    }
  }
  nvs_close(h);
}

static void _batlog_add_ev(int t_s, int mv, int pct, bool radio, bool panel, int pm,
                           int ev);   /* 전방 선언 (정의는 아래) */

/* 부팅 시 호출 — 배터리 구동 중이고 직전 세션이 살아 있으면 **이어받는다**.
 *  이어받으면 t0 를 "지금 - 저장된 경과" 로 되돌려 로그의 +초 축이 연속된다.
 *  USB 구동이면 아무것도 안 한다(정상적으로 세션 없음). */
static bool _batlog_try_resume(bool on_battery)
{
  if (!on_battery || !s_bl_saved_on || s_bl_n == 0) return false;
  s_dis_t0_us    = esp_timer_get_time() - (int64_t)s_bl_saved_el * 1000000;
  s_dis_mv0      = s_bl_saved_mv0;
  s_dis_pct0     = s_bl_saved_pct0;
  s_dis_prev_us  = esp_timer_get_time();
  s_dis_prev_mv  = s_bl_saved_mv0;
  _batlog_ls_session_begin(true);   /* 이전 부팅들의 light sleep 누적을 이어받는다 */
  s_dis_prev_pct = s_bl_saved_pct0;
  ESP_LOGW(TAG, "[BATLOG] ★재부팅 감지 — 세션 #%u 이어받음 (경과 %us, %d건 보존)",
           (unsigned)s_bl_sess, (unsigned)s_bl_saved_el, (unsigned)s_bl_n);
  _batlog_add_ev((int)s_bl_saved_el, s_bl_saved_mv0, s_bl_saved_pct0,
                 matter_blinds_get_radio_enabled(), oled_ui_is_panel_on(),
                 g_pm_state_applied, BLEV_OTHER);   /* 재부팅 지점 표시 */
  return true;
}

/* ★전체 삭제 — 콘솔 `blclear` 로 사용자가 명시적으로 요청할 때만 불린다.
 *  2026-08-16 이전에는 새 방전 세션마다 이걸 불러 기록이 사라졌다. */
static void _batlog_reset(void) {
  s_bl_n = 0; s_bl_head = 0; s_bl_sess++;
  memset(s_bl_buf, 0, sizeof(s_bl_buf));
  _batlog_save();
}

/* ★2026-08-16 새 방전 세션 — **지우지 않는다**. 세션 번호만 올린다.
 *  경계는 호출자가 BLEV_SESS 표식으로 남긴다. */
static void _batlog_new_session(void) { s_bl_sess++; }

/* 방전 세션의 light sleep 계측 기준점을 잡는다.
 *  resume=true  → 이전 부팅들의 누적을 이어받는다(재부팅해도 합계가 유지됨)
 *  resume=false → 새 세션이므로 0 부터
 *  어느 쪽이든 "지금까지 이 부팅에서 잔 것"은 세션에 넣지 않는다(cnt0/us0 로 뺀다). */
static void _batlog_ls_session_begin(bool resume) {
  s_dis_ls_cnt0     = s_ls_count;
  s_dis_ls_us0      = s_ls_total_us;
  s_dis_ls_base_cnt = resume ? s_bl_saved_lscnt : 0;
  s_dis_ls_base_us  = resume ? (int64_t)s_bl_saved_lsus : 0;
  /* ★2026-08-20 태스크 반복 카운터도 같은 시점에 기준을 잡는다.
   *  이게 없어서 -Os 회차 비교가 통째로 틀렸다(부팅 후 누적을 세션 초로 나눠
   *  회/초가 부풀려지고 "남의 몫 -9%" 라는 불가능한 값이 나왔다). */
  for (int i = 0; i < WI_COUNT; i++) s_wi_sess0[i] = g_wake_iter[i];
}

static bool     s_bl_dirty = false;
static int64_t  s_bl_last_save_us = 0;

/* 주기적으로 호출 — 쌓인 게 있고 마지막 쓰기 후 충분히 지났으면 NVS 에 반영. */
static void _batlog_flush_if_due(int64_t now_us, bool force) {
  if (!s_bl_dirty) return;
  if (!force && (now_us - s_bl_last_save_us) < (int64_t)BATLOG_FLUSH_MS * 1000) return;
  _batlog_save();
  s_bl_dirty = false;
  s_bl_last_save_us = now_us;
}

static void _batlog_add_ev(int t_s, int mv, int pct, bool radio, bool panel, int pm,
                           int ev) {
  bat_sample_t *e = &s_bl_buf[s_bl_head];
  e->t_s  = (uint32_t)(t_s < 0 ? 0 : t_s);   /* ★포화 없음(uint32) */
  e->mv   = (uint16_t)mv;
  e->pct  = (uint8_t)pct;
  e->flags = (uint8_t)((radio ? 1 : 0) | (panel ? 2 : 0) |
                       (((pm < 0 ? 0 : pm) & 3) << 2) |
                       (((ev < 0 ? 0 : ev) & 0x0F) << 4));
  e->sp   = s_bat_last_spread;      /* ★A 직전 측정의 ADC 표본 산포 */
  e->ls   = _ls_take_pct(esp_timer_get_time());  /* ★직전 샘플 이후 light sleep 비율 */
  e->rssi = s_last_rssi_cache;                   /* ★부모 RSSI(dBm), 127=불가 */
  s_bl_head = (uint16_t)((s_bl_head + 1) % BATLOG_MAX);
  if (s_bl_n < BATLOG_MAX) s_bl_n++;
  s_bl_dirty = true;       /* NVS 반영은 _batlog_flush_if_due 가 합쳐서 한다 */
}

static void _batlog_add(int t_s, int mv, int pct, bool radio, bool panel, int pm) {
  _batlog_add_ev(t_s, mv, pct, radio, panel, pm, BLEV_NONE);
}

/* 버튼 이벤트를 방전 기록에 남긴다(방전 세션 중에만). */
void somfy_app_batlog_button(int ev) {
  if (!s_dis_t0_us) return;                 /* USB 연결 중이면 기록 안 함 */
  const int64_t now = esp_timer_get_time();
  _batlog_add_ev((int)((now - s_dis_t0_us) / 1000000),
                 s_bat_last_raw_mv,
                 (s_ui.chg_percent <= 100) ? s_ui.chg_percent : 0,
                 matter_blinds_get_radio_enabled(), oled_ui_is_panel_on(),
                 g_pm_state_applied, ev);
}

/* 콘솔 `bl` — 저장된 방전 기록 전체를 출력. */
void somfy_app_batlog_dump(void) {
  ESP_LOGW(TAG, "[BATLOG] === 저장된 방전 기록: 세션 #%u, %u건 (용량 %dmAh) ===",
           (unsigned)s_bl_sess, (unsigned)s_bl_n, BAT_CAPACITY_MAH);
  if (s_bl_n == 0) { ESP_LOGW(TAG, "[BATLOG] (비어 있음)"); return; }
  const uint16_t start = (s_bl_n < BATLOG_MAX) ? 0
                       : (uint16_t)((s_bl_head + BATLOG_MAX - s_bl_n) % BATLOG_MAX);
  int p0 = -1, t0 = 0, nsess = 0;
  for (uint16_t i = 0; i < s_bl_n; i++) {
    const bat_sample_t *e = &s_bl_buf[(start + i) % BATLOG_MAX];
    const int ev = (e->flags >> 4) & 0x0F;
    /* ★2026-08-16 세션 경계에서 기준점을 다시 잡는다. 한 링에 여러 세션이 함께
     *  남으므로(더 이상 지우지 않는다) 그냥 두면 t_s 가 0 으로 되돌아가 dt 가
     *  음수가 되고 평균 전류가 엉뚱하게 찍힌다. */
    if (ev == BLEV_SESS || p0 < 0) { p0 = e->pct; t0 = e->t_s; }
    if (ev == BLEV_SESS) {
      nsess++;
      ESP_LOGW(TAG, "BL ────── 새 방전 세션 시작 (%d번째 경계) ──────", nsess);
    }
    const int dt = e->t_s - t0;
    /* ★2026-08-12 분모의 `* 10` 제거 — 아래 [BATLOG] 쪽과 같은 버그였다. */
    const int ma = (dt > 0) ? ((p0 - e->pct) * BAT_CAPACITY_MAH * 36) / dt : 0;
    static const char *EVN[] = {"주기","LEFT","RIGHT","SEL","UP","DOWN","ROT","PROG","기타",
                                "세션시작","권외-","미등록X","?","?","?"};
    /* ★A 산포를 배터리 전압 mV 로 환산(나눗셈 순서 = _read_bat_mv 와 동일) */
    const int sp_mv = (int)e->sp * 3100 / BOARD_BAT_DIV_BOT
                      * (BOARD_BAT_DIV_TOP + BOARD_BAT_DIV_BOT) / 4095;
    ESP_LOGW(TAG, "BL %4u  +%5us  %4umV  %3u%%  avg%4dmA  radio=%d screen=%d pm=%d  "
                  "sleep%3u%%  산포%3d(%2dmV)  %s",
             (unsigned)i, (unsigned)e->t_s, (unsigned)e->mv, (unsigned)e->pct, ma,
             (e->flags & 1) ? 1 : 0, (e->flags & 2) ? 1 : 0, (e->flags >> 2) & 3,
             (unsigned)e->ls, (int)e->sp, sp_mv,
             EVN[(e->flags >> 4) & 0x0F]);
  }
}
void somfy_app_batlog_clear(void) { _batlog_reset(); ESP_LOGW(TAG, "[BATLOG] 기록 삭제됨"); }
#else  /* !BOARD_BATLOG_ENABLE — 방전 로그 링버퍼·NVS 저장 전부 제외(heap 확보) */
static void _batlog_load(void) {}
static void _batlog_flush_if_due(int64_t now_us, bool force) { (void)now_us; (void)force; }
static void _batlog_reset(void) {}
static void _batlog_new_session(void) {}
static void _batlog_ls_session_begin(bool resume) { (void)resume; }
static void _ls_stats_init(void) {}   /* BATLOG 끔 — light sleep 계측 없음 */
/* 버튼/안테나 이벤트 기록 — BATLOG 끔이면 no-op (호출부는 가드 밖에 있다). */
void somfy_app_batlog_button(int ev) { (void)ev; }
static void _batlog_add(int t_s, int mv, int pct, bool radio, bool panel, int pm) {
  (void)t_s; (void)mv; (void)pct; (void)radio; (void)panel; (void)pm;
}
static void _batlog_add_ev(int t_s, int mv, int pct, bool radio, bool panel, int pm, int ev) {
  (void)t_s; (void)mv; (void)pct; (void)radio; (void)panel; (void)pm; (void)ev;
}
static bool _batlog_try_resume(bool on_battery) { (void)on_battery; return false; }
void somfy_app_batlog_dump(void) {
  ESP_LOGW(TAG, "[BATLOG] 이 보드에서는 비활성(BOARD_BATLOG_ENABLE=0 — heap 확보)");
}
void somfy_app_batlog_clear(void) { somfy_app_batlog_dump(); }
#endif /* BOARD_BATLOG_ENABLE */

/* ═══════════════════════════════════════════════
   진동센서 진단 기록 (★2026-08-13 신규, 사용자 요청)
   ──────────────────────────────────────────────
   왜 NVS 인가: 배터리 구동 중엔 시리얼이 없어 `[VIBE-stat]` 을 볼 수 없다.
   기기를 들고 흔들어 본 결과를 나중에 USB 를 꽂아 `vl` 로 확인하기 위함이다.

   ★기록/판독 규칙 — 한 창(VIBELOG_WIN_S 초)의 폴링 N 회 중 HIGH 가 H 회일 때
     H == N   계속 HIGH — 풀업 그대로(접점이 안 닫힘)
     H == 0   계속 LOW  — 접점이 붙어 있음 / GND 단락
     0<H<N    **섞임 = 실제 접점 동작** ← 흔들었을 때 이게 나와야 정상
   isr 은 에지 수. 레벨이 안 변해도 늘어나면 폴링(10ms)보다 짧은 글리치가 있다는 뜻.

   플래시 마모 대책: **섞인 창만** 남기고, 나머지는 VIBELOG_HB_S 마다 heartbeat
   1건만 남긴다. 3초마다 전부 쓰면 하루 28,800건이라 링이 금세 덮인다.
═══════════════════════════════════════════════ */
#define VIBELOG_MAX    BOARD_VIBELOG_MAX  /* 보드별(board_select.h) — 8바이트 × N */
#define VIBELOG_WIN_S    3        /* 관측 창(초) — [VIBE-stat] 과 동일 */
#define VIBELOG_HB_S    60        /* 활동이 없을 때 heartbeat 간격(초) */

#if BOARD_VIBELOG_ENABLE
typedef struct __attribute__((packed)) {
  uint16_t t_s;     /* 부팅 후 경과 초 */
  uint16_t polls;   /* 이 창의 폴링 횟수 */
  uint16_t high;    /* 그중 HIGH 로 읽힌 횟수 */
  uint16_t isr;     /* 이 창의 ISR(에지) 증가분 — 65535 에서 포화 */
} vibe_sample_t;

static vibe_sample_t s_vl_buf[VIBELOG_MAX];
static uint16_t      s_vl_n = 0, s_vl_head = 0;
static bool          s_vl_dirty = false;
static int64_t       s_vl_last_save_us = 0;

static void _vibelog_save(void) {
  nvs_handle_t h;
  if (nvs_open("vibelog", NVS_READWRITE, &h) != ESP_OK) return;
  nvs_set_u16(h, "n", s_vl_n);
  nvs_set_u16(h, "head", s_vl_head);
  nvs_set_blob(h, "buf", s_vl_buf, sizeof(s_vl_buf));
  nvs_commit(h);
  nvs_close(h);
}

static void _vibelog_load(void) {
  nvs_handle_t h;
  if (nvs_open("vibelog", NVS_READONLY, &h) != ESP_OK) return;
  size_t len = sizeof(s_vl_buf);
  if (nvs_get_blob(h, "buf", s_vl_buf, &len) != ESP_OK || len != sizeof(s_vl_buf)) {
    memset(s_vl_buf, 0, sizeof(s_vl_buf)); s_vl_n = 0; s_vl_head = 0;
  } else {
    nvs_get_u16(h, "n", &s_vl_n);
    nvs_get_u16(h, "head", &s_vl_head);
  }
  nvs_close(h);
}

static void _vibelog_add(int t_s, uint32_t polls, uint32_t high, uint32_t isr) {
  vibe_sample_t *e = &s_vl_buf[s_vl_head];
  e->t_s   = (uint16_t)(t_s > 65535 ? 65535 : t_s);
  e->polls = (uint16_t)(polls > 65535 ? 65535 : polls);
  e->high  = (uint16_t)(high  > 65535 ? 65535 : high);
  e->isr   = (uint16_t)(isr   > 65535 ? 65535 : isr);
  s_vl_head = (uint16_t)((s_vl_head + 1) % VIBELOG_MAX);
  if (s_vl_n < VIBELOG_MAX) s_vl_n++;
  s_vl_dirty = true;
}

/* 메인 루프에서 주기 호출. 카운터 델타를 떠서 필요할 때만 기록하고,
 * NVS 쓰기는 10초에 한 번으로 합친다(버튼 태스크는 전혀 관여하지 않는다). */
static void _vibelog_tick(int64_t now_us) {
  static int64_t  win_us  = 0;      /* 창 시작 시각 */
  static uint32_t p0 = 0, h0 = 0, i0 = 0;
  static int64_t  last_rec_us = 0;

  if (win_us == 0) {                /* 첫 호출 — 기준점만 잡는다 */
    win_us = now_us;
    p0 = btn_handler_vibe_poll_total();
    h0 = btn_handler_vibe_high_total();
    i0 = btn_handler_vibe_isr_count();
    return;
  }
  if (now_us - win_us < (int64_t)VIBELOG_WIN_S * 1000000LL) goto flush;

  {
    const uint32_t p1 = btn_handler_vibe_poll_total();
    const uint32_t h1 = btn_handler_vibe_high_total();
    const uint32_t i1 = btn_handler_vibe_isr_count();
    const uint32_t dp = p1 - p0, dh = h1 - h0, di = i1 - i0;
    const bool mixed = (dp > 0 && dh > 0 && dh < dp);   /* ★실제 접점 동작 */
    const bool hb    = (now_us - last_rec_us) >= (int64_t)VIBELOG_HB_S * 1000000LL;
    if (mixed || hb) {
      _vibelog_add((int)(now_us / 1000000), dp, dh, di);
      last_rec_us = now_us;
      if (mixed)
        ESP_LOGW(TAG, "[VIBELOG] ★접점 동작 감지 — polls=%u high=%u isr=%u",
                 (unsigned)dp, (unsigned)dh, (unsigned)di);
    }
    win_us = now_us; p0 = p1; h0 = h1; i0 = i1;
  }

flush:
  if (s_vl_dirty && (now_us - s_vl_last_save_us) >= 10LL * 1000000LL) {
    _vibelog_save();
    s_vl_dirty = false;
    s_vl_last_save_us = now_us;
  }
}

/* 콘솔 `vl` — 저장된 진동 기록 출력. */

void somfy_app_vibelog_dump(void) {
  ESP_LOGW(TAG, "[VIBELOG] === 진동 기록 %u건 (창 %d초, heartbeat %d초) ===",
           (unsigned)s_vl_n, VIBELOG_WIN_S, VIBELOG_HB_S);
  ESP_LOGW(TAG, "[VIBELOG] 현재: 레벨=%d 고장판정=%d  누적 poll=%u high=%u isr=%u",
           btn_handler_vibe_level(), btn_handler_vibe_stuck() ? 1 : 0,
           (unsigned)btn_handler_vibe_poll_total(),
           (unsigned)btn_handler_vibe_high_total(),
           (unsigned)btn_handler_vibe_isr_count());
  if (s_vl_n == 0) { ESP_LOGW(TAG, "[VIBELOG] (비어 있음)"); return; }
  const uint16_t start = (s_vl_n < VIBELOG_MAX) ? 0
                       : (uint16_t)((s_vl_head + VIBELOG_MAX - s_vl_n) % VIBELOG_MAX);
  int mixed_cnt = 0;
  for (uint16_t i = 0; i < s_vl_n; i++) {
    const vibe_sample_t *e = &s_vl_buf[(start + i) % VIBELOG_MAX];
    const bool mixed = (e->polls > 0 && e->high > 0 && e->high < e->polls);
    if (mixed) mixed_cnt++;
    ESP_LOGW(TAG, "VL %3u  %6us  poll=%4u high=%4u isr=%5u  %s",
             (unsigned)i, (unsigned)e->t_s, (unsigned)e->polls,
             (unsigned)e->high, (unsigned)e->isr,
             mixed                      ? "★섞임(접점 동작)"
             : (e->high == 0)           ? "계속 LOW(접점 붙음/GND 단락)"
             : (e->high == e->polls)    ? "계속 HIGH(접점 안 닫힘)"
                                        : "-");
  }
  ESP_LOGW(TAG, "[VIBELOG] 섞인 창 %d/%u — 0 이면 센서가 한 번도 동작하지 않은 것",
           mixed_cnt, (unsigned)s_vl_n);
}

void somfy_app_vibelog_clear(void) {
  s_vl_n = 0; s_vl_head = 0;
  memset(s_vl_buf, 0, sizeof(s_vl_buf));
  _vibelog_save();
  ESP_LOGW(TAG, "[VIBELOG] 기록 삭제됨");
}
#else  /* !BOARD_VIBELOG_ENABLE — 링버퍼·기록 전부 제외(heap 확보) */
static void _vibelog_tick(int64_t now_us) { (void)now_us; }
static void _vibelog_load(void) {}
static void _vibelog_save(void) {}
void somfy_app_vibelog_dump(void) {
  ESP_LOGW(TAG, "[VIBELOG] 이 보드에서는 비활성(BOARD_VIBELOG_ENABLE=0 — heap 확보)");
}
void somfy_app_vibelog_clear(void) { somfy_app_vibelog_dump(); }
#endif /* BOARD_VIBELOG_ENABLE */

/* ═══ `~INT` 선 진단 — 고장 위치를 **소프트웨어만으로** 둘로 쪼갠다 ══════════════
 *
 *  배경: ②(PCF `~INT` 인터럽트)는 `intdiag` 실측으로 "선이 안 움직인다"까지 확인됐다
 *  (버튼 조작 중 290,023 표본 / 전이 0회). 하지만 그게 **PCF 쪽인지 배선 쪽인지**는
 *  구분되지 않았다.
 *
 *  회로(kicad/somfy_blinds_h4 PCB 에서 네트 추적으로 확인, 2026-08-14):
 *      U3 pad1(~INT, PCF8575DBR) ──┬── R3 10k ── +3V3
 *                                  └── U1 pad3 = GPIO2(D2)
 *      배선 실재: segment 20개 + via 2개
 *
 *  → GPIO2 에 **내부 풀다운(~45k)** 을 걸고 전압을 재면 R3 가 이 지점에서 보이는지
 *    알 수 있다:
 *      R3 10k 가 살아 있으면  V ≈ 3.3 × 45/(10+45) ≈ 2.7V  (HIGH)
 *      GPIO2 까지 안 닿으면   V ≈ 0V                        (LOW)
 *    ADC 로 실전압을 재므로 **풀업 저항값까지 역산**된다(Rpu = Rpd×(3.3-V)/V).
 *
 *  판정:
 *    · 풀다운인데 HIGH(≈2.7V) → GPIO2~R3 정상 → 고장은 **R3~U3 pad1 사이**
 *                                (0.65mm SSOP pad1 냉납 또는 PCF INT 출력 불량)
 *    · 풀다운에서 LOW(≈0V)    → **GPIO2 까지 배선/via 단선**
 *
 *  ※ADC1 은 BAT_ADC(GPIO1)·PCF 비트뱅(GPIO6)과 공유라 버튼 뮤텍스로 직렬화한다
 *    (안 하면 변환이 교란된다 — _read_bat_mv 주석 참조). */
static int _intpd_read_mv(adc_channel_t ch) {
  if (!s_bat_adc_ok) return -1;
  int raw = 0, sum = 0, n = 0;
  for (int i = 0; i < 8; i++)
    if (adc_oneshot_read(s_bat_adc, ch, &raw) == ESP_OK) { sum += raw; n++; }
  if (n == 0) return -1;
  raw = sum / n;
  int mv = 0;
  if (s_bat_cali && adc_cali_raw_to_voltage(s_bat_cali, raw, &mv) == ESP_OK)
    return mv;
  return (raw * 3100) / 4095;            /* 캘리 없으면 근사(ATTEN_12 ≈ 3.1V FS) */
}

void somfy_app_intpd_test(void) {
  const int pin = BOARD_PIN_PCF_INT;
  ESP_LOGW(TAG, "[INTPD] ================================================");
  ESP_LOGW(TAG, "[INTPD] `~INT`(IO%d) 풀다운 진단 — 버튼은 건드리지 마세요", pin);

  adc_unit_t unit; adc_channel_t ch;
  const bool adc_ok = (adc_oneshot_io_to_channel(pin, &unit, &ch) == ESP_OK)
                      && s_bat_adc_ok;
  if (adc_ok) {
    adc_oneshot_chan_cfg_t ccfg = { .atten = ADC_ATTEN_DB_12,
                                    .bitwidth = ADC_BITWIDTH_DEFAULT };
    if (adc_oneshot_config_channel(s_bat_adc, ch, &ccfg) != ESP_OK)
      ESP_LOGW(TAG, "[INTPD] ADC 채널 설정 실패 — 디지털 판독만 진행");
  } else {
    ESP_LOGW(TAG, "[INTPD] IO%d ADC 불가 — 디지털 판독만 진행", pin);
  }

  SemaphoreHandle_t mtx = btn_handler_get_i2c_mutex();
  const bool locked = (mtx && xSemaphoreTake(mtx, pdMS_TO_TICKS(500)) == pdTRUE);
  if (mtx && !locked)
    ESP_LOGW(TAG, "[INTPD] 버튼 뮤텍스 확보 실패 — 값이 흔들릴 수 있음");

  struct { const char *name; gpio_pull_mode_t pull; } steps[] = {
    { "floating(풀 없음)", GPIO_FLOATING     },
    { "내부 풀다운 ~45k ", GPIO_PULLDOWN_ONLY},
    { "내부 풀업   ~45k ", GPIO_PULLUP_ONLY  },
  };
  int mv_pd = -1;
  for (int i = 0; i < 3; i++) {
    gpio_set_direction(pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(pin, steps[i].pull);
    vTaskDelay(pdMS_TO_TICKS(50));            /* RC 안정 — 넉넉히 */
    int lv = gpio_get_level(pin);
    int mv = adc_ok ? _intpd_read_mv(ch) : -1;
    if (i == 1) mv_pd = mv;
    if (mv >= 0)
      ESP_LOGW(TAG, "[INTPD] %-18s 디지털=%s  ADC=%dmV", steps[i].name,
               lv ? "HIGH" : "LOW ", mv);
    else
      ESP_LOGW(TAG, "[INTPD] %-18s 디지털=%s", steps[i].name, lv ? "HIGH" : "LOW ");
  }

  /* 출력 LOW 로 몰아 readback — 3V3 단락이면 안 내려간다 */
  gpio_set_pull_mode(pin, GPIO_FLOATING);
  gpio_set_direction(pin, GPIO_MODE_OUTPUT);
  gpio_set_level(pin, 0);
  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT);   /* IN 레지스터 읽기용 */
  gpio_set_level(pin, 0);
  vTaskDelay(pdMS_TO_TICKS(20));
  ESP_LOGW(TAG, "[INTPD] 출력 LOW readback = %s%s", gpio_get_level(pin) ? "HIGH" : "LOW",
           gpio_get_level(pin) ? "  ★내려가지 않음 = 3V3 단락 의심" : "  (정상 — 싱크 가능)");

  /* 원복: 입력 + 풀업(평상시 `~INT` 감시 상태) */
  gpio_set_direction(pin, GPIO_MODE_INPUT);
  gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
  if (locked) xSemaphoreGive(mtx);

  ESP_LOGW(TAG, "[INTPD] ---- 판독 ----");
  if (mv_pd >= 0) {
    /* Rpu = Rpd × (VDD - V) / V,  Rpd ≈ 45k, VDD = 3300mV */
    if (mv_pd > 50 && mv_pd < 3250) {
      const int rpu_k = (45 * (3300 - mv_pd)) / mv_pd;
      ESP_LOGW(TAG, "[INTPD] 풀다운 시 %dmV → 상단 풀업 ≈ %dk 로 환산됨", mv_pd, rpu_k);
      if (rpu_k <= 25)
        ESP_LOGW(TAG, "[INTPD] → R3 10k 가 GPIO2 에서 **보인다**. 배선 정상."
                      " 고장은 **R3~U3 pad1 구간**(pad1 냉납 또는 PCF INT 출력 불량)");
      else
        ESP_LOGW(TAG, "[INTPD] → 풀업이 예상(10k)보다 훨씬 약하다. 접촉 불량 의심");
    } else if (mv_pd <= 50) {
      ESP_LOGW(TAG, "[INTPD] 풀다운 시 %dmV(≈0V) → R3 가 **안 보인다**"
                    " = GPIO2 까지 배선/via 단선", mv_pd);
    } else {
      ESP_LOGW(TAG, "[INTPD] 풀다운인데 %dmV(거의 3V3) → 강한 소스가 물려 있음(단락 의심)",
               mv_pd);
    }
  } else {
    ESP_LOGW(TAG, "[INTPD] (ADC 없이) 풀다운에서 HIGH=배선정상/PCF측 고장,"
                  " LOW=GPIO2 배선 단선");
  }
  ESP_LOGW(TAG, "[INTPD] ================================================");
}

/* 콘솔 `usbsim off|on` — 배터리 모드 시뮬레이션 토글. */
void somfy_app_console_usbsim(int off) {
  s_usb_sim_off = (off != 0);
  ESP_LOGW(TAG, "[USBSIM] %s — 이제 %s 모드로 동작한다 (PM 은 물리 VBUS 기준 유지)",
           s_usb_sim_off ? "켬" : "끔", s_usb_sim_off ? "배터리" : "USB");
}


#define BAT_DISP_WIN 9
/* ★2026-08-12 하한(B)·세션 기준점(C)을 걸기 전 필요한 최소 표본수.
 *  중앙값이 이상치 2개를 버틸 수 있는 최소값 = 창의 절반+1 = 5 (=25초).
 *  왜 1이 아닌가: 첫 표본 하나로 하한을 잡으면 그게 저전압 무리에 걸렸을 때
 *  **세션 내내 5%p 낮게 고정**된다(sim/tools/bat_pct_monotone_sim.py [2][4] 가
 *  실제로 이 결함을 잡아냈다). 대가는 분리 후 25초간 하한이 없다는 것뿐이다. */
#define BAT_FLOOR_MIN_N (BAT_DISP_WIN / 2 + 1)
static int      s_bat_hist[BAT_DISP_WIN];
static int      s_bat_hist_n = 0, s_bat_hist_i = 0;
static int32_t  s_bat_ema_q4 = 0;         /* 1/16 mV 단위 */
static uint32_t s_bat_sm_seq = 0;         /* 평활 호출 횟수 — 세션 기준점 대기에 쓴다 */

/* ★2026-08-12 평활 상태 초기화 — 전원이 바뀌면 이전 전압을 끌고 오면 안 된다.
 *
 *  버그였다: USB 를 빼는 순간의 평활값은 **충전 중 단자전압**(실측 4056mV)이지
 *  배터리 OCV 가 아니다. 그대로 두면 중앙값 창 9개가 다 빠질 때까지(45초) + EMA
 *  지연까지 **약 90초 동안 92% → 73% 로 흘러내렸다**. 그리고 방전 로거가 이 정착
 *  기울기를 진짜 방전으로 착각해 평균 **302~819mA** 를 찍었다(700mAh 셀에서
 *  물리적으로 불가능한 값 — 실측 NVS 세션 #16).
 *  → 전환 시 창·EMA 를 비워 **다음 실측 1회로 재시드**한다(수렴 5초).
 *  USB 재연결 때도 대칭으로 부른다(계단 크기가 같아 같은 지연이 생기므로). */
static void _bat_smooth_reset(void) {
  s_bat_hist_n = 0;
  s_bat_hist_i = 0;
  s_bat_ema_q4 = 0;
}

static int _bat_smooth_mv(int mv) {
  s_bat_sm_seq++;
  s_bat_hist[s_bat_hist_i] = mv;
  s_bat_hist_i = (s_bat_hist_i + 1) % BAT_DISP_WIN;
  if (s_bat_hist_n < BAT_DISP_WIN) s_bat_hist_n++;

  int t[BAT_DISP_WIN];
  for (int i = 0; i < s_bat_hist_n; i++) t[i] = s_bat_hist[i];
  for (int i = 1; i < s_bat_hist_n; i++) {          /* 삽입정렬(최대 5개) */
    int v = t[i], j = i - 1;
    while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
    t[j + 1] = v;
  }
  int med = t[s_bat_hist_n / 2];

  if (s_bat_ema_q4 == 0) s_bat_ema_q4 = med * 16;
  else                   s_bat_ema_q4 += ((int32_t)med * 16 - s_bat_ema_q4) / 4;
  return (int)(s_bat_ema_q4 / 16);
}

/* ★2026-08-12 배터리 구동 중 표시 % 단조 비증가 (하한) ─────────────────────
 *
 *  신고: USB 분리 후 좌/우 버튼을 누르는 동안 68 → 69 → 70 → 71 % 로 **올라갔다**.
 *  방전 중 잔량이 느는 건 물리적으로 불가능하다.
 *
 *  원인(NVS 방전기록 세션 #16, 290건 실측): 배터리 구동일 때 원본 전압이 **두 무리**
 *  로 갈라진다 — 3902~3924(9개, 평균 3913) / 3932~3958(11개, 평균 3944), 간격 31mV,
 *  중간값이 거의 없다. 같은 기기가 USB 연결 중엔 4060~4068(±4mV = ADC 2카운트)로
 *  미동도 없으므로 ADC 자체는 멀쩡하고 **배터리 구동일 때만** 갈라진다.
 *  이 구간 OCV 기울기가 6.5mV/%p 라 31mV = 약 5%p.
 *  중앙값-9 는 최근 9개 중 어느 무리가 5개를 넘느냐로 출력이 정해지므로, 비율이
 *  서서히 바뀌면 출력이 31mV 씩 **툭 튀고**, EMA(α=1/4)가 그 계단을 부드럽게 따라가
 *  **단조 상승**처럼 보인다(시뮬레이터로 69→74 재현, 신고와 같은 형태).
 *
 *  두 무리의 정체는 아직 미확정이다(→ _read_bat_mv 의 표본 편차 진단 참고).
 *  다만 정체와 무관하게 **표시가 오르면 안 된다**는 건 확정이므로 하한을 건다.
 *  USB 연결 중에는 충전이라 상승이 정상이므로 하한을 푼다.
 *  검증: sim/tools/bat_pct_monotone_sim.py (5개 항목 전부 통과)
 *  ※실제 선언은 BAT_PCT_UNKNOWN 정의 뒤(아래 _nobat 블록)에 있다. */

/* ─── 배터리 미연결 판정 (5분 창마다 주기 재판정) ─────────────────────────────
 *  XIAO 는 **배터리 감지 신호가 없다**(충전 status NCHG 는 온보드 LED 전용이라
 *  어떤 패드에도 안 나옴 — wiring_xiao-c6.md). 그런데 배터리를 빼도 충전 IC 가
 *  BAT+ 를 무부하로 띄워 **약 3970mV** 가 읽히고, 이는 위 OCV 곡선상 정확히
 *  **78%** 라 "배터리 없는데 78%" 로 보인다(2026-07-17 실측).
 *
 *  전압만으론 무배터리(3970mV)와 실제 배터리 78% 가 겹쳐 구분 불가.
 *  유일한 단서는 **전압이 오르는가**:
 *    · 무배터리   : 충전 IC float → 안 오름
 *    · 충전 중 셀 : CC 충전이라 계속 오름
 *  ★min/max 폭으로 재면 안 된다(실측으로 폐기): 무배터리 ADC 노이즈가 Δ10mV
 *    (3968~3978)라 700mAh 셀의 5분 상승분(≈9.3mV)과 겹쳐 오판한다.
 *    → **전반부 평균 vs 후반부 평균**으로 재면 노이즈가 √N 상쇄(30샘플 → 약 2mV):
 *        무배터리 ≈ 0~2mV / 충전 중 ≈ 9mV → 문턱 4mV 로 깨끗이 갈린다.
 *    (검증: 무배터리에서 전반3971 후반3970, 상승 -1mV → "미연결" 정확 판정)
 *  애매하면 "배터리 있음"(=% 표시) 쪽으로 틀린다(안전한 실패 방향).            */
#define BAT_NOBAT_LO        3940   /* 무배터리 float 창 하한 mV (실측 3968~3978) */
#define BAT_NOBAT_HI        4010   /* 상한 mV */
#define BAT_NOBAT_RISE_MV      4   /* 전반→후반 평균 상승이 이 미만이면 "안 오름" */
#define BAT_NOBAT_WINDOW_MS 300000 /* 5분 관찰 후 1회 판정 */
#define BAT_NOBAT_MIN_N        3   /* 전/후반 각 최소 표본 수 */

static bool    s_nobat         = false;   /* true = 배터리 미연결 → 0% 표시 */
/* ★2026-08-11 첫 판정 완료 여부. 판정 전에는 **% 를 표시하지 않는다**(사용자 선택).
 *
 *  왜: 배터리를 빼면 충전 IC 가 BAT+ 를 3970mV 로 띄우는데, 이 값이 OCV 곡선상
 *  정확히 **78%** 라 "배터리 없는데 78%" 로 보인다. 전압만으론 실제 78% 와 구분이
 *  불가능하고, 구분 단서인 "전압이 오르는가"는 창(5분)이 차야 판정된다.
 *  → 판정 전에 78% 를 보여주느니 `--%` 로 두는 편이 정직하다.
 *  ※창을 줄이면 안 된다: 충전 상승률이 ≈9.3mV/5분 이라 창이 2분 10초보다 짧으면
 *    **진짜 충전 중인 배터리도 "안 오름"으로 오판**해 0% 로 표시된다(문턱 4mV). */
static bool    s_nobat_judged   = false;
#define BAT_PCT_UNKNOWN 255               /* >100 = 표시측이 "--%" 로 렌더 */
/* ★2026-08-12 방전 중 표시 % 하한 (근거는 _bat_smooth_mv 위 주석 참고).
 *  255 = 미설정. USB 연결 중에는 충전이므로 255 로 되돌려 상승을 허용한다. */
static uint8_t s_pct_floor     = BAT_PCT_UNKNOWN;
/* ★★2026-08-20 하한 **조건부 해제** — 잠김 사고 대응.
 *  신고: COM7 C6 장시간 방치 후 잔량 0%. NVS 방전기록(세션 #2, 300건/13.5시간)이
 *  원인을 그대로 보여준다.
 *      #0~39    4085 → 4068 mV   95%
 *      #40~59   4069 → 2651 mV    0%   ← 급락
 *      #60~99   2129 mV 바닥       0%
 *      #100~299 2656 → 3845 mV    0%   ← 8시간에 걸쳐 회복해도 0% 고정
 *  하한이 단조 비증가라 **한 번 0% 가 되면 세션 내내 못 풀린다**. 원래 의도는
 *  "31mV 두 무리" 때문에 표시가 야금야금 오르는 것을 막는 것인데, 센서가 크게
 *  거짓말했을 때의 탈출구가 없었다.
 *  ※낮은 값이 측정 오류라는 근거: (a) 사용자 확인 — 그동안 USB 를 꽂은 적이 없다.
 *    충전 없이 전압이 오르는 건 물리적으로 불가능하다. (b) 기기가 계속 돌았다 —
 *    셀이 2.1V 면 LDO 드롭아웃으로 죽는다. (c) ADC 산포는 전 구간 2~3카운트로
 *    깨끗했다(잡음 아님). **기전은 아직 미확정** — 아래 해제 로그가 다음 단서다.
 *  → 하한을 건 시점의 전압보다 RELEASE_MV 이상 높은 값이 RELEASE_N 표본 연속이면
 *    해제한다. 잡음 무리(31mV)의 3배 이상이라 원래 막으려던 상승은 그대로 막힌다.
 *  검증: sim/tools/bat_floor_release_sim.py — 실측 300건 입력 시 최종 0% → 35%. */
#define BAT_FLOOR_RELEASE_MV 100   /* 잡음 무리 31mV 의 3배 초과 */
#define BAT_FLOOR_RELEASE_N    5   /* 5초 주기 × 5 = 25초 연속 유지 */
static int     s_pct_floor_mv  = 0;    /* 하한을 건 시점의 평활 전압 */
static uint8_t s_floor_up_run  = 0;    /* 연속 상승 표본 수 */
/* ★2026-08-16 (③) 충전 중 표시% 천장 — 방전 중 하한(s_pct_floor)의 대칭.
 *  milli-percent(0~100000) 로 들고 있어야 분 단위 상승률이 정수 절사로 죽지 않는다.
 *  -1 = 비활성(방전 중). 자세한 배경은 사용 지점 주석 참조. */
static int     s_pct_ceil_mpct = -1;
static int64_t s_pct_ceil_us   = 0;
#ifndef BAT_CHG_RISE_MPCT_PER_MIN
/* ★2026-08-23 급락 표본 거부 임계 (사용 지점 주석 참조).
 *  300mV: 실측 급락(1,053mV·1,418mV)은 잡고, 정상 방전(5초당 수 mV)과
 *  USB 연결/해제로 단자전압이 움직이는 폭(≤120mV)은 통과시킨다.
 *  3회: 측정 주기 5초 × 3 = 15초. 진짜 급락은 그 뒤 인정된다. */
/* ★2026-08-27 표시 % 1회 측정(5초)당 최대 하락 %p (사용 지점 주석 참조).
 *  1 = 실측 방전(0.0014%p/5초)의 약 700배 — 진짜 방전은 전혀 안 막힌다. */
#define BAT_DISP_MAX_DROP_PCT 1
#define BAT_SPIKE_MAX_DROP_MV  300
#define BAT_SPIKE_REJECT_MAX     3
#define BAT_CHG_RISE_MPCT_PER_MIN 850   /* 0.85%p/분 = 700mAh 를 약 2시간에 충전 */
#endif
static int64_t s_nobat_t0_us   = 0;
static int32_t s_nobat_sum1 = 0, s_nobat_sum2 = 0;   /* 전반/후반 전압 합 */
static int16_t s_nobat_n1   = 0, s_nobat_n2   = 0;   /* 전반/후반 표본 수 */

/* 배터리 블록(5초 주기)에서 호출. 창의 전반/후반 평균을 비교해 판정하고,
 * ★창이 찰 때마다 **다시 판정한다**(주기 재판정, 사용자 선택 2026-07-17).
 *   → 켜둔 채로 배터리를 빼면 최대 2창(≈10분) 안에 0% 로 바뀐다. 부팅 1회 방식은
 *     다음 부팅까지 반영이 안 돼 테스트/실사용 모두 불편했다.
 *   ※재판정의 유일한 위험: 충전이 끝나 전압이 평평해졌는데 그 값이 마침 무배터리
 *     창(3940~4010mV) 안이면 오판. 실제 만충은 4.15~4.2V 라 창 밖이므로 안전하다. */
static void _nobat_track(int mv) {
  if (mv <= 0) return;
  int64_t now = esp_timer_get_time();
  if (s_nobat_t0_us == 0) s_nobat_t0_us = now;

  const int64_t elapsed = now - s_nobat_t0_us;
  const int64_t win     = (int64_t)BAT_NOBAT_WINDOW_MS * 1000;
  if (elapsed < win / 2) { s_nobat_sum1 += mv; s_nobat_n1++; return; }   /* 전반 */
  if (elapsed < win)     { s_nobat_sum2 += mv; s_nobat_n2++; return; }   /* 후반 */

  if (s_nobat_n1 >= BAT_NOBAT_MIN_N && s_nobat_n2 >= BAT_NOBAT_MIN_N) {
    const int m1   = s_nobat_sum1 / s_nobat_n1;
    const int m2   = s_nobat_sum2 / s_nobat_n2;
    const int rise = m2 - m1;
    /* 충전 IC float 창 안 + 전압이 안 오름 + USB 연결 = 배터리 없음.
     * (USB 가 없으면 기기가 배터리로 도는 중이므로 배터리는 반드시 있다.) */
    const bool in_window = (m1 >= BAT_NOBAT_LO && m1 <= BAT_NOBAT_HI &&
                            m2 >= BAT_NOBAT_LO && m2 <= BAT_NOBAT_HI);
    const bool flat      = (rise < BAT_NOBAT_RISE_MV);
    const bool nobat     = (in_window && flat && _is_usb_powered());
    static bool s_logged_once = false;
    if (nobat != s_nobat || !s_logged_once) {     /* 바뀔 때(+최초)만 로그 — 폭주 방지 */
      ESP_LOGW(TAG, "[BAT] 배터리 판정: %s — 전반%dmV(n%d) 후반%dmV(n%d) 상승%dmV "
                    "(창%s, 안오름%s, USB=%d)",
               nobat ? "미연결 → 0%% 표시" : "연결됨 → 실측 %% 표시",
               m1, s_nobat_n1, m2, s_nobat_n2, rise,
               in_window ? "O" : "X", flat ? "O" : "X", _is_usb_powered());
      s_logged_once = true;
    }
    s_nobat = nobat;
    s_nobat_judged = true;      /* ★이제부터 % 를 표시해도 된다 */
  }
  /* ★다음 창을 새로 시작 (주기 재판정) */
  s_nobat_t0_us = now;
  s_nobat_sum1 = s_nobat_sum2 = 0;
  s_nobat_n1   = s_nobat_n2   = 0;
}
#else  /* !BOARD_HAS_BAT_ADC — 배터리/충전 측정 회로가 없는 보드 ─────────────
 *  ★2026-08-16 이 #else 가 없어서 BOARD_HAS_BAT_ADC=0 은 **컴파일조차 안 됐다**
 *    (implicit declaration of '_batlog_load' / 's_bat_sm_seq' undeclared …).
 *    배터리 기계는 전부 위 #if 안에 있는데 호출부는 가드 밖이었기 때문이다.
 *    기본값이 0 인 gnpe-c6 도 같은 이유로 깨져 있었다.
 *  → 밖에서 부르는 심볼만 무해한 스텁으로 채운다. 회로가 없으면 방전 세션·
 *    잔량% 자체가 의미 없으므로 전부 no-op 이 정답이다. */
static void     _batlog_load(void) {}
static void     _vibelog_load(void) {}
static bool     _batlog_try_resume(bool on_battery) { (void)on_battery; return false; }
static void     _batlog_new_session(void) {}
static void     _batlog_ls_session_begin(bool resume) { (void)resume; }
static void     _batlog_flush_if_due(int64_t now_us, bool force) { (void)now_us; (void)force; }
static void     _batlog_add_ev(int t_s, int mv, int pct, bool radio, bool panel, int pm, int ev)
                { (void)t_s; (void)mv; (void)pct; (void)radio; (void)panel; (void)pm; (void)ev; }
static void     _bat_smooth_reset(void) {}
static uint32_t s_bat_sm_seq      = 0;
static int      s_bat_hist_n      = 0;
void somfy_app_batlog_button(int ev) { (void)ev; }
void somfy_app_batlog_dump(void)  { ESP_LOGW(TAG, "[BATLOG] 이 보드에는 배터리 측정 회로가 없다"); }
void somfy_app_batlog_clear(void) { somfy_app_batlog_dump(); }
void somfy_app_vibelog_dump(void)  { ESP_LOGW(TAG, "[VIBELOG] 이 보드에서는 비활성"); }
void somfy_app_vibelog_clear(void) { somfy_app_vibelog_dump(); }
/* 콘솔 명령이 링크 시 참조한다(app_main.cpp). 회로가 없으니 안내만 한다. */
void somfy_app_console_usbsim(int off) { (void)off;
  ESP_LOGW(TAG, "[USBSIM] 이 보드에는 배터리/충전 회로가 없어 의미 없다"); }
void somfy_app_intpd_test(void) {
  ESP_LOGW(TAG, "[INTPD] 이 보드에는 배터리/충전 회로가 없어 의미 없다"); }
#endif /* BOARD_HAS_BAT_ADC */

static uint8_t _estimate_battery_percent(void) {
#if BOARD_HAS_BAT_ADC
  int _mv = _read_bat_mv();
  if (_mv > 0) return _bat_mv_to_pct(_mv);   /* 실측 우선 (실패 시 아래 시간기반) */
#endif
  if (s_chg_start_us == 0) {
    s_chg_start_us = esp_timer_get_time();
    return 5; /* 초기값 */
  }
  int64_t elapsed_ms = (esp_timer_get_time() - s_chg_start_us) / 1000;
  if (elapsed_ms <= 0)
    return 5;
  if (elapsed_ms >= CHG_FULL_DURATION_MS)
    return 100;
  /* 5% → 100% 선형 보간 */
  int pct = 5 + (int)((elapsed_ms * 95) / CHG_FULL_DURATION_MS);
  if (pct > 100)
    pct = 100;
  if (pct < 5)
    pct = 5;
  return (uint8_t)pct;
}

/* ═══════════════════════════════════════════════
   절전 모드 진입 / 복귀
   ──────────────────────────────────────────────
   ★ v2.0 — 핀 재배치 (CC1101 SPI: IO4/5/6/7, LP_I2C: IO18/19 bit-bang).
     PCF8574 ~INT 는 IO17 로 이동 (구 IO2). IO2 는 미사용.
     light sleep wake sources (v2.0):
       • IO17 = PCF8574 ~INT  → 모든 P 변화 (UP/DOWN/SEL/PROG/SETUP/ROT 전부)
       • IO16 = VIBRATION VS1 → 진동 감지로도 wake
       • IO3  = CHG_STAT      → USB 연결로도 wake (충전 시작 즉시 화면 ON)
     timer wake 는 더 이상 필요 없음 (모든 신호가 GPIO wake source 로 커버).
═══════════════════════════════════════════════ */
/* ═══════════════════════════════════════════════
   화면 보호기 모드 (v3.2+) — USB 연결 시 사용
   ──────────────────────────────────────────────
   OLED 패널만 0xAE 로 OFF, CPU/Thread/Matter 정상 동작.
   버튼/Matter 명령 수신 시 즉시 해제됨.
═══════════════════════════════════════════════ */
static void _enter_screensaver(void) {
  if (s_screensaver_active) return;
  s_screensaver_active = true;
  ESP_LOGI(TAG, "화면 보호기 진입 (USB 연결 — 절전 미사용, Matter 수신 정상)");
  /* 설정 모드 중이면 OLED state 그대로 두고 패널만 꺼서 깨어날 때 복귀 가능 */
  if (s_setup_screen == SETUP_NONE) {
    s_ui.state = OLED_STATE_SCREENSAVER;
  }
  oled_ui_set_display_on(false);
}

static void _exit_screensaver(const char *reason) {
  /* ★ 두 가지 stuck 케이스도 처리:
   *   1. s_screensaver_active=true (정상 wake)
   *   2. s_screensaver_active=false 인데 OLED state 가 SCREENSAVER (직전 wake
   *      가 OLED I2C 실패 등으로 패널을 못 켰을 때 재시도). */
  bool was_active = s_screensaver_active;
  bool oled_stuck = (s_ui.state == OLED_STATE_SCREENSAVER);
  if (!was_active && !oled_stuck) return;          /* 이미 깨어있고 정상 */
  s_screensaver_active = false;
  /* OLED state 복원 — 설정 모드 중이면 해당 화면으로, 아니면 NORMAL */
  switch (s_setup_screen) {
    case SETUP_MENU_SCR:     s_ui.state = OLED_STATE_SETUP_MENU; break;
    case SETUP_FREQ_EDIT:    s_ui.state = OLED_STATE_FREQ_EDIT;  break;
    case SETUP_TIME_EDIT:    s_ui.state = OLED_STATE_TIME_EDIT;  break;
    case SETUP_MATTER_PAIR:  s_ui.state = OLED_STATE_PAIRING;    break;
    case SETUP_THREAD_RESET: s_ui.state = OLED_STATE_THREAD_RESET;break;
    case SETUP_FW_UPDATE:    s_ui.state = OLED_STATE_FW_UPDATE;   break;
    case SETUP_NONE:
    default:                 s_ui.state = OLED_STATE_NORMAL;     break;
  }
  s_ui.anim_frame++;
  /* ★ 패널 켜기 전에 새 화면을 framebuffer 에 미리 렌더 → 켜진 순간 잔상
   *  (스크린세이버) 안 보이고 곧장 메인 화면이 나온다. */
  if (s_setup_screen == SETUP_NONE) {
    oled_ui_render_main_once(&s_ui);
  }
  oled_ui_set_display_on(true);
  if (was_active) {
    ESP_LOGI(TAG, "화면 보호기 해제 (%s)", reason ? reason : "");
  } else {
    ESP_LOGW(TAG, "화면 보호기 강제 재해제 (%s, OLED state 가 SCREENSAVER 였음)",
             reason ? reason : "");
  }
}

/* ═══════════════════════════════════════════════
   절전 모드 진입 / 종료 (v3.3+ — PM 자동 light sleep 사용)
   ──────────────────────────────────────────────
   변경 사항 (v3.2 → v3.3):
     • esp_light_sleep_start() 직접 호출 제거.
     • CONFIG_PM_ENABLE + CONFIG_FREERTOS_USE_TICKLESS_IDLE 활성화 → CPU
       가 모든 task idle 시 자동으로 light sleep 진입. Thread SED 가
       slow-poll 마다 CPU 깨워 부모 노드에서 큐된 Matter 명령 수신.
     • 따라서 절전 진입 = OLED panel OFF + 상태 플래그 갱신 만 수행.
       실제 전력 절감은 PM 이 자동으로 처리.
     • Wake 트리거:
        - 버튼 (PCF8574 ~INT → IO17): _btn_event_cb 가 활동 표시
        - SmartThings 명령: _matter_action_cb 가 활동 표시 (sleep 중에도
          Thread SED 가 수신 → callback 실행)
        - 진동 (VIBE → IO16), 충전 시작 (CHG_STAT → IO3): main 루프 폴링
═══════════════════════════════════════════════ */
static void _enter_sleep(void) {
  if (s_is_sleeping) return;
  s_is_sleeping = true;

  ESP_LOGI(TAG, "절전 모드 진입 — OLED off, PM auto-light-sleep 활성 "
                "(Thread SED 가 slow-poll=5s 로 Matter 수신 유지)");

  /* OLED panel off — 화면만 끄고 state 는 보존 (설정 모드 등 복귀용) */
  if (s_setup_screen == SETUP_NONE) {
    s_ui.state = OLED_STATE_SCREENSAVER;
  }
  oled_ui_set_display_on(false);
}

/* sleep 모드 탈출 — 버튼/Matter 명령 등이 활동을 표시했을 때 호출 */
static void _exit_sleep(const char *reason) {
  if (!s_is_sleeping) return;
  s_is_sleeping = false;

  switch (s_setup_screen) {
    case SETUP_MENU_SCR:     s_ui.state = OLED_STATE_SETUP_MENU; break;
    case SETUP_FREQ_EDIT:    s_ui.state = OLED_STATE_FREQ_EDIT;  break;
    case SETUP_TIME_EDIT:    s_ui.state = OLED_STATE_TIME_EDIT;  break;
    case SETUP_MATTER_PAIR:  s_ui.state = OLED_STATE_PAIRING;    break;
    case SETUP_THREAD_RESET: s_ui.state = OLED_STATE_THREAD_RESET;break;
    case SETUP_FW_UPDATE:    s_ui.state = OLED_STATE_FW_UPDATE;   break;
    case SETUP_NONE:
    default:                 s_ui.state = OLED_STATE_NORMAL;     break;
  }
  s_ui.anim_frame++;
  /* ★ 패널 켜기 전 메인 미리 렌더 — 스크린세이버 잔상 안 보이게. */
  if (s_setup_screen == SETUP_NONE) {
    oled_ui_render_main_once(&s_ui);
  }
  oled_ui_set_display_on(true);
  _wake_time_task();   /* 시계 태스크 즉시 깨움 — 5분 대기 중이어도 바로 갱신 */
  _mark_activity();
  ESP_LOGI(TAG, "절전 모드 해제 (%s)", reason ? reason : "");
}

/* ═══════════════════════════════════════════════
   app_main
═══════════════════════════════════════════════ */
/* app_main.cpp(Matter 코어)가 esp_matter::start() 이후 본 함수를 별도
   태스크로 호출한다. RF(cc1101/somfy)·blind_manager 초기화와 Matter 시작은
   app_main.cpp 가 단일 소유하므로 여기서 재실행하지 않는다(중복 SPI
   init/abort 방지). 본 함수는 OLED/버튼/시계/메뉴 등 주변 애플리케이션만
   담당하며, 무거운 연속 태스크는 커미셔닝 완료 후 시작한다. */
/* ─── 패닉 진단용 RTC 메모리 브레드크럼 (소프트 리셋 살아남음) ──────
 *  매 메인 루프 tick 마다 main_loop_ticks 증가 + last_main_tick_us 갱신.
 *  btn_task 도 별도 카운터/타임스탬프 갱신.
 *  부팅 시 esp_reset_reason() 이 PANIC/WDT 면 이 값들을 로그로 덤프 →
 *  어느 task 가 어디서 멈췄는지 추정 가능. */
typedef struct {
    uint32_t magic;              /* 검증 시그니처 (RTC 노이즈 vs 유효 구분) */
    uint32_t boot_count;
    uint32_t main_loop_ticks;
    int64_t  last_main_tick_us;
    uint32_t btn_task_ticks;
    int64_t  last_btn_tick_us;
    uint8_t  last_screensaver_active;
    uint8_t  last_sleeping;
    uint8_t  last_setup_screen;
    uint8_t  pad;
    int64_t  last_activity_us;
} crash_breadcrumb_t;

#define BREADCRUMB_MAGIC  0xCB1E0001u
static RTC_NOINIT_ATTR crash_breadcrumb_t s_bc;

static void _crash_breadcrumb_init(void) {
    esp_reset_reason_t r = esp_reset_reason();
    bool was_crash = (r == ESP_RST_PANIC || r == ESP_RST_TASK_WDT ||
                      r == ESP_RST_INT_WDT || r == ESP_RST_WDT);
    bool valid = (s_bc.magic == BREADCRUMB_MAGIC);
    if (was_crash && valid) {
        ESP_LOGE(TAG, "╔══════════════════════════════════════════════════╗");
        ESP_LOGE(TAG, "║ ★ 이전 부팅 비정상 종료 — reset_reason=%d         ║", (int)r);
        ESP_LOGE(TAG, "║   PANIC=3 INT_WDT=4 TASK_WDT=5 WDT=7              ║");
        ESP_LOGE(TAG, "╠══════════════════════════════════════════════════╣");
        ESP_LOGE(TAG, "║ boot_count        : %u", (unsigned)s_bc.boot_count);
        ESP_LOGE(TAG, "║ main_loop_ticks   : %u", (unsigned)s_bc.main_loop_ticks);
        ESP_LOGE(TAG, "║ last_main_tick_us : %lld", (long long)s_bc.last_main_tick_us);
        ESP_LOGE(TAG, "║ btn_task_ticks    : %u", (unsigned)s_bc.btn_task_ticks);
        ESP_LOGE(TAG, "║ last_btn_tick_us  : %lld", (long long)s_bc.last_btn_tick_us);
        ESP_LOGE(TAG, "║ last_activity_us  : %lld (idle %lld s)",
                 (long long)s_bc.last_activity_us,
                 (long long)((s_bc.last_main_tick_us - s_bc.last_activity_us) / 1000000));
        ESP_LOGE(TAG, "║ screensaver=%d sleep=%d setup=%d",
                 s_bc.last_screensaver_active, s_bc.last_sleeping, s_bc.last_setup_screen);
        ESP_LOGE(TAG, "╚══════════════════════════════════════════════════╝");
    } else if (was_crash && !valid) {
        ESP_LOGW(TAG, "이전 부팅 비정상 (r=%d) — breadcrumb 없음(첫 panic)", (int)r);
    } else {
        ESP_LOGI(TAG, "boot reason=%d (정상)", (int)r);
    }
    if (!valid) {
        memset(&s_bc, 0, sizeof(s_bc));
        s_bc.magic = BREADCRUMB_MAGIC;
    }
    s_bc.boot_count++;
}

/* btn_handler.c 에서 호출하기 위한 weak 약속 — breadcrumb update */
void somfy_app_bc_btn_tick(void) {
    s_bc.btn_task_ticks++;
    s_bc.last_btn_tick_us = esp_timer_get_time();
}

void somfy_app_run(void *arg) {
  (void)arg;
  boot_diag_stage2(BOOT_S2_RUN_ENTRY);
  app_log_init();   /* 전역 로그 레벨 적용 (esp_log) */
  _crash_breadcrumb_init();
#ifdef SOMFY_SELFTEST
  /* 가상 테스트 빌드: 부팅 직후 1회 실행(온에어/CC1101 불필요). */
  somfy_selftest_run();
#endif
#ifdef SOMFY_ONAIR_TEST
  /* 온에어 RF 테스트 빌드: 부팅 직후 1회 실제 CC1101 송신 검증.
   *  app_main.cpp Phase2 가 g_cc1101/g_somfy/g_rf_ready 를 이미 초기화함. */
  somfy_onair_test_run();
#endif
#ifdef SOMFY_STRESS_TEST
  /* 스트레스 빌드: 부팅 직후 무한 증가 실제 RF 송신(반환 안 함). */
  somfy_stress_test_run();
#endif
#ifdef SOMFY_RXDECODE_TEST
  /* RX OOK 디코더 빌드: 부팅 직후 무한 펄스 캡처/분석(반환 안 함). */
  somfy_rxdecode_test_run();
#endif
#ifdef SOMFY_TXPROBE_TEST
  /* TX 자가진단 빌드: 부팅 직후 1회 PROG 송신+GD0 캡처/분석. */
  somfy_txprobe_test_run();
#endif
#ifdef SOMFY_RXBYTE_TEST
  /* 정품 리모컨 7바이트 디코더: 부팅 직후 무한 캡처/디코드(반환 안 함). */
  somfy_rxbyte_test_run();
#endif
#ifdef SOMFY_TXDECODE_TEST
  /* 우리 TX 송신을 GD0 직접 캡처 → 7바이트 디코드 → 기대값 비교(반환 안 함). */
  somfy_txdecode_test_run();
#endif
#ifdef SOMFY_CWTEST_TEST
  /* CW(순수 carrier) 송신 — SDR 로 carrier 안정성 확인(반환 안 함). */
  somfy_cwtest_test_run();
#endif
  ESP_LOGI(TAG, "╔════════════════════════════════════════╗");
  ESP_LOGI(TAG, "║  Somfy RTS Blind Controller  v3.5      ║");
  ESP_LOGI(TAG, "║  ESP32-C6 + CC1101 + Matter/Thread     ║");
  ESP_LOGI(TAG, "╚════════════════════════════════════════╝");

  /* ── 0. RTC를 빌드(=PC) 시각으로 초기화 (SNTP 전 fallback) ── */
  _init_rtc_from_build_time();
  boot_diag_stage2(BOOT_S2_RTC_INIT);

  /* ★2026-08-11 배터리 ADC 를 **부팅 초반으로 앞당긴다**(원래는 버튼 init 뒤).
   *  이유: "USB 없이 배터리만 연결하면 부팅 중 반복 재부팅" 진단에서, 실패 기록의
   *  전압이 항상 -1(미측정)이었다. 첫 측정이 메인 루프(5초)라 그 전에 죽으면
   *  전압 증거가 하나도 안 남는다. 여기서 재면 **붕괴 직전 전압**을 붙잡는다.
   *  안전성: _read_bat_mv 가 쓰는 두 뮤텍스는 아직 NULL 이라 잠금을 건너뛴다
   *  (btn_handler_get_i2c_mutex → NULL, oled_ui_i2c_trylock → true 반환). */
#if BOARD_HAS_BAT_ADC
  _bat_adc_init();
  _ls_stats_init();     /* ★light sleep 실측 — 배터리 로그에 실어 NVS 로 남긴다 */
  {
    int _mv0 = _read_bat_mv();
    if (_mv0 > 0) {
      boot_diag_set_bat_mv(_mv0);
      ESP_LOGW(TAG, "[BAT] 부팅 초기 전압 %dmV", _mv0);
    }
  }
#endif

  /* ── 1~3. blind_manager/CC1101/Somfy 는 app_main.cpp 가 소유·초기화 완료.
   *  여기서 재초기화하지 않는다(공유 인스턴스 g_mgr/g_somfy/g_cc1101). */
  ESP_LOGI(TAG, "블라인드 %d개 (app_main.cpp 공유)", s_mgr.count);

  /* ── 4. OLED UI 컨텍스트/하드웨어 init (1회성, 연속 태스크는 아직 X) ──
   *  ★★ v3.6 핵심 수정 (격리 검증으로 확정한 페어링 실패 근본 원인):
   *  oled_ui_start_task()(20fps 소프트웨어 bit-bang I2C)와
   *  btn_handler_start_task()(PCF8574 bit-bang 폴링)를 커미셔닝 이전에
   *  시작하면, 이 고빈도 busy-wait 태스크가 Thread 802.15.4 operational/
   *  SRP 타이밍을 굶겨 SmartThings 페어링이 영구 실패(39-517)한다.
   *  따라서 init(1회성)만 먼저 하고, 연속 태스크는 커미셔닝 완료 후
   *  _deferred_task_starter 에서 시작한다. */
  memset(&s_ui, 0, sizeof(oled_ui_ctx_t));
  for (int i = 0; i < s_mgr.count && i < BLIND_MAX_COUNT; i++) {
    strncpy(s_ui.blind_names[i], s_mgr.blinds[i].name,
            sizeof(s_ui.blind_names[i]) - 1);
  }
  strncpy(s_ui.blind_names[BLIND_MAX_COUNT], "ALL", 4);
  s_ui.selected_blind = s_mgr.selected;
  s_ui.freq_mhz =
      (s_mgr.count > 0) ? s_mgr.blinds[0].freq_mhz : CFG_FREQ_DEF_MHZ;
#if BOARD_HAS_BAT_ADC && !BOARD_BAT_SWAPPED
  /* ★첫 측정 전에는 "--%". memset 0 을 그대로 두면 첫 5초간 "0%"(=방전 직전)로
   *  보여 오해를 준다. 표시측은 >100 을 미지값으로 렌더한다. */
  s_ui.chg_percent = BAT_PCT_UNKNOWN;
#endif

  boot_diag_stage2(BOOT_S2_OLED_ENTER);   /* ★여기서 멈추면 OLED init 이 범인 */
  oled_ui_init(&s_ui);                 /* SSD1306 1회 init (연속 X) */
  boot_diag_stage2(BOOT_S2_OLED_DONE);
  btn_handler_init(_btn_event_cb, NULL); /* GPIO 설정 (연속 X) */
  boot_diag_stage2(BOOT_S2_BTN_DONE);
#if BOARD_HAS_BAT_ADC && !TEMP_NO_CHARGE
  /* (_bat_adc_init 은 부팅 초반으로 이동 — 아래 RTC init 직후 주석 참조) */
#endif

  /* ── 5. Thread/Matter 초기화 — ★ OLED/버튼 연속 태스크 시작 전, 깨끗하게 ──
   *  (Phase1/2 검증 순서: RF init → Matter init/start, 무거운 동시 태스크 0) */
  thread_prov_init();
  oled_ui_set_thread(&s_ui, false);
  if (thread_prov_is_provisioned()) {
    ESP_LOGI(TAG, "저장된 Thread dataset 발견 — 네트워크 부착 시도");
  } else {
    ESP_LOGI(TAG, "Thread 미커미셔닝 — BLE 페어링 대기");
  }

  matter_blinds_init(&s_mgr, _matter_action_cb, NULL);
  const char *pair_code = matter_blinds_start();
  ESP_LOGI(TAG, "SmartThings 페어링 코드: %s", pair_code);

  /* ── 6. 애플리케이션 태스크 ────────────────────────────────────
   *  이미 커미셔닝된 기기 → 즉시 시작.
   *  미페어링/페어링대기 → 페어링 코드를 1회 표시하고, 활성 페어링이
   *    없으면 짧은 유예 뒤 단독 모드로 자동 시작(_deferred_task_starter).
   *    즉 SmartThings 없이도 버튼/OLED/RF 가 동작한다(페어링은 옵션). */
  if (matter_blinds_is_commissioning_complete()) {
    ESP_LOGI(TAG, "Matter 이미 커미셔닝됨 — 애플리케이션 태스크 즉시 시작");
    s_ui.state = OLED_STATE_NORMAL;
    _start_app_tasks_once();
  } else {
    ESP_LOGI(TAG, "미페어링 — 부팅 진행 화면 표시 + 백그라운드 페어링 감시");
    /* ★2026-08-12 예전엔 여기서 **메인을 1회 렌더하고 방치**했다. 무거운 연속
     *  태스크(OLED 갱신·버튼 폴링)는 _deferred_task_starter 가 5초+12초 뒤에야
     *  띄우므로, 그 17초 동안 화면만 멀쩡하고 버튼이 안 먹어 "먹통" 으로 보였다
     *  (사용자 신고, 실측 16.5초). → 부팅이 끝날 때까지 진행 바를 보여준다.
     *  s_ui.state 는 NORMAL 그대로 둔다 — 나중에 _ui_task 가 뜨는 순간 바로
     *  메인을 그리게 해 전환 깜빡임을 없앤다(부팅 화면은 직접 렌더라 무관). */
    s_ui.state = OLED_STATE_NORMAL;
    oled_ui_set_matter_status(&s_ui, OLED_MT_UNPAIRED, OLED_RSSI_INVALID);
    oled_ui_show_booting(&s_ui, 0);
    xTaskCreate(_deferred_task_starter, "defer_start", 3072, NULL, 4, NULL);
  }

  /* RF 큐만 미리 생성(저비용) — 태스크 본체는 _start_app_tasks_once 에서.
   *  커미셔닝 전엔 콜백이 발생하지 않으므로 큐가 비어 무해. */
  if (s_rf_queue == NULL) {
    s_rf_queue = xQueueCreate(RF_QUEUE_DEPTH, sizeof(rf_job_t));
  }
  boot_diag_stage2(BOOT_S2_RF_DONE);
  _batlog_load();   /* 이전 방전 기록 복구 — USB 없이 쌓인 것을 나중에 조회 */
  _vibelog_load();  /* ★2026-08-13 진동 진단 기록 복구 (콘솔 `vl`) */

  /* 절전 타이머 초기화 (부팅 직후를 활동으로 기록) */
  _mark_activity();

  /* ── 8b. light sleep GPIO wake sources 설정 ──
   *   PCF8574 ~INT(IO17): active-LOW.
   *   VIBE(IO16, JYX-1210-X160): 평상시 closed(LOW) → LOW_LEVEL 웨이크는
   *     슬립 진입 즉시 발사돼 슬립 불가. 진동 시 brief open(HIGH)이므로
   *     HIGH_LEVEL 웨이크 사용 → 가만히 둘 땐 슬립 유지, 진동 시 wake.
   *   CHG_STAT(IO3): active-LOW. */
  gpio_wakeup_enable(PCF8574_INT_PIN, GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable(VIBE_PIN,        GPIO_INTR_HIGH_LEVEL);
#if BOARD_CHG_STAT_ACTIVE_HIGH
  gpio_wakeup_enable(CHG_STAT_PIN,    GPIO_INTR_HIGH_LEVEL); /* VBUS HIGH = USB 꽂힘 */
#else
  gpio_wakeup_enable(CHG_STAT_PIN,    GPIO_INTR_LOW_LEVEL);  /* STAT active-LOW */
#endif
  esp_sleep_enable_gpio_wakeup();
  /* ★ v3.15: 버튼은 PCF8574(I2C 익스팬더) 뒤에 있어 ~INT(IO17) GPIO 웨이크가
   *  이 하드웨어에서 불안정 → 절전에서 버튼으로 안 깨어나는 문제. esp-matter/
   *  Thread SED 가 자체 light sleep 을 걸 수 있으므로, 150ms 주기 타이머
   *  웨이크를 백스톱으로 추가: 라이트슬립이든 아니든 ≤150ms 마다 깨어나
   *  btn_task 가 PCF8574 를 폴링 → 눌린 버튼 감지 → _exit_sleep. (I2C
   *  익스팬더 뒤 버튼 + 절전의 정석 패턴. 짧은 폴 1회라 소비 영향 미미.) */
  esp_sleep_enable_timer_wakeup(150000);  /* 150 ms */

  /* ── 8c. PM (Power Management) ── v3.3+ / v3.6 버그 수정
   *   ★ 모든 peripheral init 완료 후에 호출 — SPI/I2C/CC1101 init 도중
   *     tickless idle 데드락 방지.
   *   ★★ v3.6: 커미셔닝 완료 전에는 light sleep 을 켜지 않는다.
   *     Thread SED + auto light sleep 이 commissioning 의 Thread operational
   *     단계(SRP 등록 + CASE 핸드셰이크)를 굶겨 SmartThings 페어링이
   *     마지막에 실패하던 버그. 커미셔닝 완료(또는 기존 fabric)일 때만
   *     _enable_pm_light_sleep() 을 1회 호출. 미완료면 메인 루프에서
   *     완료 감지 후 활성화. */
  /* ★2026-08-11: 함수 자체가 "등록 전에는 DFS 만, 등록 후 light sleep" 을 판단하므로
   *  조건 없이 부른다. 등록 완료는 메인 루프의 주기 호출이 감지해 자동 승격한다. */
  _enable_pm_light_sleep();

  /* ── 9. 메인 루프 ── */
  boot_diag_stage2(BOOT_S2_MAIN_LOOP);
  ESP_LOGI(
      TAG,
      "메인 루프 시작 (유휴 → 화면 OFF: USB %d초 / 배터리 %d초, 버튼·진동으로 복귀)",
      CFG_SCREEN_OFF_USB_SEC, CFG_SCREEN_OFF_SEC);
  /* Task WDT subscribe — 메인 루프가 N초 이상 안 돌면 자동 panic+리부트.
   *  silent hang(스크린세이버에서 wake 무반응 등) 자동 복구 + 다음 boot 시
   *  reset_reason=TASK_WDT 로 breadcrumb 와 함께 진단 가능. */
  esp_err_t wdt_err = esp_task_wdt_add(NULL);
  if (wdt_err != ESP_OK) {
    ESP_LOGW(TAG, "esp_task_wdt_add 실패: %s — WDT 보호 없음", esp_err_to_name(wdt_err));
  } else {
    ESP_LOGI(TAG, "메인 루프 Task WDT subscribe 완료");
  }
  bool was_charging = false;
  bool was_vibrating = false;
  int64_t last_vib_us_seen = 0;
  while (1) {
    /* breadcrumb + WDT reset — 매 tick 마다 갱신. silent hang 시 다음
     *  boot 의 _crash_breadcrumb_init 로그가 마지막 tick 시각을 보여줌. */
    s_bc.main_loop_ticks++;
    s_bc.last_main_tick_us = esp_timer_get_time();
    s_bc.last_screensaver_active = s_screensaver_active ? 1 : 0;
    s_bc.last_sleeping = s_is_sleeping ? 1 : 0;
    s_bc.last_setup_screen = (uint8_t)s_setup_screen;
    s_bc.last_activity_us = s_last_activity_us;
    esp_task_wdt_reset();
    /* 앱이 시작된 뒤(단독 또는 커미셔닝 완료) PM(DFS) 활성화.
     *  idempotent — 최초 1회만 적용. */
    if (s_app_started) {
      _enable_pm_light_sleep();
    }

    /* 앱 시작 전(부팅 직후 유예/활성 페어링 중)에는 메인 루프 본체를
     *  돌리지 않는다(상태 폴링/OLED/충전/진동/절전 skip — 활성 페어링
     *  타이밍 보호). 앱이 시작되면(단독 포함) 전체 루프 동작. */
    if (!s_app_started) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    /* Thread 부착 상태 OLED 동기화. 부착 '전이' 검출은 독립 static 으로
     *  추적한다 — s_ui.thread_connected 는 oled_ui_set_matter_status() 가
     *  매초 (UNPAIRED→false 로) 덮어쓰므로, 그것을 엣지 기준으로 쓰면
     *  미페어링+Thread attach 상태에서 매초 거짓 false→true 전이가 생겨
     *  _on_thread_attached() 가 반복 호출(화면 강제 초기화)된다. */
    {
      static bool s_thread_was_attached = false;
      bool thread_ok = thread_prov_is_attached();
      if (thread_ok != s_thread_was_attached) {
        oled_ui_set_thread(&s_ui, thread_ok);
        if (thread_ok) {
          _on_thread_attached();
        }
        s_thread_was_attached = thread_ok;
      }
    }

    /* v3.6: 메인 화면 상단 Matter 상태 표시 (~1s 주기로 갱신).
     *   CONNECTED : fabric 존재(커미셔닝 완료) → 부모 RSSI 4단계 신호막대만
     *   PAIRING   : fail-safe armed (commissioner 가 페어링 진행 중) → 'P' 점멸
     *   UNPAIRED  : 그 외 (페어링 대기, BLE 광고만) → X
     * 이전 버그: Thread attach 만으로 'Y' 표시 → 실제 커미셔닝 전에 연결된 것
     *           처럼 보임. 이제 fabric/fail-safe 기준으로 정확히 구분. */
    {
      static int64_t s_last_mt_us = 0;
      static bool    s_mt_panel_was_on = false;
      const bool     mt_panel_on = oled_ui_is_panel_on();
      int64_t mt_now = esp_timer_get_time();

      /* ★★2026-08-15 **화면이 켜져 있을 때만 갱신한다** (절전).
       *  왜: thread_prov_is_attached()/get_parent_rssi() 는 **OpenThread 락을
       *  잡는다**. 화면이 꺼져 있으면 아무도 안 보는 값을 위해 1초마다 OT 태스크를
       *  건드리고 CPU 를 깨우는 셈이라, tickless idle/light sleep 을 그만큼 깬다.
       *  → 패널 OFF 면 통째로 건너뛴다.
       *  ※꺼진 동안 상태가 변해도 문제없다 — 화면이 켜지는 순간 아래 전이 처리로
       *    **즉시 1회 갱신**하므로 사용자는 항상 최신값을 본다(1초 지연 없음). */
      if (!mt_panel_on) {
        s_mt_panel_was_on = false;
      } else {
        if (!s_mt_panel_was_on) {
          s_mt_panel_was_on = true;
          s_last_mt_us = 0;          /* OFF→ON 전이: 즉시 갱신 */
        }
      /* ★★2026-08-15 주기 1초 → **7초** (슬로우 폴 주기와 일치).
       *  근거: 우리는 sleepy end device(rx-off-when-idle) 라 부모 프레임을 받는 건
       *  **폴링할 때뿐**이다. 따라서 otThreadGetParentAverageRssi 는
       *  CONFIG_ICD_SLOW_POLL_INTERVAL_MS(=7000) 보다 빨리 변할 수 없고,
       *  role→DETACHED 는 폴 실패 누적이라 더 느리다(child timeout 기본 240초).
       *  1초 주기는 **같은 값을 7번 다시 읽으며 그때마다 OT 락을 잡는 것**이었다.
       *  ※'P' 점멸은 anim_frame 구동이라 무관하고, 페어링 화면은 별도 로직을 쓴다. */
      if (mt_now - s_last_mt_us >= 7000000) {   /* 7s = ICD slow poll */
        s_last_mt_us = mt_now;
        oled_matter_state_t mst;
        int8_t rssi = OLED_RSSI_INVALID;
        if (matter_blinds_is_commissioned()) {
          /* ★★2026-08-15 등록 여부(NVS)만으로 CONNECTED 를 결정하면 안 된다.
           *  사용자 신고: "신호가 안 잡히는 먼 곳으로 옮겼는데 안테나가 그대로다."
           *  원인 — 등록은 NVS 라 권외에서도 유지되고, Thread 가 detach 되면
           *  parent RSSI 가 INVALID(127) 가 되는데 _rssi_to_level 이 127 을
           *  "링크는 있으나 측정 불가 → 풀바(4)" 로 해석해 **꽉 찬 안테나로 고정**됐다.
           *  → 실제 attach 상태를 함께 보고, detach 면 NO_SIGNAL('-') 로 표시한다. */
          if (thread_prov_is_attached()) {
            mst  = OLED_MT_CONNECTED;
            rssi = thread_prov_get_parent_rssi();
          } else {
            mst  = OLED_MT_NO_SIGNAL;      /* 등록 유지 + 권외 → '-' */
          }
        } else if (matter_blinds_is_pairing_in_progress()) {
          mst  = OLED_MT_PAIRING;
        } else {
          mst  = OLED_MT_UNPAIRED;
        }
        s_last_rssi_cache = rssi;      /* ★방전 로그가 읽는 캐시 */
        oled_ui_set_matter_status(&s_ui, mst, rssi);

        /* ★★★2026-08-16 (①진단) 표시가 바뀌는 순간 **판정 근거 3종**을 남긴다.
         *  사용자 신고: 권외로 나가면 '-'(권외)가 아니라 'X'(미등록)로 바뀐다.
         *  'X' 는 FabricCount==0 일 때만 나와야 하는데 등록은 NVS 라 권외에서도
         *  유지돼야 한다 → 예상과 다르므로 fabric/attached/role 을 함께 봐야
         *  어디서 갈리는지 알 수 있다. 상태가 **바뀔 때만** 찍어 OT 락 부담을 없앤다.
         *  배터리 구동 중엔 콘솔이 없으므로 방전 로그(NVS)에도 같이 남긴다. */
        {
          static oled_matter_state_t s_mt_prev = (oled_matter_state_t)0xFF;
          static int8_t s_rssi_prev = 0;
          /* ★2026-08-16 RSSI 변화도 찍는다 — 사용자 신고 "라우터 바로 옆인데
           *  안테나가 1칸". 상태는 CONNECTED 로 유지되므로 상태전이만 보면
           *  영영 안 찍힌다. 5dB 이상 움직일 때만 남겨 스팸을 막는다. */
          const int rssi_moved = (rssi > s_rssi_prev ? rssi - s_rssi_prev
                                                     : s_rssi_prev - rssi) >= 5;
          if (mst != s_mt_prev || rssi_moved) {
            s_mt_prev   = mst;
            s_rssi_prev = rssi;
            ESP_LOGW(TAG, "[MTDIAG] 표시=%s  fabric=%d  attached=%d  role=%d  rssi=%d",
                     (mst == OLED_MT_CONNECTED) ? "막대" :
                     (mst == OLED_MT_NO_SIGNAL) ? "-(권외)" :
                     (mst == OLED_MT_PAIRING)   ? "P(페어링)" : "X(미등록)",
                     matter_blinds_is_commissioned() ? 1 : 0,
                     thread_prov_is_attached() ? 1 : 0,
                     thread_prov_get_role(), (int)rssi);
            if      (mst == OLED_MT_NO_SIGNAL) somfy_app_batlog_button(BLEV_NOSIG);
            else if (mst == OLED_MT_UNPAIRED)  somfy_app_batlog_button(BLEV_UNPAIR);
          }
        }
      }
      }   /* mt_panel_on */
    }

    _heap_watch_tick(esp_timer_get_time(), s_setup_screen == SETUP_MATTER_PAIR);

    /* v3.6: 페어링 화면 세부 단계 진행 (SETUP_MATTER_PAIR 일 때만).
     *   WAIT   : 대기 (광고만)
     *   ACTIVE : fail-safe armed (commissioner 페어링 트랜잭션 진행)
     *   FAIL   : ACTIVE 였다가 완료 못 하고 트랜잭션 종료(만료/실패)
     *   DONE   : 커미셔닝 완료 → 3초간 표시 후 메인 복귀
     * 메뉴 재진입 시 oled_ui_show_pairing() 이 phase=WAIT 로 리셋하므로,
     * 추적용 static 도 SETUP_MATTER_PAIR 가 아닐 땐 매 루프 초기화. */
    {
      /* 빠른 실패 판정 워치독: 페어링 ACTIVE 진입 후 이 시간 내 커미셔닝
       * 미완료면 fail-safe(최대 ~4분) 만료를 안 기다리고 즉시 FAIL 표시.
       * 정상 커미셔닝(BLE+operational)은 보통 20~60초 → 90초면 안전.
       * Matter fail-safe 자체는 규격대로 백그라운드 진행(동작 영향 없음). */
      #define PAIR_FAIL_WATCHDOG_US  (90LL * 1000 * 1000)
      static bool    s_pair_was_active   = false;
      static int64_t s_pair_done_us      = 0;
      static int64_t s_pair_active_since = 0;
      if (s_setup_screen != SETUP_MATTER_PAIR) {
        s_pair_was_active   = false;
        s_pair_done_us      = 0;
        s_pair_active_since = 0;
        s_pair_ready        = false;   /* 페어링 화면 이탈 — READY 해제 */
        _pairing_protect_end();        /* 보호 원복 (idempotent) */
      } else {
        oled_pair_phase_t ph;
        int64_t pnow = esp_timer_get_time();
        if (matter_blinds_is_commissioning_complete()) {
          ph = OLED_PAIR_DONE;
          if (s_pair_done_us == 0) {
            s_pair_done_us = pnow;
            ESP_LOGI(TAG, "[PAIR] 커미셔닝 완료 — SUCCESS 5초 후 메인 복귀");
          } else if (pnow - s_pair_done_us >= 5000000) {
            /* 완료 — 메인 복귀 + 상단 안테나/Matter 상태 즉시 갱신. */
            oled_ui_set_matter_status(&s_ui, OLED_MT_CONNECTED,
                                      thread_prov_get_parent_rssi());
            _setup_exit_to_main("페어링 완료 (5s 경과)");
          }
        } else if (matter_blinds_is_pairing_in_progress()) {
          if (s_pair_active_since == 0) s_pair_active_since = pnow;
          s_pair_was_active = true;
          s_pair_ready      = false;   /* 페어링 트랜잭션 시작 — READY 해제 */
          if (pnow - s_pair_active_since >= PAIR_FAIL_WATCHDOG_US) {
            /* 워치독 초과 — fail-safe 만료 전이라도 실패로 표시. */
            ph = OLED_PAIR_FAIL;
            const char *perr = matter_blinds_get_last_pair_error();
            const char *code = (perr && perr[0]) ? perr : "ERR TIMEOUT";
            strncpy(s_ui.pair_err, code, sizeof(s_ui.pair_err) - 1);
            s_ui.pair_err[sizeof(s_ui.pair_err) - 1] = '\0';
          } else {
            ph = OLED_PAIR_ACTIVE;
          }
        } else if (s_pair_was_active && !matter_blinds_is_commissioned()) {
          ph = OLED_PAIR_FAIL;     /* 진행했다가 완료 못 함 */
          /* 디바이스 관점 실패 진단 코드 표시 (SmartThings 39-xxx 는 앱
           * 측 코드라 알 수 없음 → ERR FS-NOC / FS-PRE / SESSION). */
          const char *perr = matter_blinds_get_last_pair_error();
          const char *code = (perr && perr[0]) ? perr : "ERR TIMEOUT";
          strncpy(s_ui.pair_err, code, sizeof(s_ui.pair_err) - 1);
          s_ui.pair_err[sizeof(s_ui.pair_err) - 1] = '\0';
        } else {
          /* 아직 페어링 트랜잭션 없음 — 사용자가 SETUP 으로 준비
           *  확정했으면 READY(점멸), 아니면 WAITING(점멸). */
          ph = s_pair_ready ? OLED_PAIR_READY : OLED_PAIR_WAIT;
          s_pair_active_since = 0;
          s_ui.pair_err[0] = '\0';
        }
        if (s_setup_screen == SETUP_MATTER_PAIR) {
          oled_ui_set_pair_phase(&s_ui, ph);
        }
      }
    }

    /* USB 케이블 연결 + 충전 중 검사 (MCP73831 STAT pin) */
#if TEMP_NO_CHARGE
    bool charging = false;             /* ★임시: 충전 감지 비활성(진단용) */
#else
    bool charging = btn_handler_is_charging();
#endif
    int64_t now_us = esp_timer_get_time();
    static int64_t _dbg_us = -5000000;
    if (now_us - _dbg_us >= 5LL * 1000 * 1000) {
      _dbg_us = now_us;
#if BOARD_HAS_BAT_ADC
#if BOARD_BAT_SWAPPED
      /* 현 기판 오배선: GP3(BAT_ADC핀)=VBUS, GP12(CHG_STAT핀)=BAT.
       * GP12 가 ADC/comparator 불가핀이라 잔량 % 측정 불가 → 다음으로 대체:
       *   · USB 감지 = GP3(ADC)로 VBUS 측정 (정확)
       *   · 저전압  = GP12 내부 풀업 1임계(~3V)로 판별 (HIGH=정상 / LOW=저전압) */
      int _vbus = _read_bat_mv();                /* GP3 = VBUS (USB 시 ~6000mV 환산) */
      s_ui.usb_pwr = (_vbus > 3000);
      gpio_set_pull_mode(BOARD_PIN_CHG_STAT, GPIO_PULLUP_ONLY);
      /* ★2026-08-14 vTaskDelay → esp_rom_delay_us.
       *  pdMS_TO_TICKS(3) 는 tick 100Hz 에서 0 tick → 풀업 RC 가 안정되기 전에
       *  읽어 저전압을 오판한다. tick 주기와 무관하게 고정. */
      esp_rom_delay_us(3000);
      bool _bat_ok = gpio_get_level(BOARD_PIN_CHG_STAT);   /* HIGH=배터리>~3V */
      gpio_set_pull_mode(BOARD_PIN_CHG_STAT, GPIO_PULLDOWN_ONLY);  /* button_handler 기본 복원 */
      s_ui.bat_low = (!s_ui.usb_pwr && !_bat_ok);
      ESP_LOGD(TAG, "[BAT] usb=%d bat_ok=%d (vbus=%dmV)", s_ui.usb_pwr, _bat_ok, _vbus);  /* 진단용(평소 비표시) */
#else
      /* 정상 배선(신 PCB): GP1(BAT_ADC핀)=BAT 분압 → 충전률 % 실측.
       * **충전 중에도 % 를 그대로 표시**한다(사용자 요구). 단 배터리를 빼면 충전 IC 가
       * BAT+ 를 3970mV 로 띄워 78% 처럼 보이므로, _nobat_track 이 "미연결" 로 판정하면
       * 0% 로 표시한다. 5분 창마다 재판정하므로 배터리를 빼면 최대 ~10분 안에 0% 로
       * 바뀐다(첫 판정 전 5분간은 실측값 78% 가 그대로 보인다 — 정상). */
      /* 2026-07-17 추가: _read_bat_mv() 는 OLED 전송 중이면 -1(이번 주기 건너뜀) 을
       * 준다. 그 땐 **표시를 건드리지 않고** 마지막 값을 유지한다(엉뚱한 추정치로
       * 튀지 않게). 미측정이므로 _nobat_track 에도 넣지 않는다(평균 오염 방지). */
      int _bat_mv = _read_bat_mv();
      if (_bat_mv > 0) {
        boot_diag_set_bat_mv(_bat_mv);   /* 1회만 기록 — 부팅 시 전원 상태 증거 */
        _nobat_track(_bat_mv);
        /* ★2026-08-11 첫 판정 전 % 숨김 — 단, **애매한 구간에서만** 숨긴다.
         *  무배터리 float 창(3940~4010mV) 밖이면 배터리가 있다는 게 전압만으로
         *  이미 확정이므로(충전 IC 가 그 값을 만들 수 없다) 즉시 % 를 보여준다.
         *  창 안 + USB 연결일 때만 진짜로 구분이 안 되고, 이때만 판정을 기다린다.
         *  → 실제 배터리를 꽂은 사용자는 대기 없이 바로 %, 배터리 없는 경우만
         *    5분 뒤 0% 로 확정된다(그 전엔 "--%"). */
        const bool _ambiguous = (_bat_mv >= BAT_NOBAT_LO && _bat_mv <= BAT_NOBAT_HI &&
                                 _is_usb_powered());
        /* ★★2026-08-12 숨김에 **상한**을 둔다.
         *  버그였다: "판정 전 + 애매하면 숨김"에 시간 제한이 없어, 전압이 float
         *  창(3940~4010mV) 안에 머물면 판정이 끝날 때까지 계속 "--%" 였다.
         *  판정은 5분 창이 차야 나는데 재부팅하면 처음부터 다시라, 플래시·재부팅이
         *  잦으면 **영영 숫자가 안 나온다**(실사용 신고: "충전률이 안 나온다").
         *  → 부팅 후 BAT_HIDE_MAX_S 를 넘기면 판정 여부와 무관하게 실측 % 를 보여준다.
         *    틀릴 수 있는 78% 를 잠깐 감추자는 장치였지, 영구히 감추자는 게 아니었다. */
#ifndef BAT_HIDE_MAX_S
/* ★2026-08-12 330초 → 45초 (사용자: "5분 30초는 너무 길다").
 *  45초는 임의값이 아니라 **표시 평활의 중앙값 창이 채워지는 시간**이다
 *  (BAT_DISP_WIN 9주기 × 측정주기 5초). 그 전에 보여주면 필터가 안 걸린 날값이
 *  나오므로, 필터가 찬 직후를 하한으로 잡는다.
 *  대가: 배터리가 없는데 float 창(3940~4010mV)에 걸린 경우, 판정(5분)이 날 때까지
 *  최대 4분여 동안 78% 가 보인다. 아무것도 안 보이는 것보다는 낫다는 판단. */
#define BAT_HIDE_MAX_S 45
#endif
        const bool _hide_window = (esp_timer_get_time() < (int64_t)BAT_HIDE_MAX_S * 1000000LL);
        if (!s_nobat_judged && _ambiguous && _hide_window)
          s_ui.chg_percent = BAT_PCT_UNKNOWN;          /* 표시측이 "--%" 로 렌더 */
        else
          {
            int _sm = _bat_smooth_mv(_bat_mv);
            /* ★★★2026-08-23 **급락 표본 거부** — 0% 갇힘의 근본 방어.
             *
             *  같은 사고가 두 번 났고 둘 다 NVS 방전기록에 남아 있다:
             *      08-20  4069 → 2651 → 2129 mV  (8시간 걸쳐 3845mV 로 "회복")
             *      08-23  4055 → 3002 mV         (리셋하니 4078mV 로 즉시 정상)
             *  둘 다 **산포 1~6카운트로 깨끗**했다 = ADC 잡음이 아니라 측정 전체가
             *  낮게 나왔다. 그리고 그 값이 0% 로 환산돼 표시가 0% 에 갇혔다.
             *
             *  앞서 넣은 방어 둘로는 부족했다:
             *    · 하한 조건부 해제  → 갇힌 **뒤에** 푸는 것이라 그 사이 0% 를 본다
             *    · 천장 출발점 교정  → **그 전압 자체가 틀리면** 0% 출발은 마찬가지
             *  → 여기서 표본을 아예 버린다. 700mAh 셀이 5초에 1,000mV 떨어지는 건
             *    불가능하다(그 전압이면 LDO 드롭아웃으로 죽는데 기기는 계속 돌았다).
             *  ★영원히 거부하면 안 된다 — 진짜 급락(셀 수명 끝·보호회로)도 있다.
             *    연속 BAT_SPIKE_REJECT_MAX 회까지만 버리고 그 뒤엔 인정한다(15초).
             *  검증: sim/tools/bat_spike_reject_sim.py — 실측 급락 2건을 잡고,
             *        정상 방전·USB 전환(≤120mV)은 통과. 5개 항목 전부 의도대로.
             *  ※근본 원인(왜 낮게 읽히는가)은 **아직 미해결**이다. 아래 로그가 단서다. */
            {
              static int s_bat_ok_mv = 0;
              static int s_bat_rej   = 0;
              if (s_bat_ok_mv > 0 && _sm < s_bat_ok_mv - BAT_SPIKE_MAX_DROP_MV
                  && s_bat_rej < BAT_SPIKE_REJECT_MAX) {
                s_bat_rej++;
                ESP_LOGW(TAG, "[BAT%%] ★급락 표본 거부 %d/%d — %dmV 에서 %dmV "
                              "(낙차 %dmV, 산포 %d카운트). 물리적으로 불가능하다",
                         s_bat_rej, BAT_SPIKE_REJECT_MAX, s_bat_ok_mv, _sm,
                         s_bat_ok_mv - _sm, (int)s_bat_last_spread);
                _sm = s_bat_ok_mv;          /* 직전 정상값 유지 */
              } else {
                s_bat_rej   = 0;
                s_bat_ok_mv = _sm;
              }
            }
            s_bat_last_sm_mv = _sm;   /* 1분 방전 로거가 읽는다 */
            s_bat_last_raw_mv = _bat_mv;
            s_bat_last_us = now_us;
            int _pct = s_nobat ? 0 : _bat_mv_to_pct(_sm);
            /* ★★2026-08-12 (B) 방전 중 단조 비증가 — 잔량이 느는 건 불가능하다.
             *  하한은 창이 BAT_FLOOR_MIN_N 개 찬 뒤부터 건다(첫 표본 하나로 잡으면
             *  그게 저전압 무리에 걸렸을 때 세션 내내 5%p 낮게 고정된다). */
            if (!_is_usb_powered()) {
              if (s_bat_hist_n >= BAT_FLOOR_MIN_N) {
                /* ★2026-08-20 (A) 조건부 해제 — 선언부 주석의 잠김 사고 대응.
                 *  하한을 건 시점보다 확실히 높은 전압이 연속으로 유지되면,
                 *  하한을 만든 낮은 측정이 틀렸던 것이므로 놓아준다. */
                if (s_pct_floor <= 100 &&
                    _sm >= s_pct_floor_mv + BAT_FLOOR_RELEASE_MV) {
                  if (++s_floor_up_run >= BAT_FLOOR_RELEASE_N) {
                    ESP_LOGW(TAG, "[BAT%%] ★하한 해제 — %d%%(%dmV) 에서 걸렸는데 "
                                  "%dmV 가 %d표본 연속. 그 낮은 측정이 틀렸다",
                             (int)s_pct_floor, s_pct_floor_mv, _sm,
                             (int)s_floor_up_run);
                    s_pct_floor    = BAT_PCT_UNKNOWN;
                    s_floor_up_run = 0;
                  }
                } else {
                  s_floor_up_run = 0;
                }
                if (s_pct_floor <= 100 && _pct > (int)s_pct_floor) {
                  _pct = (int)s_pct_floor;
                } else {
                  s_pct_floor    = (uint8_t)_pct;
                  s_pct_floor_mv = _sm;        /* 해제 판정의 기준 전압 */
                }
              }
              s_pct_ceil_mpct = -1;            /* 방전 복귀 → 충전 천장 해제 */
            } else {
              s_pct_floor = BAT_PCT_UNKNOWN;   /* 충전 중엔 상승이 정상 → 하한 해제 */
              /* ★★★2026-08-16 (③) 충전 중 **상승률 제한** — 위 BAT_FULL_MV 주석이
               *  "고치려면 충전 중 상승률 제한(하한 s_pct_floor 의 대칭)이 필요하다"
               *  고 지시해 놓고 구현되지 않았던 부분이다.
               *
               *  사용자 실측(2026-08-16): 84% → USB 연결 **즉시 100%** → 1분 내
               *  분리하면 96%. 원인은 충전 전류가 내부저항을 지나며 단자전압을
               *  100mV 남짓 들뜨게 하는 것인데, 하한까지 풀어버려 **상한이 없다**.
               *  즉 표시가 잔량이 아니라 단자전압을 따라간다.
               *
               *  → 천장을 둔다. USB 연결 순간의 표시값에서 출발해 실제 충전 속도보다
               *    빠르게 오르지 못하게 하고, 표시 = min(전압환산%, 천장) 으로 한다.
               *    700mAh 를 0.5C 로 충전하면 약 2시간 → 0.85%p/분. */
              if (s_pct_ceil_mpct < 0) {
                /* ★★2026-08-20 (B) 출발점을 **직전 표시값 → 연결 순간의 전압환산%**
                 *  로 바꾼다.
                 *  사고: 방전 중 하한이 0% 로 잠긴 채 USB 를 꽂으면, 천장이 그 0% 에서
                 *  출발해 0.85%/분으로 기어오른다. 실측(2026-08-20) 연결 316초 뒤 4%,
                 *  376초 뒤 5% — 참값(약 75%)까지 1.5시간이 걸린다.
                 *  왜 전압환산이 옳은가: 단자전압이 들뜨는 것은 **충전 전류가 흐른
                 *  뒤**다. USB 를 꽂은 그 순간의 평활 전압은 아직 방전 구간 값이라
                 *  참 OCV 에 가깝다. 게다가 평활(중앙값5+EMA α=1/4)이 지연을 주므로
                 *  들뜬 표본 하나가 들어와도 25% 만 반영된다(약 3%p).
                 *  → 원래 막으려던 "84% → 연결 즉시 100%" 점프는 그대로 막힌다
                 *    (연결 순간 전압환산이 곧 84% 이므로 천장도 84% 에서 출발).
                 *  검증: sim/tools/bat_floor_release_sim.py — 0% 출발 시 90분 걸려
                 *        76%, 전압환산 출발 시 **즉시 78%**. 84% 케이스는 점프 없음. */
                s_pct_ceil_mpct = _pct * 1000;   /* 연결 순간의 전압환산 % 에서 출발 */
                s_pct_ceil_us   = now_us;
              } else {
                const int64_t dms = (now_us - s_pct_ceil_us) / 1000;
                if (dms > 0) {
                  s_pct_ceil_us    = now_us;
                  s_pct_ceil_mpct += (int)((dms * BAT_CHG_RISE_MPCT_PER_MIN) / 60000);
                  if (s_pct_ceil_mpct > 100000) s_pct_ceil_mpct = 100000;
                }
              }
              if (_pct > s_pct_ceil_mpct / 1000) _pct = s_pct_ceil_mpct / 1000;
            }
            /* ★★★2026-08-27 **표시 % 하락 속도 제한** — "잔량이 내려갔다 다시 올라간다".
             *
             *  사슬: ① BAT_ADC 가 이따금 낮게 읽힌다 → ② 표시가 끌려 내려가고 방전
             *  하한(s_pct_floor)이 그 값에 잠긴다 → ③ 정상값이 돌아오면 하한 해제
             *  (2026-08-23 에 "0% 갇힘"을 풀려고 넣은 장치)로 표시가 되올라간다.
             *  ③은 의도대로 동작한 것인데, 사용자 눈에는 오락가락으로 보인다.
             *
             *  → **물리적으로 불가능한 속도의 하락을 막는다.** 그러면 이상값이 화면에
             *    안 나타나고, 하한도 잘못 잠기지 않아 되올라감도 사라진다(한 수정으로
             *    양방향 해결).
             *  근거: 실측 방전은 9.94시간에 5%p(C6) = 0.0014%p/5초. 5초에 1%p 를
             *    허용하면 **실제보다 약 700배 빠른** 셈이라 진짜 방전은 안 막힌다.
             *    반대로 스파이크(한 번에 90%p)는 100표본(8.3분)에 걸쳐야 반영되므로
             *    그 전에 정상으로 돌아오면 화면에 티가 안 난다.
             *  ★상승은 제한하지 않는다 — 충전 시 즉시 반영돼야 하고, 방전 중 상승은
             *    이미 하한이 막는다. 부팅 첫 표본도 제한하지 않는다(안 그러면 0%에서
             *    기어오른다).
             *  검증: sim/tools/bat_display_rate_sim.py (5개 항목) */
            {
              static bool s_disp_init = false;
              const int prev = (int)s_ui.chg_percent;
              if (s_disp_init && s_ui.chg_percent != BAT_PCT_UNKNOWN &&
                  _pct < prev - BAT_DISP_MAX_DROP_PCT)
                _pct = prev - BAT_DISP_MAX_DROP_PCT;
              s_disp_init = true;
            }
            s_ui.chg_percent = (uint8_t)_pct;
            /* ★진단(2026-08-11): "% 가 한 값에 고정" 신고 대응. 원본/평활/표시%
             *  를 한 줄로 남겨 **평활이 붙잡는 것인지, 전압이 실제로 안 변하는지**
             *  를 즉시 가른다. 원본이 움직이는데 평활이 안 움직이면 평활 문제,
             *  둘 다 안 움직이면 전압(충전 CV 구간 등)이 진짜 평평한 것이다. */
            static int _slog = 0;
            if ((_slog++ % BAT_DBG_EVERY) == 0)   /* ★진단 중: 매 측정 */
              ESP_LOGW(TAG, "[BAT%%] 원본%dmV -> 평활%dmV -> %d%%  (nobat=%d, judged=%d, "
                            "하한=%d, 창%d/%d)",
                       _bat_mv, _sm, (int)s_ui.chg_percent, s_nobat ? 1 : 0,
                       s_nobat_judged ? 1 : 0,
                       s_pct_floor <= 100 ? (int)s_pct_floor : -1,
                       s_bat_hist_n, BAT_DISP_WIN);
          }
      } else if (!s_bat_adc_ok) {
        s_ui.chg_percent = _estimate_battery_percent();  /* ADC 자체가 없는 경우만 */
      }
#endif
#endif
    }

    if (charging) {
      /* CHG_STAT LOW 감지 시각 갱신 — USB 모드 hold 윈도우용 */
      s_last_chg_active_us = now_us;

      if (!was_charging) {
        ESP_LOGI(TAG, "USB 케이블 연결 감지 → 충전 모드");
        if (s_screensaver_active) _exit_screensaver("USB connect");
        _mark_activity();
      }
    } else if (was_charging) {
      ESP_LOGI(TAG, "USB 케이블 분리 가능성 (CHG_STAT HIGH) — 모드 재평가");
      s_chg_start_us = 0;
    }

    /* v3.2 idle 정책: USB 모드 vs 배터리 모드 분기 ─────────────────── */
    bool usb_mode = _is_usb_powered();
    int64_t vib_us_now = btn_handler_last_vibration_us();
    int64_t since_vib_us = now_us - vib_us_now;
    bool vibration_active = btn_handler_is_vibrating();
    bool vibration_holdoff =
        (vib_us_now > 0) &&
        (since_vib_us < (int64_t)VIBRATION_HOLD_MS * 1000);

    /* v3.4: 진동 감지 = 활동 — 새 진동 이벤트 또는 진동 중이면 활성화 ─
     *  edge: 마지막 진동 시각이 직전 루프와 다르거나 진동 진행 중이면
     *  - _mark_activity() 로 idle 타이머 리셋
     *  - 화면 보호기 / 절전 모드면 즉시 해제 → OLED on, 메인 화면 복귀 */
    bool vib_event = (vib_us_now != 0) && (vib_us_now != last_vib_us_seen);
    if (vibration_active || vib_event) {
      _mark_activity();
      if (s_screensaver_active) _exit_screensaver("vibration");
      if (s_is_sleeping)        _exit_sleep("vibration");
      if (vib_event && !was_vibrating) {
        ESP_LOGI(TAG, "[VIBE] 진동 감지 → 활성 모드 복귀");
      }
    }
    last_vib_us_seen = vib_us_now;
    was_vibrating = vibration_active;

    int64_t idle_us = now_us - s_last_activity_us;

    /* 설정 모드 / 페어링 / Thread reset 화면 / charging anim 중에는
     * 절전/화면보호기 진입 안 함 */
    /* ★2026-07-24 수정 — "화면이 저절로 켜진 뒤 다시 안 꺼진다" 증상 대응.
     *  기존엔 CHARGING/PAIRING/THREAD_PROV 상태에서도 inhibit 이 걸렸는데, 이 상태들은
     *  **Matter/Thread 스택이 자동으로** 진입할 수 있다(재광고·재부착 등). inhibit 분기는
     *  매 tick _mark_activity() 로 유휴 타이머를 리셋하므로, 한 번 그 상태가 되면
     *  화면이 켜진 채 **영영 안 꺼진다**. 버튼을 누르면 상태가 풀려 10초 뒤 정상적으로
     *  꺼지던 것도 이 때문.
     *  → 사용자가 **직접** 들어간 설정 메뉴(_in_setup_mode)만 화면을 유지한다.
     *    어떤 상태 때문에 걸릴 뻔했는지는 아래 로그로 남겨 추적 가능하게 둔다. */
    bool inhibit = _in_setup_mode();
    {
      static oled_state_t last_auto = OLED_STATE_NORMAL;
      oled_state_t st = s_ui.state;
      if (!inhibit && (st == OLED_STATE_PAIRING ||
                       st == OLED_STATE_THREAD_PROV)) {
        if (st != last_auto) {
          ESP_LOGI(TAG, "[SCROFF] 자동상태(%d)에서도 화면 OFF 타이머 유지", (int)st);
          last_auto = st;
        }
      } else if (st != last_auto) {
        last_auto = OLED_STATE_NORMAL;
      }
    }
    /* ── 2026-07-23 유휴 정책(화면보호기 삭제 후) ─────────────────────
     *   1단계 활성 (idle < off)  : 정상 화면
     *   2단계 OFF  (idle >= off) : 패널 OFF (+배터리는 light sleep)
     *  중간의 "화면보호기 애니메이션" 단계는 **제거**했다(사용자 요청).
     *  꺼지는 시간은 somfy_config.h 의 CFG_SCREEN_OFF_USB_SEC(USB) /
     *  CFG_SCREEN_OFF_SEC(배터리) 두 값으로 설정한다.
     *  깨우기: 버튼(_btn_event_cb) / 진동(아래 vibration 분기)이 담당. */
    /* ★2026-08-11 USB/배터리 문턱 분리 — USB 는 전원이 무제한이라 길게(기본 5분),
     *  배터리는 화면이 소비의 큰 몫이라 짧게(기본 10초). 값은 somfy_config.h 에서. */
    int64_t off_to_us = (int64_t)(usb_mode ? CFG_SCREEN_OFF_USB_SEC
                                           : CFG_SCREEN_OFF_SEC) * 1000000LL;

    if (inhibit) {
      /* 설정/페어링/충전 화면 — 화면 OFF 보류, 타이머 재시작 */
      if (idle_us > off_to_us) _mark_activity();
    } else if (vibration_active || vibration_holdoff) {
      if (idle_us > off_to_us) {
        static int last_log_s = -1;
        int now_s = (int)(since_vib_us / 1000000);
        if (now_s != last_log_s) {
          ESP_LOGI(TAG, "진동 감지로 화면 OFF 차단 (last %ds, %s)",
                   now_s, vibration_active ? "active" : "holdoff");
          last_log_s = now_s;
        }
      }
    } else if (idle_us >= off_to_us) {
      /* 2단계: 패널 OFF (+배터리 light sleep) */
      if (usb_mode) {
        if (!s_screensaver_active) _enter_screensaver();
      } else {
        if (s_screensaver_active) s_screensaver_active = false;
        if (!s_is_sleeping) _enter_sleep();
      }
    }
    /* (화면보호기 애니메이션 단계 삭제 — 2026-07-23 사용자 요청.
     *  유휴 CFG_SCREEN_OFF_SEC 초가 지나면 위 분기에서 곧바로 패널 OFF 한다.) */

    was_charging = charging;

    /* ★2026-07-24 안정성 모니터 — 60초마다 요약.
     *  2026-08-11 충전측정 재활성 검증용으로 **락타임아웃(lockTO)** 을 추가했다.
     *  판정 기준:
     *    · 실패 0  + present=1 유지 → 직렬화 수정이 먹힌 것
     *    · lockTO 가 0 이 아님      → 누군가 뮤텍스를 200ms 넘게 쥔다(원인 추적 필요)
     *  ★함정: "실패 0"만 보면 안 된다. OLED 미검출이면 flush 를 건너뛰어 전송도
     *    에러도 안 생긴다 → **present=1 과 전송 카운터 증가를 같이** 볼 것. */
    /* ── ★2026-08-11 USB 분리 후 1분 주기 방전 로그 (사용자 요청) ────────────
     *  USB 를 빼는 순간을 세션 시작으로 잡고, 1분마다 전압·%·추정전류를 남긴다.
     *  ※세션 시작 검출은 _is_usb_powered() 의 60초 hold 창 때문에 최대 1분 늦을 수
     *    있다(그 사이 값은 기준점에 포함된다) — 누적 평균에는 영향이 미미하다. */
#if BOARD_HAS_BAT_ADC   /* ★2026-08-16 방전 세션 로직 — 배터리 ADC 가 있는 보드만.
                         *  BAT_FLOOR_MIN_N·BAT_PCT_UNKNOWN·BATLOG_* 등이 모두
                         *  #if BOARD_HAS_BAT_ADC 안에 있어, 회로 없는 보드에서
                         *  이 블록만 밖에 있으면 빌드가 깨진다. 애초에 배터리가
                         *  없으면 방전 세션 자체가 의미 없다. */
    {
      /* ★★★2026-08-15 `was_usb_pwr` 를 **부팅 시 실제 전원 상태로** 초기화한다.
       *
       *  버그였다: `= true`(항상 USB 로 가정) 라서, **배터리 구동 중 재부팅**
       *  (배터리 글리치/브라운아웃 — 부팅진단에 `bat=3990mV 리셋사유=전원투입` 실재)
       *  이 나면 첫 판독에서 "USB→배터리 전환"으로 오인하고 `_batlog_reset()` 이
       *  **직전 기록을 통째로 지웠다**. 사용자 6시간 방전 기록이 이렇게 날아갔다.
       *  → 첫 루프에서 실제 상태를 읽고, 배터리면 NVS 의 직전 세션을 **이어받는다**. */
      static bool     s_bl_first    = true;
      static bool     was_usb_pwr  = true;
      static bool     dis_pending  = false;   /* 분리는 감지, 기준점은 아직 (★C) */
      static uint32_t dis_seq0     = 0;       /* 분리 시점의 평활 호출 횟수 */
      /* ★★2026-08-13 USB→배터리 전환을 **2초 디바운스**한다.
       *
       *  버그였다: 전환을 한 번의 판독으로 확정해 `_batlog_reset()` 을 불렀다.
       *  그런데 (a) 시리얼 포트를 열면 USB-JTAG 이 칩을 리셋시키고(부팅진단
       *  "리셋사유=USB리셋(플래시/포트열기)", 누적 198회), (b) 재부팅 직후
       *  `was_usb_pwr` 은 true 로 시작하는데 VBUS 판정이 안정되기 전 한 순간
       *  false 로 읽히면 **"USB 분리"로 오인** → 새 세션 시작 → **직전 기록이
       *  통째로 지워졌다**. 실사용에서 1시간짜리 방전 측정이 두 번 날아갔다
       *  (세션 #22 → #25 → #27 로 실제 분리 횟수보다 많이 증가한 게 증거).
       *  → 연속 USB_DROP_CONFIRM_MS 동안 배터리로 읽혀야 전환으로 인정한다.
       *    실제 분리라면 2초 뒤에 시작해도 무해하다(기준점은 어차피 25초 뒤 확정). */
#ifndef USB_DROP_CONFIRM_MS
#define USB_DROP_CONFIRM_MS 2000
#endif
      static int64_t usb_low_since_us = 0;
      /* ★★★2026-08-16 **첫 판독 보정** — 이게 기록을 날려온 진짜 원인이다.
       *
       *  아래 `now_usb` 는 "usb_mode 이거나, 배터리로 읽힌 지 2초가 안 됐으면 USB"
       *  다. 그런데 부팅 직후 첫 루프에서는 `usb_low_since_us` 가 방금 now_us 로
       *  채워지므로 경과 0초 → **배터리인데도 now_usb 가 항상 true** 로 나온다.
       *  결과: (1) s_bl_first 가 USB 부팅으로 오인해 _batlog_try_resume 을 아예
       *  부르지 않고, (2) 2초 뒤 배터리로 뒤집히면 `was_usb_pwr && !now_usb` 가
       *  성립해 **"USB 분리"로 오인** → 새 세션 → 기록 삭제.
       *  즉 배터리로 부팅할 때마다 직전 기록이 지워졌다.
       *  → 배터리로 부팅한 경우엔 디바운스 창을 이미 지난 것으로 놓는다. */
      if (s_bl_first && !usb_mode && usb_low_since_us == 0) {
        usb_low_since_us = now_us - (int64_t)USB_DROP_CONFIRM_MS * 1000;
      }
      if (usb_mode) {
        usb_low_since_us = 0;
      } else if (usb_low_since_us == 0) {
        usb_low_since_us = now_us;
      }
      const bool now_usb = usb_mode ||
          ((now_us - usb_low_since_us) < (int64_t)USB_DROP_CONFIRM_MS * 1000);
      if (s_bl_first) {
        s_bl_first  = false;
        was_usb_pwr = now_usb;                /* ★가짜 전환 방지 */
        if (!now_usb) {
          /* 배터리로 부팅 — 직전 세션이 살아 있으면 이어받고, 없으면
           *  아래 dis_pending 경로로 새 세션을 시작한다(_batlog_reset 은
           *  그때 한 번만 불린다). */
          if (!_batlog_try_resume(true)) {
            dis_pending = true;
            dis_seq0    = s_bat_sm_seq;
            ESP_LOGW(TAG, "[BATLOG] 배터리로 부팅 — 이어받을 세션 없음, 새로 시작");
          }
        }
      } else if (was_usb_pwr && !now_usb) {
        /* ★★2026-08-12 (C) USB → 배터리: **평활을 비우고 기준점은 미룬다.**
         *  분리 순간의 평활값은 충전 중 단자전압(실측 4056mV)이지 배터리 OCV 가
         *  아니다. 예전엔 그 값을 그대로 기준점으로 삼아, 이후 90초에 걸친 정착
         *  (92%→73%)이 통째로 "방전"으로 계산돼 평균 302~819mA 가 찍혔다. */
        _bat_smooth_reset();
        dis_pending = true;
        dis_seq0    = s_bat_sm_seq;
        s_dis_t0_us = 0;                      /* 기준점 확정 전엔 세션 없음 */
        ESP_LOGW(TAG, "[BATLOG] USB 분리 감지 — 평활 초기화, 배터리 실측 %d회로 기준점 잡는 중",
                 BAT_FLOOR_MIN_N);
      } else if (!was_usb_pwr && now_usb) {
        ESP_LOGW(TAG, "[BATLOG] 방전 종료 (USB 연결)");
        _batlog_flush_if_due(now_us, true);   /* 남은 것 즉시 저장 */
        _bat_smooth_reset();                  /* ★C 대칭 — 배터리 전압을 충전 구간에 끌고 가지 않는다 */
        s_pct_floor = BAT_PCT_UNKNOWN;        /* ★B 하한 해제(충전 중 상승 허용) */
        dis_pending = false;
        s_dis_t0_us = 0;
      }
      was_usb_pwr = now_usb;

      /* ★C 기준점 확정 — **하한(B)이 걸리는 시점과 같이** 잡는다.
       *  왜 첫 실측이 아닌가: 첫 표본은 중앙값이 안 걸린 날값이라, 그걸 기준으로
       *  삼으면 워밍업 중 표시가 올라갈 때 누적 전류가 **음수**로 나온다.
       *  같은 수렴값에서 기준점과 하한을 함께 잡으면 그 모순이 없다.
       *  대가: 세션 시각(+0초)이 실제 분리보다 최대 25초 늦다. */
      if (dis_pending && !now_usb && s_bat_sm_seq > dis_seq0 &&
          s_bat_hist_n >= BAT_FLOOR_MIN_N && s_bat_last_sm_mv > 0) {
        dis_pending    = false;
        s_dis_t0_us    = now_us;
        s_dis_mv0      = s_bat_last_sm_mv;
        s_dis_pct0     = s_ui.chg_percent <= 100 ? s_ui.chg_percent : 0;
        s_dis_prev_us  = now_us;
        s_dis_prev_mv  = s_dis_mv0;
        s_dis_prev_pct = s_dis_pct0;
        ESP_LOGW(TAG, "[BATLOG] ★방전 시작 — %dmV %d%% / 용량 %dmAh (실측 %d회로 수렴)",
                 s_dis_mv0, (int)s_dis_pct0, BAT_CAPACITY_MAH, s_bat_hist_n);
        /* ★★★2026-08-16 `_batlog_reset()` 제거 — **기록을 지우지 않는다**.
         *  세션 번호만 올리고 경계 표식을 한 건 남긴다. 링이 가득 차면 가장 오래된
         *  것부터 밀려날 뿐이라, 세션 판정이 틀려도 과거 기록은 살아남는다.
         *  전체 삭제는 콘솔 `blclear`(somfy_app_batlog_clear) 로만 — 사용자가
         *  명시적으로 요청할 때만 일어난다. */
        _batlog_new_session();
        _batlog_ls_session_begin(false);   /* 새 세션 — light sleep 계측도 0 부터 */
        _batlog_add_ev(0, s_dis_mv0, s_dis_pct0,
                       matter_blinds_get_radio_enabled(), oled_ui_is_panel_on(),
                       g_pm_state_applied, BLEV_SESS);
      }

      /* ★기록 주기: 분리 직후 2분간은 5초, 그 뒤로는 2분마다 (사용자 지정).
       *  왜 나누나: 분리 직후는 부하가 급변하는 구간(무선 OFF·화면 OFF 전환 등)이라
       *  촘촘히 봐야 어디서 얼마나 떨어지는지 보인다. 안정되면 2분 간격으로 충분하고
       *  로그도 조용해진다.
       *  ※배터리 측정 자체가 5초 주기이므로 5초가 최소 간격이다(더 촘촘히 못 찍는다).
       *    또 표시 평활이 중앙값 9주기(=45초)라 초반 값은 아직 수렴 중임에 유의. */
      const int64_t _el_us  = now_us - s_dis_t0_us;
      const int64_t _iv_us  = (_el_us < (int64_t)BATLOG_FAST_S * 1000000LL)
                                ? (int64_t)BATLOG_FAST_IV * 1000000LL
                                : (int64_t)BATLOG_SLOW_IV * 1000000LL;
      _batlog_flush_if_due(now_us, false);   /* 쌓인 기록을 2초 단위로 NVS 반영 */
      _vibelog_tick(now_us);                 /* ★진동 진단 기록(창 3초, NVS 는 10초 합침) */
      if (!now_usb && s_dis_t0_us && s_bat_last_sm_mv > 0 &&
          (now_us - s_dis_prev_us) >= _iv_us) {
        const int mv   = s_bat_last_sm_mv;
        const int pct  = (s_ui.chg_percent <= 100) ? s_ui.chg_percent : 0;
        const int el_s = (int)((now_us - s_dis_t0_us) / 1000000);       /* 누적 초 */
        const int sg_s = (int)((now_us - s_dis_prev_us) / 1000000);     /* 구간 초 */

        /* 누적 평균 전류: 쓴 용량 / 경과시간. %p → mAh 는 용량 비례로 환산. */
        const int d_pct_tot = (int)s_dis_pct0 - pct;
        /* ★★2026-08-12 분모의 `* 10` 이 잘못이었다 — 표시가 정확히 10배 낮았다.
         *   mA = 소모mAh / 시간 = (d_pct×용량/100) / (el_s/3600)
         *      = d_pct × 용량 × 36 / el_s      ← `*10` 이 들어갈 자리가 없다.
         *   실측 대조: 2.24시간에 73%→58%(-15%p), 700mAh → 47mA 인데 로그는 4mA.
         *   `rem_min` 이 이 값을 쓰므로 예상 잔여시간도 10배 낙관적이었다. */
        const int avg_ma    = (el_s > 0)
            ? (d_pct_tot * BAT_CAPACITY_MAH * 36) / el_s          /* ×3600/100 */
            : 0;
        /* 직전 1분 구간 전류 — 부하 변화(화면·무선 ON/OFF)를 잡는다. */
        const int d_pct_seg = (int)s_dis_prev_pct - pct;
        const int seg_ma    = (sg_s > 0)
            ? (d_pct_seg * BAT_CAPACITY_MAH * 36) / sg_s          /* ★위와 같은 `*10` 제거 */
            : 0;
        /* 남은 시간: 현재 %와 누적 평균 전류 기준(전류 0 이하면 산출 불가) */
        const int rem_min = (avg_ma > 0)
            ? (pct * BAT_CAPACITY_MAH * 60) / (100 * avg_ma) : -1;

        ESP_LOGW(TAG, "[BATLOG] +%d초(%d분)  %dmV %d%%  (시작 %dmV %d%%, 누적 -%dmV -%d%%p)  "
                      "평균 %dmA / 구간 %dmA  예상잔여 %d분  무선=%s 화면=%s PM=%d",
                 el_s, el_s / 60, mv, pct, s_dis_mv0, (int)s_dis_pct0,
                 s_dis_mv0 - mv, d_pct_tot, avg_ma, seg_ma, rem_min,
                 matter_blinds_get_radio_enabled() ? "ON" : "OFF",
                 oled_ui_is_panel_on() ? "ON" : "OFF", g_pm_state_applied);

        /* ★NVS 에 즉시 영속화 — USB 없는 동안의 기록이 남아야 의미가 있다.
         *  주기는 사용자가 지정한 그대로(초반 5초 / 이후 2분). */
        _batlog_add(el_s, mv, pct,
                    matter_blinds_get_radio_enabled(), oled_ui_is_panel_on(),
                    g_pm_state_applied);

        s_dis_prev_us  = now_us;
        s_dis_prev_mv  = mv;
        s_dis_prev_pct = (uint8_t)pct;
      }
    }
#endif /* BOARD_HAS_BAT_ADC — 방전 세션 로직 */

    /* ── ★2026-08-11 무선 게이팅 (배터리 절약, 사용자 요청) ──────────────────
     *  규칙:
     *    · Thread 기기로 **등록됨**  → 무선 ON 유지 (재부팅 후에도 동일)
     *    · **미등록** + 페어링 화면 아님 → 무선 OFF (라디오가 할 일이 없다)
     *    · 페어링 화면 진입          → ON (matter_blinds_open_commissioning_window 가 켬)
     *  주기적으로 확인하는 이유: 커미셔닝 완료·fabric 삭제 같은 상태 변화를 이벤트 하나로
     *  놓치면 무선이 잘못된 상태로 굳는다. 5초마다 맞추면 어떤 경로로 바뀌어도 수렴한다.
     *  ※미등록 시 OFF 는 부팅 후 RADIO_GRACE_SEC 뒤에 적용한다 — 스택이 자리를 잡기 전에
     *    끄면 초기화와 경합할 수 있고, 사용자가 부팅 직후 커미셔닝하려는 경우도 있다. */
    {
#ifndef RADIO_GRACE_SEC
/* ★★★2026-08-17 60 → 25초. **미등록 상태에서는 광고하지 않는다**(사용자 결정).
 *  페어링은 필수 기능이 아니며, 필요할 때 설정 메뉴의 페어링 화면에 들어가면
 *  아래 `pairing` 조건으로 무선이 다시 켜진다.
 *  이 유예가 60초였을 때: 부팅(약 19초) 뒤 40초간 광고가 떠 있었고, 그 창이
 *  닫힌 뒤 페어링을 시도하면 폰이 기기를 못 찾아 SmartThings 39-100 이 났다.
 *  유예를 아예 없애지 않는 이유는 원 주석대로 **스택 초기화와의 경합** 때문이다.
 *  부팅 완료가 ~19초, 이 검사가 5초 주기이므로 25초면 초기화는 지나고
 *  광고 노출은 몇 초로 줄어든다. */
#define RADIO_GRACE_SEC 25      /* 부팅 후 이 시간 동안은 끄지 않는다(초기화 보호) */
#endif
      static int64_t last_chk_us = 0;
      if (now_us - last_chk_us >= 5LL * 1000000LL) {
        last_chk_us = now_us;
        const bool paired  = matter_blinds_is_commissioning_complete();
        const bool pairing = (s_setup_screen == SETUP_MATTER_PAIR);
        const bool booting = (now_us < (int64_t)RADIO_GRACE_SEC * 1000000LL);
        const bool want    = paired || pairing || booting;
        if (want != matter_blinds_get_radio_enabled()) {
          ESP_LOGW(TAG, "[RADIO] %s — 등록=%d 페어링화면=%d 부팅유예=%d",
                   want ? "켬" : "끔(미등록 절전)", paired, pairing, booting);
          matter_blinds_set_radio_enabled(want);
        }
      }
    }

    /* ★부팅 성공 확정 — 메인 루프가 30초 살아 있으면 "정상 가동"으로 기록한다.
     *  다음 부팅에서 이 값이 아니면 **직전 부팅이 그 단계에서 멈췄다**는 뜻. */
    {
      static bool _boot_ok_marked = false;
      if (!_boot_ok_marked && esp_timer_get_time() > 30LL * 1000000LL) {
        _boot_ok_marked = true;
        boot_diag_stage2(BOOT_S2_RUNNING);
      }
      /* 직전 부팅 요약 재출력 — begin() 시점(부팅 ~100ms) 로그는 USB-JTAG 이
       *  버려서 실제로 통째로 유실됐다. 5초·20초에 두 번 더 찍어 확실히 남긴다. */
      static int _prev_logged = 0;
      int64_t _up = esp_timer_get_time();
      if ((_prev_logged == 0 && _up > 5LL * 1000000LL) ||
          (_prev_logged == 1 && _up > 20LL * 1000000LL)) {
        _prev_logged++;
        boot_diag_log_prev();
      }
    }

    {
      extern volatile uint32_t g_bbo_tx_cnt, g_bbo_fail_cnt, g_page_skip, g_page_sent;
      extern volatile uint32_t g_bbo_lock_to_cnt;
      extern volatile bool g_oled_present_mon;
      static int64_t last_rep_us = 0;
      int64_t nu = esp_timer_get_time();
      if (nu - last_rep_us >= 60LL * 1000000LL) {
        last_rep_us = nu;
        /* ★2026-08-11 전원 상태를 같이 남긴다.
         *  PM 설정 로그는 부팅 초기에 1회만 찍혀 USB-JTAG 유실 구간에 사라진다
         *  (실제로 캡처에서 통째로 안 보였다). 60초 주기 모니터에 실제 적용값을
         *  실어 **언제 접속해도** 확인할 수 있게 한다. 값은 전부 실측 상태이며
         *  문자열을 하드코딩하지 않는다. */
        /* ★2026-08-16 RSSI 를 여기 실었다 — MTDIAG 는 '변화가 있을 때만' 찍혀
         *  신호가 안정되면 영영 안 보인다. 안테나 칸수 신고를 확인하려면
         *  **언제 접속해도** 현재값이 보여야 한다(60초 1회라 OT 락 부담 없음). */
        const int _rssi_now = (int)thread_prov_get_parent_rssi();
        s_last_rssi_cache = (int8_t)_rssi_now;   /* ★화면 OFF 구간에서도 갱신 */
        /* ★2026-08-20 LP·폴주기 추가 — 깨어남 빈도(≈1000/폴주기)의 지배 요인이라
         *  절전 진단에 매번 필요한데, LP 판정 로그는 부팅 초반에만 나와 USB 재열거가
         *  늦으면 캡처에서 놓친다. 여기 실어 언제든 읽을 수 있게 한다. */
        ESP_LOGW(TAG, "[PWR] light_sleep=%s (PM상태 %d)  무선=%s  등록=%d  화면=%s  "
                      "VBUS=%d  rssi=%d  LP=%s 폴%dms  유휴 %llds",
                 (g_pm_state_applied >= 2) ? "ON" : "OFF", g_pm_state_applied,
                 matter_blinds_get_radio_enabled() ? "ON" : "OFF",
                 matter_blinds_is_commissioning_complete() ? 1 : 0,
                 oled_ui_is_panel_on() ? "ON" : "OFF",
                 btn_handler_is_charging() ? 1 : 0, _rssi_now,
                 btn_handler_lp_active() ? "ON" : "OFF", btn_handler_poll_ms(),
                 (long long)((nu - s_last_activity_us) / 1000000));
        /* ★2026-08-23 배터리 **표시값**을 여기 싣는다.
         *  사고: 화면은 0% 인데 NVS batlog 의 pct 는 96% 였다 — 표시 경로에서
         *  갈린다는 뜻인데, [BAT%] 진단 로그는 사용자 지시로 10분 주기라 창을 계속
         *  놓쳤다. 60초 [PWR] 은 어차피 찍히므로 여기 붙이면 추가 비용이 없다.
         *  표시(chg_percent) / 평활 / 원본 / 하한 / 천장을 한 줄로 본다. */
        /* ★2026-08-24 배터리 구동 중이면 PM 락 보유자를 NVS 에 갱신 (위 주석 참조).
         *  USB 중에는 어차피 light sleep 을 끄므로 의미가 없다. */
        if (!_is_usb_powered()) _pmdump_save();
        ESP_LOGW(TAG, "[PWR2] 표시=%d%%  평활=%dmV  원본=%dmV  하한=%d  천장=%d%%  "
                      "nobat=%d judged=%d",
                 (s_ui.chg_percent == BAT_PCT_UNKNOWN) ? -1 : (int)s_ui.chg_percent,
                 s_bat_last_sm_mv, s_bat_last_raw_mv,
                 (s_pct_floor <= 100) ? (int)s_pct_floor : -1,
                 (s_pct_ceil_mpct < 0) ? -1 : (s_pct_ceil_mpct / 1000),
                 s_nobat ? 1 : 0, s_nobat_judged ? 1 : 0);
        /* ★2026-08-20 LP 깨우기 계측 — 위 btn_handler_lp_counters 주석 참조.
         *  seq 증가 = LP 가 HP 를 깨운 횟수(정지 상태면 0 이어야 정상). */
        {
          static uint32_t s_lp_p0 = 0, s_lp_s0 = 0;
          static int64_t  s_lp_t0 = 0;
          uint32_t pc = 0, sq = 0;
          btn_handler_lp_counters(&pc, &sq);
          const double dt = s_lp_t0 ? (double)(nu - s_lp_t0) / 1e6 : 0.0;
          uint32_t ntf = 0, lat = 0;
          btn_handler_lp_latch_stats(&ntf, &lat);
          if (dt > 0.5)
            ESP_LOGW(TAG, "[LPWAKE] 폴 %.1f회/초  ★HP 깨우기 %.2f회/초 "
                          "(seq +%u / %.0f초)  태스크통지 누적 %u  래치 0x%04X",
                     (pc - s_lp_p0) / dt, (sq - s_lp_s0) / dt,
                     (unsigned)(sq - s_lp_s0), dt,
                     (unsigned)ntf, (unsigned)lat);
          s_lp_p0 = pc; s_lp_s0 = sq; s_lp_t0 = nu;
        }
        /* ★2026-08-13 (②) 버스트 폴링 깨우기 출처 — `~INT` 가 실제로 쓸 만한지 판정.
         *  int 이 0 에 가까우면 `~INT` 가 죽은 것이고, 그러면 안전망 주기
         *  (BTN_IDLE_POLL_MS)가 곧 버튼 반응 지연이 된다. 코드 주석에 "현 HW 는
         *  ~INT 가 불안정" 이라는 경고가 있어 믿지 않고 **재서** 판단한다. */
        {
          uint32_t wi = 0, wv = 0, wt = 0, wid = 0;
          btn_handler_wake_stats(&wi, &wv, &wt, &wid);
          /* ★2026-08-25 `~INT` 수동 관찰 — 선이 실제로 움직이는지 본다.
           *  ISR 은 걸지 않았다(폭주·WDT 위험). 사용자가 버튼을 누르는 동안
           *  `전이` 가 늘어야 `~INT` 가 살아 있는 것이다. 0 이면 죽은 선. */
          {
            uint32_t isam = 0, ilow = 0, iedg = 0;
            btn_handler_int_observe(&isam, &ilow, &iedg);
            ESP_LOGW(TAG, "[INTOBS] 표본 %u  LOW %u (%.1f%%)  ★전이 %u "
                          "— 버튼 누를 때 전이가 늘면 ~INT 살아있음",
                     (unsigned)isam, (unsigned)ilow,
                     isam ? 100.0 * ilow / isam : 0.0, (unsigned)iedg);
          }
          ESP_LOGW(TAG, "[BTNWAKE] 유휴진입 %u  깨움: ~INT %u / 진동 %u / 타임아웃 %u"
                        "   (~INT 비율 %u%%)",
                   (unsigned)wid, (unsigned)wi, (unsigned)wv, (unsigned)wt,
                   (unsigned)((wi + wv + wt) ? (100u * wi) / (wi + wv + wt) : 0));
        }
        ESP_LOGW(TAG, "[OLEDMON] %llds  전송 %u / 실패 %u  페이지 보냄 %u / 건너뜀 %u  "
                      "lockTO %u  present=%d free=%uB",
                 nu / 1000000, (unsigned)g_bbo_tx_cnt, (unsigned)g_bbo_fail_cnt,
                 (unsigned)g_page_sent, (unsigned)g_page_skip,
                 (unsigned)g_bbo_lock_to_cnt,
                 g_oled_present_mon ? 1 : 0, (unsigned)esp_get_free_heap_size());
      }
    }

    /* FW Update 화면일 때 OTA 상태/진행률을 매 tick 갱신(OLED 가 반영). */
    if (s_setup_screen == SETUP_FW_UPDATE) {
      uint8_t prog = 0;
      s_ui.fw_ota_state    = (uint8_t)matter_ota_get_state(&prog);
      s_ui.fw_ota_progress = prog;
    }

    /* ★ 메인 루프 1000→100ms — 진동/버튼 이벤트 응답 지연(최대 1초) 단축.
     *  나머지 작업(충전, OLED, 스크린세이버 타이머)도 10× 자주 돌지만 모두
     *  경량 폴링/상태머신이라 부담 없음. light sleep 도 vTaskDelay 동안 정상 진입. */
    /* ★★2026-08-12 되돌림 — 화면 OFF 시 500ms 로 늦췄다가 되돌린다.
     *  btn_handler 가 버튼·진동을 잡아도 **화면을 켜는 처리는 이 루프**가 하므로,
     *  여기가 느리면 그만큼 켜지는 게 늦는다(150ms 폴링과 겹쳐 체감이 훨씬 나빴다).
     *  깨우기 경로에 지연을 넣지 않는다. */
    /* ★2026-08-20 흡수한 두 태스크의 일 (_time_tick 위 주석 참조) */
    _time_tick_sched(now_us);
    _time_persist_sched(now_us);
    g_wake_iter[WI_MAIN]++;            /* ★깨어남 출처 계측 */
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
