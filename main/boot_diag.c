/* boot_diag 구현 — 자세한 배경은 boot_diag.h 주석 참조. */
#include "boot_diag.h"

#include <string.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <nvs.h>
#include <nvs_flash.h>

static const char *TAG = "BOOTDIAG";

#define NS_BOOTDIAG   "bootdiag"
#define KEY_REC       "rec"
/* ★2026-08-11 설계 수정 — "직전 기록"만 두면 **증거가 한 번에 날아간다**.
 *  실제로 당했다: 배터리 부팅이 멈춘 뒤 USB 로 켰더니, 그 USB 부팅의 begin() 이
 *  rec 를 덮어써서 실패 기록이 사라졌다(로그는 19.5/20.4초에 한 번 지나가 버림).
 *  → 실패한 부팅은 별도 키(fail)에 **따로 보관**한다. 정상 부팅이 아무리 반복돼도
 *    지워지지 않고, 새로운 실패가 생길 때만 갱신된다. */
#define KEY_FAIL      "fail"
#define KEY_FAILN     "failn"

typedef struct {
    uint32_t magic;
    uint32_t boot_count;
    uint8_t  stage;        /* app_main 경로 진행도 (boot_s1_t) */
    uint8_t  stage2;       /* somfy_app 경로 진행도 (boot_s2_t) */
    uint8_t  reset_reason;
    uint8_t  sub;          /* 현재 단계 내부 세부 진행도 */
    int16_t  bat_mv;       /* 부팅 초기 1회 */
    int16_t  bat_min_mv;   /* 관측된 최저 전압(부하 시 처짐) */
    uint32_t uptime_ms;    /* 마지막 기록 시점의 uptime */
} boot_rec_t;

/* 구조가 바뀌면 magic 을 올려 옛 기록을 자동 폐기한다(오독 방지). */
#define BOOT_REC_MAGIC 0x42443034u   /* "BD04" — sub 추가 */

static boot_rec_t s_prev;
static boot_rec_t s_cur;
static boot_rec_t s_fail;
static uint32_t   s_fail_count = 0;
static bool       s_ready = false;

const char *boot_diag_s1_name(boot_s1_t s)
{
    switch (s) {
        case BOOT_S1_NONE:          return "-";
        case BOOT_S1_APP_MAIN:      return "app_main/NVS";
        case BOOT_S1_ENDPOINTS:     return "Matter 엔드포인트";
        case BOOT_S1_BLIND_MGR:     return "blind_manager";
        case BOOT_S1_MATTER_START:  return "Matter start 직전";
        case BOOT_S1_MATTER_OK:     return "Matter start 완료";
        case BOOT_S1_APP_TASK:      return "somfy_app 태스크 생성";
        case BOOT_S1_CONSOLE_ENTER: return "콘솔 섹션 진입";
        case BOOT_S1_CONSOLE_CMDS:  return "콘솔 명령 등록 완료";
        case BOOT_S1_DONE:          return "app_main 완료";
        default:                    return "?";
    }
}

const char *boot_diag_s2_name(boot_s2_t s)
{
    switch (s) {
        case BOOT_S2_NONE:       return "-(태스크 미실행)";
        case BOOT_S2_RUN_ENTRY:  return "somfy_app_run 진입";
        case BOOT_S2_RTC_INIT:   return "RTC 초기화";
        case BOOT_S2_OLED_ENTER: return "oled_ui_init 직전";
        case BOOT_S2_OLED_DONE:  return "oled_ui_init 완료";
        case BOOT_S2_BTN_DONE:   return "버튼 init 완료";
        case BOOT_S2_RF_DONE:    return "RF 큐 완료";
        case BOOT_S2_MAIN_LOOP:  return "메인 루프 진입";
        case BOOT_S2_RUNNING:    return "정상 가동";
        default:                 return "?";
    }
}

static const char *_reset_name(uint8_t r)
{
    switch ((esp_reset_reason_t)r) {
        case ESP_RST_POWERON:    return "전원투입";
        case ESP_RST_EXT:        return "외부리셋";
        case ESP_RST_SW:         return "소프트리셋";
        case ESP_RST_PANIC:      return "패닉";
        case ESP_RST_INT_WDT:    return "인터럽트WDT";
        case ESP_RST_TASK_WDT:   return "태스크WDT";
        case ESP_RST_WDT:        return "기타WDT";
        case ESP_RST_BROWNOUT:   return "★브라운아웃(전원부족)";
        case ESP_RST_DEEPSLEEP:  return "딥슬립복귀";
        case ESP_RST_SDIO:       return "SDIO";
        /* ※enum 값이라 #ifdef 로 가릴 수 없다(매크로가 아님).
         *   IDF v5.5 esp_system.h 에 전부 정의돼 있음을 확인했다. */
        case ESP_RST_USB:        return "USB리셋(플래시/포트열기)";
        case ESP_RST_JTAG:       return "JTAG리셋";
        case ESP_RST_EFUSE:      return "eFuse오류";
        case ESP_RST_PWR_GLITCH: return "★전원글리치(공급불안)";
        case ESP_RST_CPU_LOCKUP: return "★CPU 락업(더블 예외)";
        default:                 return "알수없음";
    }
}

/* 부팅이 끝까지 갔는가 — 두 경로 모두 완주해야 성공. */
static inline bool _complete(const boot_rec_t *r)
{
    return r->stage >= BOOT_S1_DONE && r->stage2 >= BOOT_S2_RUNNING;
}

static void _save(void)
{
    nvs_handle_t h;
    if (nvs_open(NS_BOOTDIAG, NVS_READWRITE, &h) != ESP_OK) return;
    s_cur.uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (nvs_set_blob(h, KEY_REC, &s_cur, sizeof(s_cur)) == ESP_OK) nvs_commit(h);
    nvs_close(h);
}

void boot_diag_begin(void)
{
#if !BOOT_DIAG_ENABLE
    return;
#else
    memset(&s_prev, 0, sizeof(s_prev));
    memset(&s_fail, 0, sizeof(s_fail));

    nvs_handle_t h;
    if (nvs_open(NS_BOOTDIAG, NVS_READWRITE, &h) == ESP_OK) {
        size_t len = sizeof(s_prev);
        if (nvs_get_blob(h, KEY_REC, &s_prev, &len) != ESP_OK ||
            len != sizeof(s_prev) || s_prev.magic != BOOT_REC_MAGIC) {
            memset(&s_prev, 0, sizeof(s_prev));
        }
        len = sizeof(s_fail);
        if (nvs_get_blob(h, KEY_FAIL, &s_fail, &len) != ESP_OK ||
            len != sizeof(s_fail) || s_fail.magic != BOOT_REC_MAGIC) {
            memset(&s_fail, 0, sizeof(s_fail));
        }
        nvs_get_u32(h, KEY_FAILN, &s_fail_count);

        /* ★직전 부팅이 끝까지 못 갔으면 **실패 기록으로 따로 보관**한다.
         *  이후 정상 부팅이 몇 번 일어나도 이 값은 남는다(위 KEY_FAIL 주석 참조). */
        if (s_prev.magic == BOOT_REC_MAGIC && !_complete(&s_prev)) {
            s_fail = s_prev;
            s_fail_count++;
            nvs_set_blob(h, KEY_FAIL, &s_fail, sizeof(s_fail));
            nvs_set_u32(h, KEY_FAILN, s_fail_count);
            nvs_commit(h);
        }
        nvs_close(h);
    }

    memset(&s_cur, 0, sizeof(s_cur));
    s_cur.magic        = BOOT_REC_MAGIC;
    s_cur.boot_count   = s_prev.boot_count + 1;
    s_cur.stage        = BOOT_S1_APP_MAIN;
    s_cur.stage2       = BOOT_S2_NONE;
    s_cur.reset_reason = (uint8_t)esp_reset_reason();
    s_cur.bat_mv       = -1;
    s_cur.bat_min_mv   = -1;
    s_ready = true;
    _save();

    boot_diag_log_prev();
#endif
}

void boot_diag_log_prev(void)
{
#if !BOOT_DIAG_ENABLE
    return;
#else
    if (!s_ready) return;
    if (s_prev.magic != BOOT_REC_MAGIC) {
        ESP_LOGW(TAG, "[BOOTDIAG] 직전 기록 없음 (첫 부팅 또는 형식 변경)");
    } else {
        bool ok = _complete(&s_prev);
        ESP_LOGW(TAG, "[BOOTDIAG] 직전 부팅(%u회차): %s app_main=%d(%s) "
                      "somfy_app=%d(%s) sub=%d uptime=%ums bat=%d/최저%dmV 리셋사유=%s",
                 (unsigned)s_prev.boot_count, ok ? "정상 가동했음" : "★부팅 미완료",
                 s_prev.stage,  boot_diag_s1_name((boot_s1_t)s_prev.stage),
                 s_prev.stage2, boot_diag_s2_name((boot_s2_t)s_prev.stage2),
                 (int)s_prev.sub, (unsigned)s_prev.uptime_ms, (int)s_prev.bat_mv,
                 (int)s_prev.bat_min_mv, _reset_name(s_prev.reset_reason));
    }

    /* ★보관된 '실패한 부팅' — 정상 부팅이 반복돼도 남아 있다. 이게 진짜 증거다. */
    if (s_fail.magic == BOOT_REC_MAGIC) {
        ESP_LOGE(TAG, "[BOOTDIAG] ★보관된 실패부팅(누적 %u회): %u회차 "
                      "app_main=%d(%s) somfy_app=%d(%s) sub=%d uptime=%ums bat=%d/최저%dmV 리셋사유=%s",
                 (unsigned)s_fail_count, (unsigned)s_fail.boot_count,
                 s_fail.stage,  boot_diag_s1_name((boot_s1_t)s_fail.stage),
                 s_fail.stage2, boot_diag_s2_name((boot_s2_t)s_fail.stage2),
                 (int)s_fail.sub, (unsigned)s_fail.uptime_ms, (int)s_fail.bat_mv,
                 (int)s_fail.bat_min_mv, _reset_name(s_fail.reset_reason));
    } else {
        ESP_LOGW(TAG, "[BOOTDIAG] 보관된 실패부팅 없음");
    }

    /* 한글/긴 줄은 유실되는 사례가 있어 **짧은 ASCII 한 줄**을 같이 남긴다. */
    ESP_LOGW(TAG, "BDPREV n=%u s1=%u s2=%u sub=%u up=%ums bat=%d min=%d rst=%u | "
                  "BDFAIL n=%u s1=%u s2=%u sub=%u up=%ums bat=%d min=%d rst=%u cnt=%u | BDNOW n=%u rst=%u",
             (unsigned)s_prev.boot_count, (unsigned)s_prev.stage,
             (unsigned)s_prev.stage2, (unsigned)s_prev.sub, (unsigned)s_prev.uptime_ms,
             (int)s_prev.bat_mv, (int)s_prev.bat_min_mv, (unsigned)s_prev.reset_reason,
             (unsigned)s_fail.boot_count, (unsigned)s_fail.stage,
             (unsigned)s_fail.stage2, (unsigned)s_fail.sub, (unsigned)s_fail.uptime_ms,
             (int)s_fail.bat_mv, (int)s_fail.bat_min_mv, (unsigned)s_fail.reset_reason,
             (unsigned)s_fail_count,
             (unsigned)s_cur.boot_count, (unsigned)s_cur.reset_reason);
#endif
}

void boot_diag_stage(boot_s1_t stage)
{
#if !BOOT_DIAG_ENABLE
    (void)stage;
#else
    if (!s_ready || (uint8_t)stage <= s_cur.stage) return;
    s_cur.stage = (uint8_t)stage;
    _save();
    ESP_LOGI(TAG, "[BOOTDIAG] app_main %d (%s) @%ums",
             (int)stage, boot_diag_s1_name(stage), (unsigned)s_cur.uptime_ms);
#endif
}

void boot_diag_stage2(boot_s2_t stage)
{
#if !BOOT_DIAG_ENABLE
    (void)stage;
#else
    if (!s_ready || (uint8_t)stage <= s_cur.stage2) return;
    s_cur.stage2 = (uint8_t)stage;
    s_cur.sub = 0;          /* 새 단계 진입 → 세부 진행도 리셋 */
    _save();
    ESP_LOGI(TAG, "[BOOTDIAG] somfy_app %d (%s) @%ums",
             (int)stage, boot_diag_s2_name(stage), (unsigned)s_cur.uptime_ms);
#endif
}

void boot_diag_sub(uint8_t step)
{
#if !BOOT_DIAG_ENABLE
    (void)step;
#else
    if (!s_ready || step <= s_cur.sub) return;
    s_cur.sub = step;
    _save();
#endif
}

void boot_diag_set_bat_mv(int mv)
{
#if !BOOT_DIAG_ENABLE
    (void)mv;
#else
    if (!s_ready || mv <= 0) return;
    bool dirty = false;
    if (s_cur.bat_mv <= 0) { s_cur.bat_mv = (int16_t)mv; dirty = true; }
    /* ★최저치 추적 — 50mV 이상 더 떨어질 때만 기록(플래시 마모 억제) */
    if (s_cur.bat_min_mv <= 0 || mv < s_cur.bat_min_mv - 50) {
        s_cur.bat_min_mv = (int16_t)mv; dirty = true;
    }
    if (dirty) _save();
#endif
}

void boot_diag_clear_fail(void)
{
#if BOOT_DIAG_ENABLE
    nvs_handle_t h;
    if (nvs_open(NS_BOOTDIAG, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_erase_key(h, KEY_FAIL);
    nvs_erase_key(h, KEY_FAILN);
    nvs_commit(h);
    nvs_close(h);
    memset(&s_fail, 0, sizeof(s_fail));
    s_fail_count = 0;
    ESP_LOGW(TAG, "[BOOTDIAG] 보관된 실패부팅 기록 삭제됨");
#endif
}
