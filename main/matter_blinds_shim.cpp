/*
 * matter_blinds_shim.cpp — somfy_app.c 가 기대하는 matter_blinds_* C API 를
 * CHIP 상태 위에 구현한 얇은 shim. Somfy RTS 블라인드 컨트롤러
 * (ESP32-C6) v3.5.
 *
 *  Matter 시작/노드/엔드포인트/이벤트핸들러/WC delegate 는 전부
 *  app_main.cpp 소유. 여기서는 상태 조회 + 메뉴발 재-커미셔닝만 한다.
 */
#include "esp_log.h"
#include <string.h>

#include <app/server/Server.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/FailSafeContext.h>
#include <credentials/FabricTable.h>
#include <platform/PlatformManager.h>
#include <system/SystemClock.h>
#include <setup_payload/OnboardingCodesUtil.h>
#include <lib/support/Span.h>
#include <app/clusters/ota-requestor/OTARequestorInterface.h>
#include <platform/ConnectivityManager.h>
#include <platform/ThreadStackManager.h>
#include <app-common/zap-generated/cluster-objects.h>
#include "esp_app_desc.h"

extern "C" {
#include "matter_blinds.h"
}

static const char *TAG = "MB_SHIM";

/* app_main.cpp 가 set 하는 커미셔닝 완료 플래그 */
extern "C" volatile bool g_commissioning_complete;

static char                       s_pair_code[24] = "00000000";
static matter_blind_action_cb_t   s_action_cb     = nullptr;
static void                      *s_action_ud     = nullptr;

/* app_main.cpp 의 WC delegate 가 SmartThings 명령 수신 시 호출 →
 * OLED/절전 해제 반영용. (없으면 무시) */
extern "C" void matter_blinds_invoke_action_cb(uint8_t ep_idx,
                                                somfy_command_t cmd,
                                                uint8_t pos_pct,
                                                uint8_t oled_action,
                                                uint8_t step_count)
{
    if (s_action_cb) s_action_cb(ep_idx, cmd, pos_pct, oled_action,
                                  step_count, s_action_ud);
}

static void _compute_pair_code(void)
{
    char buf[chip::kManualSetupLongCodeCharLength + 1] = {0};
    chip::MutableCharSpan span(buf);
    CHIP_ERROR err = ::GetManualPairingCode(span, chip::RendezvousInformationFlag::kBLE);
    if (err == CHIP_NO_ERROR) {
        snprintf(s_pair_code, sizeof(s_pair_code), "%s", buf);
    } else {
        snprintf(s_pair_code, sizeof(s_pair_code), "20202021");
    }
}

/* Matter QR 코드 내용 문자열("MT:...") — OLED 페어링 화면에서 esp_qrcode 로
 *  2D 비트맵 인코딩해 표시. VID/PID/discriminator/passcode 로 산출(고정값)
 *  이라 1회 계산 후 정적 캐시. 실패 시 빈 문자열(→ 화면은 PIN 으로 폴백). */
extern "C" const char *matter_blinds_get_qr_payload(void)
{
    static char s_qr[40] = {0};
    if (s_qr[0]) return s_qr;
    char buf[40] = {0};
    chip::MutableCharSpan span(buf);
    CHIP_ERROR err = ::GetQRCode(
        span, chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));
    if (err == CHIP_NO_ERROR) {
        snprintf(s_qr, sizeof(s_qr), "%s", buf);
    }
    return s_qr;
}

extern "C" void matter_blinds_init(blind_manager_t *mgr,
                                    matter_blind_action_cb_t action_cb,
                                    void *user_data)
{
    (void)mgr;                 /* 공유 g_mgr 사용 — app_main.cpp 소유 */
    s_action_cb = action_cb;
    s_action_ud = user_data;
}

extern "C" const char *matter_blinds_start(void)
{
    /* Matter 는 app_main.cpp 가 이미 esp_matter::start() 로 시작.
     * 여기서는 페어링 코드만 산출해 반환. */
    _compute_pair_code();
    ESP_LOGI(TAG, "Matter manual pairing code: %s", s_pair_code);
    return s_pair_code;
}

extern "C" bool matter_blinds_is_commissioning_complete(void)
{
    return g_commissioning_complete;
}

extern "C" bool matter_blinds_is_commissioned(void)
{
    return chip::Server::GetInstance().GetFabricTable().FabricCount() > 0;
}

extern "C" bool matter_blinds_is_pairing_in_progress(void)
{
    return chip::Server::GetInstance().GetFailSafeContext().IsFailSafeArmed();
}

extern "C" const char *matter_blinds_get_last_pair_error(void)
{
    return "";   /* 진단 코드 미사용(검증 코어 이벤트핸들러는 표준) */
}

/* 메뉴 → "Matter Pair" : BLE 커미셔닝 윈도우 재오픈.
 * ★ 39-517 교훈: 이미 열린 윈도우는 닫지 않는다(팩토리 광고 보존).
 *   닫혀 있을 때만 새로 연다. CHIP 스레드에서 실행. */
static void _open_cw(intptr_t)
{
    auto &cwm = chip::Server::GetInstance().GetCommissioningWindowManager();
    if (cwm.IsCommissioningWindowOpen()) {
        ESP_LOGI(TAG, "Commissioning window 이미 열림 — 유지");
        return;
    }
    CHIP_ERROR err = cwm.OpenBasicCommissioningWindow(
        chip::System::Clock::Seconds16(15 * 60));
    if (err != CHIP_NO_ERROR)
        ESP_LOGE(TAG, "OpenBasicCommissioningWindow 실패: %" CHIP_ERROR_FORMAT, err.Format());
    else
        ESP_LOGI(TAG, "BLE commissioning window 재오픈(15분)");
}

/* ── ★2026-08-11 무선 게이팅 (배터리 절약, 사용자 요청) ──────────────────────
 *  왜: Thread 기기로 **등록되지 않은 상태**에서는 라디오가 할 일이 없는데도 계속 켜져
 *  전류를 먹는다(측정 84mA 중 대부분이 무선). 등록 전에는 꺼두고, 사용자가 설정 메뉴에서
 *  페어링을 시작할 때만 켜면 그동안의 소모가 사라진다.
 *
 *  구현 메모:
 *   · 원시 OpenThread API(otThreadSetEnabled) 대신 **CHIP 매니저 API**를 쓴다.
 *     CHIP 내부 상태와 어긋나지 않고, 커미셔닝 시 CHIP 이 자격증명을 주입하며
 *     스스로 Thread 를 다시 켜는 흐름과도 충돌하지 않는다.
 *   · fabric 조작과 마찬가지로 CHIP 스레드에서만 안전하므로 ScheduleWork 로 위임한다.
 *   · BLE 광고도 같이 끈다(미등록 상태에서 15분간 광고하며 전류를 먹던 것). */
static bool s_radio_on = true;      /* 부팅 직후는 스택이 켠 상태 */

static void _radio_apply(intptr_t on)
{
    const bool en = (on != 0);
    /* Thread: 자격증명이 없으면 어차피 attach 안 하지만, 라디오 자체를 내려야 절약된다. */
    CHIP_ERROR err = chip::DeviceLayer::ThreadStackMgr().SetThreadEnabled(en);
    if (err != CHIP_NO_ERROR) {
        ESP_LOGW(TAG, "[RADIO] Thread %s 실패: %" CHIP_ERROR_FORMAT,
                 en ? "ON" : "OFF", err.Format());
    }
    /* BLE 광고: 끄면 미등록 상태의 상시 광고 소모가 사라진다. */
    chip::DeviceLayer::ConnectivityMgr().SetBLEAdvertisingEnabled(en);
    ESP_LOGW(TAG, "[RADIO] 무선 %s (Thread+BLE)", en ? "ON" : "OFF");
}

extern "C" void matter_blinds_set_radio_enabled(bool on)
{
    if (s_radio_on == on) return;      /* 같은 상태면 무동작(로그·작업 폭주 방지) */
    s_radio_on = on;
    chip::DeviceLayer::PlatformMgr().ScheduleWork(_radio_apply, on ? 1 : 0);
}

extern "C" bool matter_blinds_get_radio_enabled(void) { return s_radio_on; }

extern "C" const char *matter_blinds_open_commissioning_window(void)
{
    /* ★무선이 꺼져 있으면(미등록 절전 상태) 먼저 켠다 — 안 켜면 커미셔너가 못 찾는다. */
    matter_blinds_set_radio_enabled(true);
    _compute_pair_code();
    chip::DeviceLayer::PlatformMgr().ScheduleWork(_open_cw, 0);
    ESP_LOGI(TAG, "Commissioning window 재오픈 요청 — 코드: %s", s_pair_code);
    return s_pair_code;
}

/* 메뉴 → "Thread Reset"(완전 초기화) : Matter fabric 전체 삭제(= SmartThings 페어링 해제).
 *  fabric 조작은 CHIP 스레드에서만 안전하므로 ScheduleWork 로 위임. 호출측(somfy_app)이
 *  잠시 뒤 esp_restart 하므로 여기서는 삭제만 한다. Delete 중 iterator 무효화를 피하려
 *  인덱스를 먼저 수집한 뒤 삭제. */
static void _delete_all_fabrics(intptr_t)
{
    auto &ft = chip::Server::GetInstance().GetFabricTable();
    chip::FabricIndex idxs[CHIP_CONFIG_MAX_FABRICS];
    uint8_t n = 0;
    for (const auto &fb : ft) {
        if (n < CHIP_CONFIG_MAX_FABRICS) idxs[n++] = fb.GetFabricIndex();
    }
    ESP_LOGW(TAG, "Matter fabric 전체 삭제: %u개", (unsigned)n);
    for (uint8_t i = 0; i < n; i++) {
        CHIP_ERROR err = ft.Delete(idxs[i]);
        if (err != CHIP_NO_ERROR)
            ESP_LOGE(TAG, "fabric[%u] 삭제 실패: %" CHIP_ERROR_FORMAT, idxs[i], err.Format());
    }
}

extern "C" void matter_blinds_remove_all_fabrics(void)
{
    chip::DeviceLayer::PlatformMgr().ScheduleWork(_delete_all_fabrics, 0);
    ESP_LOGI(TAG, "Matter fabric 삭제 요청(CHIP 스레드)");
}

/* 로컬 버튼으로 블라인드를 움직였을 때 SmartThings UI 에 위치를 반영.
 * ★ 안전: ember attribute write 는 Matter 스레드에서만. 본 함수는
 *   RF worker 태스크에서 호출되므로 우선 로그만(무해 stub).
 *   추후 PlatformMgr::ScheduleWork 로 안전 반영 예정. */
extern "C" void matter_blinds_update_position(uint8_t endpoint_idx,
                                               uint8_t position_pct)
{
    ESP_LOGD(TAG, "update_position ep_idx=%u pos=%u%% (UI 반영 stub)",
             endpoint_idx, position_pct);
}

/* ═══════════════════════════════════════════════
   Matter OTA (Thread 무선 펌웨어 업데이트) 브리지
   ──────────────────────────────────────────────
   OTA Requestor 는 esp_matter::start() 가 자동 초기화(CONFIG_ENABLE_OTA_REQUESTOR).
   여기서는 현재 버전·상태 조회 + 수동 QueryImage 트리거만 C API 로 노출한다.
═══════════════════════════════════════════════ */
extern "C" const char *matter_ota_version_str(void)
{
    /* 실행 중 펌웨어의 PROJECT_VER 문자열 (예 "3.5"). */
    const esp_app_desc_t *d = esp_app_get_description();
    return (d && d->version[0]) ? d->version : "?";
}

extern "C" matter_ota_state_t matter_ota_get_state(uint8_t *progress_pct)
{
    if (progress_pct) *progress_pct = 0;
    chip::OTARequestorInterface *req = chip::GetRequestorInstance();
    if (!req) return MATTER_OTA_UNKNOWN;

    using chip::app::Clusters::OtaSoftwareUpdateRequestor::OTAUpdateStateEnum;
    OTAUpdateStateEnum st = req->GetCurrentUpdateState();
    switch (st) {
        case OTAUpdateStateEnum::kIdle:
            return MATTER_OTA_IDLE;
        case OTAUpdateStateEnum::kQuerying:
        case OTAUpdateStateEnum::kDelayedOnQuery:
            return MATTER_OTA_QUERYING;
        case OTAUpdateStateEnum::kDownloading: {
            if (progress_pct) {
                chip::app::DataModel::Nullable<uint8_t> p;
                if (req->GetUpdateStateProgressAttribute(/*root ep*/ 0, p) == CHIP_NO_ERROR
                    && !p.IsNull()) {
                    *progress_pct = p.Value();
                }
            }
            return MATTER_OTA_DOWNLOADING;
        }
        case OTAUpdateStateEnum::kApplying:
        case OTAUpdateStateEnum::kDelayedOnApply:
        case OTAUpdateStateEnum::kRollingBack:
            return MATTER_OTA_APPLYING;
        case OTAUpdateStateEnum::kDelayedOnUserConsent:
            return MATTER_OTA_DELAYED;
        default:
            return MATTER_OTA_UNKNOWN;
    }
}

/* CHIP 스레드에서 실행 — 기본 Provider 에 즉시 QueryImage. */
static void _ota_trigger_work(intptr_t)
{
    chip::OTARequestorInterface *req = chip::GetRequestorInstance();
    if (!req) {
        ESP_LOGW(TAG, "[OTA] requestor 없음 — 트리거 무시");
        return;
    }
    CHIP_ERROR err = req->TriggerImmediateQuery(chip::kUndefinedFabricIndex);
    if (err != CHIP_NO_ERROR) {
        ESP_LOGW(TAG, "[OTA] TriggerImmediateQuery 실패: %" CHIP_ERROR_FORMAT
                 " (기본 Provider 미설정 가능)", err.Format());
    } else {
        ESP_LOGI(TAG, "[OTA] 수동 업데이트 확인 트리거됨");
    }
}

extern "C" bool matter_ota_trigger_check(void)
{
    CHIP_ERROR err = chip::DeviceLayer::PlatformMgr().ScheduleWork(_ota_trigger_work, 0);
    return err == CHIP_NO_ERROR;
}
