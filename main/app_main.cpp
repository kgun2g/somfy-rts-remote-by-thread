/*
 * app_main.cpp — Matter 코어 (노드/엔드포인트/WindowCovering delegate/
 *   OpenThread/커미셔닝) 및 CC1101·Somfy RTS·blind_manager 소유.
 * Somfy RTS 블라인드 컨트롤러(채널수 BLIND_MAX_COUNT — H2=3/C6=8). v3.5.
 * Public Domain (CC0). 무보증.
 */

#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_console.h>
#include <esp_pm.h>          /* esp_pm_dump_locks — 절전 진단 */
#include <string.h>
#include <stdlib.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include "common_macros.h"
#include "log_heap_numbers.h"
#include "boot_diag.h"
#include <driver/gpio.h>
#include <esp_rom_sys.h>
#include "boards/board_select.h"   /* BOARD_PIN_CHG_STAT (VBUS 분압) */

#include <app_priv.h>
#include <app_reset.h>
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <app/clusters/window-covering-server/window-covering-delegate.h>
#include <app/clusters/window-covering-server/window-covering-server.h>
#include <app-common/zap-generated/cluster-objects.h>
#include <app-common/zap-generated/attributes/Accessors.h>

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
#include <esp_matter_providers.h>
#include <lib/support/Span.h>
#ifdef CONFIG_SEC_CERT_DAC_PROVIDER
#include <platform/ESP32/ESP32SecureCertDACProvider.h>
#elif defined(CONFIG_FACTORY_PARTITION_DAC_PROVIDER)
#include <platform/ESP32/ESP32FactoryDataProvider.h>
#endif
using namespace chip::DeviceLayer;
#endif

#include <esp_matter_providers.h>     // esp_matter::set_custom_commissionable_data_provider
#include "efuse_commissionable.h"     // eFuse 기기 고유 discriminator/passcode

static const char *TAG = "app_main";
uint16_t light_endpoint_id = 0;

/* ── Somfy RTS / CC1101 RF 드라이버 ──
   이 인스턴스들을 C-linkage 전역으로 공개하여 somfy_app.c(OLED/버튼/메뉴)
   와 단일 인스턴스를 공유한다. g_commissioning_complete 는 somfy_app.c 의
   지연-시작 게이트가 폴링한다. */
extern "C" {
#include "cc1101.h"
#include "somfy_rts.h"
#include "blind_manager.h"
cc1101_t        g_cc1101;
somfy_rts_t     g_somfy;
blind_manager_t g_mgr;
uint16_t        g_wc_ep_ids[BLIND_MAX_COUNT] = {0};
bool            g_rf_ready = false;
volatile bool   g_commissioning_complete = false;
/* somfy_app.c 진입점 (별도 태스크) */
void somfy_app_run(void *arg);
/* test/somfy_cases/oled_only_test.c — OLED 단독 부팅 테스트(OLED_ONLY_TEST 빌드) */
void oled_only_test(void);
/* matter_blinds_shim.cpp: SmartThings 명령 수신 시 somfy_app 의
   _matter_action_cb 호출 (OLED 깨우기/액션표시 + 단일 RF 큐). */
void matter_blinds_invoke_action_cb(uint8_t ep_idx, somfy_command_t cmd,
                                    uint8_t pos_pct, uint8_t oled_action,
                                    uint8_t step_count);
}
#define s_cc1101     g_cc1101
#define s_somfy      g_somfy
#define s_mgr        g_mgr
#define s_wc_ep_ids  g_wc_ep_ids
#define s_rf_ready   g_rf_ready

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

/* ── WindowCovering Delegate ──────────────────────────────────────────
 *  SmartThings 의 열기/닫기/위치이동은 WindowCovering *명령*
 *  (UpOrOpen / DownOrClose / GoToLiftPercentage) 으로 온다. CHIP
 *  window-covering-server 는 이 명령을 받으면 Target* 속성을 갱신한 뒤
 *  Delegate::HandleMovement() 를 호출한다. delegate 미설정 시
 *  "WindowCovering has no delegate set" 로그만 찍히고 아무 동작 안 함
 *  (= 조작 불가). 여기서 delegate 가 Target → Somfy RF 로 변환하고
 *  Current* 속성을 갱신해 SmartThings 상태도 반영한다. */

/* ★ SmartThings 슬라이더 동기화 문제 해결:
 *  SmartThings WindowCovering UI 는 Lift/Tilt 슬라이더가 mirror 되어 어느
 *  한쪽만 만져도 양쪽 모두 새 target 으로 갱신 → HandleMovement(Lift) +
 *  HandleMovement(Tilt) 가 ~150ms 안에 연속 호출된다.
 *  → 두 RF 명령(UP/DOWN + TILT_UP/DOWN)이 큐에 쌓여 모터가 첫 명령으로
 *    주행 시작 직후 두 번째 명령에 의해 중단·tilt 짧은 step 으로 전환되어
 *    "조금밖에 안 움직임" 현상 발생.
 *  → 같은 EP 에서 400ms 이내 후속 호출은 companion 으로 보고 RF/OLED 모두
 *    SKIP. 사용자가 진짜로 두 슬라이더를 따로 만지려면 400ms 이상 텀을 두면
 *    됨(현실적으로 사용자 손가락 속도로는 충분히 분리된다).
 *  Open/Close 버튼: Lift 가 먼저 들어와 RF 송신 → Tilt 는 companion 으로 skip.
 *  Lift 슬라이더 단독: Lift 만 들어옴(또는 Tilt 가 동시에 와도 Lift 먼저 처리됨).
 *  Tilt 슬라이더 단독: Tilt 만 들어옴(또는 Lift 가 mirror 로 와도 Tilt 가 먼저). */
#define WC_COMPANION_GUARD_US  (400 * 1000)

static int64_t s_lastLiftUs = 0;
static uint16_t s_lastLiftEp = 0xFFFF;
static int64_t s_lastTiltUs = 0;
static uint16_t s_lastTiltEp = 0xFFFF;

/* ★ Stop 버튼(일시정지) 판별: SmartThings 가 movement 직후에도 HandleStopMotion
 *  을 자동 호출할 수 있어 무조건 MY 를 보내면 열기/닫기가 즉시 취소된다.
 *  마지막 movement 시각 + EP 를 기록해두고 HandleStopMotion 호출 시 그 이후
 *  일정 시간이 지났으면 = 사용자 명시 정지 → MY 송신, 그 안이면 = auto → 무시. */
static int64_t s_lastMoveUs = 0;
static uint16_t s_lastMoveEp = 0xFFFF;
#define WC_STOP_AUTOCALL_GUARD_US  (500 * 1000)   /* 500ms 이내 = auto-stop 무시 */

class SomfyWCDelegate : public chip::app::Clusters::WindowCovering::Delegate
{
public:
    CHIP_ERROR HandleMovement(chip::app::Clusters::WindowCovering::WindowCoveringType type) override
    {
        int idx = -1;
        for (int i = 0; i < BLIND_MAX_COUNT; i++) {
            if (s_wc_ep_ids[i] == mEndpoint) { idx = i; break; }
        }

        /* oled_action_t (oled_ui.h): 1=UP 2=DOWN 3=STOP 4=TILT_UP 5=TILT_DN */
        enum { OA_UP = 1, OA_DOWN = 2, OA_STOP = 3, OA_TUP = 4, OA_TDN = 5 };
        using namespace chip::app::Clusters::WindowCovering;
        if (type == WindowCoveringType::Tilt) {
            chip::app::DataModel::Nullable<chip::Percent100ths> tgt, cur;
            if (Attributes::TargetPositionTiltPercent100ths::Get(mEndpoint, tgt) == chip::Protocols::InteractionModel::Status::Success
                && !tgt.IsNull()) {
                uint16_t v = tgt.Value();
                /* ★ Tilt 슬라이더 = 슬랫 각도 조절 → 정품 Tilt 커맨드 사용
                 *  (UP/DOWN 이 아닌 TILT_UP/TILT_DOWN, cmd nibble 0xB).
                 *  방향: target > current = 슬랫 닫는 방향(TILT_DN), 그 반대=TILT_UP. */
                bool hasCur = (Attributes::CurrentPositionTiltPercent100ths::Get(mEndpoint, cur)
                                  == chip::Protocols::InteractionModel::Status::Success)
                              && !cur.IsNull();
                bool goDown;
                if (hasCur && cur.Value() != v) {
                    goDown = (v > cur.Value());
                } else {
                    goDown = (v >= 5000);
                }
                somfy_command_t cmd = goDown ? SOMFY_CMD_TILT_DOWN : SOMFY_CMD_TILT_UP;
                uint8_t oa         = goDown ? OA_TDN : OA_TUP;
                /* ★ Lift 가 최근(400ms 이내) 같은 EP 에서 호출됐으면 이번 Tilt 는
                 *  SmartThings mirror/Open/Close 의 companion 으로 보고 RF·OLED
                 *  모두 SKIP — Lift 모션을 끝까지 보장. Current* 는 슬라이더
                 *  UI 동기화를 위해 항상 갱신. */
                int64_t now = esp_timer_get_time();
                bool companion = (s_lastLiftEp == mEndpoint) &&
                                 (now - s_lastLiftUs) <= WC_COMPANION_GUARD_US;
                s_lastTiltUs = now;
                s_lastTiltEp = mEndpoint;
                if (companion) {
                    ESP_LOGI(TAG, "[WC] ep=%u TILT companion of recent LIFT — RF/OLED skip", mEndpoint);
                } else {
                    /* ★ Tilt 슬라이더 7단계 매핑 — 정품 tilt 리모컨이 0~100%
                     *  사이를 7 detent 로 처리한다(사용자 실측). 슬라이더 delta
                     *  에 따라 1~7 step burst 송신:
                     *    steps = round(|target-current| × 7 / 10000)
                     *    delta 1429 (=100/7%) → 1 step
                     *    delta 10000 (=100%)  → 7 steps
                     *  current 미상이면 50% 기준 차이로 fallback. 최소 1 보장. */
                    uint16_t curV = hasCur ? cur.Value() : 5000;
                    int diff = (int)v - (int)curV;
                    if (diff < 0) diff = -diff;
                    int steps = (diff * 7 + 5000) / 10000;   /* nearest 1/7 */
                    if (steps < 1) steps = 1;
                    if (steps > 7) steps = 7;
                    s_lastMoveUs = now;
                    s_lastMoveEp = mEndpoint;
                    ESP_LOGI(TAG, "[WC] ep=%u idx=%d TILT=%u cmd=%d oa=%u steps=%d",
                             mEndpoint, idx, v, cmd, oa, steps);
                    if (idx >= 0) {
                        matter_blinds_invoke_action_cb((uint8_t)idx, cmd,
                                                       (uint8_t)(v / 100), oa,
                                                       (uint8_t)steps);
                    }
                }
                Attributes::CurrentPositionTiltPercent100ths::Set(mEndpoint, tgt);
            }
        } else {
            chip::app::DataModel::Nullable<chip::Percent100ths> tgt, cur;
            if (Attributes::TargetPositionLiftPercent100ths::Get(mEndpoint, tgt) == chip::Protocols::InteractionModel::Status::Success
                && !tgt.IsNull()) {
                uint16_t v = tgt.Value();
                uint8_t pct = (uint8_t)(v / 100);
                /* ★ Lift 슬라이더 = 방향 기반 (Somfy RTS 는 위치 피드백 없어
                 *  중간값 정밀 도달 불가). target > current = 닫는 방향(DOWN),
                 *  target < current = 여는 방향(UP), 같으면 무동작.
                 *  → 사용자가 슬라이더를 어디로 끌든 모터는 그 방향대로 끝까지
                 *  주행한다(슬라이더 값은 trigger 일 뿐 도달점 아님). */
                bool hasCur = (Attributes::CurrentPositionLiftPercent100ths::Get(mEndpoint, cur)
                                  == chip::Protocols::InteractionModel::Status::Success)
                              && !cur.IsNull();
                somfy_command_t cmd;
                uint8_t oa;
                if (hasCur && cur.Value() != v) {
                    bool goDown = (v > cur.Value());
                    cmd = goDown ? SOMFY_CMD_DOWN : SOMFY_CMD_UP;
                    oa  = goDown ? OA_DOWN       : OA_UP;
                } else {
                    /* current 모름 또는 동일 — 끝값 기준 fallback */
                    if (pct >= 95)      { cmd = SOMFY_CMD_DOWN; oa = OA_DOWN; }
                    else if (pct <= 5)  { cmd = SOMFY_CMD_UP;   oa = OA_UP;   }
                    else                { cmd = SOMFY_CMD_MY;   oa = OA_STOP; }
                }
                /* ★ Tilt 가 최근 같은 EP 에서 호출됐으면 이번 Lift 는 mirror
                 *  companion 으로 보고 RF/OLED skip (Tilt 슬라이더 단독 조작 시
                 *  Lift 가 따라오는 케이스). 그 외엔 정상 Lift 처리 + 추후 Tilt
                 *  companion 가드용 timestamp 기록. */
                int64_t now = esp_timer_get_time();
                bool companion = (s_lastTiltEp == mEndpoint) &&
                                 (now - s_lastTiltUs) <= WC_COMPANION_GUARD_US;
                s_lastLiftUs = now;
                s_lastLiftEp = mEndpoint;
                if (companion) {
                    ESP_LOGI(TAG, "[WC] ep=%u LIFT companion of recent TILT — RF/OLED skip", mEndpoint);
                } else {
                    s_lastMoveUs = now;
                    s_lastMoveEp = mEndpoint;
                    ESP_LOGI(TAG, "[WC] ep=%u idx=%d LIFT=%u%% cmd=%d oa=%u", mEndpoint, idx, pct, cmd, oa);
                    if (idx >= 0) {
                        matter_blinds_invoke_action_cb((uint8_t)idx, cmd, pct, oa, /*step=*/1);
                    }
                }
                Attributes::CurrentPositionLiftPercent100ths::Set(mEndpoint, tgt);
            }
        }
        return CHIP_NO_ERROR;
    }

    CHIP_ERROR HandleStopMotion() override
    {
        int idx = -1;
        for (int i = 0; i < BLIND_MAX_COUNT; i++) {
            if (s_wc_ep_ids[i] == mEndpoint) { idx = i; break; }
        }
        /* ★ SmartThings "일시정지" → Somfy MY 송신.
         *  단, CHIP 서버가 open/close 명령 처리 직후에도 HandleStopMotion 을
         *  자동 호출할 수 있어 그 경우는 무시(아니면 모든 열기/닫기가 즉시
         *  취소됨). 마지막 movement 시각과 같은 EP 면 500ms 가드 적용. */
        int64_t since = esp_timer_get_time() - s_lastMoveUs;
        bool autoCall = (s_lastMoveEp == mEndpoint) && (since < WC_STOP_AUTOCALL_GUARD_US);
        if (autoCall) {
            ESP_LOGD(TAG, "[WC] ep=%u idx=%d STOP auto-call 무시 (%lldms after move)",
                     mEndpoint, idx, (long long)(since / 1000));
            return CHIP_NO_ERROR;
        }
        /* oled_action_t: 3 = STOP (somfy_app.c 의 매핑) */
        constexpr uint8_t OA_STOP_LOCAL = 3;
        ESP_LOGI(TAG, "[WC] ep=%u idx=%d STOP → MY 송신", mEndpoint, idx);
        if (idx >= 0) {
            matter_blinds_invoke_action_cb((uint8_t)idx, SOMFY_CMD_MY,
                                           /*pct=*/50, OA_STOP_LOCAL, /*step=*/1);
        }
        return CHIP_NO_ERROR;
    }
};

/* 엔드포인트당 1개 (Delegate::SetEndpoint 가 인스턴스별 mEndpoint 설정) */
static SomfyWCDelegate s_wc_delegates[BLIND_MAX_COUNT];

/* eFuse 기반 기기 고유 커미셔닝 데이터(discriminator/passcode) provider */
static EfuseCommissionableDataProvider g_efuse_comm_provider;

constexpr auto k_timeout_seconds = 300;

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
extern const uint8_t cd_start[] asm("_binary_certification_declaration_der_start");
extern const uint8_t cd_end[] asm("_binary_certification_declaration_der_end");

const chip::ByteSpan cdSpan(cd_start, static_cast<size_t>(cd_end - cd_start));
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

/* ★ Phase 1: WindowCovering 클러스터 서버는 CONFIG_SUPPORT_WINDOW_
 *   COVERING_CLUSTER=y 로 window-covering-server.cpp 가 정식 컴파일됨
 *   (우리 원본 프로젝트와 동일). 별도 stub 불필요. */

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        g_commissioning_complete = true;   /* somfy_app.c 지연-시작 해제 */
        MEMORY_PROFILER_DUMP_HEAP_STAT("commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        MEMORY_PROFILER_DUMP_HEAP_STAT("commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved: {
        ESP_LOGI(TAG, "Fabric removed successfully");
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            /* ★ Thread 전용 기기 요구사항 (39-517 재등록 실패 방지):
             *  fabric 제거 후 kDnssdOnly(BLE 없이 DNS-SD만)로 윈도우를
             *  열면 — IP 연결이 남아있다는 가정이라 —
             *  그러나 Thread 전용 기기는 fabric 제거 시 Thread 망에서도
             *  떨어져 IP 도달이 불가 → SmartThings 가 BLE 로 못 찾아
             *  재등록이 39-517 로 실패한다.
             *  올바른 동작: 재부팅. 0 fabric 으로 부팅하면 esp-matter 가
             *  자동으로 BLE 광고 커미셔닝(최초 성공했던 그 상태)에 진입.
             *  NVS 에 fabric 제거가 영속되도록 2초 뒤 재부팅. */
            ESP_LOGW(TAG, "마지막 fabric 제거 — 2초 뒤 재부팅 (clean BLE 커미셔닝 복귀)");
            chip::DeviceLayer::SystemLayer().StartTimer(
                chip::System::Clock::Seconds16(2),
                [](chip::System::Layer *, void *) {
                    ESP_LOGW(TAG, "재부팅 — factory BLE commissioning 으로 복귀");
                    esp_restart();
                },
                nullptr);
        }
        break;
    }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        MEMORY_PROFILER_DUMP_HEAP_STAT("BLE deinitialized");
        break;

    default:
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    /* ── WindowCovering → Somfy RF 는 SomfyWCDelegate 가 단일 경로로 처리.
     *  여기(attribute 콜백)서 다시 보내면 동일 명령이 이중 송신되고
     *  방향 매핑도 충돌하므로 RF 매핑을 제거한다. (delegate 가 권위 경로) */

    if (type == PRE_UPDATE) {
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }

    return err;
}

/* ── 시리얼 콘솔 RF 제어 (자동 테스트 하네스) ──────────────────────
 *  UART0 의 Matter CHIP shell 에 tx/sel 명령을 얹어, PC(pyserial 등)가
 *  "tx updown" 한 줄로 combo(UP_DOWN) RF 를 즉시 송신하게 한다. 버튼과
 *  같은 RF 큐 경로(_send_command_ex)를 쓰되 session-gate 를 거치지 않고
 *  cmd 를 그대로 쏜다 → 물리 버튼 없이 캡처/분석 루프를 무인 자동화. */
extern "C" void somfy_app_console_tx(int cmd, uint32_t hold_ms);
extern "C" void somfy_app_console_select(int n);
extern "C" void somfy_app_console_cycle(int dir);
extern "C" void somfy_app_console_setfreq(int idx, float mhz);
extern "C" void somfy_app_console_printfreq(void);
extern "C" void somfy_app_batlog_dump(void);
extern "C" void somfy_app_batlog_clear(void);
extern "C" void somfy_app_console_usbsim(int off);
extern "C" void somfy_app_vibelog_dump(void);   /* ★2026-08-13 진동 진단 기록 */
extern "C" void somfy_app_vibelog_clear(void);
extern "C" void btn_handler_int_diag_request(void);  /* ★2026-08-13 `~INT` 관찰 진단 */
static int scmd_tx(int argc, char **argv){
    if(argc<2){ printf("usage: tx up|down|updown|myup|mydown|my|prog [hold_ms]\n"); return 0; }
    const char *a=argv[1]; int cmd=-1;
    if(!strcmp(a,"up"))cmd=SOMFY_CMD_UP; else if(!strcmp(a,"down"))cmd=SOMFY_CMD_DOWN;
    else if(!strcmp(a,"updown"))cmd=SOMFY_CMD_UP_DOWN; else if(!strcmp(a,"myup"))cmd=SOMFY_CMD_MY_UP;
    else if(!strcmp(a,"mydown"))cmd=SOMFY_CMD_MY_DOWN; else if(!strcmp(a,"my")||!strcmp(a,"stop"))cmd=SOMFY_CMD_MY;
    else if(!strcmp(a,"prog"))cmd=SOMFY_CMD_PROG;
    if(cmd<0){ printf("unknown cmd: %s\n",a); return 0; }
    uint32_t hold=(argc>=3)?(uint32_t)strtoul(argv[2],NULL,10):0;
    somfy_app_console_tx(cmd, hold);
    printf("OK tx %s hold=%lu\n", a, (unsigned long)hold);
    return 0;
}
extern "C" void somfy_app_console_tx_byte8(int raw_b8, int cmd, uint32_t hold_ms);
/* byte8 캘리브레이션: tx8 <hex> [cmd] [hold_ms]  (cmd 기본 8=PROG) */
static int scmd_tx8(int argc, char **argv){
    if(argc<2){ printf("usage: tx8 <byte8 hex> [cmd=8] [hold_ms=0]\n"); return 0; }
    int b8 = (int)strtol(argv[1], NULL, 16);
    int cmd = (argc>=3)? (int)strtol(argv[2],NULL,0) : 8;
    uint32_t hold = (argc>=4)? (uint32_t)strtoul(argv[3],NULL,10) : 0;
    somfy_app_console_tx_byte8(b8, cmd, hold);
    printf("OK tx8 b8=0x%02X cmd=%d hold=%lu\n", b8 & 0xFF, cmd, (unsigned long)hold);
    return 0;
}
static int scmd_sel(int argc, char **argv){
    if(argc<2){ printf("usage: sel <0..%d | %d=ALL>\n", BLIND_MAX_COUNT-1, BLIND_SEL_ALL); return 0; }
    int n=atoi(argv[1]); somfy_app_console_select(n);
    printf("OK sel %d\n", n); return 0;
}
static int scmd_cyc(int argc, char **argv){   /* 블라인드 선택 순환 검증 (LEFT/RIGHT 무인) */
    int dir = (argc>1 && atoi(argv[1])<0) ? -1 : 1;
    somfy_app_console_cycle(dir);
    printf("OK cyc %d\n", dir); return 0;
}
static int scmd_freq(int argc, char **argv){  /* freq [idx mhz] : set(+NVS저장) 후 전체 출력 */
    if(argc>=3){ somfy_app_console_setfreq(atoi(argv[1]), (float)atof(argv[2])); }
    somfy_app_console_printfreq();
    printf("OK freq\n"); return 0;
}
static int scmd_reboot(int argc, char **argv){ (void)argc; (void)argv; printf("rebooting\n"); esp_restart(); return 0; }
/* ★부팅 진단 조회 — "bd" 로 언제든 직전/보관된 실패 부팅 기록을 다시 찍는다.
 *  부팅 직후 몇 초짜리 로그 창을 놓쳐도 되도록(실제로 놓쳐서 증거를 날렸다).
 *  "bd clear" 로 보관 기록 삭제. */
/* ★배터리 방전 기록 조회 — USB 없는 동안 NVS 에 쌓인 것을 꺼내 본다.
 *  "bl" 덤프 / "bl clear" 삭제. */
/* ★USB 를 물리적으로 뽑지 않고 배터리 모드 동작을 확인한다. "usbsim off" / "usbsim on". */
static int scmd_usbsim(int argc, char **argv){
    if (argc < 2) { printf("usage: usbsim off|on\n"); return 0; }
    somfy_app_console_usbsim(!strcmp(argv[1], "off") ? 1 : 0);
    printf("OK usbsim %s\n", argv[1]);
    return 0;
}
static int scmd_bl(int argc, char **argv){
    if (argc >= 2 && !strcmp(argv[1], "clear")) { somfy_app_batlog_clear(); printf("OK bl clear\n"); return 0; }
    somfy_app_batlog_dump();
    printf("OK bl\n");
    return 0;
}
/* ★2026-08-14 절전 진단 — light sleep 이 **실제로** 도는지 본다.
 *  CONFIG_PM_PROFILING=y 일 때 esp_pm_dump_locks 가 각 lock 의 보유시간과
 *  각 절전 모드 체류시간을 출력한다. 지금까지 `pm=3`(=esp_pm_configure 가
 *  light sleep 을 허용) 만 보고 "자고 있다"고 추정해 왔는데 그건 증거가 아니었다.
 *  이 명령으로 몇 시간짜리 방전 대신 즉시 확인한다.
 *  ※PM_PROFILING 은 런타임 오버헤드가 있으므로 **전류 측정 빌드에서는 끌 것**. */
static int scmd_pm(int argc, char **argv){
    (void)argc; (void)argv;
#if CONFIG_PM_ENABLE
    esp_pm_dump_locks(stdout);
#else
    printf("CONFIG_PM_ENABLE 이 꺼져 있음\n");
#endif
    printf("OK pm\n");
    return 0;
}
extern "C" void somfy_app_intpd_test(void);
/* `~INT` 풀다운 진단 — PCF측 고장 vs GPIO2 배선단선 을 가른다(somfy_app.c 주석 참조) */
static int scmd_intpd(int argc, char **argv){
    (void)argc; (void)argv;
    somfy_app_intpd_test();
    printf("OK intpd\n");
    return 0;
}
static int scmd_intdiag(int argc, char **argv){
    (void)argc; (void)argv;
    btn_handler_int_diag_request();
    printf("OK intdiag - 지금부터 15초간 버튼을 여러 번 눌렀다 떼세요 (그동안 버튼 기능은 멈춤)\n");
    return 0;
}
static int scmd_vl(int argc, char **argv){
    if (argc >= 2 && !strcmp(argv[1], "clear")) { somfy_app_vibelog_clear(); printf("OK vl clear\n"); return 0; }
    somfy_app_vibelog_dump();
    printf("OK vl\n");
    return 0;
}
static int scmd_bd(int argc, char **argv){
    if (argc >= 2 && !strcmp(argv[1], "clear")) { boot_diag_clear_fail(); printf("OK bd clear\n"); return 0; }
    boot_diag_log_prev();
    printf("OK bd\n");
    return 0;
}

/* ★★2026-08-11 "USB 없이 배터리만 연결하면 부팅 중 멈춘다" 수정 — VBUS 존재 판정.
 *
 *  왜 필요한가 (boot_diag 로 확정한 근거):
 *    실패 기록이 **app_main=8(콘솔 명령 등록 완료) + somfy_app=3/sub=10** 에서 멈췄다.
 *    즉 두 태스크가 **동시에** 굳었고, 배터리는 4.0V 로 멀쩡했으며(최저치도 동일)
 *    리셋사유는 항상 "전원투입"(브라운아웃/패닉/워치독 아님)이었다.
 *    두 경로가 같이 멈추는 공통 자원은 **콘솔 출력**뿐이다.
 *
 *  기전:
 *    `esp_matter::console::init()` 은 CHIP shell 을 **우선순위 5** 태스크로 띄운다
 *    (somfy_app=4, oled_ui=3 보다 높다). 이 태스크의 RunMainLoop 는 프롬프트를
 *    출력하는데, 보조 콘솔이 USB-Serial-JTAG 이라
 *    `esp_rom_usb_serial_putc` → `usb_serial_device_tx_flush()` 가 **호스트를 기다린다**.
 *    USB 가 없으면 이 대기가 길어지고, prio 5 가 아래 태스크를 굶겨 부팅이 끝나지 않는다.
 *    USB 를 꽂으면 호스트가 FIFO 를 비워 즉시 끝나므로 증상이 사라진다 — 관측과 일치.
 *
 *  조치: **USB 가 실제로 꽂혀 있을 때만** CHIP shell 콘솔을 시작한다.
 *    콘솔은 개발용이라 배터리 단독 동작에는 필요 없다(tx/sel/cyc/bd 도 USB 전용).
 *
 *  왜 usb_serial_jtag_is_connected() 가 아니라 VBUS 핀인가:
 *    그 API 는 SOF 패킷 기반이라 부팅 ~1초 시점엔 열거가 안 끝나 false 가 나올 수 있다
 *    (그러면 USB 로 개발할 때도 콘솔이 사라진다). VBUS 분압은 꽂는 즉시 HIGH 다. */
static bool _usb_vbus_present(void)
{
#if defined(BOARD_PIN_CHG_STAT) && BOARD_CHG_STAT_ACTIVE_HIGH
    gpio_config_t io = {};
    io.pin_bit_mask = 1ULL << BOARD_PIN_CHG_STAT;
    io.mode         = GPIO_MODE_INPUT;
    io.pull_up_en   = GPIO_PULLUP_DISABLE;
    /* 내부 풀다운을 켜면 안 된다: VBUS 분압(100k/150k, 출력임피던스 60k)을
     * 3.0V → 1.29V 로 끌어내려 USB 를 LOW 로 오독한다(board_select.h 주석 참조). */
    io.pull_down_en = BOARD_CHG_STAT_EXT_PULLDOWN ? GPIO_PULLDOWN_DISABLE
                                                  : GPIO_PULLDOWN_ENABLE;
    io.intr_type    = GPIO_INTR_DISABLE;
    gpio_config(&io);
    /* ★2026-08-12 다수결 판정 — 한 번만 읽으면 전원 투입 직후 분압이 채 안 올라온
     *  순간에 걸려 USB 를 놓칠 수 있다(실제로 콘솔이 안 떠 진단이 막혔다).
     *  10ms 간격 7회 중 과반이면 USB 로 본다. 애매하면 **켜는 쪽**으로 기울인다 —
     *  콘솔이 없으면 아무것도 진단할 수 없고, 절전 손해는 미미하다. */
    int hi = 0;
    for (int i = 0; i < 7; i++) {
        if (gpio_get_level((gpio_num_t)BOARD_PIN_CHG_STAT) == 1) hi++;
        esp_rom_delay_us(10000);
    }
    ESP_LOGW(TAG, "[CONSOLE] VBUS 판정: %d/7 HIGH (GPIO%d)", hi, BOARD_PIN_CHG_STAT);
    return hi >= 3;   /* 과반보다 느슨하게 — 켜는 쪽으로 기울임 */
#else
    return true;   /* VBUS 판정 수단이 없는 보드는 기존 동작 유지 */
#endif
}

extern "C" void app_main()
{
#if OLED_ONLY_TEST
    /* ★2026-07-22 OLED 단독 부팅 테스트 — 최상단 단락.
     *  Matter/Thread/BLE/CC1101 RF/PCF8574 버튼/배터리 ADC/진동/절전 등 이후 모든
     *  초기화를 건너뛰고 OLED I2C 만 단독 구동한다(무한 루프, 반환 안 함).
     *  → 다른 기능이 하나도 안 돌므로 OLED 외 나머지 핀 입력은 자동으로 무시됨.
     *  활성화: build.ps1 -OledTest (test/somfy_cases/oled_only_test.c). */
    oled_only_test();   /* 선언: 위 extern "C" 블록 (C 링키지) */
    return;
#endif

    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* ★2026-08-11 부팅 단계 기록 시작 — "USB 없이 배터리만 연결하면 부팅 중 멈춘다"
     *  진단용. USB 를 꽂는 순간 전원이 바뀌어 증상이 사라지므로 시리얼 로그를 볼 수
     *  없다 → 전원이 끊겨도 남는 NVS 에 단계를 남기고, 다음 부팅에 읽어 찍는다.
     *  (RTC 메모리는 배터리를 빼면 지워져 쓸 수 없다.) */
    boot_diag_begin();   /* stage 는 begin() 이 APP_MAIN 으로 시작 */

    MEMORY_PROFILER_DUMP_HEAP_STAT("Bootup");

    /* ── WS2812 LED 드라이버는 RMT 채널/인터럽트를 점유해 Somfy RMT 와
     *   충돌("No free interrupt inputs for RMT")한다. 본 제품엔 WS2812 LED
     *   가 없으므로 LED 드라이버 init 을 하지 않는다. 버튼은
     *   app_reset(공장초기화)용으로만 유지. */
    app_driver_handle_t button_handle = app_driver_button_init();
    app_reset_button_register(button_handle);

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;

    // node handle can be used to add/modify other endpoints.
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    MEMORY_PROFILER_DUMP_HEAP_STAT("node created");

    /* ★ Time Synchronization 클러스터를 Root Node(EP0)에 추가.
     *  esp-matter root_node 기본 구성엔 없으므로 수동 추가. 이러면
     *  SmartThings(커미셔너)가 페어링 시 + 주기적으로 SetUTCTime 명령을
     *  보내고, CHIP TimeSynchronizationCluster → SystemClock SetClock_RealTime
     *  → ESP32 settimeofday() 로 시스템 시각이 설정된다. OLED 시계가
     *  쓰는 time()/localtime() 이 SmartThings 허브 시각과 동기화됨
     *  (인터넷/SNTP 없이도). delegate 불필요(기본 SetUTCTime 처리). */
    {
        endpoint_t *root_ep = endpoint::get(node, 0);
        if (root_ep) {
            cluster::time_synchronization::config_t ts_cfg;
            cluster_t *ts = cluster::time_synchronization::create(
                root_ep, &ts_cfg, CLUSTER_FLAG_SERVER);
            ESP_LOGI(TAG, "Time Synchronization 클러스터 추가 (root EP0) %s",
                     ts ? "OK" : "실패");
        } else {
            ESP_LOGW(TAG, "root endpoint(0) 미발견 — TimeSync 클러스터 skip");
        }
    }

    /* BLIND_MAX_COUNT-endpoint WindowCovering 브리지 구성. init/이벤트/OpenThread/start
     *   는 esp-matter 표준 흐름을 그대로 사용한다. */
    {
        using namespace chip::app::Clusters;
        const uint32_t WC_FEATURES =
            (uint32_t)WindowCovering::Feature::kLift |
            (uint32_t)WindowCovering::Feature::kTilt |
            (uint32_t)WindowCovering::Feature::kPositionAwareLift |
            (uint32_t)WindowCovering::Feature::kPositionAwareTilt;

#if BOARD_MATTER_COMPOSED
        /* ★ composed device — Aggregator/bridged_node 없이 root 아래 WindowCovering
         *  endpoint 를 직접 부착. free 빠듯한 보드(H2)에서 Aggregator endpoint + 블라인드마다의
         *  BridgedDeviceBasicInformation 클러스터를 제거해 RAM 을 아낀다(~수십KB/노드).
         *  단일 기기의 다중 endpoint 로 노출(컨트롤러에서 기기 1개 + endpoint 여러개). */
        for (int i = 0; i < BLIND_MAX_COUNT; i++) {
            window_covering::config_t wc_config;
            wc_config.window_covering.type = 0x00;  // Rollershade
            wc_config.window_covering.feature_flags = WC_FEATURES;
            wc_config.window_covering.features.position_aware_lift.current_position_lift_percent_100ths = nullable<uint16_t>(0);
            wc_config.window_covering.features.position_aware_lift.target_position_lift_percent_100ths  = nullable<uint16_t>(0);
            wc_config.window_covering.features.position_aware_tilt.current_position_tilt_percent_100ths = nullable<uint16_t>(0);
            wc_config.window_covering.features.position_aware_tilt.target_position_tilt_percent_100ths  = nullable<uint16_t>(0);
            wc_config.window_covering.delegate = &s_wc_delegates[i];
            endpoint_t *ep = window_covering::create(node, &wc_config, ENDPOINT_FLAG_NONE, NULL);
            ABORT_APP_ON_FAILURE(ep != nullptr,
                ESP_LOGE(TAG, "Failed to create window_covering %d", i));

            uint16_t wid = endpoint::get_id(ep);
            if (i == 0) light_endpoint_id = wid;
            s_wc_ep_ids[i] = wid;
            s_wc_delegates[i].SetEndpoint(wid);
            ESP_LOGI(TAG, "WindowCovering[%d] ep=%d (composed, root 직속)", i, wid);
        }
#else
        /* ★ C6: Matter 브리지 구조: EP0(root) + Aggregator, 그 아래 bridged
         *  WindowCovering N개. 단일 노드에 동일타입 N개면 SmartThings 가 기기 1개로만
         *  표시 → 브리지로 두면 SmartThings/Apple/Google 모두 *별도 기기 타일*로
         *  노출(Matter 표준 다기기 방식). */
        aggregator::config_t agg_config;
        endpoint_t *agg = aggregator::create(node, &agg_config, ENDPOINT_FLAG_NONE, NULL);
        ABORT_APP_ON_FAILURE(agg != nullptr, ESP_LOGE(TAG, "Failed to create aggregator"));
        ESP_LOGI(TAG, "Aggregator endpoint_id %d", endpoint::get_id(agg));

        for (int i = 0; i < BLIND_MAX_COUNT; i++) {
            /* 1) bridged node 엔드포인트 (Bridged Device Basic Information 포함) */
            bridged_node::config_t bn_config;
            endpoint_t *ep = bridged_node::create(node, &bn_config,
                                                  ENDPOINT_FLAG_BRIDGE, NULL);
            ABORT_APP_ON_FAILURE(ep != nullptr,
                ESP_LOGE(TAG, "Failed to create bridged_node %d", i));

            /* 2) 그 위에 WindowCovering 디바이스타입 클러스터 추가 */
            window_covering::config_t wc_config;
            wc_config.window_covering.type = 0x00;  // Rollershade
            wc_config.window_covering.feature_flags = WC_FEATURES;
            wc_config.window_covering.features.position_aware_lift.current_position_lift_percent_100ths = nullable<uint16_t>(0);
            wc_config.window_covering.features.position_aware_lift.target_position_lift_percent_100ths  = nullable<uint16_t>(0);
            wc_config.window_covering.features.position_aware_tilt.current_position_tilt_percent_100ths = nullable<uint16_t>(0);
            wc_config.window_covering.features.position_aware_tilt.target_position_tilt_percent_100ths  = nullable<uint16_t>(0);
            wc_config.window_covering.delegate = &s_wc_delegates[i];
            esp_err_t we = window_covering::add(ep, &wc_config);
            ABORT_APP_ON_FAILURE(we == ESP_OK,
                ESP_LOGE(TAG, "window_covering::add 실패 ep%d", i));

            /* 3) Aggregator 를 부모로 설정 (브리지 트리 구성) */
            esp_matter::endpoint::set_parent_endpoint(ep, agg);

            /* 4) Bridged Device Basic Information NodeLabel = 블라인드 이름 */
            cluster_t *bdbi = cluster::get(ep, BridgedDeviceBasicInformation::Id);
            if (bdbi) {
                static char s_nm[BLIND_MAX_COUNT][16];
                snprintf(s_nm[i], sizeof(s_nm[i]), "Blind %d", i + 1);
                cluster::bridged_device_basic_information::attribute::
                    create_node_label(bdbi, s_nm[i], (uint16_t)strlen(s_nm[i]));
            }

            uint16_t wid = endpoint::get_id(ep);
            if (i == 0) light_endpoint_id = wid;
            s_wc_ep_ids[i] = wid;
            s_wc_delegates[i].SetEndpoint(wid);
            ESP_LOGI(TAG, "Bridged WindowCovering[%d] ep=%d (parent=Aggregator)",
                     i, wid);
        }
#endif
    }

    /* ── Phase 2: RF 드라이버 init — ★ esp_matter::start() 이전에 수행 ──
     *  RMT TX 채널 인터럽트 슬롯을 Matter/Thread/BLE 가 점유하기 전에 확보.
     *  (Matter 이후로 미루면 "No free interrupt inputs for RMT" abort.)
     *  이건 1회성 init — 커미셔닝 중 동시 부하 없음(Phase1 검증 패턴 유지).
     *  진짜 페어링 깨짐 원인은 RF 존재가 아니라 OLED 20fps/버튼 폴링 등
     *  '커미셔닝 중 과다 연속 태스크' → 그건 Phase 3 에서 지연 시작 처리. */
    blind_manager_init(&s_mgr);
    ESP_LOGI(TAG, "[RF] 블라인드 %d개 로드", s_mgr.count);
    /* ★ 단락(&&) 금지: cc1101_init 이 false 여도 somfy_rts_init 을 반드시
     *  호출해 s_somfy 를 일관 초기화한다. (이전엔 short-circuit 으로
     *  somfy_rts_init 미호출 → s_somfy.cc1101=NULL → 버튼 RF 송신 시
     *  cc1101_set_frequency(NULL) 크래시·재부팅.) RF 활성은 둘 다 OK 일
     *  때만. somfy_app 의 _do_rf_send 도 g_rf_ready 가드로 이중 보호. */
    bool cc_ok  = cc1101_init(&s_cc1101);
    bool rts_ok = somfy_rts_init(&s_somfy, &s_cc1101);
    if (cc_ok && rts_ok) {
        s_rf_ready = true;
        ESP_LOGI(TAG, "[RF] CC1101 + Somfy RTS init 완료");
    } else {
        s_rf_ready = false;
        ESP_LOGW(TAG, "[RF] RF init 실패/하드웨어 없음 (cc=%d rts=%d) — "
                      "RF 비활성, 페어링·UI 는 계속", cc_ok, rts_ok);
    }

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD && CHIP_DEVICE_CONFIG_ENABLE_WIFI_STATION
    // Enable secondary network interface
    secondary_network_interface::config_t secondary_network_interface_config;
    endpoint = endpoint::secondary_network_interface::create(node, &secondary_network_interface_config, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create secondary network interface endpoint"));
#endif

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
    auto * dac_provider = get_dac_provider();
#ifdef CONFIG_SEC_CERT_DAC_PROVIDER
    static_cast<ESP32SecureCertDACProvider *>(dac_provider)->SetCertificationDeclaration(cdSpan);
#elif defined(CONFIG_FACTORY_PARTITION_DAC_PROVIDER)
    static_cast<ESP32FactoryDataProvider *>(dac_provider)->SetCertificationDeclaration(cdSpan);
#endif
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

    /* ★ 기기 고유 커미셔닝 — eFuse 기반 discriminator/passcode (start 이전 필수).
     *   여러 대를 만들어도 같은 QR/PIN 으로 인한 BLE 커미셔닝 충돌이 없도록
     *   칩마다 다른 값을 산출한다. (DAC provider 는 별개라 그대로 유지)
     *   CONFIG 미설정 보드(H2 등)는 호출 안 함 → 기존 경로(EXAMPLE/factory) 유지. */
#ifdef CONFIG_CUSTOM_COMMISSIONABLE_DATA_PROVIDER
    esp_matter::set_custom_commissionable_data_provider(&g_efuse_comm_provider);
#endif

    /* Matter start */
    boot_diag_stage(BOOT_S1_MATTER_START);
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));
    boot_diag_stage(BOOT_S1_MATTER_OK);

    MEMORY_PROFILER_DUMP_HEAP_STAT("matter started");

    /* 부팅 시 이미 fabric 존재(기존 페어링 기기)면 커미셔닝 완료로 간주
       → somfy_app.c 가 즉시 OLED/버튼 시작. 신규면
       kCommissioningComplete 이벤트에서 set. */
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0) {
        g_commissioning_complete = true;
        ESP_LOGI(TAG, "기존 fabric 존재 — 커미셔닝 완료 상태로 시작");
    }

    /* 주변 애플리케이션(OLED/버튼/시계/메뉴)을 별도 태스크로 시작.
       내부에서 커미셔닝 완료까지 무거운 연속 태스크는 지연한다. */
    xTaskCreate(somfy_app_run, "somfy_app", 8192, NULL, 4, NULL);   /* composed 로 free 확보 → 전 보드 8192 통일(스택 안전 마진 회복) */
    boot_diag_stage(BOOT_S1_APP_TASK);

    /* Phase 1: light 전용 기본값 설정 제거 (WindowCovering 사용) */
    /* app_driver_light_set_defaults(light_endpoint_id); */

#if CONFIG_ENABLE_ENCRYPTED_OTA
    err = esp_matter_ota_requestor_encrypted_init(s_decryption_key, s_decryption_key_len);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialized the encrypted OTA, err: %d", err));
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

#if CONFIG_ENABLE_CHIP_SHELL
    boot_diag_stage(BOOT_S1_CONSOLE_ENTER);
    const bool _usb_on = _usb_vbus_present();
    ESP_LOGW(TAG, "[CONSOLE] VBUS=%d — CHIP shell %s", _usb_on ? 1 : 0,
             _usb_on ? "시작" : "생략(배터리 단독 부팅 보호)");
    if (_usb_on) {
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
    esp_matter::console::attribute_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    { const esp_console_cmd_t txc={ .command="tx", .help="Somfy RF tx (자동테스트)", .hint=NULL, .func=&scmd_tx, .argtable=NULL };
      esp_console_cmd_register(&txc);
      const esp_console_cmd_t slc={ .command="sel", .help="블라인드 선택", .hint=NULL, .func=&scmd_sel, .argtable=NULL };
      esp_console_cmd_register(&slc);
      const esp_console_cmd_t t8c={ .command="tx8", .help="byte8 캘리브레이션 tx8 <hex> [cmd] [hold_ms]", .hint=NULL, .func=&scmd_tx8, .argtable=NULL };
      esp_console_cmd_register(&t8c);
      const esp_console_cmd_t cyc={ .command="cyc", .help="블라인드 선택 순환 -1/1 (_blind_cycle 검증)", .hint=NULL, .func=&scmd_cyc, .argtable=NULL };
      esp_console_cmd_register(&cyc);
      const esp_console_cmd_t frq={ .command="freq", .help="freq [idx mhz] 주파수 조회/설정", .hint=NULL, .func=&scmd_freq, .argtable=NULL };
      esp_console_cmd_register(&frq);
      const esp_console_cmd_t rbt={ .command="reboot", .help="재부팅(esp_restart)", .hint=NULL, .func=&scmd_reboot, .argtable=NULL };
      esp_console_cmd_register(&rbt);
      const esp_console_cmd_t bdc={ .command="bd", .help="부팅 진단 조회 (bd / bd clear)", .hint=NULL, .func=&scmd_bd, .argtable=NULL };
      esp_console_cmd_register(&bdc);
      const esp_console_cmd_t blc={ .command="bl", .help="배터리 방전 기록 조회 (bl / bl clear)", .hint=NULL, .func=&scmd_bl, .argtable=NULL };
      esp_console_cmd_register(&blc);
      const esp_console_cmd_t vlc={ .command="vl", .help="진동센서 진단 기록 조회 (vl / vl clear)", .hint=NULL, .func=&scmd_vl, .argtable=NULL };
      esp_console_cmd_register(&vlc);
      const esp_console_cmd_t idc={ .command="intdiag", .help="~INT 선 관찰 (15초간 버튼 반복 조작 필요)", .hint=NULL, .func=&scmd_intdiag, .argtable=NULL };
      esp_console_cmd_register(&idc);
      const esp_console_cmd_t ipd={ .command="intpd", .help="~INT 풀다운 진단 (고장 위치 판별)", .hint=NULL, .func=&scmd_intpd, .argtable=NULL };
      esp_console_cmd_register(&ipd);
      const esp_console_cmd_t pmc={ .command="pm", .help="절전 진단 — PM lock 보유시간/절전모드 체류시간", .hint=NULL, .func=&scmd_pm, .argtable=NULL };
      esp_console_cmd_register(&pmc);
      const esp_console_cmd_t usc={ .command="usbsim", .help="배터리 모드 시뮬 (usbsim off|on)", .hint=NULL, .func=&scmd_usbsim, .argtable=NULL };
      esp_console_cmd_register(&usc); }
    boot_diag_stage(BOOT_S1_CONSOLE_CMDS);
    esp_matter::console::init();
    }   /* if (_usb_on) */
#endif
    boot_diag_stage(BOOT_S1_DONE);   /* app_main 끝까지 도달 */

    while (true) {
        MEMORY_PROFILER_DUMP_HEAP_STAT("Idle");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
