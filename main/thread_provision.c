/*
 * thread_provision.c
 * ─────────────────────────────────────────────────────────
 * Thread 네트워크 상태 추적 — esp-matter / OpenThread 이벤트 hook.
 * 실제 OpenThread 초기화 & 자격증명 관리는 esp-matter 가 담당.
 * ─────────────────────────────────────────────────────────
 */
#include "thread_provision.h"
#include "esp_log.h"
#include <string.h>

/* OpenThread 헤더 — Thread 가 활성화된 빌드에서만 컴파일.
 * sdkconfig 의 CONFIG_OPENTHREAD_ENABLED 가 켜져 있어야 함. */
#if CONFIG_OPENTHREAD_ENABLED
#include "esp_openthread.h"
#include "esp_openthread_lock.h"
#include "openthread/thread.h"
#include "openthread/dataset.h"
#include "openthread/instance.h"
#endif

static const char *TAG = "THREAD_PROV";

static thread_prov_state_t s_state = THREAD_PROV_STATE_IDLE;
static char s_network_name[17] = {0};

void thread_prov_init(void)
{
#if CONFIG_OPENTHREAD_ENABLED
    /* esp-matter::start() 가 OpenThread 스택을 자동 시작.
     * 본 함수는 초기 상태만 조회. */
    s_state = THREAD_PROV_STATE_DETACHED;
    ESP_LOGI(TAG, "Thread provisioning 모듈 초기화 (스택은 esp-matter 가 시작)");
#else
    ESP_LOGW(TAG, "OpenThread 비활성화 빌드 — Thread 기능 없음");
    s_state = THREAD_PROV_STATE_IDLE;
#endif
}

bool thread_prov_is_provisioned(void)
{
#if CONFIG_OPENTHREAD_ENABLED
    bool provisioned = false;
    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *inst = esp_openthread_get_instance();
    if (inst) {
        otOperationalDatasetTlvs dataset_tlvs;
        otError err = otDatasetGetActiveTlvs(inst, &dataset_tlvs);
        provisioned = (err == OT_ERROR_NONE && dataset_tlvs.mLength > 0);
    }
    esp_openthread_lock_release();
    return provisioned;
#else
    return false;
#endif
}

/* ★2026-08-16 (①진단) 원시 Thread 역할을 그대로 준다.
 *  사용자 신고: "권외로 나가면 안테나가 '-'(권외)가 아니라 'X'(미등록)로 바뀐다."
 *  'X' 는 FabricCount==0 일 때만 나와야 하므로 예상과 다르다 — 판정에 쓰이는
 *  세 값(fabric / attached / role)을 함께 찍어야 어디서 갈리는지 알 수 있다.
 *  반환은 otDeviceRole 원시값: 0=DISABLED 1=DETACHED 2=CHILD 3=ROUTER 4=LEADER. */
int thread_prov_get_role(void)
{
#if CONFIG_OPENTHREAD_ENABLED
    int r = -1;
    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *inst = esp_openthread_get_instance();
    if (inst) r = (int)otThreadGetDeviceRole(inst);
    esp_openthread_lock_release();
    return r;
#else
    return -1;
#endif
}

bool thread_prov_is_attached(void)
{
#if CONFIG_OPENTHREAD_ENABLED
    bool attached = false;
    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *inst = esp_openthread_get_instance();
    if (inst) {
        otDeviceRole role = otThreadGetDeviceRole(inst);
        attached = (role == OT_DEVICE_ROLE_CHILD ||
                    role == OT_DEVICE_ROLE_ROUTER ||
                    role == OT_DEVICE_ROLE_LEADER);
        if (attached) {
            s_state = THREAD_PROV_STATE_ATTACHED;
        } else if (role == OT_DEVICE_ROLE_DETACHED) {
            s_state = thread_prov_is_provisioned()
                          ? THREAD_PROV_STATE_ATTACHING
                          : THREAD_PROV_STATE_DETACHED;
        }
    }
    esp_openthread_lock_release();
    return attached;
#else
    return false;
#endif
}

thread_prov_state_t thread_prov_get_state(void)
{
    /* attached 호출이 상태를 최신화 */
    (void)thread_prov_is_attached();
    return s_state;
}

void thread_prov_erase(void)
{
#if CONFIG_OPENTHREAD_ENABLED
    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *inst = esp_openthread_get_instance();
    if (inst) {
        /* Thread stop + dataset erase */
        otThreadSetEnabled(inst, false);
        otOperationalDataset empty = {0};
        otDatasetSetActive(inst, &empty);
        otInstanceErasePersistentInfo(inst);
        s_state = THREAD_PROV_STATE_DETACHED;
        memset(s_network_name, 0, sizeof(s_network_name));
        ESP_LOGI(TAG, "Thread 자격증명 삭제 — 재커미셔닝 필요");
    }
    esp_openthread_lock_release();
#else
    ESP_LOGW(TAG, "OpenThread 비활성화 — erase 무시");
#endif
}

void thread_prov_get_info(thread_prov_info_t *info)
{
    if (!info) return;
    memset(info, 0, sizeof(*info));

#if CONFIG_OPENTHREAD_ENABLED
    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *inst = esp_openthread_get_instance();
    if (inst) {
        const char *name = otThreadGetNetworkName(inst);
        if (name) {
            strncpy(info->network_name, name, sizeof(info->network_name) - 1);
        }
        const otExtAddress *ext = otLinkGetExtendedAddress(inst);
        if (ext) {
            snprintf(info->extaddr, sizeof(info->extaddr),
                     "%02X%02X%02X%02X%02X%02X%02X%02X",
                     ext->m8[0], ext->m8[1], ext->m8[2], ext->m8[3],
                     ext->m8[4], ext->m8[5], ext->m8[6], ext->m8[7]);
        }
        /* border router 존재 여부는 SRP/DNS 클라이언트 상태로 추정 */
        otDeviceRole role = otThreadGetDeviceRole(inst);
        strncpy(info->border_router,
                (role == OT_DEVICE_ROLE_CHILD || role == OT_DEVICE_ROLE_ROUTER ||
                 role == OT_DEVICE_ROLE_LEADER) ? "active" : "none",
                sizeof(info->border_router) - 1);
    }
    esp_openthread_lock_release();
#else
    strncpy(info->border_router, "n/a", sizeof(info->border_router) - 1);
#endif
}

const char *thread_prov_get_network_name(void)
{
#if CONFIG_OPENTHREAD_ENABLED
    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *inst = esp_openthread_get_instance();
    const char *name = inst ? otThreadGetNetworkName(inst) : NULL;
    if (name && *name) {
        strncpy(s_network_name, name, sizeof(s_network_name) - 1);
    }
    esp_openthread_lock_release();
#endif
    return (s_network_name[0]) ? s_network_name : "Thread";
}

#define THREAD_RSSI_INVALID  127

int8_t thread_prov_get_parent_rssi(void)
{
#if CONFIG_OPENTHREAD_ENABLED
    int8_t rssi = THREAD_RSSI_INVALID;
    esp_openthread_lock_acquire(portMAX_DELAY);
    otInstance *inst = esp_openthread_get_instance();
    if (inst) {
        otDeviceRole role = otThreadGetDeviceRole(inst);
        if (role == OT_DEVICE_ROLE_CHILD || role == OT_DEVICE_ROLE_ROUTER ||
            role == OT_DEVICE_ROLE_LEADER) {
            int8_t r = THREAD_RSSI_INVALID;
            /* ★★★2026-08-16 Average → **Last** 로 바꿨다.
             *  사용자 신고: "라우터 바로 옆인데 안테나가 1칸".
             *  실측 [MTDIAG] rssi=-83 → -75 (role=2 CHILD, 붙어 있는 상태).
             *  1m 이내면 -30~-40 이어야 하는데 -83 이 나온 이유는
             *  otThreadGetParentAverageRssi 가 **평균**이라, 10m 지점에서 쌓인
             *  약한 표본이 아직 안 빠졌기 때문이다(그래서 천천히 오르고 있었다).
             *  신호 세기 표시는 "지금 얼마나 강한가" 라 평균이 아니라 순시값이
             *  맞다 → otThreadGetParentLastRssi 를 먼저 쓰고, 실패하면 평균으로
             *  폴백한다(막 붙어 아직 수신 표본이 없을 때 등).
             *  ※SED 는 7초 폴 때 프레임을 받으므로 Last 도 그 주기로 갱신된다.
             *  router/leader: 부모 없음 → INVALID 유지(호출측이 풀바 처리). */
            if (otThreadGetParentLastRssi(inst, &r) == OT_ERROR_NONE &&
                r != 0 && r != THREAD_RSSI_INVALID) {
                rssi = r;
            } else if (otThreadGetParentAverageRssi(inst, &r) == OT_ERROR_NONE) {
                rssi = r;
            }
        }
    }
    esp_openthread_lock_release();
    return rssi;
#else
    return THREAD_RSSI_INVALID;
#endif
}
