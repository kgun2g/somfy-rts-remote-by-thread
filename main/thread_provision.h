#pragma once
/*
 * thread_provision.h
 * ─────────────────────────────────────────────────────────
 * Thread (Matter over Thread) 네트워크 상태 관리.
 *
 * v3.0 변경: WiFi → Thread.
 *
 *  ▸ WiFi 와 달리 Thread 는 별도 SoftAP / SSID / 패스워드 입력이 없다.
 *    Thread 네트워크 자격증명 (TLV — channel/PAN ID/network key/...) 은
 *    Matter commissioner (SmartThings/Apple Home/Google Home 등) 가
 *    BLE 커미셔닝 단계에서 자동 주입한다.
 *
 *  ▸ Thread 노드는 Border Router (eero/HomePod mini/Apple TV/Nest Hub Max
 *    등) 가 같은 네트워크에 있을 때만 인터넷 라우팅 가능. Border Router
 *    가 없어도 Matter Hub 와 직접 통신할 수는 있음.
 *
 *  ▸ 본 모듈은 ESP-IDF OpenThread 스택의 attach/detach 이벤트만 감시하고
 *    상태 비트만 노출. 실제 자격증명 저장은 OpenThread NVS 가 담당.
 *
 *  ▸ 커미셔닝 (BLE → Thread credential 주입) 자체는 esp-matter 가 자동
 *    처리 — open commissioning window 만 호출하면 됨.
 * ─────────────────────────────────────────────────────────
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ─── 상태 ─────────────────────────────────────── */
typedef enum {
    THREAD_PROV_STATE_IDLE        = 0,  // OpenThread 미초기화
    THREAD_PROV_STATE_DETACHED    = 1,  // 자격증명 없음 (커미셔닝 대기)
    THREAD_PROV_STATE_ATTACHING   = 2,  // 네트워크 부착 시도 중
    THREAD_PROV_STATE_ATTACHED    = 3,  // 부착 완료 (router/child 역할)
    THREAD_PROV_STATE_FAILED      = 4,
} thread_prov_state_t;

/* ─── 완료 콜백 ─────────────────────────────────── */
typedef void (*thread_prov_done_cb_t)(bool success, const char *info, void *user_data);

/* ─── 표시용 정보 (OLED 안내) ────────────────────── */
typedef struct {
    char network_name[17];  // Thread network name (최대 16 char + null)
    char border_router[18]; // border router presence: "active" / "none"
    char extaddr[17];       // device 64-bit EUI (16 hex + null)
} thread_prov_info_t;

/* ─── API ──────────────────────────────────────── */

/**
 * @brief Thread provisioning 모듈 초기화 — OpenThread 스택 시작.
 *        esp-matter::start() 가 자동으로 OpenThread 를 시작하므로
 *        본 함수는 상태 추적 콜백만 등록.
 */
void thread_prov_init(void);

/**
 * @brief 저장된 Thread 자격증명 존재 여부.
 *  true  = 이전 커미셔닝으로 NVS 에 네트워크 키 저장됨
 *  false = 미설정 (커미셔닝 필요)
 */
bool thread_prov_is_provisioned(void);

/**
 * @brief Thread 네트워크 부착 상태 (Matter 트랜스포트 가능 여부).
 */
bool thread_prov_is_attached(void);

/**
 * @brief 현재 상태 반환.
 */
thread_prov_state_t thread_prov_get_state(void);

/**
 * @brief Thread 자격증명 삭제 → factory reset 후 재커미셔닝 유도.
 *        Matter fabric 도 함께 erase 됨 (Matter API 별도 호출).
 */
void thread_prov_erase(void);

/**
 * @brief 표시용 정보 조회 (OLED 안내 화면 용).
 *        부착 안 된 상태에서는 network_name/border_router 비어 있음.
 */
void thread_prov_get_info(thread_prov_info_t *info);

/**
 * @brief 현재 Thread 네트워크 이름 (있으면), 없으면 "Thread".
 */
const char *thread_prov_get_network_name(void);

/**
 * @brief Thread 부모(parent) 평균 수신감도(RSSI, dBm).
 *
 *  child 역할로 부착되어 있을 때 부모와의 링크 RSSI 평균값을 반환.
 *  메인 화면 상단 안테나 신호세기 막대 표시에 사용.
 *
 * @return RSSI dBm (음수, 예: -55). 조회 불가/미부착 시 127 (INVALID).
 */
int8_t thread_prov_get_parent_rssi(void);

#ifdef __cplusplus
}
#endif
