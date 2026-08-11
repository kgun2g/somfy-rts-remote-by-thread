#pragma once

#include "blind_manager.h"
#include "somfy_rts.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Matter Window Covering 제어 콜백 타입 ── */
/* oled_action: oled_action_t 값(0=NONE,1=UP,2=DOWN,3=STOP,4=TILT_UP,
 *  5=TILT_DN,6=PROG). delegate 가 lift/tilt + 이동방향으로 산출해 전달
 *  (RF용 cmd 와 분리 — 슬라이더 중간값도 방향대로 up/down 모션).
 * step_count: 같은 cmd 를 반복 송신할 회수 (Tilt 는 7단 detent 매핑).
 *  Lift/StopMotion 등 단일 송신은 1. Tilt 슬라이더는 round(|target-current|×7/10000)
 *  step burst 를 보내 슬랫이 그만큼의 step 으로 움직이게 한다 (1~7, 실측). */
typedef void (*matter_blind_action_cb_t)(uint8_t endpoint_idx,
                                          somfy_command_t cmd,
                                          uint8_t position_pct,
                                          uint8_t oled_action,
                                          uint8_t step_count,
                                          void *user_data);

/**
 * @brief Matter 장치 초기화 (esp-matter 기반)
 *        BLIND_MAX_COUNT 개의 Window Covering 엔드포인트 등록
 *
 * @param mgr        블라인드 매니저
 * @param action_cb  SmartThings에서 제어 명령 수신 시 콜백
 * @param user_data  콜백 사용자 데이터
 */
void matter_blinds_init(blind_manager_t *mgr,
                         matter_blind_action_cb_t action_cb,
                         void *user_data);

/**
 * @brief Matter 시작 (커미셔닝 가능 상태)
 * @return 페어링 코드 문자열 (NUL-terminated)
 */
const char *matter_blinds_start(void);

/**
 * @brief 블라인드 위치 업데이트 (SmartThings UI에 반영)
 * @param endpoint_idx 0..BLIND_MAX_COUNT-1
 * @param position_pct 0~100 (0=닫힘, 100=열림)
 */
void matter_blinds_update_position(uint8_t endpoint_idx, uint8_t position_pct);

/**
 * @brief SmartThings 커미셔닝 완료 여부 (fabric 1개 이상 존재)
 */
bool matter_blinds_is_commissioned(void);

/**
 * @brief 커미셔닝(페어링)이 현재 진행 중인지 여부.
 *
 *  Matter fail-safe 타이머가 armed 상태이면(=commissioner 가 ArmFailSafe
 *  후 AddNOC/CASE 등 트랜잭션 진행 중) true. 메인 화면 상단에 "페어링 중"
 *  표시 판단에 사용.
 *
 * @return true = 페어링 트랜잭션 진행 중
 */
bool matter_blinds_is_pairing_in_progress(void);

/**
 * @brief 마지막 페어링 실패 진단 코드 문자열.
 *
 *  SmartThings 의 39-xxx 는 commissioner(앱) 측 코드라 디바이스가 알 수
 *  없다. 대신 디바이스 관점에서 커미셔닝이 어느 단계에서 멈췄는지를
 *  코드로 제공:
 *    "ERR FS-NOC" : AddNOC(인증/fabric)까지 성공, 이후 operational
 *                   (SRP 등록 / CASE / Complete) 단계에서 실패
 *    "ERR FS-PRE" : AddNOC 이전(attestation 등)에서 실패
 *    "ERR SESSION": 커미셔닝 세션 비정상 종료
 *    ""           : 실패 없음 / 클리어됨
 *
 * @return NUL-terminated 문자열 (빈 문자열 가능). 정적 버퍼 — 복사 불필요.
 */
const char *matter_blinds_get_last_pair_error(void);

/**
 * @brief Matter 커미셔닝이 완전히 끝났는지 여부.
 *
 *  kCommissioningComplete 이벤트 수신(신규 페어링) 또는 부팅 시 이미 fabric
 *  존재(기존 페어링)일 때 true.
 *
 *  ★ main.c 는 이 함수가 true 가 되기 전까지 PM auto-light-sleep 을 켜면
 *    안 된다. Thread SED + light sleep 이 commissioning 의 Thread operational
 *    단계(SRP 등록 + CASE 핸드셰이크)를 굶겨 SmartThings 페어링이 마지막에
 *    실패(39-xxx)하던 버그의 해결책.
 *
 * @return true = PM auto-light-sleep 활성화 안전
 */
bool matter_blinds_is_commissioning_complete(void);

/**
 * @brief BLE Commissioning Window 재오픈 (15분).
 *
 *  esp-matter 의 자동 commissioning window 는 부팅 직후 15분간만 열려 있고
 *  그 이후 또는 이미 커미셔닝된 디바이스에서는 닫혀 있다. 사용자가 설정 메뉴
 *  → "Matter Pair" 를 선택하면 본 함수가 호출되어 새 commissioning window
 *  를 15분간 다시 연다 (Basic Commissioning — 기존 fabric 유지, 추가 fabric
 *  허용).
 *
 * @return 페어링 코드 문자열 (NUL-terminated). 실패 시도 코드는 동일.
 */
const char *matter_blinds_open_commissioning_window(void);

/* Matter QR 코드 내용 문자열("MT:..."). OLED 페어링 화면 QR 렌더용(esp_qrcode 인코딩).
 *  고정값(VID/PID/discriminator/passcode). 정적 버퍼 — 복사 불필요. 실패 시 "". */
const char *matter_blinds_get_qr_payload(void);

/* Thread Reset(완전 초기화): Matter fabric(= SmartThings 페어링 정보) 전체 삭제.
 *  fabric 조작은 CHIP 스레드에서만 안전하므로 비동기로 위임된다. 호출 후 NVS 커밋
 *  여유를 두고 esp_restart 할 것(미커미셔닝 상태로 재부팅 → 자동 BLE 광고). */
void matter_blinds_remove_all_fabrics(void);

/* ★2026-08-11 무선(Thread+BLE) 게이팅 — 배터리 절약.
 *  Thread 기기로 등록되지 않은 상태에서는 라디오가 할 일이 없으므로 꺼둔다.
 *  설정 메뉴에서 페어링을 시작하면 자동으로 켜지고(open_commissioning_window 내부),
 *  커미셔닝이 완료되면 계속 켜둔다. somfy_app 메인 루프가 주기적으로 상태를 맞춘다. */
void matter_blinds_set_radio_enabled(bool on);
bool matter_blinds_get_radio_enabled(void);

/* ─── Matter OTA (Thread 무선 펌웨어 업데이트) ──────────────────────
 *  디바이스는 듀얼 OTA 파티션 + Matter OTA Requestor 가 활성이라, Matter OTA
 *  Provider 가 펌웨어(.ota 이미지, 현재 버전보다 높은 SoftwareVersion)를
 *  내려주면 자동 다운로드/적용한다. 설정 메뉴 "FW Update" 화면이 아래 API 로
 *  현재 버전·OTA 상태를 표시하고 수동 체크를 트리거한다. */
typedef enum {
    MATTER_OTA_IDLE        = 0,  /* 대기 */
    MATTER_OTA_QUERYING    = 1,  /* Provider 에 질의 중 */
    MATTER_OTA_DOWNLOADING = 2,  /* 다운로드 중 (progress %) */
    MATTER_OTA_APPLYING    = 3,  /* 적용/재부팅 준비 */
    MATTER_OTA_DELAYED     = 4,  /* 지연(사용자 동의/재시도 대기) */
    MATTER_OTA_UNKNOWN     = 5,
} matter_ota_state_t;

/** 현재 실행 중 펌웨어 버전 문자열 (예 "3.5"). 정적 버퍼 — 복사 불필요. */
const char *matter_ota_version_str(void);

/** 현재 OTA 상태. progress_pct(0~100) 는 DOWNLOADING 일 때만 유효(아니면 0).
 *  NULL 전달 가능. */
matter_ota_state_t matter_ota_get_state(uint8_t *progress_pct);

/** 수동 업데이트 확인 — 기본 OTA Provider 에 즉시 QueryImage.
 *  Provider 가 설정돼 있지 않으면 무동작(false). CHIP 스레드에 스케줄. */
bool matter_ota_trigger_check(void);

#ifdef __cplusplus
}
#endif
