#include "blind_manager.h"
#include "somfy_config.h"   // CFG_FREQ_MIN_MHZ / CFG_FREQ_MAX_MHZ (편집 허용 범위)
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_mac.h"        // esp_efuse_mac_get_default — 칩 고유 불변 MAC
#include <string.h>
#include <math.h>
#include <inttypes.h>

static const char *TAG = "BLIND_MGR";

/* ─── 기본 주파수 ─────────────────────────────── */
/* ★ SDR(AirSpy) 정밀 측정으로 최종 확정:
 *   - 정품 리모컨 실제 송신 = 447.678 MHz
 *   - 우리 CC1101 은 설정값보다 약 -41.5kHz 낮게 송신(crystal 오차):
 *     설정 447.65 → 실측 447.6085.
 *   → 정품 447.678 에 맞추려면 설정값 = 447.678 + 0.0415 ≈ 447.72 MHz.
 *  (이전 rfscan RSSI 추정 447.65 는 0.05MHz 간격·넓은 채널BW 로 부정확.) */
#define DEFAULT_FREQ_MHZ  BOARD_DEFAULT_FREQ_MHZ   /* board_select.h — build.ps1 -Freq 로 변경(기본 447.70, 테스트보드 447.72) */

/* ─── 블라인드 슬롯 고유 주소 (기기별 — eFuse MAC 산출) ─────────────
 *  ★ 여러 대를 만들어 같은 블라인드를 제어할 때 주소(=리모컨 ID)가 겹치면
 *  안 된다. 과거엔 하드코딩(0x3C1A85~89) → 모든 기기가 동일 → 충돌.
 *  이제 ESP32 의 **eFuse 팩토리 MAC**(칩 고유·불변)에서 결정적으로 산출:
 *    - 같은 칩 → 항상 같은 주소 (펌웨어 OTA·factory reset 무관:
 *      eFuse 는 flash 가 아니라 erase-flash 로도 안 지워짐)
 *    - 다른 칩 → 다른 주소 (기기 간 충돌 방지)
 *  _ensure_blinds 가 매 부팅 산출값을 enforce 하므로 NVS 가 지워져도 동일
 *  주소로 복원된다. */
#define BLIND_ROLLING_START  10
/* cfg_tag 세대 bump (0x...50 → 0x...60): eFuse 산출 주소 도입. 기존 기기는
 *  이 부팅에서 주소가 MAC 기반으로 바뀌므로 모터에 PROG 재등록 필요. */
#define BLIND_CFG_TAG        0xC10E0066u   /* base 고유 복원(신호 형태가 핵심, 주소 무관) → NVS 무효화(재PROG) */

/* ── eFuse 팩토리 MAC → 블록별 주소(F0 + 등차 + ID) 산출 ──────────────────
 *  실제 주소(LE, MSB=F0) = F0 [중간: 채널 등차] [하위: 보드 ID]. 4채널 = 1 블록.
 *  ※ decode_clock.py 는 byte 를 BE 로 거꾸로 출력 → 표시 3B0EF0 = 실제 F00E3B.
 *    rtl_433 의 id(0xF0…) 가 올바른 해석. 상위 F0 고정, 하위=리모컨/보드 ID(3B·67).
 *   - 중간 1번값 = (hash>>8) % 0x27  → 등차 역산 최소항(0~0x26) → ALL≤0xC2, F0 carry 없음
 *   - 하위 ID    = hash & 0xFF       → 보드 고유(블록별 salt 로 분리)
 *   - 채널 i     = base + i*0x2700,  ALL = base + 4*0x2700  (시추오 리모컨 실측 규칙) */
#define ADDR_PREFIX   0xF00000u
#define ADDR_STEP     0x2700u        /* 채널 간 공차(중간바이트 +0x27, 하위/상위 불변) */
#define ADDR_MIDSTEP  0x27u

static void _derive_addresses(uint32_t ch_out[BLIND_MAX_COUNT],
                              uint32_t all_out[BLIND_BLOCK_COUNT])
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY);
    for (int g = 0; g < BLIND_BLOCK_COUNT; g++) {
        uint32_t h = 2166136261u;                    /* FNV-1a offset basis */
        for (int i = 0; i < 6; i++) { h ^= mac[i]; h *= 16777619u; }
        h ^= (uint32_t)(g + 1) * 0x9E3779B1u;        /* 블록별 salt → 블록마다 다른 base */
        h *= 16777619u;
        /* base 중간(채널 시작 코드) — 보드 고유(eFuse). ALL 은 신호 형태(preamble/period)
         *  로 결정되며 주소와 무관하므로 base 는 고유값이면 된다(정품도 리모컨마다 다름). */
        uint8_t mid = (uint8_t)(((h >> 8) & 0xFFu) % ADDR_MIDSTEP);  /* base 중간 — eFuse 고유값(1E). 0E 강제는 정품1 mid 충돌로 1번 깸 → 원복 */
        uint8_t low = (uint8_t)(h & 0xFFu);                         /* 하위 ID(보드 고유) — eFuse 원래값(홀수 강제 B1 은 1번까지 깸 → 폐기, ID 홀짝은 동작 무관) */
        uint32_t base = ADDR_PREFIX | ((uint32_t)mid << 8) | low;
        int start = g * BLINDS_PER_BLOCK;
        int ch_n  = BLIND_MAX_COUNT - start;
        if (ch_n > BLINDS_PER_BLOCK) ch_n = BLINDS_PER_BLOCK;
        for (int i = 0; i < ch_n; i++)
            ch_out[start + i] = base + (uint32_t)i * ADDR_STEP;
        all_out[g] = base + (uint32_t)BLINDS_PER_BLOCK * ADDR_STEP;  /* ALL = base+4*step */
        ESP_LOGI(TAG, "블록[%d] base=0x%06" PRIX32 " 채널%d ALL=0x%06" PRIX32,
                 g, base, ch_n, all_out[g]);
    }
}

/* 산출 주소 캐시 — 부팅 1회 계산 후 재사용. */
static uint32_t s_ch_addr[BLIND_MAX_COUNT];
static uint32_t s_all_addr[BLIND_BLOCK_COUNT];
static bool     s_addr_ready = false;
static void _ensure_addr(void)
{
    if (!s_addr_ready) { _derive_addresses(s_ch_addr, s_all_addr); s_addr_ready = true; }
}
static const uint32_t *_blind_addr(void) { _ensure_addr(); return s_ch_addr; }

/* 모든 슬롯을 고유 주소로 채운다(빈 슬롯 없음). 슬롯 주소가 이미
 *  맞으면 건드리지 않음 — 이름/주파수/롤링코드 보존. 변경 시 true. */
static bool _ensure_blinds(blind_manager_t *mgr)
{
    bool dirty = false;
    _ensure_addr();
    for (int i = 0; i < BLIND_MAX_COUNT; i++) {
        somfy_blind_t *b = &mgr->blinds[i];
        if (b->address != s_ch_addr[i] || !b->active) {
            memset(b, 0, sizeof(*b));
            b->address      = s_ch_addr[i];
            b->rolling_code = BLIND_ROLLING_START;
            b->freq_mhz     = DEFAULT_FREQ_MHZ;
            b->active       = true;
            snprintf(b->name, sizeof(b->name), "Blind-%d", i + 1);
            ESP_LOGI(TAG, "슬롯[%d] 초기화: %s addr=0x%06" PRIX32,
                     i, b->name, s_ch_addr[i]);
            dirty = true;
        }
        /* 주파수 보정 — 편집 허용 범위(CFG_FREQ_MIN..MAX) 밖의 구버전/손상 NVS 값만
         *  기본값으로 강제. Freq-Edit 로 맞춘 정상 범위(447.20~447.79) 값은 보존한다.
         *  (과거 447.70~447.74 하드코딩은 편집기와 범위가 어긋나 사용자 설정을 되돌렸다.) */
        if (b->freq_mhz < CFG_FREQ_MIN_MHZ || b->freq_mhz > CFG_FREQ_MAX_MHZ) {
            b->freq_mhz = DEFAULT_FREQ_MHZ;
            dirty = true;
        }
    }
    /* ── ALL 블록(블록당 1개) — 주소 = base+4*step, 전용 rolling ── */
    for (int g = 0; g < BLIND_BLOCK_COUNT; g++) {
        somfy_blind_t *a = &mgr->all_blocks[g];
        if (a->address != s_all_addr[g] || !a->active) {
            memset(a, 0, sizeof(*a));
            a->address      = s_all_addr[g];
            a->rolling_code = BLIND_ROLLING_START;
            a->freq_mhz     = DEFAULT_FREQ_MHZ;
            a->active       = true;
            snprintf(a->name, sizeof(a->name), "ALL-%d", g + 1);
            dirty = true;
        }
        if (a->freq_mhz < CFG_FREQ_MIN_MHZ || a->freq_mhz > CFG_FREQ_MAX_MHZ) {
            a->freq_mhz = DEFAULT_FREQ_MHZ;
            dirty = true;
        }
    }
    mgr->block_count = BLIND_BLOCK_COUNT;
    if (mgr->count != BLIND_MAX_COUNT) {
        mgr->count = BLIND_MAX_COUNT;
        dirty = true;
    }
    return dirty;
}

/* ─── 롤링코드 전용 NVS 파티션 ───────────────────────────────
 *  롤링코드는 매 송신마다 증가하고, 모터는 직전에 받은 값보다 작은 롤링코드를
 *  "replay" 로 보고 거부한다. 기본 nvs 에만 두면 Matter factory reset(기본 nvs
 *  전체 삭제) 시 START(10)로 리셋 → 모터 무응답(재-PROG 필요). 그래서 별도
 *  'rollcode' 파티션에도 보존하고, 부팅 시 더 큰 값을 채택해 역행을 막는다.
 *  (전체 erase-flash 는 이 파티션도 지움 — 그땐 재-PROG 가 정답.) */
#define ROLLCODE_PARTITION   "rollcode"
#define ROLLCODE_NAMESPACE   "rollcode"
#define ROLLCODE_KEY         "rc5"
#define ROLLCODE_MAGIC       0x52433502u   /* 'RC5' + ver2 (블록 ALL rolling 추가) */

typedef struct {
    uint32_t magic;
    uint16_t rolling[BLIND_MAX_COUNT];
    uint16_t all_rolling[BLIND_BLOCK_COUNT];   /* 블록 ALL 전용 rolling(factory reset 내성) */
} rollcode_blob_t;

static bool s_rollcode_ok = false;

static void _rollcode_init(void)
{
    esp_err_t e = nvs_flash_init_partition(ROLLCODE_PARTITION);
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase_partition(ROLLCODE_PARTITION);
        e = nvs_flash_init_partition(ROLLCODE_PARTITION);
    }
    s_rollcode_ok = (e == ESP_OK);
    if (!s_rollcode_ok)
        ESP_LOGW(TAG, "rollcode 파티션 init 실패: %s — 롤링코드 영속 비활성",
                 esp_err_to_name(e));
}

static bool _rollcode_load(rollcode_blob_t *out)
{
    if (!s_rollcode_ok) return false;
    nvs_handle_t h;
    if (nvs_open_from_partition(ROLLCODE_PARTITION, ROLLCODE_NAMESPACE,
                                NVS_READONLY, &h) != ESP_OK) return false;
    size_t sz = sizeof(*out);
    esp_err_t e = nvs_get_blob(h, ROLLCODE_KEY, out, &sz);
    nvs_close(h);
    return (e == ESP_OK && sz == sizeof(*out) && out->magic == ROLLCODE_MAGIC);
}

/* 현재 롤링코드를 전용 파티션에 기록. */
static void _rollcode_store(const blind_manager_t *mgr)
{
    if (!s_rollcode_ok) return;
    rollcode_blob_t b = { .magic = ROLLCODE_MAGIC };
    for (int i = 0; i < BLIND_MAX_COUNT; i++)
        b.rolling[i] = mgr->blinds[i].rolling_code;
    for (int g = 0; g < BLIND_BLOCK_COUNT; g++)
        b.all_rolling[g] = mgr->all_blocks[g].rolling_code;
    nvs_handle_t h;
    if (nvs_open_from_partition(ROLLCODE_PARTITION, ROLLCODE_NAMESPACE,
                                NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_blob(h, ROLLCODE_KEY, &b, sizeof(b)) == ESP_OK) nvs_commit(h);
    nvs_close(h);
}

void blind_manager_init(blind_manager_t *mgr)
{
    memset(mgr, 0, sizeof(blind_manager_t));
    mgr->selected = 0;

    /* NVS 초기화 */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* 저장된 설정 로드 */
    bool loaded = blind_manager_load(mgr);
    if (!loaded) {
        ESP_LOGI(TAG, "저장된 설정 없음 — 블라인드 %d개 기본 생성", BLIND_MAX_COUNT);
        memset(mgr, 0, sizeof(blind_manager_t));   /* load 실패 잔여 클리어 */
        mgr->selected = 0;
    }
    bool dirty = !loaded;

    /* 설정 세대 태그 갱신 시 모든 롤링코드 1회 재기준 */
    if (mgr->cfg_tag != BLIND_CFG_TAG) {
        for (int i = 0; i < BLIND_MAX_COUNT; i++)
            mgr->blinds[i].rolling_code = BLIND_ROLLING_START;
        mgr->cfg_tag = BLIND_CFG_TAG;
        dirty = true;
        ESP_LOGW(TAG, "cfg_tag 갱신 → 롤링코드 %d 재기준", BLIND_ROLLING_START);
    }

    /* ★ 모든 슬롯 고유 주소로 채움 — 빈 슬롯 제거 */
    if (_ensure_blinds(mgr)) dirty = true;

    /* selected 보정 — 등록 범위(0~count-1) 또는 ALL(5) 외 값이면 0 */
    if (mgr->selected != BLIND_SEL_ALL && mgr->selected >= mgr->count) {
        mgr->selected = 0;
        dirty = true;
    }

    /* ★ 롤링코드 영속 — 전용 파티션에서 복원(역행 방지: 더 큰 값 채택).
     *  Matter factory reset 으로 기본 nvs 가 지워져 위에서 START(10)로
     *  리셋됐어도, 보존된 값이 더 크면 그걸 써 모터가 계속 받아준다. */
    _rollcode_init();
    rollcode_blob_t rc;
    if (_rollcode_load(&rc)) {
        for (int i = 0; i < BLIND_MAX_COUNT; i++) {
            if (rc.rolling[i] > mgr->blinds[i].rolling_code) {
                mgr->blinds[i].rolling_code = rc.rolling[i];
                dirty = true;
            }
        }
        for (int g = 0; g < BLIND_BLOCK_COUNT; g++) {
            if (rc.all_rolling[g] > mgr->all_blocks[g].rolling_code) {
                mgr->all_blocks[g].rolling_code = rc.all_rolling[g];
                dirty = true;
            }
        }
        ESP_LOGI(TAG, "rollcode 파티션에서 롤링코드 복원 "
                 "(슬롯1=%u ALL1=%u)", (unsigned)mgr->blinds[0].rolling_code,
                 (unsigned)mgr->all_blocks[0].rolling_code);
    }
    _rollcode_store(mgr);   /* 현재값을 전용 파티션에 기록(없으면 생성) */

    if (dirty) blind_manager_save(mgr);
    ESP_LOGI(TAG, "블라인드 매니저 초기화 완료: %d개 슬롯", mgr->count);
}

void blind_manager_save(const blind_manager_t *mgr)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(BLIND_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open 실패: %s", esp_err_to_name(err));
        return;
    }

    /* 구조체 전체를 blob으로 저장 */
    err = nvs_set_blob(handle, BLIND_NVS_KEY, mgr, sizeof(blind_manager_t));
    if (err == ESP_OK) {
        nvs_commit(handle);
        ESP_LOGI(TAG, "NVS 저장 완료 (%d개 블라인드)", mgr->count);
    } else {
        ESP_LOGE(TAG, "NVS 저장 실패: %s", esp_err_to_name(err));
    }
    nvs_close(handle);
}

bool blind_manager_load(blind_manager_t *mgr)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(BLIND_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return false;
    }

    size_t required_size = sizeof(blind_manager_t);
    err = nvs_get_blob(handle, BLIND_NVS_KEY, mgr, &required_size);
    nvs_close(handle);

    if (err != ESP_OK || required_size != sizeof(blind_manager_t)) {
        return false;
    }
    return true;
}

int blind_manager_add(blind_manager_t *mgr, const char *name, float freq)
{
    if (mgr->count >= BLIND_MAX_COUNT) {
        ESP_LOGW(TAG, "최대 블라인드 수(%d) 초과", BLIND_MAX_COUNT);
        return -1;
    }

    uint8_t idx = mgr->count;
    somfy_blind_t *blind = &mgr->blinds[idx];

    memset(blind, 0, sizeof(somfy_blind_t));
    strncpy(blind->name, name, sizeof(blind->name) - 1);
    /* 슬롯 고유 주소 부여 (idx 0..BLIND_MAX_COUNT-1 — eFuse MAC 산출 주소). */
    blind->address      = _blind_addr()[idx];
    blind->rolling_code = BLIND_ROLLING_START;
    blind->freq_mhz     = freq;
    blind->active       = true;

    mgr->count++;
    blind_manager_save(mgr);

    ESP_LOGI(TAG, "블라인드 추가: [%d] %s addr=0x%06" PRIx32 " freq=%.2f",
             idx, blind->name, blind->address, blind->freq_mhz);
    return idx;
}

void blind_manager_remove(blind_manager_t *mgr, uint8_t idx)
{
    if (idx >= mgr->count) return;

    /* 뒤 항목들을 앞으로 이동 */
    for (int i = idx; i < mgr->count - 1; i++) {
        mgr->blinds[i] = mgr->blinds[i + 1];
    }
    memset(&mgr->blinds[mgr->count - 1], 0, sizeof(somfy_blind_t));
    mgr->count--;

    if (mgr->selected >= mgr->count && mgr->count > 0) {
        mgr->selected = mgr->count - 1;
    }

    blind_manager_save(mgr);
    ESP_LOGI(TAG, "블라인드 삭제: idx=%d, 남은 수=%d", idx, mgr->count);
}

void blind_manager_set_freq(blind_manager_t *mgr, uint8_t idx, float freq_mhz)
{
    /* 범위 제한: 447.20 ~ 447.79 MHz */
    if (freq_mhz < 447.20f) freq_mhz = 447.20f;
    if (freq_mhz > 447.79f) freq_mhz = 447.79f;
    /* 0.01 단위로 반올림 */
    freq_mhz = roundf(freq_mhz * 100.0f) / 100.0f;

    if (idx == BLIND_SEL_ALL) {
        /* ALL — 블록별 ALL 주소(all_blocks[]) 전부에 동일 주파수 적용.
         *  실제 ALL 송신은 all_blocks[].freq 를 쓰므로 여기에 반영해야 한다
         *  (안 하면 화면만 바뀌고 송신은 기본 주파수로 나감). */
        for (int g = 0; g < mgr->block_count; g++)
            mgr->all_blocks[g].freq_mhz = freq_mhz;
        ESP_LOGI(TAG, "ALL 주파수 변경: %.2f MHz (블록 %d개)", freq_mhz, mgr->block_count);
    } else if (idx < mgr->count) {
        mgr->blinds[idx].freq_mhz = freq_mhz;
        ESP_LOGI(TAG, "블라인드[%d] 주파수 변경: %.2f MHz", idx, freq_mhz);
    } else {
        return;
    }
    blind_manager_save(mgr);
}

void blind_manager_select(blind_manager_t *mgr, uint8_t idx)
{
    /* 등록 슬롯(idx < BLIND_MAX_COUNT) 또는 ALL(인덱스 = BLIND_MAX_COUNT).
     * 채널 수가 늘어도 ALL 은 항상 마지막 인덱스(BLIND_SEL_ALL)로 일반화. */
    if (idx < BLIND_MAX_COUNT || idx == BLIND_SEL_ALL) {
        mgr->selected = idx;
        ESP_LOGI(TAG, "선택: %s",
                 (idx < mgr->count) ? mgr->blinds[idx].name : "ALL");
    }
}

void blind_manager_get_targets(blind_manager_t *mgr,
                                somfy_blind_t **out, uint8_t *out_count)
{
    *out_count = 0;

    if (mgr->selected == BLIND_SEL_ALL) {
        /* ALL — 블록별 ALL 주소를 각각 송신(ALL1, ALL2, …).
         *  H2=ALL 1개, C6=ALL 2개, 9채널=ALL 3개. */
        for (int g = 0; g < mgr->block_count; g++) {
            if (mgr->all_blocks[g].active) {
                out[(*out_count)++] = &mgr->all_blocks[g];
            }
        }
    } else if (mgr->selected < mgr->count) {
        /* 개별 선택 */
        if (mgr->blinds[mgr->selected].active) {
            out[0] = &mgr->blinds[mgr->selected];
            *out_count = 1;
        }
    }
}

void blind_manager_save_rolling(blind_manager_t *mgr, uint8_t idx)
{
    (void)idx;
    /* rolling code만 빠르게 저장 (매 전송 후 호출) — 기본 nvs + 전용 파티션. */
    blind_manager_save(mgr);
    _rollcode_store(mgr);   /* factory reset 견디는 전용 파티션에도 반영 */
}

#if defined(SOMFY_SELFTEST) || defined(SOMFY_ONAIR_TEST)
/* [테스트 전용] NVS 없이 결정적 주소로 매니저 구성 (실제 _ensure_blinds 사용). */
void blind_manager_test_populate(blind_manager_t *mgr)
{
    memset(mgr, 0, sizeof(*mgr));
    (void)_ensure_blinds(mgr);   /* 주소 산출 + ALL 블록 + active. NVS 미접근. */
    mgr->selected = 0;
}
#endif
