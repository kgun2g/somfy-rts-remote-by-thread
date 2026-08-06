// efuse_commissionable.cpp — eFuse MAC → 기기 고유 discriminator/passcode/verifier.
#include "efuse_commissionable.h"

#include <crypto/CHIPCryptoPAL.h>
#include <setup_payload/SetupPayload.h>
#include <lib/support/CodeUtils.h>
#include <esp_mac.h>
#include <esp_log.h>
#include <string.h>

using namespace chip;

static const char *TAG = "EFUSE_COMM";

static constexpr uint32_t kIterationCount = 1000;            // PBKDF2 (C6/H2 비용 ↓)
static constexpr size_t   kSaltLen        = 16;             // kSpake2p_Min_PBKDF_Salt_Length

/* eFuse 팩토리 MAC → 결정적 discriminator/passcode/salt 산출(부팅 1회, 캐싱).
 *  블라인드 주소와 같은 칩 고유 소스지만 다른 salt('COMM')로 분리한다. */
void EfuseCommissionableDataProvider::ensureDerived()
{
    if (mReady) return;

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_EFUSE_FACTORY);   // 칩 고유 6바이트(불변) — 블라인드 주소와 동일 소스

    uint32_t h = 2166136261u;                   // FNV-1a
    for (int i = 0; i < 6; i++) { h ^= mac[i]; h *= 16777619u; }
    h ^= 0x434F4D4Du;  h *= 16777619u;          // "COMM" salt → 블라인드 주소 해시와 분리

    /* 12bit discriminator — 상·하위 비트를 접어(fold) short discriminator(상위 4bit)까지
     *  고르게 분산. (단순 h&0xFFF 는 상위 4bit 가 h 의 bit8~11 에만 의존해 겹치기 쉽다 —
     *  실제로 gnpe 3640/xiao 3605 가 long 은 달라도 short 4bit 가 둘 다 0xE 로 겹쳤다.) */
    uint32_t d12 = (h ^ (h >> 12) ^ (h >> 23)) & 0x0FFFu;
    mDiscriminator = (uint16_t)d12;             // 12bit (0..4095)

    uint32_t pc = (h % kSetupPINCodeMaximumValue) + 1;   // 1..99999998
    if (!SetupPayload::IsValidSetupPIN(pc)) {             // 금지값(11111111 등) 보정
        pc = (pc % kSetupPINCodeMaximumValue) + 1;
    }
    mPasscode = pc;

    uint32_t s = h ^ 0x53414C54u;               // "SALT"
    for (size_t i = 0; i < kSaltLen; i++) {
        s ^= mac[i % 6]; s *= 16777619u;
        mSalt[i] = (uint8_t)(s >> 24);
    }

    mReady = true;
    ESP_LOGW(TAG, "eFuse 기기 고유 커미셔닝: discriminator=%u  passcode=%08u",
             (unsigned)mDiscriminator, (unsigned)mPasscode);
}

CHIP_ERROR EfuseCommissionableDataProvider::GetSetupDiscriminator(uint16_t & setupDiscriminator)
{
    ensureDerived();
    setupDiscriminator = mDiscriminator;
    return CHIP_NO_ERROR;
}

CHIP_ERROR EfuseCommissionableDataProvider::GetSpake2pIterationCount(uint32_t & iterationCount)
{
    iterationCount = kIterationCount;
    return CHIP_NO_ERROR;
}

CHIP_ERROR EfuseCommissionableDataProvider::GetSpake2pSalt(MutableByteSpan & saltBuf)
{
    ensureDerived();
    VerifyOrReturnError(saltBuf.size() >= kSaltLen, CHIP_ERROR_BUFFER_TOO_SMALL);
    memcpy(saltBuf.data(), mSalt, kSaltLen);
    saltBuf.reduce_size(kSaltLen);
    return CHIP_NO_ERROR;
}

CHIP_ERROR EfuseCommissionableDataProvider::GetSpake2pVerifier(MutableByteSpan & verifierBuf, size_t & verifierLen)
{
    uint32_t passcode = 0, iter = 0;
    uint8_t  salt[Crypto::kSpake2p_Max_PBKDF_Salt_Length] = {0};
    MutableByteSpan saltSpan(salt, sizeof(salt));

    ReturnErrorOnFailure(GetSetupPasscode(passcode));
    ReturnErrorOnFailure(GetSpake2pIterationCount(iter));
    ReturnErrorOnFailure(GetSpake2pSalt(saltSpan));

    Crypto::Spake2pVerifier verifier;
    ReturnErrorOnFailure(verifier.Generate(iter, saltSpan, passcode));
    ReturnErrorOnFailure(verifier.Serialize(verifierBuf));   // 97바이트로 reduce_size
    verifierLen = verifierBuf.size();
    return CHIP_NO_ERROR;
}

CHIP_ERROR EfuseCommissionableDataProvider::GetSetupPasscode(uint32_t & setupPasscode)
{
    ensureDerived();
    setupPasscode = mPasscode;
    return CHIP_NO_ERROR;
}
