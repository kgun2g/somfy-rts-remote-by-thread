// efuse_commissionable.h
// ─────────────────────────────────────────────────────────────
// eFuse 팩토리 MAC 으로 Matter setup discriminator/passcode 를 기기마다
// 결정적·고유하게 산출하는 CommissionableDataProvider.
//
// 블라인드 주소(blind_manager _derive_addresses)와 같은 칩 고유 eFuse MAC 에서
// 파생하므로, fctry NVS(factory_nvs_gen.py) 를 플래시하지 않아도 기기별로 다른
// discriminator/passcode 를 갖는다 → 여러 대를 만들어도 BLE 커미셔닝 충돌 없음.
//
// 등록: app_main 에서 esp_matter::start() 직전에
//       esp_matter::set_custom_commissionable_data_provider(&provider).
//       (sdkconfig: CONFIG_CUSTOM_COMMISSIONABLE_DATA_PROVIDER=y 필요)
// DAC(attestation)는 별개 provider 라 그대로 유지된다.
// ─────────────────────────────────────────────────────────────
#pragma once

#include <crypto/CHIPCryptoPAL.h>
#include <platform/CommissionableDataProvider.h>

class EfuseCommissionableDataProvider : public chip::DeviceLayer::CommissionableDataProvider {
public:
    EfuseCommissionableDataProvider() = default;

    CHIP_ERROR GetSetupDiscriminator(uint16_t & setupDiscriminator) override;
    CHIP_ERROR SetSetupDiscriminator(uint16_t) override { return CHIP_ERROR_NOT_IMPLEMENTED; }
    CHIP_ERROR GetSpake2pIterationCount(uint32_t & iterationCount) override;
    CHIP_ERROR GetSpake2pSalt(chip::MutableByteSpan & saltBuf) override;
    CHIP_ERROR GetSpake2pVerifier(chip::MutableByteSpan & verifierBuf, size_t & verifierLen) override;
    CHIP_ERROR GetSetupPasscode(uint32_t & setupPasscode) override;
    CHIP_ERROR SetSetupPasscode(uint32_t) override { return CHIP_ERROR_NOT_IMPLEMENTED; }

private:
    void ensureDerived();                 // eFuse → 값 1회 산출 (캐싱)
    bool     mReady          = false;
    uint16_t mDiscriminator  = 0;         // 0..4095 (12bit)
    uint32_t mPasscode       = 0;         // 1..99999998 (유효 PIN)
    uint8_t  mSalt[16]       = {0};       // PBKDF salt (eFuse 결정적)
};
