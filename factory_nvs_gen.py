#!/usr/bin/env python3
"""
factory_nvs_gen.py
─────────────────────────────────────────────────────────────────
ESP32 팩토리 파티션(fctry) NVS 이미지 생성 — Matter 커미셔닝 설정
(discriminator/passcode 등) 을 고정값으로 지정하고 싶을 때 사용.

※ 기본 빌드는 main/efuse_commissionable.cpp 가 eFuse MAC 에서 discriminator/
   passcode 를 기기마다 자동 산출한다(CONFIG_CUSTOM_COMMISSIONABLE_DATA_PROVIDER=y).
   따라서 이 스크립트는 **선택사항** — 특정 고정 코드를 fctry 에 넣을 때만 쓴다.

사용법:
    python factory_nvs_gen.py [--passcode 12345678] [--discriminator 3840]

출력:
    factory_nvs.bin  →  파티션 테이블의 'fctry' 파티션에 플래시

플래시 명령:
    esptool.py -p COM3 write_flash 0x3E0000 factory_nvs.bin
─────────────────────────────────────────────────────────────────
"""

import argparse
import os
import sys
import struct
import random

# esp-idf의 nvs_partition_gen.py 활용
# (IDF_PATH)/tools/mass_mfr/mfr_gen.py 또는
# (IDF_PATH)/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py

NVS_PARTITION_SIZE = 0x6000  # 24 KB (partitions.csv의 fctry 크기)

def gen_passcode():
    """유효한 Matter passcode 생성 (11자리 숫자)"""
    # Matter 스펙: 00000001 ~ 99999998, 일부 값 제외
    invalid = {11111111, 22222222, 33333333, 44444444,
               55555555, 66666666, 77777777, 88888888,
               99999999, 12345678, 87654321}
    while True:
        code = random.randint(1, 99999998)
        if code not in invalid:
            return code

def gen_discriminator():
    """12비트 discriminator 생성 (0~4095)"""
    return random.randint(0, 4095)

def main():
    parser = argparse.ArgumentParser(
        description='Somfy Blind Controller - Factory NVS 생성')
    parser.add_argument('--passcode',
                        type=int,
                        default=None,
                        help='Matter passcode (기본: 랜덤)')
    parser.add_argument('--discriminator',
                        type=int,
                        default=None,
                        help='Matter discriminator 0~4095 (기본: 랜덤)')
    parser.add_argument('--vendor-id',
                        type=lambda x: int(x, 16),
                        default=0xFFF1,
                        help='Vendor ID (hex, 기본: 0xFFF1)')
    parser.add_argument('--product-id',
                        type=lambda x: int(x, 16),
                        default=0x8001,
                        help='Product ID (hex, 기본: 0x8001)')
    parser.add_argument('--output',
                        default='factory_nvs.bin',
                        help='출력 파일 이름')
    args = parser.parse_args()

    passcode      = args.passcode     or gen_passcode()
    discriminator = args.discriminator if args.discriminator is not None \
                                      else gen_discriminator()

    print("=" * 60)
    print("  Somfy Blind Controller - Factory NVS 설정")
    print("=" * 60)
    print(f"  Passcode:      {passcode:08d}")
    print(f"  Discriminator: {discriminator}")
    print(f"  Vendor ID:     0x{args.vendor_id:04X}")
    print(f"  Product ID:    0x{args.product_id:04X}")
    print("=" * 60)

    # CSV 파일 생성 (nvs_partition_gen.py 입력 형식)
    csv_content = f"""key,type,encoding,value
chip-factory,namespace,,
mfr-passcode,data,u32,{passcode}
mfr-discriminator,data,u16,{discriminator}
vendor-id,data,u16,{args.vendor_id}
product-id,data,u16,{args.product_id}
"""

    csv_path = 'factory_nvs_input.csv'
    with open(csv_path, 'w') as f:
        f.write(csv_content)
    print(f"\n  CSV 생성: {csv_path}")

    # nvs_partition_gen.py 실행
    idf_path = os.environ.get('IDF_PATH') or os.path.join(os.environ.get('WORKSPACES_PATH', ''), 'esp-idf')
    nvs_gen  = os.path.join(idf_path,
        'components', 'nvs_flash', 'nvs_partition_generator', 'nvs_partition_gen.py')

    if os.path.exists(nvs_gen):
        cmd = f'python "{nvs_gen}" generate {csv_path} {args.output} {NVS_PARTITION_SIZE}'
        print(f"\n  실행: {cmd}")
        ret = os.system(cmd)
        if ret == 0:
            print(f"\n  ✓ NVS 이미지 생성 완료: {args.output}")
            print(f"\n  플래시 명령:")
            print(f"  esptool.py -p COM3 write_flash 0x3E0000 {args.output}")
        else:
            print("  ✗ NVS 이미지 생성 실패")
    else:
        print(f"\n  ⚠ nvs_partition_gen.py를 찾을 수 없습니다.")
        print(f"    경로: {nvs_gen}")
        print(f"    IDF_PATH 환경변수를 확인하세요.")
        print(f"\n  CSV 파일은 생성되었습니다: {csv_path}")

    # QR 코드 URL 생성 (Matter 온보딩)
    # MT: 형식: MT:{base38_encoded_payload}
    print(f"\n  Matter 설정 정보:")
    print(f"  - SmartThings 앱 → 디바이스 추가 → Matter 디바이스")
    print(f"  - 수동 코드: {passcode:08d}-{discriminator:04d}")
    print(f"  - OLED 화면의 QR코드로도 페어링 가능합니다")

if __name__ == '__main__':
    main()
