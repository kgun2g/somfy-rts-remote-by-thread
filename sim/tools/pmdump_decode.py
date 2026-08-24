# -*- coding: utf-8 -*-
"""NVS 덤프에서 `esp_pm_dump_locks()` 출력을 꺼내 보여준다.

왜 필요한가 (2026-08-24): H2 가 배터리 13.9시간 동안 light sleep 을 **0.0%** 밖에
안 했다. batlog 의 pm 열은 3(= light sleep ON + 배터리)이라 **펌웨어 설정은 정상**
이다. 즉 누군가 ESP_PM_NO_LIGHT_SLEEP 락을 계속 쥐고 있다.
(같은 조건 C6 는 96.2% 잔다. USJ_NO_AUTO_LS_ON_CONNECTION 을 꺼봤지만 무변화 —
 그 가설은 틀렸다.)

esp_pm_dump_locks 는 stdout 으로만 출력하는데, H2 는 배터리 구동 중 USB 콘솔이
죽고 write 도 불안정해 콘솔로는 못 받는다. 그래서 펌웨어가 open_memstream 으로
받아 NVS("pmdump"/"locks")에 남기고, 여기서 꺼내 읽는다.

    esptool ... read_flash 0x10000 0xC000 nvs.bin
    python sim/tools/pmdump_decode.py nvs.bin
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nvs_parse import parse   # noqa: E402


def main(path):
    _, items = parse(open(path, 'rb').read())
    txt = None
    for it in items:
        if it['ns_name'] == 'pmdump' and it['key'] == 'locks':
            v = it['value']
            if isinstance(v, (bytes, bytearray)):
                txt = v.split(b'\x00')[0].decode('utf-8', 'replace')
            elif isinstance(v, str):
                txt = v
    if not txt:
        print('pmdump/locks 가 없다 — 배터리 구동 중 60초 주기로 저장된다.')
        print('USB 를 뺀 채 최소 1분 이상 두었다가 다시 연결해 덤프할 것.')
        return 1
    print('=== esp_pm_dump_locks (배터리 구동 중 마지막 저장분) ===')
    print(txt)
    print('=== 끝 ===')
    print()
    print('읽는 법: NO_LIGHT_SLEEP 류 락을 잡고 있는 항목이 범인이다.')
    print('        CONFIG_PM_PROFILING=y 면 보유 시간(%)까지 나온다.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]) if len(sys.argv) > 1 else (print(__doc__) or 1))
