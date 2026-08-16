# -*- coding: utf-8 -*-
"""NVS 덤프에서 부팅 진단(bootdiag)을 읽는다 — 기기를 켜지 않고.

H2 는 콘솔을 못 쓰므로(heap → linenoise 폭주) `bd` 명령을 보낼 수 없다.
배터리 구동 중 재부팅이 반복됐을 때 **무엇 때문에 죽었는지**는 여기서만 읽힌다.

    esptool --chip esp32h2 -p COM3 --before default_reset --after hard_reset \
            read_flash 0x10000 0xC000 nvs.bin
    python sim/tools/bootdiag_decode.py nvs.bin

main/boot_diag.c 의 boot_rec_t (20B) 와 짝을 이룬다:
    uint32 magic / uint32 boot_count / uint8 stage stage2 reset_reason sub
    int16 bat_mv / int16 bat_min_mv / uint32 uptime_ms
"""
import struct
import sys

sys.path.insert(0, __file__.rsplit('\\', 1)[0].rsplit('/', 1)[0])
from nvs_parse import parse   # noqa: E402

REC = struct.Struct('<IIBBBBhhI')
MAGIC = 0x42443034     # "BD04"

# esp_reset_reason_t
RST = {0: '알수없음', 1: '전원투입', 2: '외부리셋', 3: 'SW재시작',
       4: '★패닉/예외', 5: '★INT워치독', 6: '★TASK워치독', 7: '★기타워치독',
       8: 'deep sleep 복귀', 9: '★★브라운아웃(전압강하)', 10: 'SDIO',
       11: 'USB리셋(플래시/포트열기)', 12: 'JTAG', 13: 'eFuse오류',
       14: '★전원글리치', 15: '★CPU락업'}

S1 = {0: '-', 1: 'app_main/NVS', 2: 'Matter 엔드포인트', 3: 'blind_manager',
      4: 'Matter start 직전', 5: 'Matter start 완료', 6: 'somfy_app 태스크 생성',
      7: '콘솔 섹션 진입', 8: '콘솔 명령 등록 완료', 9: 'app_main 완료'}
S2 = {0: '-', 7: '메인 루프 진입', 8: '정상 가동'}


def show(name, raw):
    if not raw or len(raw) < REC.size:
        print('  %s: (없음)' % name); return
    m, cnt, s1, s2, rst, sub, mv, mmv, up = REC.unpack_from(raw, 0)
    if m != MAGIC:
        print('  %s: magic 불일치 0x%08X (형식이 바뀐 옛 기록)' % (name, m)); return
    print('  %-6s %4d회차  app_main=%d(%s)  somfy_app=%d(%s)  sub=%d'
          % (name, cnt, s1, S1.get(s1, '?'), s2, S2.get(s2, '?'), sub))
    print('         uptime %s  bat=%dmV/최저%dmV  리셋사유=%s'
          % (('%.1f초' % (up / 1000.0)) if up < 600000 else
             ('%.1f분' % (up / 60000.0)), mv, mmv, RST.get(rst, '?%d' % rst)))


def main(path):
    _, items = parse(open(path, 'rb').read())
    got = {}
    for it in items:
        if it['ns_name'] != 'bootdiag':
            continue
        if isinstance(it['value'], (bytes, bytearray)) or it['key'] not in got:
            got[it['key']] = it['value']
    print('=== 부팅 진단 ===')
    show('직전', got.get('rec'))
    show('실패', got.get('fail'))
    n = got.get('failn')
    if isinstance(n, int):
        print('  보관된 실패부팅 누적: %d회' % n)
    if not got:
        print('  bootdiag 네임스페이스 없음')
    return 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    sys.exit(main(sys.argv[1]))
