# -*- coding: utf-8 -*-
"""ESP-IDF NVS 파티션 덤프를 오프라인으로 읽는다 — 기기를 건드리지 않고.

왜 필요한가 (2026-08-16)
────────────────────────
진단 기록(`bl`/`vl`/`bd`)을 콘솔로 읽으려면 **포트를 열어야 하고, 그 순간 칩이
리셋된다**. 게다가 H2 는 heap 이 빠듯해 콘솔을 켜면 linenoise 4KB 할당 실패로
콘솔이 폭주하며 somfy_app 을 굶겨 task_wdt 가 터진다(app_main.cpp 주석 참조)
→ H2 에서는 콘솔 자체가 선택지가 아니다.

대신 파티션을 통째로 떠서(리셋은 나지만 앱은 안 뜸) 오프라인에서 읽는다:

    esptool --chip esp32h2 -p COM3 --before default_reset --after no_reset \
            read_flash 0x10000 0xC000 nvs.bin
    python sim/tools/nvs_parse.py nvs.bin --ns batlog

NVS 온-플래시 형식 (components/nvs_flash)
──────────────────────────────────────────
  페이지 4096B = 헤더 32B + 엔트리상태 비트맵 32B + 엔트리 126개 × 32B
  엔트리상태: 2bit/엔트리 — 0b11 비어있음 / 0b10 기록됨 / 0b00 지워짐
  엔트리 32B = NsIndex1 Type1 Span1 ChunkIdx1 CRC4 Key16 Data8
  네임스페이스는 NsIndex==0 인 U8 엔트리로 정의된다(키=이름, 값=인덱스).
  가변길이(STR/BLOB_DATA)는 Data 앞 2B 가 길이이고 실제 내용은 **뒤따르는
  엔트리들의 raw 바이트**에 있다(Span 이 그 개수를 포함).
  큰 blob 은 BLOB_IDX(0x48) + BLOB_DATA(0x42) 청크로 쪼개진다.
"""
import struct
import sys

PAGE = 4096
ENTRY = 32
ENTRIES_PER_PAGE = 126

T_U8, T_I8 = 0x01, 0x11
T_U16, T_I16 = 0x02, 0x12
T_U32, T_I32 = 0x04, 0x14
T_U64, T_I64 = 0x08, 0x18
T_STR = 0x21
T_BLOB = 0x41
T_BLOB_DATA = 0x42
T_BLOB_IDX = 0x48

_FIXED = {
    T_U8: ('<B', 1), T_I8: ('<b', 1),
    T_U16: ('<H', 2), T_I16: ('<h', 2),
    T_U32: ('<I', 4), T_I32: ('<i', 4),
    T_U64: ('<Q', 8), T_I64: ('<q', 8),
}


def _entry_states(page):
    """2bit × 126 상태를 리스트로."""
    bm = page[32:64]
    out = []
    for i in range(ENTRIES_PER_PAGE):
        b = bm[i // 4]
        out.append((b >> ((i % 4) * 2)) & 0x3)
    return out


def parse(blob):
    """(namespaces, items) 반환. items = [dict(ns, key, type, value, chunk)]"""
    ns_names = {}
    raw = []
    for p0 in range(0, len(blob) - PAGE + 1, PAGE):
        page = blob[p0:p0 + PAGE]
        state = struct.unpack('<I', page[0:4])[0]
        if state == 0xFFFFFFFF:
            continue                     # 미사용 페이지
        st = _entry_states(page)
        body = page[64:64 + ENTRIES_PER_PAGE * ENTRY]
        i = 0
        while i < ENTRIES_PER_PAGE:
            if st[i] != 0x2:             # 기록됨(0b10) 만 취급
                i += 1
                continue
            e = body[i * ENTRY:(i + 1) * ENTRY]
            ns, typ, span, chunk = e[0], e[1], e[2], e[3]
            key = e[8:24].split(b'\x00')[0].decode('utf-8', 'replace')
            data = e[24:32]
            if span < 1:
                span = 1
            val = None
            if typ in _FIXED:
                fmt, n = _FIXED[typ]
                val = struct.unpack(fmt, data[:n])[0]
            elif typ in (T_STR, T_BLOB_DATA, T_BLOB):
                size = struct.unpack('<H', data[0:2])[0]
                start = (i + 1) * ENTRY
                val = body[start:start + size]
            elif typ == T_BLOB_IDX:
                total, cnt, cstart = struct.unpack('<IBB', data[0:6])
                val = {'size': total, 'chunks': cnt, 'chunk_start': cstart}
            if ns == 0 and typ == T_U8:
                ns_names[val] = key      # 네임스페이스 정의
            raw.append({'ns': ns, 'key': key, 'type': typ, 'value': val,
                        'chunk': chunk})
            i += span
    # blob 청크 합치기
    items = []
    chunks = {}
    for it in raw:
        if it['type'] == T_BLOB_DATA:
            chunks.setdefault((it['ns'], it['key']), {})[it['chunk']] = it['value']
        else:
            items.append(it)
    for (ns, key), parts in chunks.items():
        joined = b''.join(parts[k] for k in sorted(parts))
        items.append({'ns': ns, 'key': key, 'type': T_BLOB,
                      'value': joined, 'chunk': 0})
    for it in items:
        it['ns_name'] = ns_names.get(it['ns'], '?%d' % it['ns'])
    return ns_names, items


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    path = sys.argv[1]
    want = None
    if '--ns' in sys.argv:
        want = sys.argv[sys.argv.index('--ns') + 1]
    blob = open(path, 'rb').read()
    ns_names, items = parse(blob)
    print('네임스페이스: %s' % ', '.join(sorted(ns_names.values())))
    for it in sorted(items, key=lambda x: (x['ns_name'], x['key'])):
        if it['ns'] == 0:
            continue
        if want and it['ns_name'] != want:
            continue
        v = it['value']
        if isinstance(v, (bytes, bytearray)):
            print('  %-10s %-10s blob %5dB' % (it['ns_name'], it['key'], len(v)))
        else:
            print('  %-10s %-10s %s' % (it['ns_name'], it['key'], v))
    return 0


if __name__ == '__main__':
    sys.exit(main())
