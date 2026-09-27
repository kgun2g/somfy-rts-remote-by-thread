# -*- coding: utf-8 -*-
"""LP halt A/B — 두 보드 NVS 를 뜨고 절전 지표를 나란히 비교한다.

왜 필요한가 (2026-09-10)
────────────────────────
`SOMFY_LP_HALT`(pcf_lp_config.h) 전환 뒤 깨어남이 32.6 → 339회/초로 늘고 sleep 이
96 → 81% 로 떨어졌다는 기록이 있는데, 그 A/B 를 끝내지 못했다. 원인 가설(“LP 타이머가
HP 를 깨운다”)은 IDF 소스로 **반증**됐다 — HP 기상은 LP_TIMER target[0], LP 코어는
target[1] 로 분리돼 있고 HP 는 `ulp_lp_core_wakeup_main_processor()` 의 명시적 PMU
소프트 인터럽트로만 깨어난다. 그러니 기전 추정을 접고 **인과부터 실측**한다.

배치 (이 값이 곧 라벨이다 — 바꾸면 여기도 고칠 것):
    COM7 = SOMFY_LP_HALT 0   (옛 방식: while(1) + busy-wait)
    COM8 = SOMFY_LP_HALT 1   (현재:   폴 1회 + halt + LP 타이머)

기준선(2026-09-10, 양쪽 다 HALT=1 로 11.7시간):
    COM7 334.6회/초 · 2.73ms · sleep 91.3%
    COM8 337.1회/초 · 2.72ms · sleep 91.7%
→ 두 보드가 같은 조건에서 사실상 동일했으므로, 이번에 갈리면 그건 스위치 탓이다.

주의: 배터리 구간에서만 light sleep 이 걸린다. USB 를 뽑은 채 최소 몇 시간 두고,
다시 꽂은 **직후** 이 스크립트를 돌릴 것(재부팅되면 카운터가 날아간다 — 다만
`pmdump/locks`·`locks0` 는 NVS 라 재부팅에도 남는다).

    python sim/tools/lp_ab_collect.py
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
ESPTOOL = os.path.expanduser(
    r"~/.espressif/python_env/idf5.5_py3.14_env/Scripts/esptool.exe")

# 포트 → (라벨, SOMFY_LP_HALT 값)
BOARDS = [("COM7", "HALT=0  옛 방식(busy-wait)"),
          ("COM8", "HALT=1  현재(halt+LP타이머)")]


def dump(port, path):
    for _ in range(3):
        r = subprocess.run([ESPTOOL, "--chip", "esp32c6", "-p", port,
                            "--before", "default_reset", "--after", "hard_reset",
                            "read_flash", "0x10000", "0xC000", path],
                           capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        if r.returncode == 0 and os.path.exists(path) and os.path.getsize(path):
            return True
        time.sleep(3)
    print("   [%s] 덤프 실패: %s" % (port, (r.stdout or '')[-200:].strip()))
    return False


def main():
    outdir = os.path.join(ROOT, "logs")
    os.makedirs(outdir, exist_ok=True)
    stamp = time.strftime("%m%d_%H%M")
    for port, label in BOARDS:
        print("\n" + "=" * 72)
        print("  %s   %s" % (port, label))
        print("=" * 72)
        path = os.path.join(outdir, "nvs_%s_ab_%s.bin" % (port, stamp))
        if not dump(port, path):
            continue
        env = dict(os.environ, PYTHONIOENCODING="utf-8")
        r = subprocess.run([sys.executable,
                            os.path.join(HERE, "pmdump_decode.py"), path],
                           capture_output=True, text=True,
                           encoding="utf-8", errors="replace", env=env)
        # 차분 요약만 먼저 보여주고, 원문은 파일로 남는다
        for line in (r.stdout or "").splitlines():
            if line.startswith("=== esp_pm_dump_locks"):
                break
            print(line)
        print("   (원문: %s)" % os.path.relpath(path, ROOT))
    print("\n판정: **초당 절전 횟수**를 본다. sleep%% 는 지표가 아니다 —")
    print("      C6 는 절전 1회당 약 2.52ms 가 고정비용이라, 평균 절전이 그보다")
    print("      짧으면 sleep 90%% 여도 실제 절약은 거의 없다.")
    return 0


if __name__ == '__main__':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.exit(main())
