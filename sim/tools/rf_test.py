#!/usr/bin/env python3
"""rf_test — 무인 RF 테스트 루프: 기기 제어 → RF 송신 → 캡처 → 분석.

  python rf_test.py COM8 "tx updown"
  python rf_test.py COM8 "tx myup" --freq 447.675M --secs 3

동작:
  1) somfy_cli record 를 백그라운드로 시작(rtl_433 이 RF 를 계속 감시)
  2) 시리얼로 기기에 명령 전송 → 펌웨어가 RF 송신
  3) --secs 동안 캡처
  4) record 종료
  5) 캡처 JSON(rtl_433 -F json, 라인별) 요약 출력

전제: RTL-SDR 동글 연결 + rtl_433.exe(somfy_cli 가 자동탐지). SDR# GUI 는 꺼야 함
(동글을 동시에 못 쓴다).
"""
import subprocess, sys, os, time, json, signal, argparse, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import serial_tx
SOMFY_CLI = r"D:\dev\workspaces\plugin-Rtl433-for-SdrSharp-master\Rtl_433_Plugin\tools\somfy_cli.py"

def run(port, cmd, freq="447.675M", rate="250000", secs=3.0, warmup=3.0, capdir=None):
    capdir = capdir or os.path.join(HERE, "caps")
    os.makedirs(capdir, exist_ok=True)
    jf = os.path.join(capdir, "rf.json")
    if os.path.exists(jf):
        try: os.remove(jf)
        except OSError: pass

    print(f"[1] record 시작 (freq={freq})")
    rec = subprocess.Popen(
        [sys.executable, SOMFY_CLI, "record", "--freq", freq, "--rate", rate,
         "--out", capdir, "--json", jf],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
    time.sleep(warmup)                      # rtl_433 동글 warmup

    if rec.poll() is not None:               # 이미 죽음 = 동글/exe 문제
        print("[!] record 즉시 종료 — 동글 미연결/rtl_433 없음/SDR# 점유 의심")
        print((rec.stdout.read() or b"").decode("utf-8", "replace")[-1500:])
        return

    print(f"[2] 기기 제어: {cmd}")
    print(serial_tx.send(port, cmd))
    print(f"[3] {secs}s 캡처…")
    time.sleep(secs)

    print("[4] record 종료")
    try:
        rec.send_signal(signal.CTRL_BREAK_EVENT)
        rec.wait(timeout=5)
    except Exception:
        rec.terminate()
        try: rec.wait(timeout=3)
        except Exception: rec.kill()

    print("[5] 분석")
    if not os.path.exists(jf):
        print("  캡처 JSON 없음 — 신호 미검출(freq 불일치?) 또는 동글 문제")
        return
    with open(jf, encoding="utf-8-sig") as f:
        msgs = [json.loads(l) for l in f if l.strip()]
    somfy = [m for m in msgs if str(m.get("model", "")).lower().startswith("somfy")]
    print(f"  총 {len(msgs)}개 메시지, Somfy {len(somfy)}개")
    for m in somfy[-12:]:
        print("   ", {k: m.get(k) for k in ("id", "control", "counter", "cmd") if k in m} or m)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("port"); ap.add_argument("cmd")
    ap.add_argument("--freq", default="447.675M"); ap.add_argument("--rate", default="250000")
    ap.add_argument("--secs", type=float, default=3.0)
    a = ap.parse_args()
    run(a.port, a.cmd, a.freq, a.rate, a.secs)
