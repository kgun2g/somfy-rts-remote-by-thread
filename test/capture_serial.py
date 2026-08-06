#!/usr/bin/env python3
# 단순 시리얼 캡처기. ESC 처리/색상 디코드 없이 raw bytes 를 utf-8(errors=replace)
# 로 디코딩해 시간/태그 그대로 stdout + 파일에 동시 기록.
# 종료: Ctrl-C 또는 부모 프로세스 종료.
import sys, time, os, serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
log  = sys.argv[2] if len(sys.argv) > 2 else "rxbyte_serial.log"
baud = 115200

print(f"[capture] opening {port} @ {baud} -> {log}", flush=True)
ser = None
for i in range(20):
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
        break
    except Exception as e:
        print(f"[capture] open attempt {i+1}: {e}", flush=True)
        time.sleep(1)
if ser is None:
    sys.exit(2)

f = open(log, "ab", buffering=0)
f.write(f"\n===== capture start {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n".encode())

# 시리얼 보유 상태에서 RTS 토글 → 디바이스 재부팅(부팅 로그 0초부터 캡처).
# ESP32-C6 reset 회로: RTS=LOW(=DTR don't care) → EN low → reset.
ser.setDTR(False)
ser.setRTS(True);  time.sleep(0.1)
ser.setRTS(False); time.sleep(0.05)
print("[capture] RTS toggled - device reset", flush=True)

try:
    while True:
        chunk = ser.read(4096)
        if chunk:
            sys.stdout.buffer.write(chunk)
            sys.stdout.flush()
            f.write(chunk)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    f.close()
