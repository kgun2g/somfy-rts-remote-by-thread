import serial, time, sys
# read_serial.py 성공 패턴 그대로: 포트지정 즉시 생성(DTR 기본) + readline(timeout).
# (DTR off + read(N) 은 이 USB JTAG 에서 timeout 무시 → 무한 hang.)
ser = serial.Serial('COM8', 115200, timeout=1)
print('OPENED', flush=True)
time.sleep(0.3)
ser.write(b'help\r\n')
out = ''
for _ in range(12):
    out += ser.readline().decode('utf-8', 'replace')
print('=== help: tx/sel 등록? ===', flush=True)
hit = False
for ln in out.splitlines():
    if any(k in ln.lower() for k in ('tx', 'sel', 'somfy')):
        print('  ', ln, flush=True); hit = True
if not hit:
    print('  (tx/sel 안 보임) help 마지막 500자:', flush=True)
    print(out[-500:], flush=True)
ser.write(b'tx up\r\n')
out2 = ''
for _ in range(12):
    out2 += ser.readline().decode('utf-8', 'replace')
print('=== after "tx up" ===', flush=True)
print(out2[-900:], flush=True)
ser.close(); print('DONE', flush=True)
