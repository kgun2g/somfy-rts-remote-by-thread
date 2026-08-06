# 시리얼 캡처 — VIBE 진단용. UTF-8 출력(cp949 인코딩 에러 방지).
import serial, sys, time, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
try:
    ser = serial.Serial(port, baud, timeout=1)
except Exception as e:
    print('OPEN FAIL: %s' % e, flush=True); sys.exit(1)
print('=== OPEN %s @ %d ===' % (port, baud), flush=True)
try:
    while True:
        line = ser.readline()
        if not line: continue
        text = line.decode('utf-8', errors='replace').rstrip('\r\n')
        print('[%s] %s' % (time.strftime('%H:%M:%S'), text), flush=True)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
