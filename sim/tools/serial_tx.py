#!/usr/bin/env python3
"""serial_tx — 펌웨어 CHIP shell 에 tx/sel 명령을 보내고 응답을 읽는다.

  python serial_tx.py COM8 "tx updown"
  python serial_tx.py COM8 "sel 0"

★ ESP32 USB Serial JTAG 안전 규칙(이 환경에서 반드시):
   - reset_input_buffer()/flush()/in_waiting 은 hang → 절대 사용 금지
   - DTR/RTS off + read(N) 조합도 timeout 무시하고 hang
   - 안전한 건 '포트지정 즉시 생성(DTR 기본) + readline(timeout)' 뿐 (read_serial.py 패턴)
   - 콘솔이 USB Serial JTAG primary 여야 write(명령)가 먹힘(secondary=stdin없음→write hang)
"""
import serial, time, sys

def send(port, cmd, baud=115200, lines=15):
    ser = serial.Serial(port, baud, timeout=1)     # 즉시 open, DTR 기본
    time.sleep(0.25)
    ser.write((cmd + "\r\n").encode())
    out = ""
    for _ in range(lines):
        out += ser.readline().decode("utf-8", "replace")
    ser.close()
    return out

if __name__ == "__main__":
    if len(sys.argv) < 3:
        sys.exit('usage: serial_tx.py <COM> "<cmd>" [baud]')
    port, cmd = sys.argv[1], sys.argv[2]
    baud = int(sys.argv[3]) if len(sys.argv) > 3 else 115200
    print(send(port, cmd, baud), end="")
