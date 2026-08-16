# -*- coding: utf-8 -*-
"""리셋 없이 ESP32 USB-Serial-JTAG 로그를 읽는다.

★왜 리셋이 났는가
   pyserial 은 open() 시 DTR/RTS 를 **assert** 한다. ESP32 의 USB-Serial-JTAG 는
   CDC SET_CONTROL_LINE_STATE 를 esptool 의 EN/BOOT 자동리셋 배선처럼 해석하므로,
   포트를 여는 것만으로 칩이 리셋된다. → 측정 데이터가 매번 날아갔다.

★대책
   open() **전에** dtr/rts 를 False 로 지정하면 pyserial 이 DCB 의
   fDtrControl/fRtsControl 을 DISABLE 로 열어 assert 자체가 일어나지 않는다.

★기존 지뢰(그대로 유지)
   flush / reset_input_buffer / in_waiting / read(N) 은 hang → **readline 만** 쓴다.
"""
import serial
import time


def open_noreset(port='COM3', baud=115200, timeout=0.3, write_timeout=None):
    """리셋을 유발하지 않고 포트를 연다."""
    sp = serial.Serial()
    sp.port = port
    sp.baudrate = baud
    sp.timeout = timeout
    sp.dtr = False          # ★ open 전에 내려야 효과가 있다
    sp.rts = False          # ★
    if write_timeout is not None:
        sp.write_timeout = write_timeout
    sp.open()
    return sp


def read_for(sp, seconds):
    """readline 만 사용해 seconds 동안 수집한다."""
    out = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        ln = sp.readline()
        if ln:
            out.append(ln)
    return b''.join(out).decode('utf-8', 'replace')
