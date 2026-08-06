#!/usr/bin/env python3
"""sim_server — web_sim.html 서빙 + 빌드/플래시/CLI 실행 백엔드.

  python sim/sim_server.py     → http://localhost:8765/web_sim.html

브라우저는 프로세스를 못 돌리므로, 웹의 [빌드]/[플래시]/[CLI] 버튼이 여기로 fetch 하면
이 파이썬이 build.ps1 / esptool / somfy_cli.py 를 실행하고 로그를 돌려준다.
(WDAC: python·powershell·idf gcc·esptool 은 통과. emcc(WASM) 만 별도 예외 필요.)
"""
import http.server, socketserver, subprocess, json, os, sys

ROOT = os.path.dirname(os.path.abspath(__file__))   # sim/
PROJ = os.path.dirname(ROOT)                          # 프로젝트 루트
PORT = 8765
BOARD_MAP = {'h2': 'esp32-h2', 'c6': 'xiao-c6'}       # web → build.ps1 -Board
CLI = r'D:\dev\workspaces\plugin-Rtl433-for-SdrSharp-master\Rtl_433_Plugin\tools\somfy_cli.py'
sys.path.insert(0, os.path.join(ROOT, 'tools'))
import serial_tx
import threading
_ser_lock = threading.Lock()    # USB JTAG 는 동시 열기 금지 → 시리얼 요청 직렬화

def run(cmd, cwd=PROJ, timeout=2400):
    try:
        p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True,
                           timeout=timeout, encoding='utf-8', errors='replace')
        return p.returncode, (p.stdout or '') + (p.stderr or '')
    except subprocess.TimeoutExpired:
        return 1, '[TIMEOUT]'
    except Exception as e:
        return 1, f'[ERROR] {e}'

class H(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k): super().__init__(*a, directory=ROOT, **k)
    def _send(self, code, txt):
        b = txt.encode('utf-8', 'replace')
        self.send_response(code)
        self.send_header('Content-Type', 'text/plain; charset=utf-8')
        self.send_header('Content-Length', str(len(b)))
        self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        if self.path == '/api/ping': return self._send(200, 'ok')
        return super().do_GET()
    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0))
        try: data = json.loads(self.rfile.read(n) or b'{}')
        except Exception: data = {}
        board = BOARD_MAP.get(data.get('board', 'h2'), 'esp32-h2')
        ps1 = os.path.join(PROJ, 'build.ps1')
        def variant():   # web 콤보 → build.ps1 변형 파라미터(빈값 생략)
            a = []
            for k, flag in (('pcf','-Pcf'), ('rotary','-Rotary'), ('oled','-Oled'), ('rotate','-Rotate'), ('freq','-Freq')):
                v = str(data.get(k, '')).strip()
                if v: a += [flag, v]
            return a
        if self.path == '/api/tx':          # 실기기 RF 송신 (tx up/updown/…)
            c = str(data.get('cmd', '')).strip()
            port = data.get('port', 'COM8')
            if not c: return self._send(400, 'no cmd')
            with _ser_lock:
                try: out = serial_tx.send(port, 'tx ' + c)
                except Exception as e: return self._send(500, f'serial err: {e}')
            return self._send(200, out)
        if self.path == '/api/sel':         # 블라인드 선택
            port = data.get('port', 'COM8')
            with _ser_lock:
                try: out = serial_tx.send(port, 'sel %s' % data.get('n', 0))
                except Exception as e: return self._send(500, str(e))
            return self._send(200, out)
        if self.path == '/api/clean':
            cmd = ['powershell','-ExecutionPolicy','Bypass','-File',ps1,'-Board',board,'-Action','clean']
            rc, out = run(cmd)
            return self._send(200 if rc == 0 else 500, '> ' + ' '.join(cmd) + '\n\n' + out[-9000:])
        if self.path == '/api/build':
            cmd = ['powershell','-ExecutionPolicy','Bypass','-File',ps1,'-Board',board,'-Action','build'] + variant()
            rc, out = run(cmd)
            return self._send(200 if rc == 0 else 500, '> ' + ' '.join(cmd) + '\n\n' + out[-9000:])
        if self.path == '/api/flash':
            port = data.get('port', 'COM8')
            cmd = ['powershell','-ExecutionPolicy','Bypass','-File',ps1,'-Board',board,'-Port',port,'-Action','flash'] + variant()
            rc, out = run(cmd)
            return self._send(200 if rc == 0 else 500, '> ' + ' '.join(cmd) + '\n\n' + out[-9000:])
        if self.path == '/api/cli':
            # v1: somfy_cli 연결 확인(--help). 실제 extract 는 캡처 경로 인자로 확장.
            if not os.path.isfile(CLI):
                return self._send(500, f'somfy_cli.py 없음: {CLI}')
            rc, out = run([sys.executable, CLI, '--help'])
            return self._send(200 if rc == 0 else 500, out[-9000:])
        return self._send(404, 'no route')
    def log_message(self, *a): pass

if __name__ == '__main__':
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    with socketserver.ThreadingTCPServer(('127.0.0.1', PORT), H) as srv:
        print(f'sim_server  →  http://localhost:{PORT}/web_sim.html   (Ctrl-C 종료)')
        try: srv.serve_forever()
        except KeyboardInterrupt: print('\nbye')
