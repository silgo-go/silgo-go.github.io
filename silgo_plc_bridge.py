#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""실고 자설기 ↔ 진짜 PLC 브리지 (파이썬 3.8+, 따로 설치할 것 없음)

실고(자동화설비기능사 실습대의 '외부 PLC' 모드)가 0.1초마다 보내는 센서·스위치 16개(X)를
PLC 메모리에 쓰고, PLC 출력 10개(Y)를 읽어서 실고로 돌려줍니다.

  실고 X 순서: LS1~LS8, S1~S4, SW1~SW3, 비상   (X0~X7, X10~X17)
  실고 Y 순서: SOL1~SOL5, M1, M2, PL1~PL3      (Y0~Y7, Y10, Y11)

사용 예
  python silgo_plc_bridge.py --plc sim                          # PLC 없이 연결만 시험
  python silgo_plc_bridge.py --plc ls   --host 192.168.0.10     # LS XGT (XGB/XGK) 이더넷
  python silgo_plc_bridge.py --plc mc   --host 192.168.0.20 --port 5007   # 미쓰비시 MC 3E (Q/L/iQ-R/FX5U)
  python silgo_plc_bridge.py --plc modbus --host 192.168.0.30   # Modbus TCP

그다음 같은 PC의 크롬에서 실고 → 자설기 → 제어 모드 '외부 PLC' → ws://localhost:8765 연결.

주의: 진짜 PLC의 입력(P000x / X)은 실제 단자 상태로 매 스캔 덮어쓰이므로, 기본값은 센서 값을
M 영역에 씁니다. 래더에서 입력 접점을 그 M 주소로 바꾸거나(--x 로 주소 지정), 입력 모듈이
없는 시뮬레이터라면 --x 로 입력 주소를 직접 지정하세요.
"""
import argparse, asyncio, base64, hashlib, json, socket, struct, sys, time

NX, NY = 16, 10

# ---------------------------------------------------------------- PLC 드라이버
class Plc:
    def __init__(s, a): s.a = a; s.sock = None
    def connect(s):
        s.close(); s.sock = socket.create_connection((s.a.host, s.a.port), timeout=2); s.sock.settimeout(2)
    def close(s):
        try:
            if s.sock: s.sock.close()
        except OSError: pass
        s.sock = None
    def recvn(s, n):
        b = b""
        while len(b) < n:
            c = s.sock.recv(n - len(b))
            if not c: raise ConnectionError("PLC가 연결을 끊었어요")
            b += c
        return b

class Sim(Plc):
    """PLC 없이 시험: PB1(SW1)을 누르면 컨베이어, 매거진에 공작물이 있으면 녹색등, 비상정지면 적색 점멸"""
    def connect(s): pass
    def cycle(s, X):
        Y = [0] * NY
        if X[15]: Y[7] = int(time.time() * 2) % 2; return Y
        Y[9] = X[8]; Y[6] = X[12]; Y[8] = X[13]
        return Y

class Modbus(Plc):
    """Modbus TCP: 코일 쓰기(FC15) / 코일 읽기(FC1). --x, --y 는 코일 번호(0부터)"""
    def __init__(s, a): super().__init__(a); s.tid = 0; s.xa = int(a.x or 100); s.ya = int(a.y or 0)
    def req(s, pdu):
        s.tid = (s.tid + 1) & 0xFFFF
        s.sock.sendall(struct.pack(">HHHB", s.tid, 0, len(pdu) + 1, s.a.unit) + pdu)
        h = s.recvn(7); body = s.recvn(struct.unpack(">H", h[4:6])[0] - 1)
        if body[0] & 0x80: raise IOError("Modbus 오류 코드 %d" % body[1])
        return body
    def cycle(s, X):
        v = sum(b << i for i, b in enumerate(X))
        s.req(struct.pack(">BHHB", 15, s.xa, NX, 2) + struct.pack("<H", v))
        r = s.req(struct.pack(">BHH", 1, s.ya, NY)); v = int.from_bytes(r[2:2 + r[1]], "little")
        return [(v >> i) & 1 for i in range(NY)]

class Xgt(Plc):
    """LS XGT 전용 프로토콜 (FEnet, TCP 2004). 워드 단위로 읽고 씀.
    --x %MW10 → M0100~M010F 에 센서 16개,  --y %PW2 → P0020~P0029 에서 출력 10개"""
    def __init__(s, a): super().__init__(a); s.iv = 0; s.xv = a.x or "%MW10"; s.yv = a.y or "%PW2"
    def frame(s, body):
        s.iv = (s.iv + 1) & 0xFFFF
        h = b"LSIS-XGT" + b"\x00\x00" + b"\x00\x00" + b"\xA0" + b"\x33" + struct.pack("<HH", s.iv, len(body)) + b"\x00"
        return h + bytes([sum(h) & 0xFF]) + body
    def xfer(s, body):
        s.sock.sendall(s.frame(body)); h = s.recvn(20); d = s.recvn(struct.unpack("<H", h[16:18])[0])
        if struct.unpack("<H", d[6:8])[0]: raise IOError("XGT 오류 0x%04X (주소 %s / %s 확인)" % (struct.unpack("<H", d[8:10])[0], s.xv, s.yv))
        return d
    def cycle(s, X):
        v = sum(b << i for i, b in enumerate(X)); n = s.xv.encode()
        s.xfer(struct.pack("<HHHH", 0x58, 2, 0, 1) + struct.pack("<H", len(n)) + n + struct.pack("<HH", 2, v))
        n = s.yv.encode(); d = s.xfer(struct.pack("<HHHH", 0x54, 2, 0, 1) + struct.pack("<H", len(n)) + n)
        v = struct.unpack("<H", d[12:14])[0]
        return [(v >> i) & 1 for i in range(NY)]

class Mc3e(Plc):
    """미쓰비시 MC 프로토콜 3E 프레임(바이너리, SLMP). 비트 단위 일괄 쓰기/읽기.
    --x M100 → M100~M115 에 센서 16개,  --y Y0 → Y0부터 10점 (FX: Y0~Y7·Y10·Y11 / Q: 16진 Y10~Y19 면 --y Y10)"""
    CODE = {"X": (0x9C, 16), "Y": (0x9D, 16), "M": (0x90, 10), "L": (0x92, 10), "B": (0xA0, 16)}
    def __init__(s, a): super().__init__(a); s.x = s.dev(a.x or "M100"); s.y = s.dev(a.y or "Y0")
    def dev(s, t):
        t = t.upper(); c, base = s.CODE[t[0]]
        if s.a.octal and t[0] in "XY": base = 8
        return c, int(t[1:], base)
    def xfer(s, cmd, sub, dev, pts, data=b""):
        body = struct.pack("<HHH", 0x0010, cmd, sub) + struct.pack("<I", dev[1])[:3] + bytes([dev[0]]) + struct.pack("<H", pts) + data
        s.sock.sendall(b"\x50\x00\x00\xFF\xFF\x03\x00" + struct.pack("<H", len(body)) + body)
        h = s.recvn(9); d = s.recvn(struct.unpack("<H", h[7:9])[0]); end = struct.unpack("<H", d[:2])[0]
        if end: raise IOError("MC 프로토콜 종료 코드 0x%04X (주소·통신 설정 확인)" % end)
        return d[2:]
    def cycle(s, X):
        s.xfer(0x1401, 1, s.x, NX, bytes((X[i] << 4) | X[i + 1] for i in range(0, NX, 2)))
        d = s.xfer(0x0401, 1, s.y, NY)
        return [(d[i // 2] >> (4 if i % 2 == 0 else 0)) & 1 for i in range(NY)]

DRIVERS = {"sim": (Sim, 0), "modbus": (Modbus, 502), "ls": (Xgt, 2004), "mc": (Mc3e, 5007)}

# ---------------------------------------------------------------- 웹소켓 (표준 라이브러리만)
async def ws_handshake(r, w, origins):
    req = (await r.readuntil(b"\r\n\r\n")).decode("latin1"); hd = {}
    for line in req.split("\r\n")[1:]:
        if ":" in line: k, v = line.split(":", 1); hd[k.strip().lower()] = v.strip()
    o = hd.get("origin", "")
    if "sec-websocket-key" not in hd or (origins and o and o not in origins):
        w.write(b"HTTP/1.1 403 Forbidden\r\n\r\n"); await w.drain(); return False
    acc = base64.b64encode(hashlib.sha1((hd["sec-websocket-key"] + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest())
    w.write(b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: " + acc + b"\r\n\r\n")
    await w.drain(); return True

async def ws_recv(r, w):
    while True:
        b0, b1 = await r.readexactly(2); op, n = b0 & 15, b1 & 127
        if n == 126: n = struct.unpack(">H", await r.readexactly(2))[0]
        elif n == 127: n = struct.unpack(">Q", await r.readexactly(8))[0]
        if n > 65536: raise ConnectionError("메시지가 너무 커요")
        mask = await r.readexactly(4) if b1 & 128 else b"\0\0\0\0"
        d = bytes(c ^ mask[i & 3] for i, c in enumerate(await r.readexactly(n)))
        if op == 8: raise ConnectionError("실고가 연결을 끊었어요")
        if op == 9: w.write(bytes([0x8A, len(d)]) + d); await w.drain(); continue
        if op in (1, 2): return d

def ws_send(w, text):
    d = text.encode(); n = len(d)
    w.write(bytes([0x81]) + (bytes([n]) if n < 126 else b"\x7e" + struct.pack(">H", n)) + d)

# ---------------------------------------------------------------- 본체
async def serve(a):
    plc = DRIVERS[a.plc][0](a); state = {"busy": False}
    async def client(r, w):
        peer = w.get_extra_info("peername")
        try:
            if not await ws_handshake(r, w, a.origin): return
            if state["busy"]: ws_send(w, json.dumps({"error": "busy"})); return
            state["busy"] = True; print("실고 연결됨:", peer); ok = False; last = 0
            try:
                while True:
                    msg = json.loads(await ws_recv(r, w)); X = [1 if v else 0 for v in (msg.get("X") or [])][:NX]
                    if len(X) < NX: continue
                    try:
                        if not ok: await asyncio.get_running_loop().run_in_executor(None, plc.connect); ok = True; print("PLC 연결됨:", a.plc, a.host or "")
                        Y = await asyncio.get_running_loop().run_in_executor(None, plc.cycle, X)
                        ws_send(w, json.dumps({"Y": Y})); await w.drain()
                    except (OSError, IOError, struct.error, IndexError) as e:
                        ok = False; plc.close()
                        if time.time() - last > 3: last = time.time(); print("PLC 통신 실패:", e)
                        await asyncio.sleep(0.5)      # 값을 안 보내면 실고 워치독(2초)이 출력을 모두 끔
            finally:
                state["busy"] = False; plc.close(); print("실고 연결 끊김 → PLC 연결도 닫음")
        except (ConnectionError, asyncio.IncompleteReadError, ValueError): pass
        finally:
            try: w.close()
            except OSError: pass
    srv = await asyncio.start_server(client, a.listen, a.ws_port)
    print("실고 PLC 브리지 준비됨  →  실고 '외부 PLC'에 ws://localhost:%d 입력" % a.ws_port)
    print("PLC: %s %s  (X→%s, Y←%s)   끝내기: Ctrl+C" % (a.plc, ("%s:%d" % (a.host, a.port)) if a.host else "", a.x or "기본", a.y or "기본"))
    async with srv: await srv.serve_forever()

def main():
    p = argparse.ArgumentParser(description="실고 자설기 ↔ PLC 브리지")
    p.add_argument("--plc", choices=list(DRIVERS), default="sim", help="sim(시험용) / ls(XGT) / mc(미쓰비시 3E) / modbus")
    p.add_argument("--host", help="PLC IP 주소"); p.add_argument("--port", type=int, help="PLC 포트 (기본: ls 2004, mc 5007, modbus 502)")
    p.add_argument("--x", help="센서 16개를 쓸 주소 (ls: %%MW10, mc: M100, modbus: 100)")
    p.add_argument("--y", help="출력 10개를 읽을 주소 (ls: %%PW2, mc: Y0, modbus: 0)")
    p.add_argument("--octal", action="store_true", help="mc: X·Y 번호를 8진수로 해석 (FX 계열)")
    p.add_argument("--unit", type=int, default=1, help="modbus 국번")
    p.add_argument("--ws-port", type=int, default=8765); p.add_argument("--listen", default="127.0.0.1", help="기본은 이 PC에서만 접속 허용")
    p.add_argument("--origin", action="append", default=["https://silgo-go.github.io", "null", "http://localhost", "http://127.0.0.1"], help="접속을 허용할 웹 주소 (추가)")
    a = p.parse_args()
    if a.plc != "sim" and not a.host: p.error("--host 에 PLC IP 주소를 넣어 주세요")
    a.port = a.port or DRIVERS[a.plc][1]
    try: asyncio.run(serve(a))
    except KeyboardInterrupt: print("\n끝")

if __name__ == "__main__": main()
