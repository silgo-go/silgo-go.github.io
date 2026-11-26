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

스마트공장산업기사 (--ind)
  python silgo_plc_bridge.py --plc sim --ind                    # PLC 없이: 브리지 안의 가짜 PLC가 스텝 래더로 설비를 돌림
  python silgo_plc_bridge.py --plc ls --host 192.168.0.10 --ind # LS XGT (XG5000 시뮬레이터면 --host 127.0.0.1)
  실고 → PLC 비트 16개 (--x, ls 기본 %MW10 = M0100~M010F):
    후진끝, 전진끝, 매거진, 용량형, 유도형, 비전 위치, 촬영 완료, HMI 연속, HMI 종료, HMI 리셋, HMI 단속,
    비상정지 정상, 로봇 작업 완료, 종단 센서, 로봇 대기, 로봇 픽업 완료
  PLC → 실고 출력 비트 (--y, ls 기본 %PW2 = P0020~P002A):
    공급 실린더, (비움), (비움), 컨베이어, 녹색등, 황색등, 적색등, 로봇 작업 요청, 적재시트로, 불량함으로, 비전 촬영
  실고 → PLC 숫자: D110~D117 비전 결과(검사 완료·합격·불합격·빨강·파랑·흰색·OCR 판독·글자 코드), D200~D215 로봇 상태
  PLC → 실고 숫자: D224 로봇에 넘길 번호, D226 재질(1=금속)
  숫자 주소는 PLC의 D 번호 그대로 씁니다 (modbus 는 보유 레지스터 번호 = D 번호 + --dbase).

주의: 진짜 PLC의 입력(P000x / X)은 실제 단자 상태로 매 스캔 덮어쓰이므로, 기본값은 센서 값을
M 영역에 씁니다. 래더에서 입력 접점을 그 M 주소로 바꾸거나(--x 로 주소 지정), 입력 모듈이
없는 시뮬레이터라면 --x 로 입력 주소를 직접 지정하세요.
"""
import argparse, asyncio, base64, hashlib, json, socket, struct, sys, time

NX, NY, NYI = 16, 10, 16

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

class SimInd:
    """--plc sim --ind: 가짜 PLC. 실고 산업기사 정답 래더(스텝 방식)를 그대로 흉내 냄"""
    def __init__(s): s.m0 = 0; s.st = [0] * 6; s.t0 = None; s.x12 = 0; s.d224 = 0; s.d226 = 0
    def cycle_ind(s, X, W, R):
        (x0, x1, x2, _, _, x5, x6, h_run, h_end, h_rst, h_one, x11, x12, x13, _, _) = X
        v, r = W.get("110", [0] * 8), W.get("200", [0] * 16); st = s.st
        ready = r[2] == 1 and r[5] == 1
        if h_run and x2 and x11 and ready: s.m0 = 1
        if s.m0 and x2 and x0 and not st[0]: st[0] = 1
        if st[0] and x1: st[1] = 1
        if st[1] and x0: st[2] = 1
        if st[2] and x5: st[3] = 1
        if st[3] and x6: st[4] = 1
        if st[4] and x13:
            s.t0 = s.t0 or time.time()
        else: s.t0 = None
        if st[4] and s.t0 and time.time() - s.t0 >= 0.3: st[5] = 1
        if v[0] == 1: s.d224 = max(0, v[7] - 48)
        if st[5]: s.d226 = 1
        if x12 and not s.x12: s.st = st = [0] * 6
        if x12: s.d224 = 0; s.d226 = 0
        s.x12 = x12
        if not x2 and not st[0]: s.m0 = 0
        if h_end or not x11: s.st = st = [0] * 6; s.m0 = 0
        Y = [0] * NYI
        Y[0] = int(st[0] and not st[1]); Y[3] = int((st[2] and not st[3]) or (st[4] and not st[5])); Y[10] = int(st[3] and not st[4])
        Y[8] = st[5]; Y[7] = int(st[5] and ready); Y[4] = s.m0; Y[5] = int(not s.m0); Y[6] = int(not x11)
        return Y, [s.d224, 0, s.d226][:R[1]]

class Sim(Plc):
    """PLC 없이 시험: PB1(SW1)을 누르면 컨베이어, 매거진에 공작물이 있으면 녹색등, 비상정지면 적색 점멸"""
    def __init__(s, a): super().__init__(a); s.ind = SimInd()
    def connect(s): pass
    def cycle_ind(s, X, W, R): return s.ind.cycle_ind(X, W, R)
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
    def cycle_ind(s, X, W, R):
        v = sum(b << i for i, b in enumerate(X))
        s.req(struct.pack(">BHHB", 15, s.xa, NX, 2) + struct.pack("<H", v))
        for a0, vals in W.items():      # 보유 레지스터 여러 개 쓰기 (FC16)
            s.req(struct.pack(">BHHB", 16, s.a.dbase + int(a0), len(vals), len(vals) * 2) + b"".join(struct.pack(">H", x & 0xFFFF) for x in vals))
        r = s.req(struct.pack(">BHH", 1, s.ya, NYI)); v = int.from_bytes(r[2:2 + r[1]], "little")
        d = s.req(struct.pack(">BHH", 3, s.a.dbase + R[0], R[1]))      # 보유 레지스터 읽기 (FC3)
        return [(v >> i) & 1 for i in range(NYI)], [struct.unpack(">H", d[2 + i * 2:4 + i * 2])[0] for i in range(R[1])]

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
    def cycle_ind(s, X, W, R):
        v = sum(b << i for i, b in enumerate(X)); n = s.xv.encode()
        s.xfer(struct.pack("<HHHH", 0x58, 2, 0, 1) + struct.pack("<H", len(n)) + n + struct.pack("<HH", 2, v))
        for a0, vals in W.items():      # 연속 쓰기 (데이터 타입 0x14, 바이트 주소 %DB = D 번호 × 2)
            n = ("%%DB%d" % (int(a0) * 2)).encode(); data = b"".join(struct.pack("<H", x & 0xFFFF) for x in vals)
            s.xfer(struct.pack("<HHHH", 0x58, 0x14, 0, 1) + struct.pack("<H", len(n)) + n + struct.pack("<H", len(data)) + data)
        n = s.yv.encode(); d = s.xfer(struct.pack("<HHHH", 0x54, 2, 0, 1) + struct.pack("<H", len(n)) + n)
        v = struct.unpack("<H", d[12:14])[0]
        n = ("%%DB%d" % (R[0] * 2)).encode(); d = s.xfer(struct.pack("<HHHH", 0x54, 0x14, 0, 1) + struct.pack("<H", len(n)) + n + struct.pack("<H", R[1] * 2))
        return [(v >> i) & 1 for i in range(NYI)], [struct.unpack("<H", d[12 + i * 2:14 + i * 2])[0] for i in range(R[1])]

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
    def cycle_ind(s, X, W, R):
        s.xfer(0x1401, 1, s.x, NX, bytes((X[i] << 4) | X[i + 1] for i in range(0, NX, 2)))
        for a0, vals in W.items():      # D 워드 일괄 쓰기
            s.xfer(0x1401, 0, (0xA8, int(a0)), len(vals), b"".join(struct.pack("<H", x & 0xFFFF) for x in vals))
        d = s.xfer(0x0401, 1, s.y, NYI); y = [(d[i // 2] >> (4 if i % 2 == 0 else 0)) & 1 for i in range(NYI)]
        w = s.xfer(0x0401, 0, (0xA8, R[0]), R[1])
        return y, [struct.unpack("<H", w[i * 2:i * 2 + 2])[0] for i in range(R[1])]

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
                        if msg.get("ind"):
                            if not a.ind: ws_send(w, json.dumps({"error": "ind"})); await w.drain(); print("실고가 산업기사 모드예요. 브리지를 --ind 로 다시 켜 주세요."); await asyncio.sleep(1); continue
                            W = {str(k): [int(x) & 0xFFFF for x in v][:32] for k, v in (msg.get("W") or {}).items()}; R = [int(x) for x in (msg.get("R") or [224, 3])][:2]
                            Y, RV = await asyncio.get_running_loop().run_in_executor(None, plc.cycle_ind, X, W, R)
                            ws_send(w, json.dumps({"Y": Y, "R": RV})); await w.drain(); continue
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
    p.add_argument("--ind", action="store_true", help="스마트공장산업기사: 비전·로봇 숫자(D110~D117, D200~D215, D224~D226)도 주고받음")
    p.add_argument("--dbase", type=int, default=0, help="modbus: 보유 레지스터 번호 = D 번호 + 이 값")
    p.add_argument("--ws-port", type=int, default=8765); p.add_argument("--listen", default="127.0.0.1", help="기본은 이 PC에서만 접속 허용")
    p.add_argument("--origin", action="append", default=["https://silgo-go.github.io", "null", "http://localhost", "http://127.0.0.1"], help="접속을 허용할 웹 주소 (추가)")
    a = p.parse_args()
    if a.plc != "sim" and not a.host: p.error("--host 에 PLC IP 주소를 넣어 주세요")
    a.port = a.port or DRIVERS[a.plc][1]
    try: asyncio.run(serve(a))
    except KeyboardInterrupt: print("\n끝")

if __name__ == "__main__": main()
