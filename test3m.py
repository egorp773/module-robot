"""Drive 3m straight, log heading every 100ms."""
import math, time
from websockets.sync.client import connect

URI = "ws://192.168.31.222:81/ws"
PCT = 35
GAIN = 2.0

def haversine(lat1, lon1, lat2, lon2):
    R = 6378137.0; phi1 = math.radians(lat1)
    dx = math.radians(lon2 - lon1) * math.cos(phi1) * R
    dy = math.radians(lat2 - lat1) * R
    return dx, dy

def parse_tel(line):
    if not line.startswith("TEL,"): return None
    p = line.split(",")
    if len(p) < 20: return None
    try:
        car = p[6].strip()
        return {
            "lat": float(p[1]), "lon": float(p[2]),
            "head": float(p[4]),
            "carSol": 2 if car == "fixed" else (1 if car == "float" else 0),
            "hacc_m": float(p[9]) / 1000.0,
            "spd": float(p[11]),
            "imuYaw": float(p[16]),
        }
    except: return None

with connect(URI, open_timeout=5) as ws:
    ws.send("PING"); time.sleep(0.5)
    try:
        for _ in range(15):
            line = ws.recv(timeout=0.3)
            if not line: break
    except: pass
    P0 = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < 30 and not P0:
        try: line = ws.recv(timeout=0.5)
        except: continue
        for ln in line.splitlines():
            tel = parse_tel(ln.strip())
            if tel and tel["carSol"] == 2 and tel["hacc_m"] < 0.05:
                P0 = tel; break
    if not P0:
        print("[!] no FIX"); exit(1)
    print(f"[P0] imu={P0['imuYaw']:.1f} head={P0['head']:.1f}")
    target_head = P0["head"]
    origin = P0

    # Sample heading for 0.5s before moving - is it stable?
    print("[*] sampling heading for 0.5s without moving:")
    t1 = time.monotonic()
    while time.monotonic() - t1 < 0.5:
        try: line = ws.recv(timeout=0.05)
        except: continue
        if not line: continue
        for ln in line.splitlines():
            tel = parse_tel(ln.strip())
            if tel and tel["carSol"] == 2:
                print(f"  h={tel['head']:.1f} imu={tel['imuYaw']:.1f}")

    # Drive
    last_t = time.monotonic()
    last_log = 0.0
    sent = 0
    heads_at_d = {}
    while time.monotonic() - last_t < 60:
        try: line = ws.recv(timeout=0.05)
        except: continue
        if not line: time.sleep(0.005); continue
        cur = None
        for ln in line.splitlines():
            tel = parse_tel(ln.strip())
            if tel and tel["carSol"] == 2 and tel["hacc_m"] < 0.05:
                cur = tel; break
        if cur is None: continue
        dH = ((target_head - cur["head"] + 540) % 360) - 180
        corr = max(-30, min(30, dH * GAIN))
        L = int(PCT + corr); R = int(PCT - corr)
        L = max(-70, min(70, L)); R = max(-70, min(70, R))
        ws.send(f"M,{L},{R}")
        sent += 1
        dx, dy = haversine(origin["lat"], origin["lon"], cur["lat"], cur["lon"])
        d = math.hypot(dx, dy)
        # log every 0.2m
        dkey = round(d * 5) / 5
        if dkey not in heads_at_d:
            heads_at_d[dkey] = (cur["head"], cur["imuYaw"], dH)
            print(f"  d={d:.2f} h={cur['head']:.1f} imu={cur['imuYaw']:.1f} dH={dH:+.1f} spd={cur['spd']:.3f}")
        if d >= 3.0:
            ws.send("STOP"); time.sleep(0.2); ws.send("STOP")
            print(f"[ARRIVED 3m]")
            break

    time.sleep(0.5)
    P1 = None
    t2 = time.monotonic()
    while time.monotonic() - t2 < 3 and not P1:
        try: line = ws.recv(timeout=0.5)
        except: continue
        for ln in line.splitlines():
            tel = parse_tel(ln.strip())
            if tel and tel["carSol"] == 2:
                P1 = tel; break
    if P1:
        dx, dy = haversine(origin["lat"], origin["lon"], P1["lat"], P1["lon"])
        print(f"[FINAL] d={math.hypot(dx,dy):.3f} dx={dx:+.3f} dy={dy:+.3f}")
        # heading jump analysis
        heads = [h for h, imu, dH in heads_at_d.values()]
        if heads:
            print(f"[HEADING] start={heads[0]:.1f} end={heads[-1]:.1f} min={min(heads):.1f} max={max(heads):.1f} range={max(heads)-min(heads):.1f}")
