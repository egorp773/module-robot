"""Smart drive: 1) read imuYaw from TEL, 2) SET_HEADING, 3) drive 1.5m."""
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

    # Wait for FIX + sample imuYaw
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

    # Set heading from imu
    target_head = P0["imuYaw"]
    ws.send(f"SET_HEADING,{target_head:.1f}")
    time.sleep(0.5)
    try: print("[cal]", ws.recv(timeout=0.5).strip()[:200])
    except: pass

    # Re-wait for FIX with new origin (should be same point, just new heading)
    time.sleep(0.5)
    P1 = None
    t1 = time.monotonic()
    while time.monotonic() - t1 < 5 and not P1:
        try: line = ws.recv(timeout=0.5)
        except: continue
        for ln in line.splitlines():
            tel = parse_tel(ln.strip())
            if tel and tel["carSol"] == 2 and tel["hacc_m"] < 0.05:
                P1 = tel; break
    if not P1:
        print("[!] no FIX after SET_HEADING"); exit(1)
    print(f"[after SET] head={P1['head']:.1f} imu={P1['imuYaw']:.1f}")
    target_head = P1["head"]
    origin = P1

    last_t = time.monotonic()
    last_log = 0.0
    sent = 0
    while time.monotonic() - last_t < 25:
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
        if time.monotonic() - last_t - last_log > 0.3:
            print(f"  d={d:.3f} dx={dx:+.3f} dy={dy:+.3f} h={cur['head']:.1f} dH={dH:+.1f} L={L} R={R} spd={cur['spd']:.3f}")
            last_log = time.monotonic() - last_t
        if d >= 1.5:
            ws.send("STOP"); time.sleep(0.2); ws.send("STOP")
            print(f"[ARRIVED] d={d:.3f}")
            break

    time.sleep(0.5)
    P2 = None
    t2 = time.monotonic()
    while time.monotonic() - t2 < 3 and not P2:
        try: line = ws.recv(timeout=0.5)
        except: continue
        for ln in line.splitlines():
            tel = parse_tel(ln.strip())
            if tel and tel["carSol"] == 2:
                P2 = tel; break
    if P2:
        dx, dy = haversine(origin["lat"], origin["lon"], P2["lat"], P2["lon"])
        print(f"[FINAL] d={math.hypot(dx,dy):.3f} dx={dx:+.3f} dy={dy:+.3f}")
