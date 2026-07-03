"""Drive 3m: target_head from GPS when moving, else imuYaw."""
import math, time
from collections import deque
from websockets.sync.client import connect

URI = "ws://192.168.31.222:81/ws"
PCT = 30
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
    if not P0: print("[!] no FIX"); exit(1)

    # Set heading from imu (initial)
    target_head = P0["imuYaw"]
    ws.send(f"SET_HEADING,{target_head:.1f}")
    time.sleep(0.3)
    try: print("[cal]", ws.recv(timeout=0.5).strip()[:120])
    except: pass

    origin = P0
    history = deque(maxlen=30)
    last_t = time.monotonic()
    last_log = 0.0
    sent = 0

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

        history.append((cur["lat"], cur["lon"], cur["spd"], time.monotonic()))

        # Compute GPS heading from last 0.5s
        gps_head = None
        if len(history) >= 3:
            old = None
            for h in history:
                if (time.monotonic() - h[3]) > 0.5:
                    old = h; break
            if old:
                dx, dy = haversine(old[0], old[1], cur["lat"], cur["lon"])
                d_ = math.hypot(dx, dy)
                if d_ > 0.05:  # >5cm
                    gps_head = (math.degrees(math.atan2(dx, dy)) + 360) % 360

        # Trust gps_head when available
        cur_head_truth = gps_head if gps_head is not None else cur["imuYaw"]

        # slowly drift target_head towards truth (so corrections are smooth)
        if gps_head is not None:
            dT = ((gps_head - target_head + 540) % 360) - 180
            dT = max(-3, min(3, dT))
            target_head = (target_head + dT) % 360
        else:
            target_head = cur["imuYaw"]  # use imu when not moving

        dH = ((target_head - cur["imuYaw"] + 540) % 360) - 180
        corr = max(-30, min(30, dH * GAIN))
        L = int(PCT + corr); R = int(PCT - corr)
        L = max(-70, min(70, L)); R = max(-70, min(70, R))
        ws.send(f"M,{L},{R}")
        sent += 1
        dx, dy = haversine(origin["lat"], origin["lon"], cur["lat"], cur["lon"])
        d = math.hypot(dx, dy)
        if time.monotonic() - last_t - last_log > 0.3:
            gp = f"gpsH={gps_head:.0f}" if gps_head is not None else "gpsH=-"
            print(f"  d={d:.2f} tgt={target_head:.0f} imu={cur['imuYaw']:.1f} dH={dH:+.1f} L={L} R={R} spd={cur['spd']:.3f} [{gp}]")
            last_log = time.monotonic() - last_t
        if d >= 3.0:
            ws.send("STOP"); time.sleep(0.2); ws.send("STOP")
            print(f"[ARRIVED]")
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
