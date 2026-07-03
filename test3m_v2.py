"""Drive 3m with dynamic heading (GPS displacement-based)."""
import math, time
from collections import deque
from websockets.sync.client import connect

URI = "ws://192.168.31.222:81/ws"
PCT = 30
GAIN = 2.5

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

    # Wait FIX, get origin + initial heading from GPS
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
    # SET heading from imuYaw (initial)
    target_head = P0["imuYaw"]
    ws.send(f"SET_HEADING,{target_head:.1f}")
    time.sleep(0.3)
    try: print("[cal]", ws.recv(timeout=0.5).strip()[:120])
    except: pass

    origin = P0
    # History of positions for heading computation
    history = deque(maxlen=15)  # last 15 samples (~1-2 sec)
    last_t = time.monotonic()
    last_log = 0.0
    sent = 0
    last_head_from_gps = None
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

        history.append((cur["lat"], cur["lon"], cur["spd"], cur["head"], cur["imuYaw"], time.monotonic()))

        # Compute heading from GPS displacement (last 1s of samples)
        gps_head = None
        if len(history) >= 5:
            old = None
            for h in history:
                if (time.monotonic() - h[5]) > 1.0:
                    old = h; break
            if old:
                dx, dy = haversine(old[0], old[1], cur["lat"], cur["lon"])
                if math.hypot(dx, dy) > 0.1:  # moved > 10cm
                    gps_head = (math.degrees(math.atan2(dx, dy)) + 360) % 360

        # Use gps_head as truth when available, else fall back to imuYaw
        truth_head = gps_head if gps_head is not None else cur["imuYaw"]
        # If very recent gps_head, slowly update target_head to it (rate-limit to ±5°/s)
        if gps_head is not None and last_head_from_gps is not None:
            dT = 1.0  # per second
            dH = ((gps_head - last_head_from_gps + 540) % 360) - 180
            dH = max(-5*dT, min(5*dT, dH))
            target_head = (target_head + dH) % 360
            last_head_from_gps = target_head
        elif gps_head is not None:
            target_head = gps_head
            last_head_from_gps = target_head

        dH = ((target_head - cur["imuYaw"] + 540) % 360) - 180
        # use imuYaw for current heading (smoother than head)
        corr = max(-30, min(30, dH * GAIN))
        L = int(PCT + corr); R = int(PCT - corr)
        L = max(-70, min(70, L)); R = max(-70, min(70, R))
        ws.send(f"M,{L},{R}")
        sent += 1
        dx, dy = haversine(origin["lat"], origin["lon"], cur["lat"], cur["lon"])
        d = math.hypot(dx, dy)
        if time.monotonic() - last_t - last_log > 0.3:
            gp = f"gpsH={gps_head:.0f}" if gps_head is not None else "gpsH=-"
            print(f"  d={d:.2f} truth={truth_head:.1f} imu={cur['imuYaw']:.1f} dH={dH:+.1f} L={L} R={R} spd={cur['spd']:.3f} [{gp}]")
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
