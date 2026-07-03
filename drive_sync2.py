import math, time
from websockets.sync.client import connect

URI = "ws://192.168.31.222:81/ws"
PCT_GO = 50
TURN = 30

def haversine(lat1, lon1, lat2, lon2):
    R = 6378137.0
    phi1 = math.radians(lat1)
    dx = math.radians(lon2 - lon1) * math.cos(phi1) * R
    dy = math.radians(lat2 - lat1) * R
    return dx, dy

def parse_tel(line):
    if not line.startswith("TEL,"):
        return None
    p = line.split(",")
    if len(p) < 12:
        return None
    try:
        car = p[6].strip()
        car_sol = 0
        if car == "fixed": car_sol = 2
        elif car == "float": car_sol = 1
        else: car_sol = 0
        # hAcc is in millimeters field, p[9]=numSv (uint), p[10]=hAcc_mm (uint? no: ints but lroundf)
        # Actually format p[9]=numSv, p[10]=(int)(hAcc*1000), so hAcc_m = p[10]/1000
        # BUT! Count: lat(1),lon(2),alt(3),head(4),fixType(5),carrier(6),diff(7),numSv(8),hAcc(9),vAcc(10),speed(11)
        # So hAcc = float(p[9]) / 1000.0
        return {
            "lat": float(p[1]), "lon": float(p[2]), "alt": float(p[3]),
            "head": float(p[4]), "fixType": int(p[5]), "carSol": car_sol,
            "diff": int(p[7]), "numSv": int(p[8]),
            "hacc_m": float(p[9]) / 1000.0, "spd": float(p[11])
        }
    except (ValueError, IndexError):
        return None

def wait_fix(ws, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            line = ws.recv(timeout=1.0)
        except Exception:
            continue
        if not line: continue
        for ln in line.splitlines():
            tel = parse_tel(ln.strip())
            if tel and tel["carSol"] >= 2 and tel["hacc_m"] < 0.05:
                return tel
    return None

def drive_straight(ws, start, target_dist, pct, label, max_sec=30, ct_max=0.5):
    print(f"[{label}] pct={pct} target={target_dist:.1f}m")
    sent = 0; last_log = 0.0; t0 = time.monotonic()
    while time.monotonic() - t0 < max_sec:
        try: ws.send(f"M,{pct},{pct}")
        except: break
        sent += 1
        time.sleep(0.03)
        try: line = ws.recv(timeout=0.05)
        except: continue
        if not line: continue
        for ln in line.splitlines():
            tel = parse_tel(ln.strip())
            if tel and tel["carSol"] >= 2 and tel["hacc_m"] < 0.05:
                dx, dy = haversine(start["lat"], start["lon"], tel["lat"], tel["lon"])
                d = math.hypot(dx, dy)
                if time.monotonic() - t0 - last_log > 0.4:
                    print(f"  [{label}] d={d:.3f} dx={dx:+.3f} dy={dy:+.3f} spd={tel['spd']:.3f}")
                    last_log = time.monotonic() - t0
                if d >= target_dist:
                    print(f"[{label}] ARRIVED d={d:.3f}")
                    try: ws.send("STOP"); time.sleep(0.2); ws.send("STOP")
                    except: pass
                    return tel
                if ct_max and abs(dx) > ct_max:
                    print(f"[{label}] CROSS-TRACK dx={dx:.2f}")
                    try: ws.send("STOP"); time.sleep(0.2); ws.send("STOP")
                    except: pass
                    return tel
    try: ws.send("STOP")
    except: pass
    return None

def turn_right(ws, start_head, target_dh=90, turn=30, label="P2", max_sec=15):
    print(f"[{label}] head={start_head:.1f} target=+{target_dh}")
    t0 = time.monotonic(); last_log = 0.0
    while time.monotonic() - t0 < max_sec:
        try: ws.send(f"M,{-abs(turn)},{abs(turn)}")
        except: break
        time.sleep(0.03)
        try: line = ws.recv(timeout=0.05)
        except: continue
        if not line: continue
        for ln in line.splitlines():
            tel = parse_tel(ln.strip())
            if tel and tel["carSol"] >= 2:
                dH = ((tel["head"] - start_head + 540) % 360) - 180
                if time.monotonic() - t0 - last_log > 0.3:
                    print(f"  [{label}] h={tel['head']:.1f} dH={dH:+.1f} spd={tel['spd']:.3f}")
                    last_log = time.monotonic() - t0
                if dH >= target_dh - 5 and dH <= target_dh + 5:
                    print(f"[{label}] TURNED dH={dH:.1f}")
                    try: ws.send("STOP"); time.sleep(0.2); ws.send("STOP")
                    except: pass
                    return tel["head"]
    try: ws.send("STOP")
    except: pass
    return None

def main():
    with connect(URI, open_timeout=5) as ws:
        ws.send("PING")
        time.sleep(0.5)
        # drain
        try:
            for _ in range(20):
                line = ws.recv(timeout=0.3)
                if not line: break
        except Exception:
            pass
        P0 = wait_fix(ws, 30.0)
        if not P0:
            print("[!] no FIXED P0"); return
        print(f"[+] P0 lat={P0['lat']:.7f} lon={P0['lon']:.7f} head={P0['head']:.1f}")

        P1 = drive_straight(ws, P0, 1.5, PCT_GO, "P1", max_sec=20)
        if P1:
            dx, dy = haversine(P0["lat"], P0["lon"], P1["lat"], P1["lon"])
            print(f"[=] P1: d={math.hypot(dx,dy):.3f} dx={dx:+.3f} dy={dy:+.3f}")
        time.sleep(0.5)
        head_for_turn = P1["head"] if P1 else P0["head"]
        new_head = turn_right(ws, head_for_turn, 90, TURN, "P2")
        if new_head:
            print(f"[=] P2: new head={new_head:.1f} dH={((new_head-head_for_turn+540)%360)-180:+.1f}")
        time.sleep(0.5)
        P2 = wait_fix(ws, 4.0)
        if not P2:
            lat0 = P1["lat"] if P1 else P0["lat"]
            lon0 = P1["lon"] if P1 else P0["lon"]
            P2 = {"lat": lat0, "lon": lon0, "head": new_head or head_for_turn, "spd": 0, "carSol": 2, "hacc_m": 0.014}
        P3 = drive_straight(ws, P2, 1.0, PCT_GO, "P3", max_sec=20)
        if P3:
            dx, dy = haversine(P2["lat"], P2["lon"], P3["lat"], P3["lon"])
            print(f"[=] P3: d={math.hypot(dx,dy):.3f} dx={dx:+.3f} dy={dy:+.3f}")
            dx2, dy2 = haversine(P0["lat"], P0["lon"], P3["lat"], P3["lon"])
            print(f"[TOTAL] P0->P3 d={math.hypot(dx2,dy2):.3f}m dx={dx2:+.3f} dy={dy2:+.3f}")

main()
