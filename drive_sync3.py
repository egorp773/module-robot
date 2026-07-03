"""Drive with heading feedback.
P1: 1.5m straight M,(base+st),(base-st)
P2: turn right 90deg with stop when dH=+90
P3: 1m straight
"""
import math, time
from websockets.sync.client import connect

URI = "ws://192.168.31.222:81/ws"
BASE = 45       # base speed
GAIN_HEAD = 1.5 # deg to %, so 10deg error => 15% diff
TURN = 35

def haversine(lat1, lon1, lat2, lon2):
    R = 6378137.0; phi1 = math.radians(lat1)
    dx = math.radians(lon2 - lon1) * math.cos(phi1) * R
    dy = math.radians(lat2 - lat1) * R
    return dx, dy

def parse_tel(line):
    if not line.startswith("TEL,"): return None
    p = line.split(",")
    if len(p) < 12: return None
    try:
        car = p[6].strip()
        car_sol = 2 if car == "fixed" else (1 if car == "float" else 0)
        return {
            "lat": float(p[1]), "lon": float(p[2]),
            "head": float(p[4]),
            "carSol": car_sol,
            "hacc_m": float(p[9]) / 1000.0,
            "spd": float(p[11])
        }
    except (ValueError, IndexError):
        return None

def wait_fix(ws, timeout=30):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try: line = ws.recv(timeout=1.0)
        except: continue
        if not line: continue
        for ln in line.splitlines():
            tel = parse_tel(ln.strip())
            if tel and tel["carSol"] >= 2 and tel["hacc_m"] < 0.05:
                return tel
    return None

def drive_to(ws, start, target_dist, target_head, label, max_sec=30, ct_max=0.4):
    print(f"[{label}] target d={target_dist}m, head={target_head:.1f}")
    sent = 0; last_log = 0.0; t0 = time.monotonic()
    while time.monotonic() - t0 < max_sec:
        # Read one TEL packet first to compute correction
        try: line = ws.recv(timeout=0.05)
        except: continue
        if not line:
            time.sleep(0.005); continue
        cur = None
        for ln in line.splitlines():
            tel = parse_tel(ln.strip())
            if tel and tel["carSol"] >= 2 and tel["hacc_m"] < 0.05:
                cur = tel; break
        if cur is None:
            time.sleep(0.005); continue

        dx, dy = haversine(start["lat"], start["lon"], cur["lat"], cur["lon"])
        d = math.hypot(dx, dy)
        # heading error (positive = need to turn right)
        dH = ((target_head - cur["head"] + 540) % 360) - 180
        # apply: left = base + GAIN*dH  (right turn->dH positive -> right faster)
        # To turn right: left faster. left > right when target_head > cur_head
        d_pct = max(-30, min(30, dH * GAIN_HEAD))
        # use base -/+ d_pct
        left = int(BASE + d_pct)
        right = int(BASE - d_pct)
        left = max(-70, min(70, left))
        right = max(-70, min(70, right))
        try: ws.send(f"M,{left},{right}")
        except: break
        sent += 1
        if time.monotonic() - t0 - last_log > 0.3:
            print(f"  [{label}] d={d:.3f} dx={dx:+.3f} dy={dy:+.3f} h={cur['head']:.1f} dH={dH:+.1f} L={left} R={right} spd={cur['spd']:.3f}")
            last_log = time.monotonic() - t0
        if d >= target_dist:
            print(f"[{label}] ARRIVED d={d:.3f}")
            try: ws.send("STOP"); time.sleep(0.2); ws.send("STOP")
            except: pass
            return cur
        if ct_max and abs(dx) > ct_max:
            print(f"[{label}] CROSS-TRACK dx={dx:.2f}")
            try: ws.send("STOP"); time.sleep(0.2); ws.send("STOP")
            except: pass
            return cur
    try: ws.send("STOP")
    except: pass
    return None

def turn_right_in_place(ws, start_head, target_dh=90, max_sec=20):
    print(f"[P2] TURN head={start_head:.1f} target=+{target_dh}")
    t0 = time.monotonic(); last_log = 0.0
    while time.monotonic() - t0 < max_sec:
        try: ws.send(f"M,{-TURN},{TURN}")
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
                    print(f"  [P2] h={tel['head']:.1f} dH={dH:+.1f} spd={tel['spd']:.3f}")
                    last_log = time.monotonic() - t0
                if dH >= target_dh - 8 and dH <= target_dh + 8:
                    try: ws.send("STOP"); time.sleep(0.2); ws.send("STOP")
                    except: pass
                    return tel["head"]
    try: ws.send("STOP")
    except: pass
    return None

def main():
    with connect(URI, open_timeout=5) as ws:
        ws.send("PING")
        time.sleep(0.3)
        try:
            for _ in range(20):
                line = ws.recv(timeout=0.3)
                if not line: break
        except: pass
        P0 = wait_fix(ws, 30.0)
        if not P0:
            print("[!] no FIXED P0"); return
        print(f"[+] P0 lat={P0['lat']:.7f} lon={P0['lon']:.7f} head={P0['head']:.1f}")

        # P1: 1.5m with heading control
        P1 = drive_to(ws, P0, 1.5, P0["head"], "P1", max_sec=25)
        if P1:
            dx, dy = haversine(P0["lat"], P0["lon"], P1["lat"], P1["lon"])
            print(f"[=] P1: d={math.hypot(dx,dy):.3f} dx={dx:+.3f} dy={dy:+.3f}")
            head1 = P1["head"]
        else:
            head1 = P0["head"]

        # P2: turn 90 right (in place)
        time.sleep(0.5)
        new_head = turn_right_in_place(ws, head1, target_dh=90, max_sec=20)
        if new_head is None:
            print("[!] P2 fail"); return
        time.sleep(0.5)
        P2 = wait_fix(ws, 4.0)
        if not P2:
            P2 = {"lat": P1["lat"] if P1 else P0["lat"],
                  "lon": P1["lon"] if P1 else P0["lon"],
                  "head": new_head, "spd": 0, "carSol": 2, "hacc_m": 0.014}

        # P3: 1m straight at new_head
        P3 = drive_to(ws, P2, 1.0, P2["head"], "P3", max_sec=25)
        if P3:
            dx, dy = haversine(P2["lat"], P2["lon"], P3["lat"], P3["lon"])
            print(f"[=] P3: d={math.hypot(dx,dy):.3f} dx={dx:+.3f} dy={dy:+.3f}")
            dx2, dy2 = haversine(P0["lat"], P0["lon"], P3["lat"], P3["lon"])
            print(f"[TOTAL] P0->P3 d={math.hypot(dx2,dy2):.3f}m dx={dx2:+.3f} dy={dy2:+.3f}")

main()
