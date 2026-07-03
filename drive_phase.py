"""Drive phases with held WS connection.
Phase 1: 1.5m straight (M,PCT,PCT)
Phase 2: 90 deg right turn (M,-TURN,TURN)
Phase 3: 1m straight (M,PCT,PCT)
Stops on distance/heading threshold with cross-track guard.
"""
import asyncio, sys, math, time
from websockets.asyncio.client import connect

URI = "ws://192.168.31.222:81/ws"
PCT_GO = 50
TURN = 30

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6378137.0
    phi1 = math.radians(lat1)
    dx = math.radians(lon2 - lon1) * math.cos(phi1) * R
    dy = math.radians(lat2 - lat1) * R
    return dx, dy

async def get_fix(ws, timeout=5.0):
    """Wait for one TEL with FIXED sol and hAcc < 50mm."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            line = await asyncio.wait_for(ws.recv(), timeout=1.0)
        except asyncio.TimeoutError:
            continue
        for ln in line.splitlines():
            ln = ln.strip()
            if ln.startswith("TEL,"):
                p = ln.split(",")
                if len(p) > 11:
                    sol = int(p[5])
                    hacc = float(p[8])
                    if sol >= 2 and hacc < 0.05:
                        return {
                            "lat": float(p[1]),
                            "lon": float(p[2]),
                            "head": float(p[4]),
                            "spd": float(p[11]),
                        }
    return None

async def drive_phase(ws, start, target_dist, pct, label, max_sec=30.0, cross_track_max=None):
    print(f"[{label}] start lat={start['lat']:.7f} lon={start['lon']:.7f} head={start['head']:.1f}")
    t0 = time.monotonic()
    sent = 0
    last_print = 0.0
    while time.monotonic() - t0 < max_sec:
        try:
            await ws.send(f"M,{pct},{pct}" if pct >= 0 else f"M,{-pct},{pct}")
        except Exception as e:
            print(f"[{label}] send error: {e}"); break
        sent += 1
        await asyncio.sleep(0.03)
        try:
            line = await asyncio.wait_for(ws.recv(), timeout=0.02)
        except asyncio.TimeoutError:
            continue
        for ln in line.splitlines():
            ln = ln.strip()
            if ln.startswith("TEL,"):
                p = ln.split(",")
                if len(p) > 11:
                    sol = int(p[5])
                    hacc = float(p[8])
                    if sol >= 2 and hacc < 0.05:
                        cur = {
                            "lat": float(p[1]),
                            "lon": float(p[2]),
                            "head": float(p[4]),
                            "spd": float(p[11]),
                        }
                        dx, dy = haversine_m(start["lat"], start["lon"], cur["lat"], cur["lon"])
                        dist = math.hypot(dx, dy)
                        if time.monotonic() - t0 - last_print > 0.3:
                            print(f"  [{label}] t={time.monotonic()-t0:.1f}s sent={sent} d={dist:.3f} dx={dx:+.3f} dy={dy:+.3f} spd={cur['spd']:.3f} head={cur['head']:.1f}")
                            last_print = time.monotonic() - t0
                        if dist >= target_dist:
                            print(f"[{label}] ARRIVED d={dist:.3f}")
                            await ws.send("STOP")
                            await asyncio.sleep(0.3)
                            await ws.send("STOP")
                            return cur
                        if cross_track_max is not None and abs(dx) > cross_track_max:
                            print(f"[{label}] CROSS-TRACK dx={dx:.2f}")
                            await ws.send("STOP")
                            await asyncio.sleep(0.3)
                            await ws.send("STOP")
                            return cur
        await asyncio.sleep(0.0)
    await ws.send("STOP"); await asyncio.sleep(0.2); await ws.send("STOP")
    return None

async def turn_phase(ws, start_head, target_dh, turn_val, label, max_sec=20.0):
    print(f"[{label}] start head={start_head:.1f}, target dH=+{target_dh:.0f}")
    t0 = time.monotonic()
    sent = 0
    while time.monotonic() - t0 < max_sec:
        try:
            await ws.send(f"M,{-abs(turn_val)},{abs(turn_val)}")
        except Exception as e:
            print(f"[{label}] send error: {e}"); break
        sent += 1
        await asyncio.sleep(0.03)
        try:
            line = await asyncio.wait_for(ws.recv(), timeout=0.02)
        except asyncio.TimeoutError:
            continue
        for ln in line.splitlines():
            ln = ln.strip()
            if ln.startswith("TEL,"):
                p = ln.split(",")
                if len(p) > 11:
                    sol = int(p[5])
                    if sol >= 2:
                        head = float(p[4])
                        dH = ((head - start_head + 540) % 360) - 180
                        if (time.monotonic() - t0) > 0.5:
                            print(f"  [{label}] t={time.monotonic()-t0:.1f}s head={head:.1f} dH={dH:+.1f}")
                        if dH >= target_dh - 5 and dH <= target_dh + 5:
                            print(f"[{label}] TURNED dH={dH:.1f}")
                            await ws.send("STOP"); await asyncio.sleep(0.3); await ws.send("STOP")
                            return head
        await asyncio.sleep(0.0)
    await ws.send("STOP"); await asyncio.sleep(0.2); await ws.send("STOP")
    return None

async def main():
    async with connect(URI, ping_interval=None) as ws:
        await ws.send("PING")
        # Wait for first TEL
        for _ in range(30):
            line = await asyncio.wait_for(ws.recv(), timeout=1.0)
            for ln in line.splitlines():
                if ln.startswith("TEL,"):
                    print(ln[:120])
                    break
            break

        P0 = await get_fix(ws, 30.0)
        if not P0:
            print("[!] no FIXED P0"); return
        print(f"[+] P0 lat={P0['lat']:.7f} lon={P0['lon']:.7f} head={P0['head']:.1f}")

        P1 = await drive_phase(ws, P0, 1.5, PCT_GO, "P1", max_sec=20, cross_track_max=0.5)
        if not P1:
            print("[!] P1 no arrive"); return
        print(f"[=] P1: dx={(P1['lat']-P0['lat'])*111320:.3f} dy={(P1['lon']-P0['lon'])*111320*math.cos(math.radians(P0['lat'])):.3f}")

        P1H = P1['head']
        new_head = await turn_phase(ws, P1H, 90, TURN, "P2", max_sec=15)
        if not new_head:
            print("[!] P2 no turn"); return

        # wait for stabilized TEL
        await asyncio.sleep(0.5)
        P2 = await get_fix(ws, 4.0)
        if not P2:
            P2 = {"lat": P1['lat'], "lon": P1['lon'], "head": new_head, "spd": 0.0}

        P3 = await drive_phase(ws, P2, 1.0, PCT_GO, "P3", max_sec=20, cross_track_max=0.5)
        if not P3:
            print("[!] P3 no arrive"); return
        print(f"[=] P3: dx={(P3['lat']-P2['lat'])*111320:.3f} dy={(P3['lon']-P2['lon'])*111320*math.cos(math.radians(P2['lat'])):.3f}")

        dx, dy = haversine_m(P0['lat'], P0['lon'], P3['lat'], P3['lon'])
        print(f"[SUMMARY] P0->P3 total: d={math.hypot(dx,dy):.3f}m  dx={dx:+.3f} dy={dy:+.3f}")

asyncio.run(main())
