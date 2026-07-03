"""Calibrate IMU: get current BNO085 yaw from imuYaw, set via SET_HEADING."""
import time
from websockets.sync.client import connect

URI = "ws://192.168.31.222:81/ws"
with connect(URI, open_timeout=5) as ws:
    ws.send("PING")
    time.sleep(0.5)
    # drain
    try:
        for _ in range(15):
            line = ws.recv(timeout=0.3)
            if not line: break
    except: pass

    # Read 2s of TEL, average imuYaw field
    samples = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        try:
            line = ws.recv(timeout=0.2)
        except: continue
        if not line: continue
        for ln in line.splitlines():
            ln = ln.strip()
            if ln.startswith("TEL,"):
                p = ln.split(",")
                if len(p) > 17:
                    try:
                        # imuYawDeg is in field index... check format
                        # format: lat,lon,alt,headingFilt,fixType,carrier,diff,numSv,hAcc,vAcc,speed,pDop,pvtAge,rtcmBytes,rtcmAge,imuYaw,imuAge,imuFresh,rtcm2,gnssRtcmAge,rtcmSrc,msgCount,crcFail,lastType
                        # Let's find positions 0..23
                        imu_yaw = float(p[16]) if len(p) > 16 else None
                        head_filt = float(p[4])
                        spd = float(p[11])
                        if imu_yaw is not None:
                            samples.append(imu_yaw)
                            print(f"imuYaw={imu_yaw:.1f} headFilt={head_filt:.1f} spd={spd:.3f}")
                    except (ValueError, IndexError):
                        pass
    if samples:
        avg = sum(samples) / len(samples)
        print(f"[AVG IMU YAW] {avg:.1f}")
        # Send SET_HEADING
        ws.send(f"SET_HEADING,{avg:.1f}")
        time.sleep(0.3)
        line = ws.recv(timeout=0.5)
        print(f"[CAL response] {line.strip()[:200]}")
    else:
        print("[!] no imuYaw samples")
