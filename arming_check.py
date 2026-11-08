"""Read or change the drone's ARMING_CHECK over the Solo's Wi-Fi (MAVLink v1, which QGroundControl 5 can't talk).

  python3 arming_check.py            # show the current value (and save it to arming_check_before.txt)
  python3 arming_check.py no-compass # every pre-flight check except the compass
  python3 arming_check.py restore    # put back the value saved before

Quit QGroundControl first: both want UDP port 14550.
"""
import os, sys, time
from pymavlink import mavutil

SAVED = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arming_check_before.txt")
# ArduCopter bits: 1 all, 2 baro, 4 compass, 8 GPS, 16 INS, 32 params, 64 RC, 128 voltage, 256 battery,
# 512 airspeed, 1024 logging, 2048 safety switch, 4096 GPS config. "No compass" = every check bit but 1 and 4.
# Older firmware (the Solo's ArduCopter 3.x) only knows bits up to 64 and turns anything else into 127 (all
# checks), so use only those: 2 baro + 8 GPS + 16 INS + 32 params + 64 RC = 122 (no "all", no compass).
NO_COMPASS = 2 + 8 + 16 + 32 + 64   # 122

m = mavutil.mavlink_connection("udpin:0.0.0.0:14550", source_system=255, source_component=190)
print("waiting for the drone…")
if not m.wait_heartbeat(timeout=20):
    sys.exit("No drone heard on UDP 14550. Is the Mac on SoloLink and QGroundControl closed?")


def read():
    m.mav.param_request_read_send(m.target_system, m.target_component, b"ARMING_CHECK", -1)
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
    while msg and msg.param_id.strip("\x00") != "ARMING_CHECK":
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
    return int(msg.param_value) if msg else None


def write(v):
    m.mav.param_set_send(m.target_system, m.target_component, b"ARMING_CHECK", float(v),
                         mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(1)
    return read()


now = read()
if now is None:
    sys.exit("The drone didn't answer the parameter request.")
print("ARMING_CHECK is", now)
cmd = sys.argv[1] if len(sys.argv) > 1 else ""
if cmd == "no-compass":
    if not os.path.exists(SAVED):
        open(SAVED, "w").write(str(now))
        print("saved the old value to", SAVED)
    got = write(NO_COMPASS)
    if got is not None and (got & 1 or got & 4):
        print("now", got, "-> the drone still has the compass check ON (it changed the value).")
    else:
        print("now", got, "(compass check off; restart the drone)")
elif cmd == "restore":
    old = int(open(SAVED).read())
    print("restored to", write(old))
elif not os.path.exists(SAVED):
    open(SAVED, "w").write(str(now))
