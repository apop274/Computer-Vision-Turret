import math, os, threading, time
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"     # hide the spammy DSHOW warnings
import cv2, serial
from ultralytics import YOLO

# ---------------- settings ----------------
PORT = "COM3"
CAMERA_NAME = "Arducam"   # finds the camera by NAME, even if Windows renumbers it
CAMERA = 1                # only used if pygrabber isn't installed
CAM_W, CAM_H = 640, 480
UPSIDE_DOWN = True
CCW_SIGN = -1             # your motor: -1. The program asks at startup anyway.
START_CCW = 30.0          # degrees CCW to turn after startup (your "center")
HFOV = 95.0               # camera horizontal field of view (degrees)
KP = 3.0                  # speed per degree of error (deg/s per deg). Wobbles -> 2. Lazy -> 4
MAX_SPEED = 86.0          # deg/s - top speed (main.cpp MAX_VEL is the hard cap)
DEADBAND = 2.0            # degrees - this close to center = stop
SMOOTH = 0.5              # 0-1, how much to smooth YOLO's jitter (higher = smoother but laggier)
PAN_CCW_MAX = 120.0       # furthest CCW (degrees from power-up)
PAN_CW_MAX = 45.0         # furthest CW - camera cable gets tugged past here
# ------------------------------------------
# pan = degrees CCW from where the head was at power-up.
# You walk to YOUR RIGHT -> you show up on the LEFT of the screen -> head turns CCW.
# You walk to YOUR LEFT  -> you show up on the RIGHT of the screen -> head turns CW.

ser = serial.Serial(PORT, 115200, timeout=0.1)
print("Waiting for ESP32...")
deadline = time.time() + 30
while time.time() < deadline:
    line = ser.readline().decode(errors="ignore").strip()
    if line and not line.startswith("S "):
        print("  ESP32:", line)
    if line == "Ready":
        break
else:
    raise SystemExit("ESP32 never said Ready - 12 V plugged in? firmware uploaded? serial monitor closed?")

status = {"pan": 0.0, "pp": 0.0}   # pan = REAL commanded position (deg CCW) from the ESP32
rx = ""


def send(cmd):
    ser.write((cmd + "\n").encode())


def pan_speed(ccw_deg_s):          # + = turn CCW
    send(f"V{math.radians(CCW_SIGN * ccw_deg_s):.4f}")


def pan_goto(ccw_deg):
    send(f"X{math.radians(CCW_SIGN * ccw_deg):.4f}")


def read_status():
    global rx
    if ser.in_waiting:
        rx += ser.read(ser.in_waiting).decode(errors="ignore")
        *lines, rx = rx.split("\n")
        for l in lines:
            p = l.strip().split()
            if len(p) == 3 and p[0] == "S":
                try:
                    status["pan"] = CCW_SIGN * math.degrees(float(p[1]))
                    status["pp"] = float(p[2])
                except ValueError:
                    pass


# ---- direction check: tell it once which way is CCW ----
while True:
    pan_goto(20)
    time.sleep(1.2)
    ans = input("Did the head just turn CCW? y/n: ").strip().lower()
    pan_goto(0)
    time.sleep(1.2)
    if ans == "y":
        break
    CCW_SIGN = -CCW_SIGN
    print("Flipped direction - trying again...")
print(f"Direction locked in. (Put CCW_SIGN = {CCW_SIGN} in the settings to skip the question next time.)")
pan_goto(START_CCW)
time.sleep(1.0)
ser.reset_input_buffer()


# ---------------- camera (reconnects by name if it drops) ----------------
def find_camera():
    try:
        from pygrabber.dshow_graph import FilterGraph
        for i, name in enumerate(FilterGraph().get_input_devices()):
            if CAMERA_NAME.lower() in name.lower():
                return i
        return None
    except ImportError:
        return CAMERA


def open_camera(index):
    c = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    return c


latest = [None, 0.0]
drops = [0]


def grab():
    try:
        import comtypes
        comtypes.CoInitialize()
    except Exception:
        pass
    cap, last_good, waiting_msg = None, time.time(), False
    while True:
        if cap is None:
            idx = find_camera()
            if idx is None:
                if not waiting_msg:
                    print("  Arducam is OFF the USB bus - waiting for it to come back...")
                    waiting_msg = True
                time.sleep(1.0)
                continue
            cap = open_camera(idx)
            if not cap.isOpened():
                cap.release()
                cap = None
                time.sleep(1.0)
                continue
            if waiting_msg or drops[0]:
                print(f"  camera back (as camera {idx})")
            waiting_msg = False
            last_good = time.time()
        ok, f = cap.read()
        if ok and f is not None and f.mean() > 12:
            latest[0], latest[1] = f, time.time()
            last_good = time.time()
        elif time.time() - last_good > 1.5:
            drops[0] += 1
            print(f"  camera dropout #{drops[0]} at {time.strftime('%H:%M:%S')} - reconnecting...")
            cap.release()
            cap = None
            time.sleep(1.0)


threading.Thread(target=grab, daemon=True).start()

model = YOLO("yolo11n.pt")
cv2.namedWindow("tracker", cv2.WINDOW_NORMAL)
cv2.resizeWindow("tracker", 960, 720)
print("Tracking. CLICK THE VIDEO WINDOW, then: a = CCW, d = CW, c start position, q quit")

err_s = None
fps, t_last = 0.0, time.time()
seen = False

while True:
    read_status()
    frame = latest[0]
    if frame is None or time.time() - latest[1] > 0.5:      # no picture / frozen -> stand still
        pan_speed(0)
        if frame is not None:
            stale = frame.copy()
            cv2.putText(stale, "CAMERA LOST - holding still", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imshow("tracker", stale)
        if cv2.waitKey(30) & 0xFF == ord("q"):
            break
        continue

    frame = cv2.rotate(frame, cv2.ROTATE_180) if UPSIDE_DOWN else frame.copy()
    h, w = frame.shape[:2]
    boxes = model(frame, imgsz=416, classes=[0], conf=0.45, verbose=False)[0].boxes.xyxy.cpu().numpy()
    fps = 0.9 * fps + 0.1 / max(time.time() - t_last, 1e-3)
    t_last = time.time()

    v = 0.0
    if len(boxes):
        if not seen:
            print("Person found - following")
            seen = True
        x1, y1, x2, y2 = max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))  # biggest person
        cx = (x1 + x2) / 2
        # degrees from center: + = RIGHT of green line (YOUR left) -> turn CW
        #                      - = LEFT of green line (YOUR right) -> turn CCW
        err = math.degrees(math.atan((cx - w / 2) / (w / 2) * math.tan(math.radians(HFOV / 2))))
        err_s = err if err_s is None else SMOOTH * err_s + (1 - SMOOTH) * err

        # NUDGE every frame: speed proportional to how far off you are
        if abs(err_s) > DEADBAND:
            v = max(-MAX_SPEED, min(MAX_SPEED, -KP * err_s))     # + = CCW
        if status["pan"] >= PAN_CCW_MAX and v > 0:
            v = 0.0
        if status["pan"] <= -PAN_CW_MAX and v < 0:
            v = 0.0

        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 165, 255), 2)
        cv2.line(frame, (int(cx), 0), (int(cx), h), (0, 0, 255), 2)
        txt = f"TRACKING  off {err_s:+.0f} deg  speed {v:+.0f} deg/s"
    else:
        err_s = None
        if seen:
            print("Person lost - holding")
            seen = False
        txt = "no person - holding"
    pan_speed(v)

    cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 255, 0), 2)
    cv2.putText(frame, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"power (est): pan {status['pp']:.1f} W  drops {drops[0]}",
                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    cv2.putText(frame, f"pan {status['pan']:+.0f} (+ = CCW)  {fps:.0f} fps", (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.imshow("tracker", frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("c"):
        pan_goto(START_CCW)
    elif key == ord("a"):
        pan_goto(min(PAN_CCW_MAX, status["pan"] + 10))
    elif key == ord("d"):
        pan_goto(max(-PAN_CW_MAX, status["pan"] - 10))

pan_speed(0)
pan_goto(START_CCW)
time.sleep(2.0)
ser.close()
cv2.destroyAllWindows()