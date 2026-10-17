import time, socket, argparse
import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks.python import vision

from gesture import (ensure_model, make_landmarker_options, extract_features,
                     classify_gesture, GESTURE_TO_TOOL, dist, draw_skeleton,
                     CAMERA_BACKEND)

CAMERA_INDEX = 1
UDP_HOST, UDP_PORT = "127.0.0.1", 5005
DRY_RUN = True                    # default: print only. Use  --live  to really send (no need to edit this line)

HOLD_SEC = 0.6                    # seconds a gesture must be held
GRACE_SEC = 0.15                  # a bad frame shorter than this does not reset the hold
COOLDOWN_SEC = 3.0                # minimum seconds between two sends
REARM_SEC = 1.0                   # seconds with no tool gesture needed to re-arm after a send
REQUIRE_STILL = True              # True: a moving hand does not count
MOTION_MAX_PER_SEC = 0.8          # palm-lengths per second; faster than this counts as moving

ARMED, LOCKED = "ARMED", "LOCKED"
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_udp(text):
    if DRY_RUN:
        print(f"[dry run] would send: {text!r}")
        return
    sock.sendto(text.encode(), (UDP_HOST, UDP_PORT))
    print(f"sent: {text!r}")

class Hold:
    def __init__(self, grace):
        self.grace = grace
        self.reset()

    def reset(self):
        self.label, self.start, self.last_good = None, 0.0, 0.0

    def update(self, label, now, need):
        if label is not None and label == self.label:
            self.last_good = now
        elif label is not None:
            self.label, self.start, self.last_good = label, now, now
        elif self.label is not None and now - self.last_good > self.grace:
            self.reset()
        if self.label is None:
            return 0.0, False
        held = now - self.start
        return min(1.0, held / need), held >= need

def draw_ui(frame, state, status, progress, banner):
    h, w = frame.shape[:2]
    x0, y0, bar_w = w // 2 - 150, 30, 300
    cv2.rectangle(frame, (x0, y0), (x0 + bar_w, y0 + 12), (200, 200, 200), 1)
    if progress > 0:
        cv2.rectangle(frame, (x0, y0), (x0 + int(bar_w * progress), y0 + 12), (0, 255, 0), -1)
    cv2.putText(frame, state, (20, h - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, status, (20, h - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    if banner:
        cv2.putText(frame, banner, (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)

def run(camera_index=CAMERA_INDEX):
    ensure_model()
    cap = cv2.VideoCapture(camera_index, CAMERA_BACKEND)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera {camera_index}. Run find_camera.py to see which indexes work.")
    print(f"Using camera {camera_index}. Sending to {UDP_HOST}:{UDP_PORT} (DRY_RUN={DRY_RUN}). Tools: {GESTURE_TO_TOOL}")
    print("Keys: q/Esc quit, d debug, r send 'reset'")

    state = ARMED
    hold = Hold(GRACE_SEC)
    prev_P, prev_t = None, None
    now0 = time.monotonic()
    last_send_t = now0 - COOLDOWN_SEC
    no_gesture_since = None
    last_ts = -1
    banner, banner_until = "", 0.0
    debug = False

    def fire(gesture, now):
        nonlocal state, last_send_t, banner, banner_until, no_gesture_since
        tool = GESTURE_TO_TOOL[gesture]
        send_udp(tool)
        state, last_send_t, no_gesture_since = LOCKED, now, None
        banner, banner_until = f"SENT: {tool}", now + 1.5
        hold.reset()

    try:
        with vision.HandLandmarker.create_from_options(make_landmarker_options()) as landmarker:
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                frame = cv2.flip(frame, 1)
                h, w = frame.shape[:2]
                now = time.monotonic()

                ts = int(now * 1000)
                if ts <= last_ts:
                    ts = last_ts + 1
                last_ts = ts
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                result = landmarker.detect_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)

                progress, status, gesture, speed = 0.0, "", None, 0.0
                can_fire = (now - last_send_t) >= COOLDOWN_SEC

                if result.hand_landmarks:
                    W = np.array([[p.x, p.y, p.z] for p in result.hand_world_landmarks[0]])
                    P = np.array([[p.x * w, p.y * h] for p in result.hand_landmarks[0]])
                    draw_skeleton(frame, [(int(x), int(y)) for x, y in P])

                    F = extract_features(W)
                    gesture = classify_gesture(F)

                    palm_px = dist(P, 0, 9) + 1e-9
                    if prev_P is not None and now > prev_t:
                        speed = float(np.mean(np.linalg.norm(P - prev_P, axis=1))) / palm_px / (now - prev_t)
                    else:
                        speed = float("inf")
                    prev_P, prev_t = P, now
                    still = (speed < MOTION_MAX_PER_SEC) or not REQUIRE_STILL

                    if state == ARMED:
                        if not can_fire:
                            hold.reset()
                            status = "Cooldown..."
                        else:
                            cand = gesture if (gesture in GESTURE_TO_TOOL and still) else None
                            progress, done = hold.update(cand, now, HOLD_SEC)
                            if done:
                                fire(cand, now)
                            elif cand is not None:
                                status = f"Hold: {GESTURE_TO_TOOL[cand]}"
                            else:
                                status = "Show a tool gesture"
                else:
                    prev_P, prev_t = None, None
                    hold.update(None, now, HOLD_SEC)
                    status = "Show your hand"

                if state == LOCKED:
                    hold.reset()
                    status = "Sent. Relax your hand to re-arm"
                    if gesture in GESTURE_TO_TOOL:
                        no_gesture_since = None
                    else:
                        if no_gesture_since is None:
                            no_gesture_since = now
                        if now - no_gesture_since >= REARM_SEC and can_fire:
                            state = ARMED

                if now > banner_until:
                    banner = ""
                draw_ui(frame, state, status, progress, banner)
                if debug:
                    cv2.putText(frame, f"gesture: {gesture}  speed: {speed:.2f}", (20, 130),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1, cv2.LINE_AA)

                cv2.imshow("Gesture command", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("d"):
                    debug = not debug
                if key == ord("r"):
                    send_udp("reset")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send tool commands to the sim from hand gestures")
    parser.add_argument("--camera", type=int, default=CAMERA_INDEX, help="camera index (see find_camera.py)")
    parser.add_argument("--host", default=UDP_HOST, help="IP of the computer running the sim")
    parser.add_argument("--port", type=int, default=UDP_PORT, help="port the sim listens on")
    parser.add_argument("--live", action="store_true", help="really send commands (default: dry run, print only)")
    args = parser.parse_args()
    UDP_HOST, UDP_PORT = args.host, args.port
    DRY_RUN = not args.live
    run(args.camera)