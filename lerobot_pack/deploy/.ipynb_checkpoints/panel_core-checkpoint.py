"""Core of the SO-101 multi-workspace deployment: config, rail, arm stow, policy runner, state machine, inputs.

Everything hardware-specific sits behind small classes so the whole menu flow can be tested with mocks
(`mock: true` in workspaces.yaml). The notebook only builds widgets on top of `Controller`.
"""
from __future__ import annotations

import os
import queue
import re
import shlex
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Callable

import yaml

# ----------------------------------------------------------------------------------------------- config

def load_config(path: str | Path = "workspaces.yaml") -> dict:
    cfg = yaml.safe_load(open(path))
    cfg.setdefault("mock", False)
    return cfg


def parse_keywords(text: str, options: dict) -> str | None:
    """Return the option key if the sentence names EXACTLY ONE option of this workspace, else None.
    Keywords are scoped to the workspace's own options (so 'hammer' in WS2 never collides with WS3)."""
    t = " " + re.sub(r"[^a-z0-9' ]+", " ", text.lower()) + " "
    hits = {k for k, o in options.items()
            if any(re.search(rf"(?<![a-z0-9]){re.escape(w.lower())}(?![a-z0-9])", t) for w in o.get("keywords", []))}
    return hits.pop() if len(hits) == 1 else None


# ----------------------------------------------------------------------------------------------- rail

class RailError(RuntimeError):
    pass


class RailController:
    def connect(self): ...
    def home(self): ...
    def move_to(self, name: str): ...
    def position(self) -> float | None: return None
    def stop(self): ...
    def close(self): ...


class MockRail(RailController):
    def __init__(self, positions, delay=1.0):
        self.positions, self.delay, self.at = positions, delay, None

    def move_to(self, name):
        if name not in self.positions:
            raise RailError(f"unknown rail position {name}")
        time.sleep(self.delay)
        self.at = name


class SerialRail(RailController):
    """Rail over serial. Drivers:
       esp32: the firmware in hardware/rail_esp32 (MOVE/MOVEPCT/STOP/HOME protocol, replies ACK/DONE/FAULT). DEFAULT.
       grbl:  '$H' homes, 'G90 G1 X<pos> F<feed>' moves, '?' polls '<Idle|...>'. Units = mm.
    `unit: pct` (esp32) means positions are percent of the measured travel (0 = base switch, 100 = end switch);
    `unit: mm` means millimetres from the base switch. Positions are clamped by the firmware to stay 3 mm inside the switches."""

    def __init__(self, cfg, logf=print, serial_factory=None):
        self.cfg, self.log = cfg, logf
        self.positions = cfg["positions"]
        self.timeout = cfg.get("move_timeout_s", 60)
        self.driver = cfg.get("driver", "esp32")
        self.ser = None
        self._factory = serial_factory

    # -- low level ---------------------------------------------------------------------------------
    def connect(self):
        if self._factory:
            self.ser = self._factory()
        else:
            import serial  # pyserial
            self.ser = serial.Serial(self.cfg["port"], self.cfg.get("baud", 115200), timeout=0.2)
        if self.driver == "esp32":
            self._wait_boot(self.cfg.get("boot_timeout_s", 15))   # opening the port resets the ESP32; it then WAITS (UNHOMED)
        else:
            time.sleep(2.0)
            self.ser.reset_input_buffer()

    def _send(self, line: str):
        self.ser.write((line + "\n").encode())

    def _readline(self) -> str:
        return self.ser.readline().decode(errors="ignore").strip()

    # -- esp32 -------------------------------------------------------------------------------------
    def _status(self, timeout=3.0) -> dict:
        """Ask the firmware for its state. Returns {'state': 'UNHOMED|HOMING|READY|FAULT', 'pos': mm, ...}."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._send("STATUS?")
            t1 = time.time()
            while time.time() - t1 < 1.0:
                line = self._readline()
                if line.startswith("STATE"):
                    t = line.split()
                    d = {"state": t[1]}
                    d.update({t[i].lower(): t[i + 1] for i in range(2, len(t) - 1, 2)})
                    return d
                if line.startswith("FAULT"):
                    raise RailError(line)
                if line and not line.startswith(("PONG", "DBG")):
                    self.log("rail: " + line)
        raise RailError("rail: no answer to STATUS? (is the ESP32 powered, flashed with rail_esp32.ino, and the port correct?)")

    def _wait_boot(self, timeout):
        st = self._status(timeout)
        if st["state"] == "FAULT":
            raise RailError("rail firmware is in FAULT (send HOME after fixing the cause)")
        self.log(f"rail: connected, state {st['state']} (the rail only moves after HOME)")

    def _wait_homed(self, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            st = self._status()
            if st["state"] == "READY":
                self.log(f"rail: homed, travel {st.get('travel')} mm")
                return
            if st["state"] == "UNHOMED":
                raise RailError("rail: homing was stopped")
            time.sleep(1.0)
        self.stop()
        raise RailError(f"rail: homing not finished after {timeout}s")

    def _expect(self, prefix: str, timeout: float) -> str:
        t0 = time.time()
        while time.time() - t0 < timeout:
            line = self._readline()
            if not line:
                continue
            if line.startswith(prefix):
                return line
            if line.startswith(("FAULT", "ERR", "STOPPED")):
                raise RailError(line)
        raise RailError(f"rail: no '{prefix}' within {timeout}s")

    def home(self):
        if self.driver == "esp32":
            self.ser.reset_input_buffer()
            self.log("rail: homing (full-length travel to both switches, up to ~2 min)")
            self._send("HOME")
            self._wait_homed(self.cfg.get("home_timeout_s", self.cfg.get("boot_timeout_s", 180)))
        else:
            self._send("$H")
            self._wait_ok("homing")

    def move_to(self, name):
        pos = self.positions[name]
        if self.driver == "esp32":
            if self._status()["state"] != "READY":      # first move after power-up / FAULT: home first (arm is already stowed)
                self.home()
            self.ser.reset_input_buffer()
            cmd = "MOVEPCT" if self.cfg.get("unit", "pct") == "pct" else "MOVE"
            self._send(f"{cmd} {pos}")
            self._expect("ACK", 3.0)
            self._expect("DONE", self.timeout)
            return
        self._send(f"G90 G1 X{pos} F{self.cfg.get('feed', 600)}")
        self._wait_ok("move")
        t0 = time.time()
        while time.time() - t0 < self.timeout:        # wait until GRBL reports Idle
            self._send("?")
            if "Idle" in self._readline():
                return
            time.sleep(0.1)
        raise RailError("rail move: timeout waiting for Idle")

    def position(self):
        if self.driver != "esp32":
            return None
        self.ser.reset_input_buffer()
        return float(self._status()["pos"])

    def _wait_ok(self, what):
        t0 = time.time()
        while time.time() - t0 < self.timeout:
            line = self._readline()
            if line.lower().startswith("ok"):
                return
            if line.lower().startswith(("error", "alarm")):
                raise RailError(f"rail {what}: {line}")
        raise RailError(f"rail {what}: timeout after {self.timeout}s")

    def stop(self):
        if self.ser:
            self.ser.write(b"STOP\n" if self.driver == "esp32" else b"!")

    def close(self):
        if self.ser:
            self.ser.close()


def make_rail(cfg, logf=print) -> RailController:
    if cfg.get("mock"):
        return MockRail(cfg["rail"]["positions"], delay=0.5)
    r = SerialRail(cfg["rail"], logf)
    r.connect()
    return r


# ----------------------------------------------------------------------------------------------- arm stow

def stow_arm(cfg, logf: Callable[[str], None]) -> bool:
    """Move the follower to robot.rest_pose (no policy involved) and keep torque ON so it holds the pose."""
    rob = cfg["robot"]
    if cfg.get("mock") or not rob.get("rest_pose"):
        logf("stow: skipped (mock or no rest_pose configured)")
        return False
    from lerobot.robots.so_follower import SO101Follower, SOFollowerRobotConfig
    rc = SOFollowerRobotConfig(port=rob["port"], id=rob["id"], use_degrees=rob.get("use_degrees", True),
                               max_relative_target=rob.get("max_relative_target"), disable_torque_on_disconnect=False)
    robot = SO101Follower(rc)
    robot.connect()
    try:
        target = dict(rob["rest_pose"])
        t0 = time.time()
        while time.time() - t0 < 30:
            obs = robot.get_observation()
            err = max(abs(obs[k] - v) for k, v in target.items())
            if err < 2.0:
                break
            robot.send_action(target)       # max_relative_target makes this a slow, bounded approach
            time.sleep(0.05)
        else:
            raise RuntimeError(f"arm did not reach rest pose within 30 s (err {err:.1f})")
    finally:
        robot.disconnect()
    logf("stow: arm at rest pose")
    return True


# ----------------------------------------------------------------------------------------------- policy runner

class MockRunner:
    """Same interface as RolloutRunner, no hardware."""
    state = "idle"

    def __init__(self, logf):
        self.log, self._stop = logf, threading.Event()

    def run(self, model_id, model, duration_s, on_state):
        self._stop.clear()
        on_state("LOADING")
        time.sleep(0.3)
        on_state("RUNNING")
        self.log(f"[mock] running {model_id} task={model['task']!r} for {duration_s}s")
        self._stop.wait(min(duration_s, 2.0))
        on_state("RESETTING")
        time.sleep(0.3)

    def stop(self):
        self._stop.set()


class RolloutRunner:
    """Runs one model through the official `lerobot-rollout --interactive=true` CLI as a subprocess.
    Launch -> wait until it is waiting for '/start' -> '/start' -> (duration or Stop) -> '/stop' -> process exits
    and (return_to_initial_position=true, the default) the arm goes back to where it was at launch = rest pose."""

    def __init__(self, cfg, logf):
        self.cfg, self.log = cfg, logf
        self.proc: subprocess.Popen | None = None
        self._stop = threading.Event()
        self.server = None

    # -- command line -------------------------------------------------------------------------------
    def build_cmd(self, model_id: str, model: dict, duration_s: float) -> list[str]:
        rob, inf = self.cfg["robot"], self.cfg["inference"]
        cams = ", ".join(
            f"{n}: {{type: opencv, index_or_path: {c['index_or_path']}, width: {c['width']}, height: {c['height']}, fps: {c['fps']}}}"
            for n, c in rob["cameras"].items())
        exe = shutil.which("lerobot-rollout") or "lerobot-rollout"
        cmd = [exe, "--strategy.type=base", "--interactive=true",
               f"--robot.type={rob['type']}", f"--robot.port={rob['port']}", f"--robot.id={rob['id']}",
               f"--robot.use_degrees={str(rob.get('use_degrees', True)).lower()}",
               f"--robot.cameras={{ {cams} }}",
               "--robot.disable_torque_on_disconnect=false",   # arm keeps holding the rest pose between runs
               f"--task={model['task']}", f"--fps={model.get('fps', 30)}"]
        if rob.get("max_relative_target"):
            cmd.append(f"--robot.max_relative_target={rob['max_relative_target']}")
        if inf["backend"] == "remote":
            cmd += ["--inference.type=remote", f"--inference.endpoint={inf['endpoint']}",
                    f"--inference.deployment={model_id}", "--inference.mode=chunk", "--inference.chunk_merge=aligned",
                    f"--inference.semantics={inf.get('semantics', 'so101-degrees-gripper-percent-v1')}"]
        else:
            cmd += [f"--policy.path={model.get('path') or model['repo']}"]
            if inf.get("device"):
                cmd.append(f"--device={inf['device']}")
            if inf["backend"] == "rtc":
                cmd.append("--inference.type=rtc")
        return cmd

    # -- lifecycle ---------------------------------------------------------------------------------
    def run(self, model_id, model, duration_s, on_state):
        self._stop.clear()
        on_state("LOADING")
        if self.cfg["inference"]["backend"] == "remote" and self.cfg["inference"].get("ssh_host"):
            self.server = RemoteServer(self.cfg["inference"], self.log)
            self.server.start(model_id)
            if self._stop.is_set():
                return self._cleanup()
        env = dict(os.environ)
        if self.cfg["inference"].get("offline"):
            env["HF_HUB_OFFLINE"] = "1"
        cmd = self.build_cmd(model_id, model, duration_s)
        self.log("$ " + " ".join(shlex.quote(c) for c in cmd))
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, bufsize=1, env=env, start_new_session=True)
        ready = threading.Event()

        def pump():
            for line in self.proc.stdout:
                line = line.rstrip()
                if line:
                    self.log("  | " + line)
                if "/start" in line:
                    ready.set()
        threading.Thread(target=pump, daemon=True).start()

        t0, limit = time.time(), self.cfg["inference"].get("load_timeout_s", 120)
        while not ready.is_set():
            if self.proc.poll() is not None:
                return self._fail(f"rollout exited during load (code {self.proc.returncode})")
            if self._stop.is_set():
                return self._cleanup()
            if time.time() - t0 > limit:
                return self._fail(f"rollout not ready after {limit}s")
            time.sleep(0.2)
        self._send("/start")
        on_state("RUNNING")
        end = time.time() + duration_s
        while time.time() < end and not self._stop.is_set() and self.proc.poll() is None:
            time.sleep(0.1)
        on_state("RESETTING")
        self._cleanup()

    def _send(self, line):
        try:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, ValueError, AttributeError):
            pass

    def _fail(self, msg):
        self.log("ERROR: " + msg)
        self._cleanup(force=True)
        raise RuntimeError(msg)

    def _cleanup(self, force=False):
        p = self.proc
        if p and p.poll() is None:
            if not force:
                self._send("/stop")                       # graceful: arm returns to its start pose, then exits
                try:
                    p.wait(timeout=self.cfg["inference"].get("stop_timeout_s", 45))
                except subprocess.TimeoutExpired:
                    self.log("rollout did not exit after /stop -> terminating")
            if p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    p.kill()
        self.proc = None
        if self.server:
            self.server.stop()
            self.server = None

    def stop(self):
        self._stop.set()


class RemoteServer:
    """Optionally starts `lerobot-policy-server` for the chosen model on the GPU box over SSH and tunnels port 7447.
    One ssh process = tunnel + server; killing it stops both. Needs ~/serve/run_server.sh on the GPU box (see server/)."""

    def __init__(self, inf, logf):
        self.inf, self.log, self.p = inf, logf, None

    def start(self, model_id):
        port = self.inf.get("port", 7447)
        cmd = ["ssh", "-tt", "-o", "ExitOnForwardFailure=yes", "-L", f"{port}:127.0.0.1:{port}", self.inf["ssh_host"],
               f"bash ~/serve/run_server.sh {shlex.quote(model_id)}"]
        self.log("$ " + " ".join(cmd))
        self.p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
                                  start_new_session=True)
        ready = threading.Event()

        def pump():
            for line in self.p.stdout:
                line = line.rstrip()
                if line:
                    self.log("  [server] " + line)
                if "Policy server ready" in line:
                    ready.set()
        threading.Thread(target=pump, daemon=True).start()
        t0 = time.time()
        while not ready.is_set():
            if self.p.poll() is not None or time.time() - t0 > self.inf.get("server_timeout_s", 180):
                self.stop()
                raise RuntimeError("policy server did not become ready (see [server] lines above)")
            time.sleep(0.3)

    def stop(self):
        if self.p and self.p.poll() is None:
            self.p.terminate()
            try:
                self.p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.p.kill()
        self.p = None


# ----------------------------------------------------------------------------------------------- state machine

class Controller:
    """IDLE -> STOWING -> RAIL_MOVING -> READY(ws) -> LOADING -> RUNNING -> RESETTING -> READY(ws).
    Commands arriving while busy are ignored. Any error -> FAULT (needs `acknowledge_fault()`)."""

    BUSY = {"STOWING", "RAIL_MOVING", "LOADING", "RUNNING", "RESETTING"}

    def __init__(self, cfg, logf=print, on_change=lambda s, ws: None, rail=None, runner=None):
        self.cfg, self.log, self.on_change = cfg, logf, on_change
        self.rail = rail if rail is not None else make_rail(cfg, logf)
        self.runner = runner if runner is not None else (MockRunner(logf) if cfg.get("mock") else RolloutRunner(cfg, logf))
        self.state, self.ws = "IDLE", None
        self._lock = threading.Lock()
        self._jobs: queue.Queue = queue.Queue()
        self.arm_at_rest = bool(cfg.get("arm_starts_at_rest", False)) or bool(cfg.get("mock"))
        threading.Thread(target=self._worker, daemon=True).start()

    # ---- public API (all non-blocking) ----
    def select_workspace(self, ws_id: int):
        with self._lock:
            if self.state not in ("IDLE", "READY"):
                busy = self.state
            else:
                busy = None
                self._set("STOWING")          # flip state immediately so a second click is ignored
        if busy:
            self.log(f"ignored select_workspace: state is {busy}")
            return
        self._jobs.put(("ws", ws_id))

    def choose(self, option_key: str, source: str = "ui"):
        """Select the tool / recipe. `source` is only for the log (text|voice|vision)."""
        with self._lock:
            if not (self.state == "READY" and self.ws is not None):
                reason = f"state is {self.state}"
            elif option_key not in self._ws_cfg()["options"]:
                reason = f"not an option of workspace {self.ws}"
            else:
                reason = None
                self._set("LOADING")          # flip state immediately so repeated commands are ignored
        if reason:
            self.log(f"ignored '{option_key}' ({source}): {reason}")
            return
        self.log(f"command via {source}: {option_key}")
        self._jobs.put(("run", option_key))

    def stop(self):
        """Always works: ends the current run (arm returns to rest). During a rail move/homing it halts the rail
        (the move then fails -> FAULT, acknowledge it once the rail is safe)."""
        if self.state == "RAIL_MOVING":
            self.rail.stop()
        self.runner.stop()

    def back(self):
        self.runner.stop()
        with self._lock:
            if self.state == "READY":
                self.ws = None
                self._set("IDLE")

    def acknowledge_fault(self):
        with self._lock:
            if self.state == "FAULT":
                self.ws = None
                self._set("IDLE")

    # ---- internals ----
    def _ws_cfg(self):
        return next(w for w in self.cfg["workspaces"] if w["id"] == self.ws)

    def _set(self, state, ws=None):
        self.state = state
        if ws is not None:
            self.ws = ws
        self.on_change(self.state, self.ws)

    def _worker(self):
        while True:
            kind, arg = self._jobs.get()
            try:
                if kind == "ws":
                    self._do_workspace(arg)
                else:
                    self._do_run(arg)
            except Exception as e:  # noqa
                self.log(f"FAULT: {e}")
                with self._lock:
                    self._set("FAULT")

    def _do_workspace(self, ws_id):
        w = next(x for x in self.cfg["workspaces"] if x["id"] == ws_id)
        if stow_arm(self.cfg, self.log):
            self.arm_at_rest = True
        pos = w["rail_position"]
        if not self.arm_at_rest:
            raise RuntimeError("interlock: arm is not at rest, refusing to move the rail")
        with self._lock:
            self._set("RAIL_MOVING")
        self.log(f"rail -> {pos}")
        self.rail.move_to(pos)
        with self._lock:
            self.ws = ws_id
            self._set("READY", ws_id)
        self.log(f"workspace {ws_id} ({w['name']}) ready")

    def _do_run(self, option_key):
        opt = self._ws_cfg()["options"][option_key]
        model_id = opt["model"]
        model = dict(self.cfg["models"][model_id])
        model.setdefault("fps", next(iter(self.cfg["robot"]["cameras"].values()))["fps"])

        def on_state(s):
            with self._lock:
                self._set(s)
        try:
            self.runner.run(model_id, model, model["duration_s"], on_state)
        finally:
            self.arm_at_rest = True       # rollout returns the arm to its start pose on /stop
        with self._lock:
            self._set("READY", self.ws)
        self.log(f"run finished; workspace {self.ws} ready")

    def close(self):
        self.runner.stop()
        self.rail.stop()
        self.rail.close()


# ----------------------------------------------------------------------------------------------- inputs

class VoiceInput:
    """Microphone -> energy-based speech segments -> faster-whisper -> workspace-scoped keyword match.
    Imports are lazy so a missing package never breaks text mode. Speech heard while busy is ignored."""

    def __init__(self, controller: Controller, logf, model_size="base", device=None):
        import numpy as np
        import sounddevice as sd
        from faster_whisper import WhisperModel
        self.np, self.sd, self.c, self.log = np, sd, controller, logf
        # optional: voice.input_device in workspaces.yaml (index or name substring from `python3 -m sounddevice`)
        if device is None:
            device = (getattr(controller, "cfg", None) or {}).get("voice", {}).get("input_device")
        self.device = device
        self.model = WhisperModel(model_size, device="cpu", compute_type="int8")
        self.running = False

    def start(self, prompt: str = ""):
        self.running = True
        threading.Thread(target=self._loop, args=(prompt,), daemon=True).start()

    def stop(self):
        self.running = False

    def _loop(self, prompt):
        np, sd, sr = self.np, self.sd, 16000
        blk = int(sr * 0.1)
        rms = lambda a: float(np.sqrt(np.mean(a ** 2)))          # noqa: E731
        try:
            st = sd.InputStream(samplerate=sr, channels=1, dtype="float32", blocksize=blk, device=self.device)
        except Exception as e:
            self.log(f"voice: cannot open microphone ({e}). List devices with `python3 -m sounddevice`.")
            return
        with st:
            self.log(f"voice: using input device {sd.query_devices(st.device)['name']!r}")
            for _ in range(5):                                  # discard the start-up pop / settling (0.5 s)
                st.read(blk)
            levels = []
            for _ in range(15):                                 # 1.5 s of room noise, DC offset removed
                x = st.read(blk)[0][:, 0]
                levels.append(rms(x - x.mean()))
            noise = float(np.median(levels))                    # median: robust to a single click or cough
            thr = min(max(noise * 3.0, 0.01), 0.15)             # never above 0.15 (speech is ~0.02-0.3 RMS)
            self.log(f"voice: listening (noise {noise:.4f}, threshold {thr:.4f})")
            if noise > 0.1:
                self.log("voice: WARNING room-noise level is very high -> mic gain too high / wrong device. "
                         "Lower the input volume in Settings > Sound or set voice.input_device.")
            buf, quiet = [], 0
            while self.running:
                x = st.read(blk)[0][:, 0]
                x = x - x.mean()
                loud = rms(x) > thr
                if loud or buf:
                    buf.append(x)
                    quiet = 0 if loud else quiet + 1
                    if quiet >= 8 or len(buf) > 100:           # 0.8 s of silence or 10 s max
                        audio = np.concatenate(buf)
                        buf, quiet = [], 0
                        if len(audio) < sr * 0.3:               # ignore clicks / bumps shorter than 0.3 s
                            continue
                        if self.c.state != "READY":              # busy -> ignore
                            continue
                        peak = float(np.abs(audio).max())
                        if peak > 0:
                            audio = audio * min(0.9 / peak, 20.0)  # normalise level for Whisper
                        segs, _ = self.model.transcribe(audio, language="en", initial_prompt=prompt or None, beam_size=1)
                        text = " ".join(s.text for s in segs).strip()
                        self.log(f"voice heard: {text!r} (peak {peak:.3f}, {len(audio)/sr:.1f} s)")
                        key = parse_keywords(text, self.c._ws_cfg()["options"]) if self.c.ws else None
                        if key:
                            self.c.choose(key, "voice")


class GestureInput:
    """Webcam + MediaPipe hand landmarks: hold N extended fingers (still) for `hold_s` -> option with gesture "N".
    Uses the laptop webcam (not a robot camera: the rollout process owns those)."""

    def __init__(self, controller: Controller, logf, camera_index=0, model_path="hand_landmarker.task",
                 hold_s=0.6, cooldown_s=3.0, on_frame: Callable[[bytes], None] | None = None):
        import cv2
        import mediapipe as mp
        from mediapipe.tasks import python as mpt
        from mediapipe.tasks.python import vision
        self.cv2, self.mp, self.c, self.log = cv2, mp, controller, logf
        self.cam, self.hold_s, self.cooldown_s, self.on_frame = camera_index, hold_s, cooldown_s, on_frame
        opts = vision.HandLandmarkerOptions(
            base_options=mpt.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO, num_hands=1)
        self.landmarker = vision.HandLandmarker.create_from_options(opts)
        self.running = False

    @staticmethod
    def count_fingers(lm) -> int:
        """Index..pinky extended if the tip is farther from the wrist than the PIP joint. Thumb is ignored (1-4)."""
        import math
        d = lambda a, b: math.hypot(lm[a].x - lm[b].x, lm[a].y - lm[b].y)  # noqa: E731
        return sum(d(tip, 0) > d(pip, 0) * 1.05 for tip, pip in ((8, 6), (12, 10), (16, 14), (20, 18)))

    def start(self):
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self.running = False

    def _loop(self):
        cv2, mp = self.cv2, self.mp
        cap = cv2.VideoCapture(self.cam)
        if not cap.isOpened():
            self.log(f"gesture: cannot open camera {self.cam}")
            return
        held, since, last_fire, rearmed, last_ts = None, 0.0, 0.0, True, -1
        try:
            while self.running:
                ok, frame = cap.read()
                if not ok:
                    break
                frame = cv2.flip(frame, 1)
                ts = max(int(time.monotonic() * 1000), last_ts + 1)
                last_ts = ts
                res = self.landmarker.detect_for_video(
                    mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)), ts)
                n = self.count_fingers(res.hand_landmarks[0]) if res.hand_landmarks else 0
                now = time.monotonic()
                if n == 0:
                    rearmed, held = True, None                  # relax the hand to re-arm
                elif rearmed and now - last_fire > self.cooldown_s and self.c.state == "READY":
                    if n != held:
                        held, since = n, now
                    elif now - since >= self.hold_s:
                        key = next((k for k, o in self.c._ws_cfg()["options"].items() if str(o.get("gesture")) == str(n)), None)
                        if key:
                            self.c.choose(key, "vision")
                            last_fire, rearmed, held = now, False, None
                cv2.putText(frame, f"fingers: {n}  state: {self.c.state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                if self.on_frame:
                    self.on_frame(cv2.imencode(".jpg", frame)[1].tobytes())
        finally:
            cap.release()


# ----------------------------------------------------------------------------------------------- pre-flight

def preflight(cfg, methods=("text",)) -> list[str]:
    """Return a list of problems (empty = OK). Only checks what the chosen input methods need."""
    bad = []
    if sys.version_info < (3, 12):
        bad.append(f"Python {sys.version_info.major}.{sys.version_info.minor}: LeRobot 0.6 needs >= 3.12")
    if not cfg.get("mock"):
        if not shutil.which("lerobot-rollout"):
            bad.append("lerobot-rollout not found: install LeRobot (see README) in this environment")
        if cfg["inference"]["backend"] == "remote" and not cfg["inference"].get("endpoint"):
            bad.append("inference.endpoint missing for remote backend")
        for k, v in (("robot.port", cfg["robot"]["port"]), ("rail.port", cfg["rail"].get("port"))):
            if v and str(v).startswith("/dev") and not Path(v).exists():
                bad.append(f"{k} {v} does not exist (plug in / check lerobot-find-port)")
        for n, m in cfg["models"].items():
            loc = m.get("path") or m.get("repo") or "<"
            if "<" in loc or "<" in m.get("task", "<"):
                bad.append(f"models.{n}: fill path and task")
            elif cfg["inference"]["backend"] != "remote" and m.get("path") and not (Path(m["path"]) / "config.json").exists():
                bad.append(f"models.{n}: no config.json in {m['path']} (point path at the pretrained_model folder)")
    mods = {"voice": ["sounddevice", "faster_whisper"], "vision": ["cv2", "mediapipe"]}
    import importlib.util
    for meth in methods:
        for mod in mods.get(meth, []):
            if importlib.util.find_spec(mod) is None:
                bad.append(f"input '{meth}' needs python package '{mod}'")
    if "vision" in methods and not Path("hand_landmarker.task").exists():
        bad.append("hand_landmarker.task missing next to the notebook")
    return bad