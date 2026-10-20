"""
sim_server.py - the simulation process: MuJoCo viewer + SmolVLA policy, controlled over UDP.

The control-panel notebook starts this program for you. To run it by hand:
    python sim_server.py --ckpt ./ckpt/smolvla_garage/checkpoints/020000/pretrained_model

Commands (UDP, plain text, localhost):
    wrench | screwdriver | hammer | pliers      (or any sentence that names exactly ONE tool)
    reset                                       reshuffle the tools back into the source tray
    quit

Status lines it prints (the notebook reads them):
    [state] idle | run:<tool> | home
    [event] success:<tool> | timeout:<tool> | refused:<tool> | reset
"""
import argparse
import os
import queue
import re
import socket
import sys
import threading
import time

import numpy as np

# ------------------------------------------------------------------ command parsing (no heavy imports)
TOOL_SYNONYMS = {
    "wrench": ["wrench", "spanner"],
    "screwdriver": ["screwdriver", "screw driver"],
    "hammer": ["hammer"],
    "pliers": ["pliers", "plier", "plyers"],
}


def tool_from_text(text):
    """The tool named in `text`, or None if no tool or more than one tool is named."""
    t = text.lower()
    hits = [tool for tool, words in TOOL_SYNONYMS.items()
            if any(re.search(r"\b" + re.escape(w) + r"\b", t) for w in words)]
    return hits[0] if len(hits) == 1 else None


def parse_command(text):
    """-> ('tool', name) | ('reset', None) | ('quit', None) | (None, None)"""
    t = " ".join(text.strip().lower().split())
    if t.startswith("task\t"):                       # old notebook protocol
        t = t[5:].strip()
    if t in ("quit", "exit"):
        return ("quit", None)
    tool = tool_from_text(t)
    if tool:
        return ("tool", tool)
    if re.search(r"\breset\b", t):
        return ("reset", None)
    return (None, None)


# ------------------------------------------------------------------ the state machine (no IO of its own)
class ArmController:
    """idle -> run (policy drives the arm) -> home (scripted return) -> idle.
    * a command that arrives while the arm is busy is ignored (gesture/voice programs can repeat themselves)
    * a tool that is already in the green tray is refused
    * every run starts from the home pose, exactly like the training episodes"""

    def __init__(self, reset_policy, max_run_ticks=600, min_run_ticks=30, home_ticks=60):
        self.reset_policy = reset_policy
        self.max_run_ticks, self.min_run_ticks, self.home_ticks = max_run_ticks, min_run_ticks, home_ticks
        self.mode, self.tool, self.t, self.home_from = "idle", None, 0, None

    def _state_line(self):
        return f"[state] run:{self.tool}" if self.mode == "run" else f"[state] {self.mode}"

    def _start_home(self, env):
        self.mode, self.t, self.home_from = "home", 0, env.get_action_vec()

    def tick(self, env, get_action, commands, emit=print):
        """one 20 Hz control step. returns 'quit' to stop the program, else None."""
        for kind, arg in commands:
            if kind is None:
                emit("[event] unknown command (name exactly one tool: wrench, screwdriver, hammer, pliers)")
            elif kind == "quit":
                return "quit"
            elif kind == "reset":
                env.reset()
                env.set_instruction("")
                self.reset_policy()
                self.mode, self.tool, self.t = "idle", None, 0
                emit("[event] reset")
            elif kind == "tool":
                if self.mode != "idle":
                    emit(f"[event] busy - ignored '{arg}'")
                elif env.tool_in_tray(arg, "dst"):
                    emit(f"[event] refused:{arg} (it is already in the green tray; send 'reset' to reshuffle)")
                else:
                    from mujoco_env.y_env_tools import PROMPT          # imported lazily so tests need no lerobot
                    self.mode, self.tool, self.t = "run", arg, 0
                    env.set_instruction(PROMPT.format(arg))
                    self.reset_policy()                                # clear the previous action chunk
                    emit(f"[event] start:{arg} -> \"{env.instruction}\"")
            emit(self._state_line())

        if self.mode == "run":
            action = np.asarray(get_action(env, env.instruction), dtype=np.float64)
            if not np.isfinite(action).all():
                emit("[event] non-finite action; going home")
                self._start_home(env)
                emit(self._state_line())
                return None
            env.step(action)
            self.t += 1
            if self.t >= self.min_run_ticks and env.check_success(self.tool):
                emit(f"[event] success:{self.tool}")
                self._start_home(env)
                emit(self._state_line())
            elif self.t >= self.max_run_ticks:
                emit(f"[event] timeout:{self.tool} (send 'reset' if it is stuck)")
                self._start_home(env)
                emit(self._state_line())
        elif self.mode == "home":
            self.t += 1
            a = min(1.0, self.t / self.home_ticks)
            vec = (1 - a) * self.home_from + a * env.home_action_vec()
            vec[7] = 0.0                                               # open the gripper
            env.step(vec)
            if self.t >= self.home_ticks + 10:
                self.mode, self.tool = "idle", None
                env.set_instruction("")
                emit(self._state_line())
        return None


# ------------------------------------------------------------------ the checkpoint carries its own normalisation statistics
def check_normalization(policy):
    """The mean/std used to (un)normalise state and action are stored INSIDE the checkpoint weights (the dataset is not needed).
    They are created as +inf and must have been overwritten while loading. Returns how many tensors were checked."""
    import numpy as np
    items = list(policy.named_parameters()) + list(policy.named_buffers())
    stats = [(n, p) for n, p in items if "normalize_" in n]            # also matches 'unnormalize_'
    if len(stats) < 4:
        raise RuntimeError(f"expected the normalisation statistics inside the checkpoint but found {len(stats)} tensors")
    bad = [n for n, p in stats if not np.isfinite(p.detach().cpu().numpy()).all()]
    if bad:
        raise RuntimeError("the checkpoint has no valid normalisation statistics (still infinite): " + ", ".join(bad[:3]))
    return len(stats)


# ------------------------------------------------------------------ UDP
def open_socket(host, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)               # no SO_REUSEADDR: a second server must fail loudly
    s.bind((host, port))
    s.settimeout(0.5)
    return s


def udp_listener(sock, q, stop):
    while not stop.is_set():
        try:
            data, _ = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        q.put(data.decode("utf-8", errors="ignore"))


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--ckpt", required=True, help=".../checkpoints/<step>/pretrained_model")
    p.add_argument("--xml", default="./asset/example_scene_tools.xml")
    p.add_argument("--device", default="cpu")
    p.add_argument("--hz", type=int, default=20)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--max_run_ticks", type=int, default=600)
    p.add_argument("--min_run_ticks", type=int, default=30)
    p.add_argument("--home_ticks", type=int, default=60)
    p.add_argument("--parent_pid", type=int, default=0, help="exit when this process dies (set by the notebook)")
    return p.parse_args()


def main():
    a = parse_args()
    q, stop = queue.Queue(), threading.Event()
    try:
        sock = open_socket(a.host, a.port)                              # fail fast, before the slow policy load
    except OSError as exc:
        print(f"[error] cannot listen on {a.host}:{a.port} ({exc}). Another simulation (or deploy_policy.ipynb) "
              f"is probably still running.", flush=True)
        sys.exit(2)
    threading.Thread(target=udp_listener, args=(sock, q, stop), daemon=True).start()

    for f in ("config.json", "model.safetensors"):
        if not os.path.exists(os.path.join(a.ckpt, f)):
            print(f"[error] {a.ckpt}/{f} not found. Point CKPT at the 'pretrained_model' folder of a checkpoint.", flush=True)
            sys.exit(2)

    print("loading the policy (this can take a minute)...", flush=True)
    import torch
    from PIL import Image
    from torchvision import transforms
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    from mujoco_env.y_env_tools import SimpleEnvTools

    cfg = PreTrainedConfig.from_pretrained(a.ckpt)
    cfg.device = a.device
    policy = SmolVLAPolicy.from_pretrained(a.ckpt, config=cfg).to(a.device).eval()     # no dataset needed: stats are in the checkpoint
    try:
        n_stats = check_normalization(policy)
    except RuntimeError as exc:
        print(f"[error] {exc}", flush=True)
        sys.exit(2)
    print(f"normalisation statistics loaded from the checkpoint ({n_stats} tensors, all finite)", flush=True)
    to_tensor = transforms.ToTensor()

    def get_action(env, task):
        rgb, wrist = env.grab_image()
        f = lambda img: to_tensor(Image.fromarray(img).resize((256, 256))).unsqueeze(0).to(a.device)
        obs = {"observation.state": torch.tensor(env.get_joint_state()).unsqueeze(0).to(a.device),   # 8-D
               "observation.image": f(rgb), "observation.wrist_image": f(wrist), "task": [task]}
        with torch.no_grad():
            return policy.select_action(obs)[0, :8].cpu().numpy()                                    # 6 joints, slide, gripper

    env = SimpleEnvTools(a.xml, action_type="joint_angle", seed=0)
    ctrl = ArmController(policy.reset, a.max_run_ticks, a.min_run_ticks, a.home_ticks)
    policy.reset()
    print(f"READY. listening on {a.host}:{a.port}", flush=True)
    print("[state] idle", flush=True)

    emit = lambda s: print(s, flush=True)
    try:
        while env.env.is_viewer_alive():
            if a.parent_pid and os.getppid() != a.parent_pid:           # the notebook is gone: do not leave an orphan window
                break
            env.step_env()                                              # physics at 500 Hz
            if not env.env.loop_every(HZ=a.hz):
                continue                                                # control at 20 Hz
            commands = []
            while not q.empty():
                commands.append(parse_command(q.get()))
            if ctrl.tick(env, get_action, commands, emit) == "quit":
                break
            if ctrl.mode != "run":
                env.grab_image()                                        # keep the overlay cameras fresh
            env.render()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        sock.close()
        try:
            env.env.close_viewer()
        except Exception:
            pass
        print("[state] stopped", flush=True)


if __name__ == "__main__":
    main()
