"""
Always-on deployment: the sim keeps running, you type (or a queue receives) a tool name, the arm picks it
and drops it in the destination tray, returns home, and waits for the next command.

    python deploy_language.py            # then type:  hammer   /  wrench  /  reset  /  quit
"""
import queue, threading
import numpy as np
import torch
from PIL import Image
from torchvision import transforms
from lerobot.common.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.configs.policies import PreTrainedConfig
from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from mujoco_env.y_env_tools import SimpleEnvTools, TOOLS, PROMPT

ROOT = './demo_data_garage'
REPO_NAME = 'garage_pnp'
CKPT = './ckpt/smolvla_garage/checkpoints/last/pretrained_model'
XML = './asset/example_scene_tools.xml'
MAX_RUN_TICKS = 600          # ~2x the longest demo; give up after this
MIN_RUN_TICKS = 30           # don't check success right after start (arm starts high with gripper open)
HOME_TICKS = 60              # 3 s interpolation back to the home pose between commands

device = 'cuda' if torch.cuda.is_available() else 'cpu'
meta = LeRobotDatasetMetadata(REPO_NAME, root=ROOT)
cfg = PreTrainedConfig.from_pretrained(CKPT)      # reads the 'type' field -> SmolVLAConfig
cfg.device = device
policy = SmolVLAPolicy.from_pretrained(CKPT, config=cfg, dataset_stats=meta.stats).to(device).eval()
to_tensor = transforms.ToTensor()

# ---------- command source (swap this thread for a socket / ROS / file reader for the gesture module) ----------
cmd_q = queue.Queue()
def stdin_reader():
    while True:
        try:
            cmd_q.put(input("tool> "))
        except EOFError:
            return
threading.Thread(target=stdin_reader, daemon=True).start()

def parse(text):
    t = text.strip().lower()
    if t in ('reset', 'quit', 'exit'):
        return t
    for name in TOOLS:
        if name in t:
            return name
    return None

def obs_for(env):
    rgb, wrist = env.grab_image()
    f = lambda a: to_tensor(Image.fromarray(a).resize((256, 256))).unsqueeze(0).to(device)
    return {'observation.state': torch.tensor(env.get_joint_state()).unsqueeze(0).to(device),
            'observation.image': f(rgb), 'observation.wrist_image': f(wrist),
            'task': [env.instruction]}

env = SimpleEnvTools(XML, action_type='joint_angle', seed=0)
mode, tool, t, home_from = 'idle', None, 0, None
print("ready. tools:", TOOLS)

while env.env.is_viewer_alive():
    env.step_env()
    if not env.env.loop_every(HZ=20):
        continue

    # ---------------- idle: look for a new command ----------------
    if mode == 'idle' and not cmd_q.empty():
        cmd = parse(cmd_q.get())
        if cmd in ('quit', 'exit'):
            break
        elif cmd == 'reset':
            env.reset(); env.set_instruction('')
            print("scene reset")
        elif cmd is None:
            print("unknown command; use one of", TOOLS)
        elif env.tool_in_tray(cmd, 'dst'):
            print(f"{cmd} is already in the destination tray")
        else:
            tool, t, mode = cmd, 0, 'run'
            env.set_instruction(PROMPT.format(cmd))
            policy.reset()                      # clear the old action chunk
            print("->", env.instruction)

    # ---------------- run: policy drives the arm ----------------
    if mode == 'run':
        with torch.no_grad():
            action = policy.select_action(obs_for(env))[0, :8].cpu().numpy()
        env.step(action)
        t += 1
        if t >= MIN_RUN_TICKS and env.check_success(tool):
            print(f"success: {tool}")
            mode, t, home_from = 'home', 0, env.get_action_vec()
        elif t >= MAX_RUN_TICKS:
            print(f"timeout on {tool} (it may be dropped somewhere; type 'reset' to reshuffle)")
            mode, t, home_from = 'home', 0, env.get_action_vec()

    # ---------------- home: scripted return so every command starts from the training start pose ----------------
    elif mode == 'home':
        t += 1
        a = min(1.0, t / HOME_TICKS)
        vec = (1 - a) * home_from + a * env.home_action_vec()
        vec[7] = 0.0                            # open gripper
        env.step(vec)
        if t >= HOME_TICKS + 10:
            mode = 'idle'
            env.set_instruction('')

    if mode != 'run':
        env.grab_image()                        # keep the overlay cameras fresh while idle / homing
    env.render()
env.env.close_viewer()
