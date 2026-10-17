"""
Generate a language-labelled LeRobot dataset automatically with a scripted pick-and-place expert.
Run from the repo root:   python collect_scripted.py
It APPENDS to an existing dataset if you answer 'n' to the delete prompt, so you can mix it with teleop demos.
"""
import os, shutil
import numpy as np
from PIL import Image
from mujoco_env.y_env_tools import SimpleEnvTools, TOOLS, PROMPT
from mujoco_env.scripted_expert import ScriptedPnP
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

REPO_NAME = 'garage_pnp'
ROOT = './demo_data_garage'
EPISODES_PER_TOOL = 75          # 25*4 = 100 scripted episodes
XML = './asset/example_scene_tools.xml'
SEED = None                     # None -> different layouts every run
MAX_TICKS = 700                 # 35 s at 20 Hz; abort a failed episode
MIN_FRAMES = 40
RENDER_EVERY = 3                # draw the viewer every Nth tick (faster); cameras for the dataset are always rendered


def make_dataset():
    if os.path.exists(ROOT):
        if input(f"{ROOT} exists. Delete it? (y = delete, n = append) ") == 'y':
            shutil.rmtree(ROOT)
        else:
            return LeRobotDataset(REPO_NAME, root=ROOT)
    img = lambda n: {"dtype": "image", "shape": (256, 256, 3), "names": ["height", "width", n]}
    return LeRobotDataset.create(
        repo_id=REPO_NAME, root=ROOT, robot_type="omy_slider", fps=20,
        features={
            "observation.image": img("channels"),
            "observation.wrist_image": img("channel"),
            "observation.state": {"dtype": "float32", "shape": (8,), "names": ["state"]},
            "action": {"dtype": "float32", "shape": (8,), "names": ["action"]},
            "obj_init": {"dtype": "float32", "shape": (16,), "names": ["obj_init"]},
        },
        image_writer_threads=10, image_writer_processes=5)


def resize(a):
    return np.array(Image.fromarray(a).resize((256, 256)))


def main():
    dataset = make_dataset()
    env = SimpleEnvTools(XML, action_type='eef_pose', seed=SEED)      # slide_mode stays 'follow'
    plan = [t for t in TOOLS for _ in range(EPISODES_PER_TOOL)]
    np.random.shuffle(plan)
    ep, fails = 0, 0
    while ep < len(plan) and env.env.is_viewer_alive():
        tool = plan[ep]
        env.reset(n_pre_in_dst=int(np.random.choice([0, 0, 1, 2])), target=tool)   # target never starts in the green tray
        env.set_instruction(PROMPT.format(tool))
        expert, ticks, ok = ScriptedPnP(env, tool), 0, False
        while env.env.is_viewer_alive():
            env.step_env()
            if env.env.loop_every(HZ=20):
                action, finished = expert.act()
                rgb, wrist = env.grab_image()
                state = env.step(action)
                dataset.add_frame({
                    "observation.image": resize(rgb), "observation.wrist_image": resize(wrist),
                    "observation.state": state, "action": env.get_action_vec(),
                    "obj_init": env.obj_init_pose}, task=env.instruction)
                ticks += 1
                if ticks % RENDER_EVERY == 0:
                    env.render()
                if ticks >= MIN_FRAMES and env.check_success(tool):
                    ok = True
                    break
                if finished or ticks > MAX_TICKS:
                    break
        if ok:
            dataset.save_episode(); ep += 1; fails = 0
            print(f"episode {ep}/{len(plan)} saved ({tool}, {ticks} ticks)")
        else:
            dataset.clear_episode_buffer()
            fails += 1
            print(f"FAILED ({tool}) - discarded ({fails}/10 in a row)")
            if fails >= 10:
                print("10 failures in a row: calibrate HOVER_Z / GRASP_DZ / RELEASE_DZ in mujoco_env/scripted_expert.py")
                break
    env.env.close_viewer()
    shutil.rmtree(dataset.root / 'images', ignore_errors=True)


if __name__ == '__main__':
    main()
