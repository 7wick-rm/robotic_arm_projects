#!/usr/bin/env python3
"""Run ON THE BREV GPU BOX (LeRobot venv active). Builds ~/serve/<model>.yaml for every model + ~/serve/run_server.sh.
The model folders must already be on the box, one per model id:   ~/models/<id>/pretrained_model/{config.json,model.safetensors,...}
(copy them from the laptop, see README step 3). The server schema is derived from each model's config.json.

    python make_server_yaml.py [--models-dir ~/models] [--ids screw,hammer,wrench,burger,thor_hammer]
"""
import argparse, json, stat
from pathlib import Path
import yaml

JOINTS = ["shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos", "wrist_flex.pos", "wrist_roll.pos", "gripper.pos"]  # SO-101 order

ap = argparse.ArgumentParser()
ap.add_argument("--models-dir", default=str(Path.home() / "models"))
ap.add_argument("--ids", default="screw,hammer,wrench,burger,thor_hammer")
ap.add_argument("--fps", type=float, default=30.0, help="dataset fps (same as robot.cameras fps in workspaces.yaml)")
ap.add_argument("--semantics", default="so101-degrees-gripper-percent-v1")
ap.add_argument("--out", default=str(Path.home() / "serve"))
a = ap.parse_args()
out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
for mid in a.ids.split(","):
    mp = Path(a.models_dir) / mid / "pretrained_model"
    cfg = json.load(open(mp / "config.json"))
    st = cfg["input_features"]["observation.state"]["shape"]
    act = cfg["output_features"]["action"]["shape"]
    if st != [6] or act != [6]:
        raise SystemExit(f"{mid}: state/action shape {st}/{act} is not the 6-joint SO-101 layout")
    feats = [{"name": "observation.state", "shape": st, "dtype": "float32", "names": list(JOINTS), "semantics": a.semantics}]
    for k, v in cfg["input_features"].items():
        if k.startswith("observation.images."):
            c, h, w = v["shape"]                      # policy config stores (C, H, W); the server schema wants H, W, 3
            feats.append({"name": k, "shape": [h, w, c], "dtype": "uint8", "kind": "rgb", "semantics": f"rgb-{k.split('.')[-1]}-v1"})
    y = {"deployment": mid,
         "model": {"repo_or_path": str(mp), "revision": None, "device": "cuda"},
         "execution": {"supported_modes": ["chunk"], "action_fps": a.fps, "action_deadline_s": 5.0, "idle_timeout_s": 10.0, "warmup_calls": 2},
         "language": {"enabled": False, "deadline_s": 60.0},
         "zenoh": {"mode": "peer", "listen_endpoints": ["tcp/127.0.0.1:7447"]},
         "semantics": a.semantics, "robot_type": "so101_follower", "features": feats,
         "action_feature": {"name": "action", "shape": act, "dtype": "float32", "names": list(JOINTS), "semantics": a.semantics}}
    (out / f"{mid}.yaml").write_text(yaml.safe_dump(y, sort_keys=False))
    print("wrote", out / f"{mid}.yaml")
sh = out / "run_server.sh"
sh.write_text(f'#!/usr/bin/env bash\n# usage: run_server.sh <model_id>   (called by the laptop over ssh; stays in the foreground)\nsource "$HOME/lr-venv/bin/activate"\nexec lerobot-policy-server --config_path="{out}/$1.yaml" 2>&1\n')
sh.chmod(sh.stat().st_mode | stat.S_IEXEC)
print("wrote", sh, "\nTest by hand first:  bash ~/serve/run_server.sh hammer   -> wait for 'Policy server ready'")
