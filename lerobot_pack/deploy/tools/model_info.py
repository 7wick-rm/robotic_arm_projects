#!/usr/bin/env python3
"""Print what a trained model expects, to fill workspaces.yaml (camera names, resolution, fps hints).
    python tools/model_info.py /path/to/model/pretrained_model [more paths...]
Reads config.json (+ train_config.json if present). Nothing is loaded onto a GPU."""
import json, sys
from pathlib import Path

for p in map(Path, sys.argv[1:]):
    cfg = json.load(open(p / "config.json"))
    print(f"\n== {p}\n type={cfg.get('type')}  chunk_size={cfg.get('chunk_size')}  n_action_steps={cfg.get('n_action_steps')}")
    for k, v in cfg.get("input_features", {}).items():
        print(f"  in : {k:35s} {v.get('type'):7s} shape={v.get('shape')}")
    for k, v in cfg.get("output_features", {}).items():
        print(f"  out: {k:35s} {v.get('type'):7s} shape={v.get('shape')}")
    cams = [k.split(".")[-1] for k in cfg.get("input_features", {}) if k.startswith("observation.images.")]
    print(f"  -> robot.cameras names in workspaces.yaml must be exactly: {cams}")
    tc = p / "train_config.json"
    if tc.exists():
        t = json.load(open(tc))
        print(f"  trained on dataset: {t.get('dataset', {}).get('repo_id')}  (task string + fps: see that dataset's meta/tasks.parquet and meta/info.json)")
