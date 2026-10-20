#!/usr/bin/env python3
"""Read ONLY the metadata of every dataset (a few KB each) and print what training/deployment need:
fps, episodes, frames, cameras, task string, longest episode -> run duration for deployment.
Writes plan.json (used by train_all.py and copied into the deployment config).

    python prepare.py [--config train_models.yaml]
"""
import argparse
import json
import math
from pathlib import Path

import pandas as pd
import yaml
from huggingface_hub import snapshot_download


def read_meta(repo: str) -> dict:
    root = Path(snapshot_download(repo, repo_type="dataset", allow_patterns=["meta/**"]))
    info = json.loads((root / "meta/info.json").read_text())
    fps = info["fps"]
    tasks = pd.read_parquet(root / "meta/tasks.parquet")
    # v3: the task string is the index (older exports: a "task" column)
    task_list = [str(t) for t in (tasks["task"].tolist() if "task" in tasks.columns else tasks.index.tolist())]
    eps = pd.concat([pd.read_parquet(p) for p in sorted((root / "meta/episodes").rglob("*.parquet"))])
    lengths = eps["length"].tolist()
    feats = info["features"]
    cams = {k: v["shape"] for k, v in feats.items() if v.get("dtype") in ("video", "image")}
    codec = next((v.get("info", {}).get("video.codec") for v in feats.values() if v.get("dtype") == "video"), None)
    return dict(
        repo=repo, fps=fps, codebase_version=info.get("codebase_version"), robot_type=info.get("robot_type"),
        episodes=info["total_episodes"], frames=info["total_frames"], cameras=cams, video_codec=codec,
        state_names=feats.get("observation.state", {}).get("names"),
        action_names=feats.get("action", {}).get("names"),
        tasks=task_list, longest_ep_s=max(lengths) / fps, mean_ep_s=sum(lengths) / len(lengths) / fps,
        run_duration_s=math.ceil(1.3 * max(lengths) / fps),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="train_models.yaml")
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    plan = {}
    for name, m in cfg["models"].items():
        if "<" in m["dataset"]:
            raise SystemExit(f"Fill the dataset repo for '{name}' in {a.config}")
        plan[name] = read_meta(m["dataset"]) | {"policy": m["policy"]}
        p = plan[name]
        print(f"\n[{name}] {p['repo']}  ({p['policy']})")
        print(f"  version={p['codebase_version']} robot={p['robot_type']} fps={p['fps']} episodes={p['episodes']} frames={p['frames']}")
        print(f"  cameras={ {k: tuple(v) for k, v in p['cameras'].items()} } codec={p['video_codec']}")
        print(f"  task={p['tasks']}")
        print(f"  longest episode {p['longest_ep_s']:.1f}s -> deployment duration_s ~ {p['run_duration_s']}")
        if p["codebase_version"] and not str(p["codebase_version"]).startswith("v3"):
            print("  !! NOT a v3 dataset: convert it first (lerobot/scripts/convert_dataset_v21_to_v30.py)")
        if p["video_codec"] and "av1" in str(p["video_codec"]).lower():
            print("  note: AV1 video is slower to decode; if the GPU sits idle during training use more workers")
    cams = {tuple(sorted(p["cameras"])) for p in plan.values()}
    if len(cams) > 1:
        print(f"\n!! datasets do not share the same camera names: {cams}")
    Path("plan.json").write_text(json.dumps(plan, indent=2))
    print("\nWrote plan.json")


if __name__ == "__main__":
    main()
