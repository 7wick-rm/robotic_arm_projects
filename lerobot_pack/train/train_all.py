#!/usr/bin/env python3
"""Time-budgeted training of all models on ONE GPU (built for the Brev L40S 48GB).

    python train_all.py --budget-hours 5.5                 # all models in train_models.yaml
    python train_all.py --budget-hours 3.5 --models burger,thor_hammer   # a subset (e.g. second instance)
    python train_all.py --dry-run                          # print the commands, run nothing

What it does
 1. Pre-flight: GPU, HF login, plan.json (run prepare.py first).
 2. Calibration: runs ~80 real training steps once for each policy type, measures samples/s (and OOM-halves
    the batch size if needed). Fails in ~3 min if a flag/dataset is wrong, instead of after hours.
 3. For every model: steps = (time slot - overhead) * measured samples/s / batch_size, scheduler decay set to
    that step count, then trains with bf16, pushes the final model to <hf_user>/<name> and saves a mid checkpoint.
 4. Re-running resumes: finished models are skipped, an interrupted model continues from its last checkpoint.
Outputs: runs/<name>/ (checkpoints), logs/<name>.log, runs/state.json, runs/trained_models.json
"""
import argparse
import json
import os
import re
import shlex
import shutil
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
RUNS, LOGS = ROOT / "runs", ROOT / "logs"
OVERHEAD_S = 300          # per model: startup, dataset download, 2 checkpoint saves, hub push
SAFETY = 0.92             # use 92% of the computed step count (speed varies, saves take time)
ENV = dict(os.environ, TOKENIZERS_PARALLELISM="false", PYTORCH_CUDA_ALLOC_CONF="expandable_segments:True")


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def train_bin():
    exe = shutil.which("lerobot-train")
    return [exe] if exe else [sys.executable, "-m", "lerobot.scripts.lerobot_train"]


def model_cfg(cfg, name):
    m = cfg["models"][name]
    d = dict(cfg["defaults"][m["policy"]])
    d.update({k: v for k, v in m.items()})
    return d


def build_cmd(cfg, name, m, steps, bs, out_dir, calibrate=False):
    hw = cfg["hardware"]
    cmd = train_bin() + [
        f"--dataset.repo_id={m['dataset']}",
        f"--dataset.video_backend={cfg.get('video_backend', 'torchcodec')}",
        f"--dataset.return_uint8={str(m.get('return_uint8', True)).lower()}",
        f"--batch_size={bs}",
        f"--num_workers={hw['num_workers']}",
        f"--steps={steps}",
        f"--output_dir={out_dir}",
        f"--job_name={name}",
        "--policy.device=cuda",
        f"--accelerator.mixed_precision={hw['mixed_precision']}",
        "--wandb.enable=false",
        "--env_eval_freq=0",
        "--seed=1000",
    ]
    if m["policy"] == "smolvla":
        if m.get("pretrained"):
            cmd += [f"--policy.path={m['pretrained']}"]
        else:
            cmd += ["--policy.type=smolvla", f"--policy.load_vlm_weights={str(m.get('load_vlm_weights', True)).lower()}"]
        cmd += [f"--policy.optimizer_lr={m['lr']}",
                f"--policy.scheduler_warmup_steps={max(100, min(1000, steps // 20))}",
                f"--policy.scheduler_decay_steps={steps}"]
    else:  # act
        cmd += ["--policy.type=act", f"--policy.optimizer_lr={m['lr']}", f"--policy.optimizer_lr_backbone={m['lr']}"]
    if calibrate:
        cmd += ["--log_freq=10", "--save_checkpoint=false", "--policy.push_to_hub=false"]
    else:
        cmd += ["--log_freq=100", f"--save_freq={max(500, steps // 2)}", "--save_checkpoint_to_hub=true",
                "--policy.push_to_hub=true", f"--policy.repo_id={cfg['hf_user']}/{name}"]
    return cmd


def kill_group(p):
    try:
        os.killpg(p.pid, signal.SIGTERM)
        p.wait(timeout=20)
    except Exception:
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except Exception:
            pass


def calibrate(cfg, name, m):
    """Measure samples/s for this policy type. Returns (samples_per_s, batch_size_used)."""
    bs = m["batch_size"]
    while bs >= 4:
        cmd = build_cmd(cfg, name, m, steps=100, bs=bs, out_dir=RUNS / f"_calib_{name}", calibrate=True)
        shutil.rmtree(RUNS / f"_calib_{name}", ignore_errors=True)
        log(f"calibrating {m['policy']} on '{name}' with batch {bs} ...")
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=ENV,
                             start_new_session=True)
        sps, data_s, step_s, oom, tail, t0 = [], [], [], False, [], time.time()
        for line in p.stdout:
            tail = (tail + [line.rstrip()])[-25:]
            if "out of memory" in line.lower():
                oom = True
            for rx, acc in ((r"smp/s:(\d+)", sps), (r"data_s:([\d.]+)", data_s), (r"\bstep_s:([\d.]+)", step_s)):
                mt = re.search(rx, line)
                if mt and "step:" in line:
                    acc.append(float(mt.group(1)))
            if len(sps) >= 6 or time.time() - t0 > 20 * 60:
                break
        kill_group(p)
        shutil.rmtree(RUNS / f"_calib_{name}", ignore_errors=True)
        if oom:
            bs //= 2
            log(f"  out of memory -> retry with batch {bs}")
            continue
        if len(sps) < 3:
            print("\n".join(tail))
            raise SystemExit(f"Calibration failed for '{name}' (see the last log lines above). Fix this before spending GPU hours.")
        v = statistics.median(sps[2:])          # skip the first (warm-up) windows
        d = statistics.median(data_s[2:]) if len(data_s) > 2 else 0
        s = statistics.median(step_s[2:]) if len(step_s) > 2 else 0
        log(f"  {m['policy']}: {v:.0f} samples/s at batch {bs} (step {s:.2f}s, of which data wait {d:.2f}s)")
        if s and d / s > 0.4:
            log("  !! data loading is >40% of the step: GPU is waiting for video decoding. "
                "Raise hardware.num_workers (max ~14) or check the video codec (prepare.py prints it).")
        return v, bs
    raise SystemExit("Even batch 4 runs out of memory: something else is using the GPU (nvidia-smi).")


def last_ckpt(out_dir):
    c = out_dir / "checkpoints" / "last" / "pretrained_model" / "train_config.json"
    return c if c.exists() else None


def run_logged(cmd, logfile):
    log("RUN " + " ".join(shlex.quote(c) for c in cmd))
    with open(logfile, "a") as lf:
        p = subprocess.Popen(cmd, stdout=lf, stderr=subprocess.STDOUT, env=ENV, start_new_session=True)
        try:
            return p.wait()
        except KeyboardInterrupt:
            kill_group(p)
            raise


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="train_models.yaml")
    ap.add_argument("--budget-hours", type=float, required=True,
                    help="wall-clock hours available for the whole run (calibration included)")
    ap.add_argument("--models", default="", help="comma list; default = all")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    cfg = yaml.safe_load(open(ROOT / a.config))
    plan = json.loads((ROOT / "plan.json").read_text()) if (ROOT / "plan.json").exists() else None
    if plan is None:
        raise SystemExit("Run `python prepare.py` first.")
    if "<" in cfg["hf_user"]:
        raise SystemExit("Set hf_user in train_models.yaml")
    names = [n for n in (a.models.split(",") if a.models else cfg["models"]) if n]
    for n in names:
        if n not in cfg["models"]:
            raise SystemExit(f"unknown model '{n}'")
    RUNS.mkdir(exist_ok=True)
    LOGS.mkdir(exist_ok=True)
    state_f = RUNS / "state.json"
    state = json.loads(state_f.read_text()) if state_f.exists() else {}
    t_start = time.time()
    deadline = t_start + a.budget_hours * 3600

    if a.dry_run:
        for n in names:
            m = model_cfg(cfg, n)
            print(" ".join(shlex.quote(c) for c in build_cmd(cfg, n, m, steps=10000, bs=m["batch_size"], out_dir=RUNS / n)))
        return

    subprocess.run(["nvidia-smi", "--query-gpu=name,memory.used,memory.total", "--format=csv"], check=False)
    speed = {}
    todo = [n for n in names if state.get(n, {}).get("status") != "done"]
    for pol in dict.fromkeys(cfg["models"][n]["policy"] for n in todo):
        first = next(n for n in todo if cfg["models"][n]["policy"] == pol)
        speed[pol] = calibrate(cfg, first, model_cfg(cfg, first))

    for i, n in enumerate(todo):
        m = model_cfg(cfg, n)
        sps, bs = speed[m["policy"]]
        remaining = deadline - time.time()
        weights = [model_cfg(cfg, x)["weight"] for x in todo[i:]]
        slot = remaining * m["weight"] / sum(weights)
        steps = int((slot - OVERHEAD_S) * SAFETY * sps / bs)
        steps = max(m["min_steps"], min(m["max_steps"], steps // 100 * 100))
        frames = plan[n]["frames"]
        log(f"=== {n} ({m['policy']}): slot {slot/60:.0f} min -> {steps} steps x batch {bs} "
            f"= {steps*bs/frames:.1f} epochs over {frames} frames; est. {steps*bs/sps/60:.0f} min")
        if steps * bs / sps + OVERHEAD_S > remaining:
            log("  !! minimum steps do not fit in the remaining budget; continuing anyway (raise --budget-hours or lower min_steps)")
        out = RUNS / n
        ck = last_ckpt(out)
        if ck:
            cmd = train_bin() + [f"--config_path={ck}", "--resume=true"]
            log(f"  resuming from {ck}")
        else:
            shutil.rmtree(out, ignore_errors=True)
            cmd = build_cmd(cfg, n, m, steps, bs, out)
        t0 = time.time()
        rc = run_logged(cmd, LOGS / f"{n}.log")
        if rc != 0:
            state[n] = {"status": "failed", "rc": rc}
            state_f.write_text(json.dumps(state, indent=2))
            log(f"!! {n} failed (rc={rc}); see logs/{n}.log. Continuing with the next model; re-run the script to retry.")
            continue
        state[n] = {"status": "done", "steps": steps, "minutes": round((time.time() - t0) / 60, 1),
                    "repo": f"{cfg['hf_user']}/{n}"}
        state_f.write_text(json.dumps(state, indent=2))
        # everything the deployment config needs for this model
        tm_f = RUNS / "trained_models.json"
        tm = json.loads(tm_f.read_text()) if tm_f.exists() else {}
        tm[n] = {"policy": m["policy"], "repo": f"{cfg['hf_user']}/{n}", "task": plan[n]["tasks"][0],
                 "duration_s": plan[n]["run_duration_s"], "fps": plan[n]["fps"], "cameras": plan[n]["cameras"]}
        tm_f.write_text(json.dumps(tm, indent=2))
        log(f"=== {n} done in {state[n]['minutes']} min. Total elapsed {(time.time()-t_start)/3600:.2f} h")

    log("ALL DONE: " + json.dumps({k: v.get("status") for k, v in state.items()}))
    log("Now STOP the Brev instance (console -> Stop) so credits stop being spent. Models are on the Hub.")


if __name__ == "__main__":
    main()
