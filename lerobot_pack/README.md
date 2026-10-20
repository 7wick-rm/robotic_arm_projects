# SO-101 + linear rail: training on Brev and deployment (LeRobot 0.6.2, v3 datasets)

```
train/                      -> runs ON the Brev GPU box
  setup_brev.sh             install LeRobot (pinned commit) + checks
  train_models.yaml         the 5 datasets, batch sizes, time weights   (you fill 6 values)
  prepare.py                reads dataset metadata: cameras, task strings, run durations  -> plan.json
  train_all.py              time-budgeted training of all 5 models, pushes each to the Hub
deploy/                     -> runs ON your laptop (Ubuntu, Python >= 3.12)
  control_panel.ipynb       the menu notebook (WS1/2/3, text / voice / vision)
  workspaces.yaml           ONE config: robot, cameras, rail, models, workspaces
  panel_core.py             rail driver, rollout runner, state machine, voice, gesture
  hardware/rail_esp32/      ESP32 firmware (+ sim/ = firmware simulator), hardware/test_rail.py
  server/make_server_yaml.py  (Brev box) policy-server configs for remote inference
  setup_laptop.sh, tests/, hand_landmarker.task
```

## 0. RUN ORDER (models already trained: skip `train/` and section 1)

Put this folder next to your old project, e.g. `~/robotic_arm_projects/lerobot_pack`. Do NOT use the old `lerobot-sim` env (Python 3.10, old LeRobot); nothing from the MuJoCo project (`sim_server.py`, `gesture_command.py`, `9_multimodal_control.ipynb`, `ckpt/smolvla_garage`) is used.

| # | Where | Do | File |
|---|---|---|---|
| 1 | laptop | `bash setup_laptop.sh` (new Python 3.12 env `~/lr-deploy`), log out/in once | `deploy/setup_laptop.sh` |
| 2 | laptop (Arduino IDE) | **Flash the ESP32 with `deploy/hardware/rail_esp32/rail_esp32.ino` only.** Delete/ignore the `rail_esp32.ino` in your project root (older copy). Move the MIN switch wire GPIO12 -> GPIO27 first | `deploy/hardware/rail_esp32/rail_esp32.ino` |
| 3 | laptop | `lerobot-find-port` (arm), `ls /dev/ttyUSB* /dev/ttyACM*` (ESP32), `lerobot-find-cameras opencv` | |
| 4 | laptop | test the rail alone: `python deploy/hardware/test_rail.py --port /dev/ttyUSB0 cycle` (e-stop in hand) | `deploy/hardware/test_rail.py` |
| 5 | laptop | read what each model expects: `python deploy/tools/model_info.py <model>/pretrained_model ...` -> camera names for the yaml | `deploy/tools/model_info.py` |
| 6 | laptop | fill `deploy/workspaces.yaml`: robot port + **id used when recording**, cameras, `rest_pose`, rail port/positions, and per model `path:` (placeholders `<PATH_TO_..._MODEL>`) + `task:` (exact training string) + `duration_s` | `deploy/workspaces.yaml` |
| 7 | laptop | `cd deploy && python tests/test_core.py` (should print OK), then `jupyter lab` -> `control_panel.ipynb` with `mock: true`, run all cells, click through the menus | `deploy/control_panel.ipynb` |
| 8 | Brev box (only for `backend: remote`) | copy the 5 model folders: `brev copy <model>/pretrained_model isaac-launchable-70b6b8:~/models/<id>/pretrained_model` for ids `screw hammer wrench burger thor_hammer`; run `setup_brev.sh` once (train/ folder), then `python make_server_yaml.py`, then `bash ~/serve/run_server.sh hammer` until "Policy server ready" (Ctrl-C) | `train/setup_brev.sh`, `deploy/server/make_server_yaml.py` |
| 9 | laptop | set `mock: false`; first real run: WS1 (ACT) with `max_relative_target: 5`, hand on the e-stops | notebook |

Without a GPU on the laptop SmolVLA must use `backend: remote` (step 8, Brev instance Running = $3.65/h during the demo). With a local GPU use `backend: sync`, `device: cuda` and skip step 8.

## 1. (Only if you still need to train) Spending the Brev credits

Your instance: **L40S 48 GB, 16 vCPU, $3.65/h**. Assuming 1 credit = $1 (confirm under Billing): 40 credits = **10.9 GPU-hours**.

* **It bills while "Running", trained or not.** The console showed it created at 21:49 on 17/10. If you are not about to train, press **Stop** now.
* Plan so you finish with a reserve: `--budget-hours = min(time you have left, credits/3.65) - 0.7`. Examples: 7 h left -> `6.0`; 4 h left -> `3.2`. The script divides that time between the 5 models by the weights in `train_models.yaml` (ACT 0.8, each SmolVLA 1.0), measures real speed first, and picks the steps that fit.
* **Option B (uses the credits fully): two instances.** Cost is per GPU-hour, so 2 instances x 4.5 h = $32.9 gives ~9 GPU-hours instead of 7. Create a second L40S, run setup on both, then
  instance 1: `python train_all.py --budget-hours 4.3 --models screw,hammer,wrench`
  instance 2: `python train_all.py --budget-hours 4.3 --models burger,thor_hammer`
  (both need the same yaml + `prepare.py`). Finishes in 4.5 h wall-clock and leaves ~2 h for deployment testing.
* Switch to a faster GPU only if Brev lists one at <= ~$5.5/h and you can set it up in < 15 min; otherwise stay on this one.

What the scripts tuned for this GPU (all verified against LeRobot 0.6.2 source): `bf16` mixed precision, `num_workers=12` of 16 vCPUs, uint8 frames, batch 32 (more updates per hour than 64), cosine schedule decayed over *your* step count (the default 30k-step decay would never finish in a short run), mid-run checkpoint pushed to the Hub, auto-resume after a crash, no wandb/eval overhead. SmolVLA is trained as `--policy.type=smolvla` with the SmolVLM2 backbone weights loaded and the action expert trained from scratch (not `smolvla_base`), so it needs more steps than a fine-tune: it will use the whole slot (up to 40k steps).

### Steps (on your laptop, then on the box)
```bash
curl -fsSL https://raw.githubusercontent.com/brevdev/brev-cli/main/bin/install-latest.sh | bash   # or the one-liner from the console
brev login
brev shell isaac-launchable-70b6b8                      # terminal on the GPU box
```
Get `train/` onto the box (pick one): `brev copy ./train isaac-launchable-70b6b8:~/train` (check `brev copy --help`), or `git clone` a private repo, or paste the files.
```bash
cd ~/train && bash setup_brev.sh                        # ~5 min; asks for your HF WRITE token
source ~/lr-venv/bin/activate
nano train_models.yaml                                  # hf_user + 5 dataset repo ids
python prepare.py                                       # check: v3, same camera names, task strings, durations
tmux new -s train                                       # survives a dropped connection (re-attach: tmux a -t train)
python train_all.py --budget-hours 6.0
```
Watch: `tail -f logs/hammer.log`. Healthy log line: `step:... loss:... smp/s:~100 ... data_s:<0.1`. The calibration step (first ~4 min) prints the measured speed, the planned steps/epochs per model, and **fails fast** if a flag/dataset is wrong. If it prints "data loading is >40% of the step", raise `num_workers` to 14.

When it prints `ALL DONE`: models are on the Hub as `<hf_user>/<name>`. **Stop the instance in the console.** `runs/trained_models.json` holds repo, task string and duration for each model: copy them into `deploy/workspaces.yaml` (`models:`).

## 2. Deployment on the laptop
1. `bash setup_laptop.sh` (Python 3.12 env, same LeRobot commit, mic/vision packages). Log out/in once (serial group).
2. **Rail firmware** (`hardware/rail_esp32/rail_esp32.ino`, Arduino IDE, ESP32 + AccelStepper): **move the MIN switch wire from GPIO12 to GPIO27** (GPIO12 is a boot-strapping pin: a pressed/broken NC switch at power-up can stop the ESP32 booting), or set `LIMIT_MIN_NC_PIN` back to 12 at your own risk. The firmware never moves on its own: it waits for `HOME`.
3. Test the rail alone: `python hardware/test_rail.py --port /dev/ttyUSB0 cycle` (homes first, then 0 -> 50 -> 100 -> 50 -> 0 %). Hand on the e-stop. Check that "100 %" is the far end; else flip `ENCODER_DIR`/wiring.
4. Fill `workspaces.yaml`: robot port/id (same `id` as recording!), cameras (indices + names from `prepare.py`), `rest_pose` (measure it), ports, models. Camera names and `use_degrees` must equal the training data.
5. `jupyter lab` -> open `control_panel.ipynb`. First run with `mock: true` (whole menu, no hardware). Then `mock: false`.
6. First real runs: `max_relative_target: 5`, hand on the arm e-stop, run workspace 1 (ACT) first.

**SmolVLA needs a GPU at run time.** Set `inference.backend: remote`: the notebook SSHes to the Brev box, starts `lerobot-policy-server` for the chosen model and tunnels port 7447 (details in `server/make_server_yaml.py`; run it once on the box after training, then `bash ~/serve/run_server.sh hammer` by hand to see "Policy server ready"). The instance must be **Running** during the demo (3.65/h, keep ~1 h of credits for it). For a local GPU use `backend: sync`, `device: cuda`.

## 3. Not verified (no hardware here), check first
* Remote inference through the SSH tunnel (Zenoh over `tcp/127.0.0.1:7447`) and the semantics label; fall back to `sync` on a GPU machine if it fails.
* `rollout`'s "waiting for /start" text is detected by the substring `/start`; if the load wait times out, raise `inference.load_timeout_s`.
* `rest_pose` values and gravity-safety of that pose (`disable_torque_on_disconnect=false` keeps torque on between runs).
* Firmware verified only in simulation (`hardware/rail_esp32/sim`): check `HOME_SPEED`, `HOME_TIMEOUT_MS` (a full-length seek must finish inside 60 s) for your microstepping.
* Vision uses the laptop webcam and finger count (1 = hammer, 2 = wrench); `gesture.py` was not in your upload, so this is a rewrite on MediaPipe + your `hand_landmarker.task`.
