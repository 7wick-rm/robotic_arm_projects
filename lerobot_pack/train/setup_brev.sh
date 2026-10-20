#!/usr/bin/env bash
# Run ONCE on the Brev instance (inside `brev shell <instance>`), from the folder holding these files.
#   bash setup_brev.sh
# Installs the same LeRobot commit the scripts were checked against (0.6.2, commit below) so that
# training here and deployment on your laptop use matching code.
set -euo pipefail

LEROBOT_SHA="146d8474932c8b566717eb2738ba459337ab71c4"   # LeRobot main, 2026-10-16, version 0.6.2
VENV="$HOME/lr-venv"

echo "== 1/6 GPU check"
nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv
# This is an Isaac Launchable: if Isaac Sim is running it can hold GPU memory. Training needs ~30+ GB free.
USED=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -1)
if [ "${USED:-0}" -gt 4000 ]; then
  echo "WARNING: ${USED} MiB of GPU memory already in use. See processes below; stop Isaac Sim/other jobs if not needed:"
  nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
fi

echo "== 2/6 system packages (ffmpeg for video decoding, tmux for long runs)"
if ! command -v ffmpeg >/dev/null || ! command -v tmux >/dev/null; then
  sudo apt-get update -y && sudo apt-get install -y ffmpeg tmux git
fi

echo "== 3/6 python 3.12 venv (uv)"
if ! command -v uv >/dev/null; then
  curl -LsSf https://astral.sh/uv/install.sh | sh
  export PATH="$HOME/.local/bin:$PATH"
fi
uv python install 3.12
[ -d "$VENV" ] || uv venv --python 3.12 "$VENV"
# shellcheck disable=SC1091
source "$VENV/bin/activate"

echo "== 4/6 install LeRobot (smolvla + training extras) at the pinned commit"
uv pip install "lerobot[smolvla,training,dataset] @ git+https://github.com/huggingface/lerobot.git@${LEROBOT_SHA}" pyyaml

echo "== 5/6 sanity checks"
python - <<'EOF'
import torch, lerobot
print("lerobot", lerobot.__version__, "| torch", torch.__version__, "| cuda", torch.cuda.is_available(),
      "|", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "NO GPU")
assert torch.cuda.is_available(), "CUDA not available - check the driver / instance"
try:
    import torchcodec  # noqa
    print("torchcodec OK (default video backend)")
except Exception as e:  # noqa
    print("torchcodec NOT usable:", e, "\n -> set video_backend: pyav in train_models.yaml")
EOF
which lerobot-train

echo "== 6/6 Hugging Face login (needs a WRITE token: https://huggingface.co/settings/tokens)"
if ! hf auth whoami >/dev/null 2>&1; then
  hf auth login
fi
hf auth whoami

cat <<EOF

Setup done. Next (every new shell):  source $VENV/bin/activate
  1) edit train_models.yaml   (your HF username + the 5 dataset repo ids)
  2) python prepare.py        (reads dataset metadata, prints the plan; ~1 min)
  3) tmux new -s train
     python train_all.py --budget-hours 5.5
EOF
