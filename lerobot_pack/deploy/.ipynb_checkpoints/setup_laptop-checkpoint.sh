#!/usr/bin/env bash
# Laptop (Ubuntu) setup for deployment. Python >= 3.12 required. Run once:  bash setup_laptop.sh
set -euo pipefail
LEROBOT_SHA="146d8474932c8b566717eb2738ba459337ab71c4"     # same commit as the training box
command -v uv >/dev/null || { curl -LsSf https://astral.sh/uv/install.sh | sh; export PATH="$HOME/.local/bin:$PATH"; }
uv python install 3.12
uv venv --python 3.12 "$HOME/lr-deploy"
source "$HOME/lr-deploy/bin/activate"
sudo apt-get install -y ffmpeg portaudio19-dev   # video + microphone
uv pip install "lerobot[feetech,remote,core_scripts] @ git+https://github.com/huggingface/lerobot.git@${LEROBOT_SHA}" \
    jupyterlab ipywidgets pyserial pyyaml sounddevice faster-whisper opencv-python mediapipe
sudo usermod -aG dialout "$USER" || true          # serial ports; log out/in once
echo "Done. Next:  source ~/lr-deploy/bin/activate && jupyter lab   (open control_panel.ipynb)"
