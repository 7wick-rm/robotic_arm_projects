"""
Upload the recorded dataset to a PRIVATE Hugging Face dataset repo, so Colab can download it (no Google Drive needed).
Resumable: if your connection drops, just run the same command again; finished files are not sent twice.

One-time setup:
    pip install -U huggingface_hub
    huggingface-cli login            # paste a token with WRITE access (huggingface.co -> Settings -> Access Tokens)

Usage (from the repo root):
    python upload_dataset_hf.py --repo <your-username>/garage_pnp
    python upload_dataset_hf.py --repo <your-username>/garage_pnp --max_episodes 500    # smaller upload: first 500 episodes only

Only meta/ and data/*.parquet are uploaded (the images are inside the parquet files).
"""
import argparse
import sys
from pathlib import Path

from huggingface_hub import HfApi


def build_patterns(max_episodes=None):
    if max_episodes is None:
        return ["meta/*", "data/*/*.parquet"]
    return ["meta/*"] + [f"data/*/episode_{i:06d}.parquet" for i in range(max_episodes)]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--repo", required=True, help="<your-username>/garage_pnp")
    p.add_argument("--folder", default="./demo_data_garage")
    p.add_argument("--max_episodes", type=int, default=None, help="upload only the first N episodes")
    p.add_argument("--token", default=None, help="only if you did not run huggingface-cli login")
    a = p.parse_args()

    folder = Path(a.folder)
    if not (folder / "meta" / "info.json").exists():
        sys.exit(f"{folder}/meta/info.json not found: is --folder the dataset folder?")
    n_files = len(list(folder.glob("data/*/*.parquet")))
    size_gb = sum(f.stat().st_size for f in folder.glob("data/*/*.parquet")) / 1e9
    print(f"{n_files} episode files, {size_gb:.1f} GB on disk")
    if a.max_episodes is not None and a.max_episodes > n_files:
        sys.exit(f"--max_episodes {a.max_episodes} is more than the {n_files} episodes you have")

    api = HfApi(token=a.token)
    print("logged in as:", api.whoami()["name"])
    api.create_repo(a.repo, repo_type="dataset", private=True, exist_ok=True)
    print(f"uploading to https://huggingface.co/datasets/{a.repo}  (private). This can take a while; it is resumable.")
    api.upload_large_folder(repo_id=a.repo, repo_type="dataset", folder_path=str(folder),
                            allow_patterns=build_patterns(a.max_episodes), num_workers=4)
    print("done. In the Colab notebook leave HF_DATASET_REPO = None (it uses <your-username>/garage_pnp) or set it to", a.repo)


if __name__ == "__main__":
    main()
