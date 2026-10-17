"""
Upload a LeRobot dataset folder to a PRIVATE Hugging Face dataset repo, so Colab can download it.

Run on your laptop, from the tutorial repo root, with the (lerobot-sim) venv active:

    huggingface-cli login                      # once; paste a token with WRITE access
    python upload_dataset_to_hf.py --repo YOUR_USERNAME/garage_pnp

The upload is resumable: if your internet drops or you press Ctrl-C, run the same command again
and it continues where it stopped (progress is kept in <folder>/.cache/huggingface).
"""
import argparse
import json
import sys
from pathlib import Path

from huggingface_hub import HfApi


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--repo", required=True, help="e.g. myname/garage_pnp")
    p.add_argument("--folder", default="./demo_data_garage")
    p.add_argument("--public", action="store_true", help="make the repo public (default: private)")
    args = p.parse_args()

    root = Path(args.folder)
    info_path = root / "meta" / "info.json"
    if not info_path.exists():
        sys.exit(f"{info_path} not found. Is --folder the dataset folder (the one with data/ and meta/)?")

    # Do not spend hours uploading a dataset that will not load.
    info = json.loads(info_path.read_text())
    n_files = len(list((root / "data").rglob("episode_*.parquet")))
    n_meta = len((root / "meta" / "episodes.jsonl").read_text().splitlines())
    print(f"info.json says {info.get('total_episodes')} episodes, episodes.jsonl has {n_meta}, "
          f"{n_files} parquet files on disk")
    if not (n_files == n_meta == info.get("total_episodes")):
        sys.exit("These numbers disagree. Run `python repair_dataset.py` first (dry run), fix it, then upload.")

    total_gb = sum(f.stat().st_size for f in root.rglob("*") if f.is_file()
                   and ("data" in f.parts or "meta" in f.parts)) / 1e9
    print(f"About to upload {total_gb:.1f} GB (data/ and meta/ only) to {args.repo} "
          f"({'PUBLIC' if args.public else 'private'}).")

    api = HfApi()
    api.create_repo(repo_id=args.repo, repo_type="dataset", private=not args.public, exist_ok=True)
    patterns = ["data/**", "meta/**"]               # skips images/, backups and caches
    if hasattr(api, "upload_large_folder"):         # present in huggingface_hub 0.36.2, the version your venv pins
        api.upload_large_folder(
            repo_id=args.repo,
            repo_type="dataset",
            folder_path=str(root),
            allow_patterns=patterns,
        )
    else:
        print("This huggingface_hub has no upload_large_folder; falling back to upload_folder "
              "(not resumable). Prefer running this in the lerobot-sim venv.")
        api.upload_folder(
            repo_id=args.repo,
            repo_type="dataset",
            folder_path=str(root),
            allow_patterns=patterns,
        )
    print(f"\nDone. In the Colab notebook set HF_DATASET = '{args.repo}'")


if __name__ == "__main__":
    main()
