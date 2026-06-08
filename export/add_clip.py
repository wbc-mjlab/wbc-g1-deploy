#!/usr/bin/env python3
"""Copy an NPZ clip into config/clips and update manifest.yaml."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def load_manifest(path: Path) -> dict:
  if path.is_file():
    return yaml.safe_load(path.read_text()) or {}
  return {"clips": []}


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--motion-file", type=Path, required=True)
  parser.add_argument("--name", required=True, help="clip id in manifest")
  parser.add_argument(
    "--clips-dir",
    type=Path,
    default=Path("robots/g1/config/clips"),
    help="clips directory containing manifest.yaml",
  )
  parser.add_argument("--set-default", action="store_true")
  args = parser.parse_args()

  clips_dir = args.clips_dir.resolve()
  clips_dir.mkdir(parents=True, exist_ok=True)
  manifest_path = clips_dir / "manifest.yaml"

  dest_name = f"{args.name}.npz"
  dest_path = clips_dir / dest_name
  dest_path.write_bytes(args.motion_file.read_bytes())

  doc = load_manifest(manifest_path)
  clips = doc.setdefault("clips", [])
  clips = [c for c in clips if c.get("name") != args.name]
  clips.append({"name": args.name, "file": dest_name})
  doc["clips"] = clips
  if args.set_default or "default" not in doc:
    doc["default"] = args.name

  manifest_path.write_text(yaml.safe_dump(doc, sort_keys=False))
  print(f"Registered clip '{args.name}' → {dest_path}")
  print(f"Updated {manifest_path}")


if __name__ == "__main__":
  main()
