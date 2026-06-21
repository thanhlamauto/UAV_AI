#!/usr/bin/env python3
"""Unwrap the 4TU ODA download when it contains a nested dataset ZIP."""

from __future__ import annotations

import argparse
import shutil
import zipfile
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("zip_path", help="Downloaded ODA ZIP path")
    parser.add_argument(
        "--output",
        default=None,
        help="Output path for the inner dataset ZIP. Defaults to a sibling file.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    zip_path = Path(args.zip_path)

    with zipfile.ZipFile(zip_path) as zf:
        files = [info for info in zf.infolist() if not info.is_dir()]
        if len(files) != 1 or not files[0].filename.lower().endswith(".zip"):
            print(zip_path)
            return

        inner = files[0]
        output = Path(args.output) if args.output else zip_path.parent / Path(inner.filename).name
        if output.exists() and output.stat().st_size == inner.file_size:
            print(output)
            return

        output.parent.mkdir(parents=True, exist_ok=True)
        tmp = output.with_suffix(output.suffix + ".tmp")
        if tmp.exists():
            tmp.unlink()

        free_bytes = shutil.disk_usage(output.parent).free
        if free_bytes < inner.file_size + 1024 * 1024 * 1024:
            raise SystemExit(
                "Not enough free space to unwrap nested ODA ZIP: "
                f"need {inner.file_size} bytes plus 1 GiB slack, have {free_bytes} bytes"
            )

        print(f"Extracting nested ZIP {inner.filename} -> {output}")
        with zf.open(inner) as src, tmp.open("wb") as dst:
            shutil.copyfileobj(src, dst, length=16 * 1024 * 1024)
        tmp.rename(output)
        print(output)


if __name__ == "__main__":
    main()
