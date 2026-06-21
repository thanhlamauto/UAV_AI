#!/usr/bin/env bash
set -euo pipefail

REPO_URL="https://github.com/JuSquare/ODA_Dataset.git"
TARGET_DIR="${1:-data/raw/ODA_Dataset}"

if [[ ! -d "${TARGET_DIR}/.git" ]]; then
  mkdir -p "$(dirname "${TARGET_DIR}")"
  git clone --depth 1 --filter=blob:none --sparse "${REPO_URL}" "${TARGET_DIR}"
fi

git -C "${TARGET_DIR}" sparse-checkout init --no-cone
git -C "${TARGET_DIR}" sparse-checkout set \
  /Readme.md \
  /dataset/trial_overview.csv \
  /dataset/3/optitrack.csv \
  /dataset/10/optitrack.csv \
  /dataset/345/optitrack.csv

echo "Fetched minimal ODA metadata and OptiTrack CSV samples into ${TARGET_DIR}"
