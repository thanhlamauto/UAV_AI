#!/usr/bin/env bash
set -euo pipefail

TARGET_DIR="${1:-data/raw/ODA_Dataset}"
SAMPLE_ID="${2:-345}"

if [[ ! -d "${TARGET_DIR}/.git" ]]; then
  "$(dirname "$0")/fetch_oda_minimal.sh" "${TARGET_DIR}"
fi

git -C "${TARGET_DIR}" sparse-checkout add "/dataset/${SAMPLE_ID}/${SAMPLE_ID}.avi"

echo "Fetched ODA RGB video sample ${SAMPLE_ID} into ${TARGET_DIR}/dataset/${SAMPLE_ID}/"
