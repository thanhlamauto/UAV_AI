#!/usr/bin/env bash
set -euo pipefail

OUTPUT="${1:-data/raw/Dupeyroux_et_al_2021_ODA_DATASET_Full.zip}"
URL="https://data.4tu.nl/file/b74ea8f4-b20e-4b3e-9399-826a7953475d/a1cd3fba-09c4-4c84-a3e4-2daf502b18e4"
EXPECTED_MD5="189639db8176ccdbd728b88d99c27309"
EXPECTED_BYTES=98186579073
SPACE_SLACK_BYTES=$((1024 * 1024 * 1024))

file_size_bytes() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo 0
    return
  fi
  if stat -f%z "${path}" >/dev/null 2>&1; then
    stat -f%z "${path}"
  else
    stat -c%s "${path}"
  fi
}

available_bytes_for_path() {
  local path="$1"
  local dir
  dir="$(dirname "${path}")"
  mkdir -p "${dir}"
  df -Pk "${dir}" | awk 'NR == 2 {print $4 * 1024}'
}

human_gib() {
  awk -v bytes="$1" 'BEGIN {printf "%.1f GiB", bytes / 1024 / 1024 / 1024}'
}

mkdir -p "$(dirname "${OUTPUT}")"

PARTIAL_BYTES="$(file_size_bytes "${OUTPUT}")"
REMAINING_BYTES=$((EXPECTED_BYTES - PARTIAL_BYTES))
if (( REMAINING_BYTES < 0 )); then
  REMAINING_BYTES=0
fi
FREE_BYTES="$(available_bytes_for_path "${OUTPUT}")"
NEEDED_BYTES=$((REMAINING_BYTES + SPACE_SLACK_BYTES))

echo "ODA full ZIP target: ${OUTPUT}"
echo "Expected archive size: $(human_gib "${EXPECTED_BYTES}") (${EXPECTED_BYTES} bytes)"
echo "Existing partial size: $(human_gib "${PARTIAL_BYTES}") (${PARTIAL_BYTES} bytes)"
echo "Available space: $(human_gib "${FREE_BYTES}")"

if [[ "${ODA_SKIP_SPACE_CHECK:-0}" != "1" ]] && (( FREE_BYTES < NEEDED_BYTES )); then
  echo "Not enough free space for the remaining download." >&2
  echo "Need at least $(human_gib "${NEEDED_BYTES}") including 1 GiB slack, but only $(human_gib "${FREE_BYTES}") is available." >&2
  echo "Free space, choose another output drive/path, or set ODA_SKIP_SPACE_CHECK=1 to override." >&2
  exit 2
fi

echo "Downloading ODA full ZIP to ${OUTPUT}"
echo "Expected size is about 98 GB. This may take a long time."
curl -L -C - --fail --output "${OUTPUT}" "${URL}"

echo "Checking MD5..."
if command -v md5 >/dev/null 2>&1; then
  ACTUAL_MD5="$(md5 -q "${OUTPUT}")"
elif command -v md5sum >/dev/null 2>&1; then
  ACTUAL_MD5="$(md5sum "${OUTPUT}" | awk '{print $1}')"
else
  echo "No md5/md5sum command available; skipping checksum."
  exit 0
fi

if [[ "${ACTUAL_MD5}" != "${EXPECTED_MD5}" ]]; then
  echo "Checksum mismatch: got ${ACTUAL_MD5}, expected ${EXPECTED_MD5}" >&2
  exit 1
fi

echo "Checksum OK."
