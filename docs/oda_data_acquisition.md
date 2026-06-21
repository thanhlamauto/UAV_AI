# ODA Data Acquisition Notes

Checked again on 2026-06-21. The 4TU ODA Dataset record provides one full
archive:

- `Dupeyroux_et_al_2021_ODA_DATASET_Full.zip`
- size: `98,186,579,073` bytes
- MD5: `189639db8176ccdbd728b88d99c27309`
- DOI: <https://doi.org/10.4121/14214236.v1>

The server did not honor HTTP range requests during this run. A `Range: bytes=0-15`
request returned `HTTP/1.1 200 OK` with the full `Content-Length`, not
`206 Partial Content`, so individual trials could not be extracted remotely
without downloading the full archive.
Search results did not reveal a split-trial mirror. The GitHub repository only
includes sample trials `3`, `10`, and `345`.

Local feasibility check on 2026-06-21:

- no existing full/partial ODA ZIP was found in the workspace, Downloads,
  Documents, or Desktop search paths;
- the current filesystem has about `11 GiB` free;
- the full 4TU archive needs about `91.4 GiB` (`98,186,579,073` bytes) plus
  slack, so downloading it on the current disk is not feasible yet.

The download script now checks available space before starting `curl` and exits
early with a clear message if the target drive cannot hold the archive.

Prepared workflow:

```bash
python3 scripts/create_oda_20_trial_manifest.py
python3 scripts/check_oda_trial_readiness.py
# Optional, large download:
scripts/download_oda_full_zip.sh
# Or use a large external drive:
scripts/download_oda_full_zip.sh /Volumes/LargeDrive/Dupeyroux_et_al_2021_ODA_DATASET_Full.zip
# After the full ZIP is available locally:
python3 scripts/extract_oda_trials_from_full_zip.py /path/to/Dupeyroux_et_al_2021_ODA_DATASET_Full.zip
scripts/run_oda_target20_benchmark.sh
```

Selected target trials are balanced between one-obstacle and two-obstacle
full-light trials with RGB video flags and non-missing obstacle coordinates.
