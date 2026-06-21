# Multi-LiDAR Download Link Probe

Purpose: check whether the SharePoint-hosted Multi-LiDAR Multi-UAV samples are currently reachable before treating them as downloadable stress-test data.

- Probed links: 27
- Direct download-ready links: 0
- Login-required links: 27
- Download-ready hard-sequence links: 0

## Recommended Use

- Use Multi-LiDAR as UAV perception/tracking generalization evidence only.
- Do not replace ODA as the primary obstacle-avoidance benchmark.
- If SharePoint links fail, cite the dataset page and keep this as a documented access blocker rather than spending the main project budget on data wrangling.

## Hard Sequences To Try First

- `Autel05`: `login_required` (HTTP `200`), unstructured indoor
- `AutelOut01`: `login_required` (HTTP `200`), unstructured outdoor
- `AutelOut02`: `login_required` (HTTP `200`), unstructured outdoor
- `Tello03`: `login_required` (HTTP `200`), unstructured indoor
- `Tello04`: `login_required` (HTTP `200`), unstructured indoor
- `Tello05`: `login_required` (HTTP `200`), unstructured indoor
- `TelloOut01`: `login_required` (HTTP `200`), unstructured outdoor
- `TelloOut02`: `login_required` (HTTP `200`), unstructured outdoor
