# Qualitative Results

Generated wide-panel videos compare camera input, obstacle safety-distance map,
and OptiTrack MAV trajectory over time.

| sample | video | interpretation |
| --- | --- | --- |
| 3 | `outputs/videos/qualitative_sample_3.mp4` | One-obstacle case. The MAV enters the 0.50 m safety-clearance region; minimum boundary clearance is 0.435 m. |
| 345 | `outputs/videos/qualitative_sample_345.mp4` | Two-obstacle case. The MAV avoids both obstacles with minimum boundary clearance of 0.969 m. |
| 3 | `outputs/videos/qualitative_sensor_sample_3.mp4` | Five-panel qualitative result: RGB camera, 24 GHz radar FFT, 6-axis IMU, safety map, and OptiTrack trajectory. Shows a safety-distance violation. |
| 345 | `outputs/videos/qualitative_sensor_sample_345.mp4` | Five-panel qualitative result with the same RGB/radar/IMU/safety/trajectory panels for a two-obstacle trial. |
| 3 | `outputs/videos/qualitative_depth_sensor_sample_3.mp4` | Six-panel qualitative result: RGB, monocular predicted depth, radar FFT, IMU, safety map, and trajectory. Shows the safety-distance violation case. |
| 345 | `outputs/videos/qualitative_depth_sensor_sample_345.mp4` | Six-panel qualitative result with predicted depth plus radar/IMU for the two-obstacle trial. |

These videos are qualitative visualizations. The safety map is a geometric
risk map reconstructed from ODA obstacle metadata and the same safety-distance
threshold used in the metrics. The five-panel videos add real radar and IMU CSV
streams, but they still do not perform sensor fusion.

The six-panel videos add monocular relative-depth predictions from
`Intel/dpt-hybrid-midas` through Hugging Face Transformers. This is qualitative
relative depth only, not calibrated metric depth. The depth cache is stored as
compressed `.npz` files under `data/processed/`.

Reproduce:

```bash
scripts/fetch_oda_video_sample.sh data/raw/ODA_Dataset 3
scripts/fetch_oda_video_sample.sh data/raw/ODA_Dataset 345
scripts/fetch_oda_sensor_sample.sh data/raw/ODA_Dataset 3
scripts/fetch_oda_sensor_sample.sh data/raw/ODA_Dataset 345
python3 experiments/make_qualitative_video.py --trial-id 3 --output outputs/videos/qualitative_sample_3.mp4
python3 experiments/make_qualitative_video.py --trial-id 345 --output outputs/videos/qualitative_sample_345.mp4
python3 experiments/make_qualitative_video.py --trial-id 3 --include-sensors --output outputs/videos/qualitative_sensor_sample_3.mp4
python3 experiments/make_qualitative_video.py --trial-id 345 --include-sensors --output outputs/videos/qualitative_sensor_sample_345.mp4
python3 experiments/cache_monocular_depth.py --trial-id 3 --fps 5 --output data/processed/depth_sample_3_5fps.npz
python3 experiments/cache_monocular_depth.py --trial-id 345 --fps 5 --output data/processed/depth_sample_345_5fps.npz
python3 experiments/make_qualitative_video.py --trial-id 3 --include-depth --include-sensors --output outputs/videos/qualitative_depth_sensor_sample_3.mp4
python3 experiments/make_qualitative_video.py --trial-id 345 --include-depth --include-sensors --output outputs/videos/qualitative_depth_sensor_sample_345.mp4
```

Next predicted-depth step:

1. Validate whether the relative-depth response is temporally stable near
   obstacle approach.
2. Calibrate the panel as qualitative depth only at first, without claiming
   metric depth unless scale is validated.
3. Compare depth-derived near-obstacle cues against radar FFT changes and
   OptiTrack obstacle distance.
