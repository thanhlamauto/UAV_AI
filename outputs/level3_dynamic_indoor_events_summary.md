# Level 3 Dynamic Indoor Events Video

Video:
- `outputs/videos/level3_dynamic_indoor_events_esdf_mppi.mp4`

Metrics:
- `outputs/tables/level3_dynamic_indoor_events_mppi.csv`

Scenario:

```text
indoor corridor/lab
  -> static furniture/walls/pillars
  -> surprise person crossing
  -> door panel closing
  -> cart/box appearing
  -> voxel map / ESDF update
  -> MPPI replanning in x,y,z
```

Stage metrics:

| Stage | Event | Min ESDF [m] | Compute [ms] | Safety violation |
|---|---|---:|---:|---:|
| Baseline | Nominal indoor corridor | 0.4280 | 70.443 | 0 |
| Event 1 | Person suddenly crosses the flight corridor | 0.4588 | 26.704 | 0 |
| Event 2 | Door panel narrows the passage | 0.4197 | 27.867 | 0 |
| Event 3 | Cart/box appears near the old route | 0.4780 | 26.412 | 0 |

Scope note: this is a simulated dynamic indoor Level-3 visualization.
It demonstrates online replanning against changing 3D occupancy/ESDF maps,
but it is not yet a physical robot log or PX4 closed-loop flight.
