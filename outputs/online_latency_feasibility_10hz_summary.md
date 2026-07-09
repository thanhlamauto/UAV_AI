# Online Latency Feasibility: 10 Hz 3D LiDAR -> ESDF -> MPPI

This is a narrow online timing experiment, not a full AvoidBench/PX4 run.
The latency budget is worst-case LiDAR period + occupancy/ESDF update + MPPI compute + command publish.

Warm-runtime map update samples ms: min=5.768, median=5.851, max=6.200.

| Sensor Hz | UAV speed | Map update ms | MPPI ms | Total delay ms | Delay distance m | Min body clearance m | Collision | Safety violation |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 | 1.0 | 5.9 | 28.9 | 144.7 | 0.145 | 0.389 | 0 | 0 |
| 10 | 2.0 | 5.9 | 87.4 | 203.3 | 0.407 | 0.268 | 0 | 0 |
| 10 | 3.0 | 5.9 | 187.8 | 303.7 | 0.911 | 0.523 | 0 | 0 |
| 10 | 4.0 | 5.9 | 184.5 | 300.3 | 1.201 | 0.403 | 0 | 0 |
| 10 | 5.0 | 5.9 | 205.5 | 321.4 | 1.607 | 0.125 | 0 | 1 |
| 10 | 6.0 | 5.9 | 184.6 | 300.4 | 1.803 | -0.015 | 1 | 1 |

Interpretation: at 10 Hz, the LiDAR period dominates the fixed part of the delay.
At 2 m/s, the measured stack delay moves the UAV roughly 0.4 m before the new MPPI command can take effect.
At 5 m/s, the UAV already enters the safety radius during the delay; at 6 m/s, the body-clearance estimate crosses zero.
The reported buffer columns in the CSV quantify how much distance remains before a straight-line command would enter the safety radius or body contact.
