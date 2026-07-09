# Defense Narrative

## One-sentence Positioning

This project is not claiming a new planner algorithm; its contribution is a measured UAV obstacle-avoidance pipeline that turns ODA ground truth and multi-sensor cues into safety metrics, planner comparison, perception-risk analysis, and perception-to-planner demonstrations.

## Slide Flow

1. Problem: indoor/GNSS-denied UAV obstacle avoidance needs measurable safety evidence, not only a visual demo.
2. Dataset: ODA is the main benchmark because it contains MAV trajectory, obstacle metadata, RGB, radar, IMU and OptiTrack ground truth.
3. Benchmark: 300 ODA trials produce 2100 planner metric rows across human, straight-line, geometric bypass, A*, RRT, RRT* and MPPI.
4. Main result: A*/RRT/RRT* reach 0 collision and 0 safety violation in the current 2D geometric benchmark; MPPI reaches 0 collision and 0.0033 safety-violation rate with the highest mean clearance.
5. Research angle: once geometry is solvable with a known obstacle map, the hard part shifts to sensing, mapping and control.
6. Perception-risk: Depth Anything V2 Small + radar + IMU gives a future-risk baseline on 50 trials / 2584 frames; relative depth is treated as a cue, not metric distance.
7. Perception-to-planner: LiDAR bbox, metric depth, relative depth and fused costmaps are converted to the same occupancy-grid contract consumed by A*/RRT/MPPI.
8. 3D extension: voxel/ESDF + MPPI shows the path-planning direction toward continuous `[x,y,z]` UAV motion, while ROS2/Gazebo/Isaac/nvBlox artifacts provide integration evidence.
9. Limitations: no TensorRT engine claim yet, no monocular metric-depth claim yet, and no full PX4/Gazebo closed-loop claim yet.
10. Next step: one strong PX4/Gazebo closed-loop fused-costmap recording would most directly improve the completeness score.

## Suggested Opening

Em bắt đầu từ yêu cầu của mentor là không làm khảo sát rộng mà phải có kết quả đo được. Vì vậy em chọn ODA làm benchmark chính, dựng pipeline từ ground truth và sensor data sang metric an toàn, rồi dùng cùng điều kiện start/goal/obstacle để so sánh các planner. Sau đó em mở rộng sang perception-risk và perception-to-planner để chứng minh dữ liệu cảm biến có thể đi vào planner chứ không chỉ dùng để minh họa.

## Three Contributions

1. ODA benchmark: 300 trial, 2100 metric rows, có collision, safety violation, clearance, path length, smoothness và compute time.
2. Perception-risk: tạo feature depth/radar/IMU và future-risk label, có threshold tuning cho recall khi cần cảnh báo sớm.
3. Integration evidence: LiDAR/depth/fused perception outputs được đưa về occupancy grid, kiểm tra 5 nguồn perception x 3 planner; bổ sung 3D ESDF/MPPI và ROS2/Gazebo/Isaac/nvBlox artifacts.

## Key Metrics For Slides

Use `outputs/tables/defense_key_metrics.csv` as the compact metric table. The most useful numbers to say aloud are: 300/300 ODA trials, 2100 planner metric rows, A*/RRT/RRT* at 0 collision and 0 safety violation in the current 2D benchmark, MPPI at 0 collision and 0.0033 safety-violation rate, 2584 perception-risk frames, recall-tuned depth+radar+IMU risk recall 0.6667, and 15/15 local perception-planner matrix cases collision-free after inflation.

## Safe Claims

- Có thể claim: benchmark ODA 300 trial, planner metrics, relative-depth/radar/IMU risk baseline, point-cloud 3D bbox, local perception-to-planner contract, 3D ESDF/MPPI demo, ROS2/Gazebo fused costmap evidence.
- Không nên claim: monocular depth là metric depth theo mét, TensorRT speedup thật, full PX4/Gazebo closed-loop, direct planner query toàn bộ NVBlox 3D ESDF volume.

## Short Answer For Originality

Tính mới của đề tài không nằm ở việc phát minh A* hay MPPI mới, mà nằm ở cách biến ODA thành một pipeline đánh giá hoàn chỉnh: từ ground truth sang risk label, từ planner benchmark sang perception-risk, rồi từ LiDAR/depth/fusion sang map đầu vào cho planner. Đây là đóng góp hệ thống và thực nghiệm, phù hợp với đồ án kỹ thuật.

## Short Answer For Completeness

Sản phẩm đã hoàn thiện ở mức benchmark, artifact và demo: audit pass, video/PDF/CSV đầy đủ, source compile được, demo 3D chạy local, contract perception-to-planner pass. Giới hạn còn lại là phần robot runtime cao nhất: full PX4/Gazebo closed-loop fused costmap và hiệu chuẩn metric depth.
