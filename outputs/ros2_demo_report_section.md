# Mục Báo Cáo: ROS2/Gazebo Perception-to-Planning Demo

## Mục tiêu

Mục tiêu của phần mở rộng này là kiểm chứng luồng mô phỏng từ đầu ra perception sang planner UAV:

```text
LiDAR bbox / PointCloud2 / synthetic depth / cached predicted-depth / bbox-depth mux / Gazebo depth / Gazebo LaserScan / Gazebo fused sensor costmap
  -> OccupancyGrid costmap
  -> A*/RRT/MPPI planner
  -> nav_msgs/Path
  -> kinematic UAV marker trong RViz
```

Pipeline này bổ sung cho benchmark ODA chính bằng cách chứng minh obstacle representation có thể được đưa vào planner dưới dạng costmap, thay vì chỉ đánh giá offline bằng bảng metric.

## Trạng thái kiểm chứng

Trạng thái tổng quát: **đã có runtime evidence cho nhánh sensor fusion Gazebo: PointCloud2 + depth + LaserScan -> source costmaps -> costmap mux -> planner; các mode còn thiếu/chưa chạy lại gồm: bbox, synthetic, depth_image, cached_depth**.

| Mode | Planner | Status | Topics | Messages | Mux | Video | Evidence folder |
|---|---|---|---:|---:|---|---|---|
| bbox_cached_depth_mux | astar | đạt | 11/11 | 11/11 | passed | có | `outputs/ros2_demo_runtime/bbox_cached_depth_mux_astar_20260623_043046` |
| gazebo_laserscan | astar | đạt | 6/6 | 6/6 | not_applicable | có | `outputs/ros2_demo_runtime/gazebo_laserscan_astar_20260623_052303` |
| gazebo_depth | astar | đạt | 7/7 | 7/7 | not_applicable | có | `outputs/ros2_demo_runtime/gazebo_depth_astar_20260623_052445` |
| gazebo_fused | astar | đạt | 13/13 | 13/13 | passed | có | `outputs/ros2_demo_runtime/gazebo_fused_astar_20260623_071634` |
| gazebo_fused | mppi | đạt | 13/13 | 13/13 | passed | có | `outputs/ros2_demo_runtime/gazebo_fused_mppi_20260623_112924` |

Kiểm chứng offline không cần ROS đã được bổ sung tại `outputs/tables/perception_to_planner_contract.csv`: LiDAR bbox CSV, metric depth image, relative predicted-depth proxy, fused LiDAR bbox + relative-depth mux và fused LiDAR bbox + cached predicted-depth mux đều tạo được occupancy grid không rỗng và planner A* sinh được path từ grid đó.
Bổ sung thêm `outputs/tables/perception_planner_matrix.csv`: 5 obstacle map nguồn x 3 planner (`astar`, `rrt`, `mppi`) đều sinh path collision-free sau khi inflate theo bán kính UAV và safety distance.

## Lệnh kiểm chứng

```bash
scripts/verify_ros2_costmap_runtime.sh bbox astar
scripts/verify_ros2_costmap_runtime.sh synthetic astar
scripts/verify_ros2_costmap_runtime.sh depth_image astar
scripts/verify_ros2_costmap_runtime.sh cached_depth astar
scripts/verify_ros2_costmap_runtime.sh bbox_cached_depth_mux astar
scripts/verify_ros2_costmap_runtime.sh gazebo_depth astar
scripts/verify_ros2_costmap_runtime.sh gazebo_laserscan astar
scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar
```

## Nội dung đã triển khai

- `bbox_costmap_publisher`: chuyển 3D bbox CSV từ Multi-LiDAR thành `OccupancyGrid`.
- `pointcloud_costmap`: project `PointCloud2` thành costmap 2D.
- `depth_image_costmap`: project metric depth hoặc relative predicted-depth proxy thành costmap 2D.
- `cached_depth_image_publisher`: phát cache monocular predicted-depth `.npz` thành ROS2 `mono8` image.
- `costmap_mux`: đợi đủ các source costmap rồi merge thành một `OccupancyGrid` duy nhất cho planner; đồng thời publish `/perception/costmap_mux_status`. Runtime verifier chỉ cho nhánh fusion đạt khi status này có `state=merged`, đủ source grid và không còn input bị thiếu.
- Gazebo depth camera: bridge `/camera/depth/image` qua `ros_gz_bridge` rồi dùng chung `depth_image_costmap`.
- `laserscan_costmap`: chuyển Gazebo `LaserScan` qua `ros_gz_bridge` thành costmap.
- `gazebo_fused`: chạy đồng thời PointCloud2, Gazebo depth và Gazebo LaserScan; ba source này publish `/perception/pointcloud_occupancy_grid`, `/perception/depth_occupancy_grid`, `/perception/laserscan_occupancy_grid`; `costmap_mux` merge thành `/perception/occupancy_grid` cho planner.
- `costmap_planner`: dùng costmap để sinh `nav_msgs/Path` bằng A*/RRT/MPPI.
- `kinematic_path_follower`: tạo UAV marker di chuyển theo path trong RViz khi chưa dùng PX4.
- `px4_waypoint_follower`: cầu nối optional từ path sang PX4 Offboard setpoint.
- `run_headless_ros2_runtime_video.sh`: chạy focused fused runtime trên server không GUI, validate mux status và copy MP4 ra `outputs/videos/ros2_fused_perception_runtime_astar.mp4`.

## Giới hạn hiện tại

- Demo hiện dùng fixed-altitude 2D planning, chưa phải dynamic 3D UAV control đầy đủ.
- PX4 bridge đã có nhưng chỉ nên bật sau khi server có PX4 SITL, `px4_msgs` và DDS bridge ổn định.
- Depth image mode dùng synthetic metric depth hoặc relative-depth proxy; cần calibration nếu dùng monocular depth như metric distance.
- Gazebo depth camera và Gazebo LaserScan đã đưa được vào costmap; bước tiếp theo là thêm PointCloudPacked bridge nếu cần point cloud 3D trực tiếp từ mô phỏng.

## Câu kết luận ngắn

Phần ROS2/Gazebo mở rộng biến project từ benchmark offline sang pipeline perception-to-planning có thể demo: cảm biến hoặc output perception được chuyển thành costmap, planner sinh đường tránh vật cản, và UAV marker di chuyển theo trajectory trong RViz.
