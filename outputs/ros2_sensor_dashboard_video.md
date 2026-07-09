# ROS2 Sensor Dashboard Video

Video: `outputs/videos/ros2_sensor_dashboard_flight_astar.mp4`

| Panel | What it shows | Source |
|---|---|---|
| UAV FLIGHT | UAV marker moving along A* path with obstacle/safety footprints | Gazebo demo geometry + planner helper |
| POINTCLOUD XYZ | Synthetic PointCloud2-style obstacle points visible from the moving UAV | Same geometry as ROS2 synthetic point cloud publisher |
| GAZEBO LIDAR | 180-degree LiDAR scan fan and obstacle hits | Same range model as Gazebo LaserScan runtime evidence |
| DEPTH IMAGE | Metric depth image changing with UAV pose | Same depth projection convention as `depth_image_costmap` |
| DEPTH COSTMAP | Depth-derived occupied cells in world coordinates | `depth_image_to_grid` helper |
| PLANNER OUTPUT | Inflated occupancy grid and A* path consumed by follower | `costmap_planner` helper |

This qualitative video complements the ROS2 runtime evidence folders. It is rendered offline so it can be viewed without RViz/Gazebo GUI, while preserving the same perception-to-costmap-to-planner story.
