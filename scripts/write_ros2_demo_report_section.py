#!/usr/bin/env python3
"""Write a concise Vietnamese report section from ROS2 runtime evidence."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def _read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _is_complete(row: dict[str, str]) -> bool:
    base_ok = (
        row.get("status") == "passed"
        and row.get("topics_present") == row.get("topics_expected")
        and row.get("messages_received") == row.get("messages_expected")
    )
    if row.get("mode") in {"bbox_cached_depth_mux", "gazebo_fused"}:
        return base_ok and row.get("mux_status_valid") == "passed"
    return base_ok


def _status_vn(row: dict[str, str]) -> str:
    return "đạt" if _is_complete(row) else row.get("status", "chưa có")


def write_report_section(rows: list[dict[str, str]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)

    required_modes = [
        "bbox",
        "synthetic",
        "depth_image",
        "cached_depth",
        "bbox_cached_depth_mux",
        "gazebo_depth",
        "gazebo_laserscan",
        "gazebo_fused",
    ]
    passed_modes = {
        mode
        for mode in required_modes
        if any(row.get("mode") == mode and _is_complete(row) for row in rows)
    }
    if set(required_modes).issubset(passed_modes):
        overall = "đã có runtime evidence cho LiDAR bbox, PointCloud2, synthetic depth, cached predicted-depth, bbox-depth mux, Gazebo depth, Gazebo LaserScan và Gazebo fused sensor costmap"
    elif "gazebo_fused" in passed_modes:
        missing_modes = ", ".join(mode for mode in required_modes if mode not in passed_modes)
        overall = (
            "đã có runtime evidence cho nhánh sensor fusion Gazebo: PointCloud2 + depth + LaserScan "
            "-> source costmaps -> costmap mux -> planner; các mode còn thiếu/chưa chạy lại gồm: "
            f"{missing_modes}"
        )
    elif "bbox_cached_depth_mux" in passed_modes:
        missing_modes = ", ".join(mode for mode in required_modes if mode not in passed_modes)
        overall = (
            "đã có runtime evidence cho nhánh focused LiDAR bbox + cached predicted-depth -> "
            "costmap mux -> planner; các mode all-mode còn lại để mở rộng/gỡ lỗi gồm: "
            f"{missing_modes}"
        )
    else:
        missing_modes = ", ".join(mode for mode in required_modes if mode not in passed_modes)
        overall = f"chưa đủ runtime evidence; cần chạy verifier trên server ROS2/Gazebo cho: {missing_modes}"

    lines = [
        "# Mục Báo Cáo: ROS2/Gazebo Perception-to-Planning Demo",
        "",
        "## Mục tiêu",
        "",
        "Mục tiêu của phần mở rộng này là kiểm chứng luồng mô phỏng từ đầu ra perception sang planner UAV:",
        "",
        "```text",
        "LiDAR bbox / PointCloud2 / synthetic depth / cached predicted-depth / bbox-depth mux / Gazebo depth / Gazebo LaserScan / Gazebo fused sensor costmap",
        "  -> OccupancyGrid costmap",
        "  -> A*/RRT/MPPI planner",
        "  -> nav_msgs/Path",
        "  -> kinematic UAV marker trong RViz",
        "```",
        "",
        "Pipeline này bổ sung cho benchmark ODA chính bằng cách chứng minh obstacle representation có thể được đưa vào planner dưới dạng costmap, thay vì chỉ đánh giá offline bằng bảng metric.",
        "",
        "## Trạng thái kiểm chứng",
        "",
        f"Trạng thái tổng quát: **{overall}**.",
        "",
    ]

    if rows:
        lines.extend(
            [
                "| Mode | Planner | Status | Topics | Messages | Mux | Video | Evidence folder |",
                "|---|---|---|---:|---:|---|---|---|",
            ]
        )
        for row in rows:
            video = "có" if int(row.get("video_bytes", "0") or 0) > 0 else "chưa có"
            mux = row.get("mux_status_valid") or "n/a"
            lines.append(
                "| "
                f"{row.get('mode', '')} | "
                f"{row.get('planner', '')} | "
                f"{_status_vn(row)} | "
                f"{row.get('topics_present', '0')}/{row.get('topics_expected', '0')} | "
                f"{row.get('messages_received', '0')}/{row.get('messages_expected', '0')} | "
                f"{mux} | "
                f"{video} | "
                f"`{row.get('run_dir', '')}` |"
            )
        lines.append("")
    else:
        lines.extend(
            [
                "Chưa có folder runtime evidence. Trên server cần chạy:",
                "",
                "```bash",
                "scripts/verify_ros2_costmap_runtime.sh bbox astar",
                "scripts/verify_ros2_costmap_runtime.sh synthetic astar",
                "scripts/verify_ros2_costmap_runtime.sh depth_image astar",
                "scripts/verify_ros2_costmap_runtime.sh cached_depth astar",
                "scripts/verify_ros2_costmap_runtime.sh bbox_cached_depth_mux astar",
                "scripts/verify_ros2_costmap_runtime.sh gazebo_depth astar",
                "scripts/verify_ros2_costmap_runtime.sh gazebo_laserscan astar",
                "scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar",
                "",
                "# hoặc chạy focused headless video trên server không GUI:",
                "scripts/run_headless_ros2_runtime_video.sh astar",
                "```",
                "",
            ]
        )

    lines.extend(
        [
            "Kiểm chứng offline không cần ROS đã được bổ sung tại `outputs/tables/perception_to_planner_contract.csv`: LiDAR bbox CSV, metric depth image, relative predicted-depth proxy, fused LiDAR bbox + relative-depth mux và fused LiDAR bbox + cached predicted-depth mux đều tạo được occupancy grid không rỗng và planner A* sinh được path từ grid đó.",
            "Bổ sung thêm `outputs/tables/perception_planner_matrix.csv`: 5 obstacle map nguồn x 3 planner (`astar`, `rrt`, `mppi`) đều sinh path collision-free sau khi inflate theo bán kính UAV và safety distance.",
            "",
        ]
    )

    lines.extend(
        [
            "## Lệnh kiểm chứng",
            "",
            "```bash",
            "scripts/verify_ros2_costmap_runtime.sh bbox astar",
            "scripts/verify_ros2_costmap_runtime.sh synthetic astar",
            "scripts/verify_ros2_costmap_runtime.sh depth_image astar",
            "scripts/verify_ros2_costmap_runtime.sh cached_depth astar",
            "scripts/verify_ros2_costmap_runtime.sh bbox_cached_depth_mux astar",
            "scripts/verify_ros2_costmap_runtime.sh gazebo_depth astar",
            "scripts/verify_ros2_costmap_runtime.sh gazebo_laserscan astar",
            "scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar",
            "```",
            "",
            "## Nội dung đã triển khai",
            "",
            "- `bbox_costmap_publisher`: chuyển 3D bbox CSV từ Multi-LiDAR thành `OccupancyGrid`.",
            "- `pointcloud_costmap`: project `PointCloud2` thành costmap 2D.",
            "- `depth_image_costmap`: project metric depth hoặc relative predicted-depth proxy thành costmap 2D.",
            "- `cached_depth_image_publisher`: phát cache monocular predicted-depth `.npz` thành ROS2 `mono8` image.",
            "- `costmap_mux`: đợi đủ các source costmap rồi merge thành một `OccupancyGrid` duy nhất cho planner; đồng thời publish `/perception/costmap_mux_status`. Runtime verifier chỉ cho nhánh fusion đạt khi status này có `state=merged`, đủ source grid và không còn input bị thiếu.",
            "- Gazebo depth camera: bridge `/camera/depth/image` qua `ros_gz_bridge` rồi dùng chung `depth_image_costmap`.",
            "- `laserscan_costmap`: chuyển Gazebo `LaserScan` qua `ros_gz_bridge` thành costmap.",
            "- `gazebo_fused`: chạy đồng thời PointCloud2, Gazebo depth và Gazebo LaserScan; ba source này publish `/perception/pointcloud_occupancy_grid`, `/perception/depth_occupancy_grid`, `/perception/laserscan_occupancy_grid`; `costmap_mux` merge thành `/perception/occupancy_grid` cho planner.",
            "- `costmap_planner`: dùng costmap để sinh `nav_msgs/Path` bằng A*/RRT/MPPI.",
            "- `kinematic_path_follower`: tạo UAV marker di chuyển theo path trong RViz khi chưa dùng PX4.",
            "- `px4_waypoint_follower`: cầu nối optional từ path sang PX4 Offboard setpoint.",
            "- `run_headless_ros2_runtime_video.sh`: chạy focused fused runtime trên server không GUI, validate mux status và copy MP4 ra `outputs/videos/ros2_fused_perception_runtime_astar.mp4`.",
            "",
            "## Giới hạn hiện tại",
            "",
            "- Demo hiện dùng fixed-altitude 2D planning, chưa phải dynamic 3D UAV control đầy đủ.",
            "- PX4 bridge đã có nhưng chỉ nên bật sau khi server có PX4 SITL, `px4_msgs` và DDS bridge ổn định.",
            "- Depth image mode dùng synthetic metric depth hoặc relative-depth proxy; cần calibration nếu dùng monocular depth như metric distance.",
            "- Gazebo depth camera và Gazebo LaserScan đã đưa được vào costmap; bước tiếp theo là thêm PointCloudPacked bridge nếu cần point cloud 3D trực tiếp từ mô phỏng.",
            "",
            "## Câu kết luận ngắn",
            "",
            "Phần ROS2/Gazebo mở rộng biến project từ benchmark offline sang pipeline perception-to-planning có thể demo: cảm biến hoặc output perception được chuyển thành costmap, planner sinh đường tránh vật cản, và UAV marker di chuyển theo trajectory trong RViz.",
            "",
        ]
    )
    output.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-csv", type=Path, default=Path("outputs/tables/ros2_demo_runtime_summary.csv"))
    parser.add_argument("--output", type=Path, default=Path("outputs/ros2_demo_report_section.md"))
    args = parser.parse_args()

    rows = _read_rows(args.input_csv)
    write_report_section(rows, args.output)
    print(f"Wrote {args.output} from {len(rows)} runtime row(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
