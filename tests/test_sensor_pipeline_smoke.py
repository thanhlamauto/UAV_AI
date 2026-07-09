from __future__ import annotations

import unittest

import numpy as np
from PIL import Image

from src.depth_metric import MockDepthProvider, depth_to_esdf, depth_to_point_cloud, intrinsics_from_fov, point_cloud_to_occupancy
from src.esdf3d import compute_esdf
from src.oda_io import load_radar_spectra
from src.planners.mppi_3d_esdf import MPPI3DConfig, mppi_3d_esdf_path


class SensorPipelineSmokeTest(unittest.TestCase):
    def test_radar_range_doppler_keys_and_shape(self) -> None:
        radar = load_radar_spectra("data/raw/ODA_Dataset/dataset", "345")
        self.assertIn("rd_mag", radar)
        self.assertEqual(radar["rd_mag"].ndim, 3)
        self.assertGreater(radar["rd_mag"].shape[1], 1)
        self.assertGreater(radar["rd_mag"].shape[2], 1)
        for key in [
            "radar_rd_peak",
            "radar_rd_range_bin",
            "radar_rd_doppler_bin",
            "radar_rd_energy",
            "radar_rd_near_energy",
            "radar_rd_doppler_spread",
            "radar_rd_range_spread",
        ]:
            self.assertIn(key, radar)

    def test_metric_depth_provider_interface(self) -> None:
        provider = MockDepthProvider(depth_m=2.0)
        result = provider.predict(Image.new("RGB", (32, 24)))
        self.assertEqual(result.depth_m.shape, (24, 32))
        self.assertAlmostEqual(float(np.mean(result.depth_m)), 2.0)

    def test_depth_to_point_cloud_occupancy_esdf(self) -> None:
        depth = np.full((24, 32), 2.0, dtype=np.float32)
        intrinsics = intrinsics_from_fov(32, 24, horizontal_fov_deg=70.0)
        points = depth_to_point_cloud(depth, intrinsics, stride=4)
        self.assertGreater(len(points), 0)
        occupancy, spec = point_cloud_to_occupancy(points, voxel_size_m=0.25, safety_radius_m=0.25)
        self.assertEqual(occupancy.shape, spec.shape)
        self.assertGreater(int(np.sum(occupancy)), 0)
        esdf = compute_esdf(occupancy, spec)
        queried = esdf.query(np.asarray([[0.0, 0.0, 2.0]], dtype=float))
        self.assertEqual(queried.shape, (1,))

    def test_depth_to_esdf_and_mppi(self) -> None:
        depth = np.full((24, 32), 4.0, dtype=np.float32)
        depth[:, 14:18] = 1.5
        intrinsics = intrinsics_from_fov(32, 24, horizontal_fov_deg=70.0)
        esdf, stats = depth_to_esdf(depth, intrinsics, voxel_size_m=0.30, safety_radius_m=0.30, stride=3)
        self.assertGreater(int(stats["occupied_voxels"]), 0)
        result = mppi_3d_esdf_path(
            np.asarray([0.0, -1.0, 1.0], dtype=float),
            np.asarray([0.0, 1.0, 1.0], dtype=float),
            esdf,
            MPPI3DConfig(num_rollouts=16, horizon_steps=12, max_iterations=2, seed=3),
        )
        self.assertEqual(result.trajectory_xyz.shape, (12, 3))


if __name__ == "__main__":
    unittest.main()

