#!/usr/bin/env python3
import argparse
import math
import random
import time

import numpy as np
import rospy
from avoid_msgs.msg import TaskState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Float32


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class AvoidBenchPlanner:
    def __init__(self, planner, speed, seed):
        self.planner = planner
        self.speed = float(speed)
        self.rng = random.Random(seed)
        np.random.seed(seed)
        self.state = 0
        self.goal = None
        self.odom = None
        self.depth = None
        self.depth_stamp = None
        self.sent_mission_start = False
        self.last_cmd = np.array([0.0, 0.0], dtype=np.float64)

        self.cmd_pub = rospy.Publisher(
            "/hummingbird/autopilot/velocity_command", TwistStamped, queue_size=1
        )
        self.iter_pub = rospy.Publisher("/hummingbird/iter_time", Float32, queue_size=10)
        self.start_pub = rospy.Publisher("/hummingbird/mission_start", Empty, queue_size=1)

        rospy.Subscriber("/hummingbird/task_state", TaskState, self.on_task, queue_size=1)
        rospy.Subscriber("/hummingbird/goal_point", Path, self.on_goal, queue_size=1)
        rospy.Subscriber(
            "/hummingbird/ground_truth/odometry", Odometry, self.on_odom, queue_size=1
        )
        rospy.Subscriber("/depth", Image, self.on_depth, queue_size=1, buff_size=2**24)

    def on_task(self, msg):
        self.state = int(msg.Mission_state)

    def on_goal(self, msg):
        if msg.poses:
            p = msg.poses[-1].pose.position
            self.goal = np.array([p.x, p.y, p.z], dtype=np.float64)
            self.sent_mission_start = False

    def on_odom(self, msg):
        self.odom = msg

    def on_depth(self, msg):
        if msg.height == 0 or msg.width == 0:
            return
        if msg.encoding == "mono16":
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            depth = arr.astype(np.float32) / 1000.0
        elif msg.encoding == "32FC1":
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        else:
            return
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        self.depth = depth
        self.depth_stamp = msg.header.stamp

    def sector_clearance(self):
        if self.depth is None:
            return np.ones(31, dtype=np.float64) * 8.0
        h, w = self.depth.shape
        band = self.depth[int(h * 0.38) : int(h * 0.72), :]
        sectors = 31
        clear = np.zeros(sectors, dtype=np.float64)
        for i in range(sectors):
            x0 = int(i * w / sectors)
            x1 = int((i + 1) * w / sectors)
            patch = band[:, x0:x1]
            vals = patch[(patch > 0.25) & (patch < 30.0)]
            clear[i] = float(np.percentile(vals, 20)) if vals.size else 0.3
        return clear

    def pick_angle(self, desired_angle):
        fov = math.radians(91.0)
        if abs(desired_angle) > fov / 2.0:
            return desired_angle
        sectors = np.linspace(-fov / 2.0, fov / 2.0, 31)
        clear = self.sector_clearance()
        safe = np.clip((clear - 1.0) / 5.0, 0.0, 1.0)

        if self.planner == "rrt":
            samples = [desired_angle] + [self.rng.uniform(-fov / 2.0, fov / 2.0) for _ in range(96)]
            best = min(samples, key=lambda a: abs(wrap(a - desired_angle)) + 2.5 * (1.0 - np.interp(a, sectors, safe)))
            return float(np.clip(best, -fov / 2.0, fov / 2.0))

        if self.planner == "rrt_star":
            samples = [desired_angle] + [self.rng.uniform(-fov / 2.0, fov / 2.0) for _ in range(320)]
            best_cost = 1e9
            best = desired_angle
            for a in samples:
                neighborhood = np.linspace(a - 0.12, a + 0.12, 5)
                local_safe = max(np.interp(np.clip(x, -fov / 2.0, fov / 2.0), sectors, safe) for x in neighborhood)
                cost = abs(wrap(a - desired_angle)) + 3.0 * (1.0 - local_safe) + 0.12 * abs(a)
                if cost < best_cost:
                    best_cost = cost
                    best = a
            return float(np.clip(best, -fov / 2.0, fov / 2.0))

        # MPPI-style one-step stochastic control optimization.
        controls = np.linspace(-fov / 2.0, fov / 2.0, 121)
        costs = []
        for a in controls:
            c = abs(wrap(a - desired_angle)) ** 2
            c += 5.0 * (1.0 - np.interp(a, sectors, safe)) ** 2
            c += 0.2 * abs(wrap(a - math.atan2(self.last_cmd[1], max(self.last_cmd[0], 1e-6))))
            costs.append(c)
        costs = np.asarray(costs)
        beta = max(1e-6, costs.min())
        weights = np.exp(-(costs - costs.min()) / (0.12 + beta))
        return float(np.sum(controls * weights) / np.sum(weights))

    def step(self):
        if self.odom is None or self.goal is None:
            return
        if self.state == 2 and not self.sent_mission_start:
            self.start_pub.publish(Empty())
            self.sent_mission_start = True
        if self.state not in (3, 4):
            return

        t0 = time.perf_counter()
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        delta = self.goal - pos
        yaw = yaw_from_quat(q)
        desired_world = math.atan2(delta[1], delta[0])
        desired_body = wrap(desired_world - yaw)
        steer = self.pick_angle(desired_body)
        forward = math.cos(steer)
        lateral = math.sin(steer)
        vz = float(np.clip(0.8 * (self.goal[2] - pos[2]), -0.6, 0.6))
        gain = min(self.speed, max(0.4, np.linalg.norm(delta[:2])))
        self.last_cmd = np.array([gain * forward, gain * lateral])

        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        world_angle = yaw + steer
        vx = gain * math.cos(world_angle)
        vy = gain * math.sin(world_angle)
        self.last_cmd = np.array([vx, vy])

        msg.header.frame_id = "world"
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = vz
        msg.twist.angular.z = float(np.clip(1.5 * steer, -1.2, 1.2))
        self.cmd_pub.publish(msg)
        self.iter_pub.publish(Float32((time.perf_counter() - t0) * 1000.0))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--planner", choices=["rrt", "rrt_star", "mppi"], required=True)
    parser.add_argument("--speed", type=float, required=True)
    parser.add_argument("--seed", type=int, default=32)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("avoidbench_planner_%s" % args.planner)
    node = AvoidBenchPlanner(args.planner, args.speed, args.seed)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.step()
        rate.sleep()


if __name__ == "__main__":
    main()
