# AvoidBench Official Runtime And Server Runbook

This runbook is for running the real AvoidBench simulator, not the local
costmap-only adapter benchmark.

## Do You Need A Server?

You do not need a paid server if you already have:

```text
Ubuntu 20.04
ROS Noetic
NVIDIA GPU + NVIDIA driver
Docker + nvidia-container-toolkit, or native catkin workspace
working display/X11 for Unity
```

You probably need a rented GPU machine if you are on macOS/Windows without a
Linux NVIDIA GPU. AvoidBench uses Unity/Flightmare/RotorS/ROS Noetic; the
official README recommends Ubuntu 20.04 + ROS Noetic and suggests a GPU because
Unity rendering and stereo depth work much better with NVIDIA acceleration.

Do not rent a CPU-only server for the official benchmark. It may build parts of
the workspace, but it is the wrong target for Unity rendering, stereo/CUDA
depth, and full flight episodes.

## Suggested Machine Profile

Minimum practical profile:

```text
OS: Ubuntu 20.04
GPU: NVIDIA GPU with display/graphics capability
VRAM: 8 GB or more preferred
RAM: 32 GB preferred
Disk: 80-120 GB free
Docker: yes, with nvidia-container-toolkit
```

Most convenient option:

```text
GPU workstation or cloud GPU instance with Ubuntu desktop/noVNC/X11.
```

More painful option:

```text
Headless GPU server.
```

Headless can work, but Unity/X11/graphics forwarding is the fragile part. A
desktop GPU machine is easier than a pure SSH server.

## Official AvoidBench Setup

Docker path from the official README:

```bash
# host
xhost +

distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list \
  | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

sudo docker pull hangyutud/noetic_avoidbench:latest
sudo docker run -it \
  --device=/dev/dri \
  --group-add video \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --gpus all \
  --name=noetic_ab \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -e NVIDIA_VISIBLE_DEVICES=all \
  hangyutud/noetic_avoidbench:latest /bin/bash
```

Inside the container:

```bash
cd AvoidBench
catkin build
source devel/setup.bash
roslaunch avoid_manage rotors_gazebo.launch
```

The README also has a Python-oriented launcher:

```bash
roslaunch avoid_manage test_py.launch
```

## Topics To Connect

AvoidBench provides these user-facing topics:

```text
/depth                                      SGM depth image, mono16
/rgb/left                                   left RGB, bgr8
/rgb/right                                  right RGB, bgr8
/hummingbird/camera_depth/camera/camera_info
/hummingbird/ground_truth/odometry
/hummingbird/ground_truth/imu
/hummingbird/goal_point
/hummingbird/task_state
```

Your algorithm should publish one of:

```text
/hummingbird/autopilot/pose_command
/hummingbird/autopilot/velocity_command
/hummingbird/autopilot/reference_state
/hummingbird/autopilot/control_command_input
```

And it should publish timing:

```text
/hummingbird/iter_time
```

Important task-state rule:

```text
Do not send control commands while task_state is 0, 1, or 5.
```

## How To Run This Project's Planners In AvoidBench

The project needs one ROS Noetic bridge node, conceptually:

```text
/depth
  -> decode mono16 depth
  -> depth_image_to_grid equivalent
  -> costmap

/hummingbird/ground_truth/odometry
  -> current position/velocity

/hummingbird/goal_point
  -> goal

costmap + state + goal
  -> A* / RRT / RRT* / MPPI / MPC-style
  -> velocity_command or reference_state
  -> iter_time
```

Recommended first runtime target:

```text
MPPI or MPC-style -> /hummingbird/autopilot/velocity_command
```

A*/RRT/RRT* are better as global/reference path generators. For actual episode
flight, wrap them with a path follower that publishes velocity/reference-state
commands.

## Official Baselines

AvoidBench's README gives setup instructions for Agile-Autonomy:

```bash
source devel/setup.bash
cd src/
git clone --recursive https://github.com/NPU-yuhang/agile_autonomy.git
cd ..
catkin build

source devel/setup.bash
roslaunch agile_autonomy simulation.launch
```

Then in another terminal:

```bash
source devel/setup.bash
conda activate tf_24
roscd planner_learning
python test_trajectories.py --settings_file=config/test_settings.yaml
```

AvoidBench also reports tests with Agile-Autonomy, Ego-Planner, and MBPlanner.
Those are ROS/runtime baselines, not simple CPU-only scripts.

## What To Measure

For every episode, record:

```text
environment
map_id / trial_id
planner
sensor mode
success
collision
timeout
flight_time_s
path_length_m
min_clearance_m
mean_iter_time_ms
max_iter_time_ms
command topic
```

Minimum result worth adding to the report:

```text
10-30 episodes in one AvoidBench scene, same start/goal/task config,
comparing at least MPPI/MPC-style with one official baseline or a straight/path
follower baseline.
```

## Practical Recommendation

If the goal is a defensible student-report extension:

```text
1. Do not rent yet if you only need local tables and stress tests.
2. Rent or borrow a GPU Ubuntu machine only when you are ready to run official
   AvoidBench episodes and record videos/metrics.
3. Prefer a GUI-capable GPU workstation/cloud desktop over a bare SSH GPU box.
```

The current local work already proves the costmap-planner side. The server is
needed only for the official claim: "ran AvoidBench runtime episodes."
