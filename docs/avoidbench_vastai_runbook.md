# AvoidBench On Vast.ai Runbook

Yes, Vast.ai is a reasonable option for an official AvoidBench runtime attempt,
but choose the instance type carefully.  AvoidBench is not just a Python script:
it uses ROS Noetic, Flightmare/RotorS, and Unity rendering.

## Instance Choice

Use:

```text
Ubuntu 20.04-compatible environment
NVIDIA GPU
16 GB VRAM preferred, 8 GB minimum for first tests
32 GB RAM preferred
80-120 GB disk
on-demand / reliable host, not interruptible for first setup
SSH access enabled
desktop/noVNC template preferred
```

## GPU Recommendation

Best cost/performance targets for this project:

```text
1. RTX 3090 / RTX 4090
   Best practical choice on Vast if available as VM. Strong graphics path,
   high VRAM, good for Unity + ROS + recording.

2. RTX A4000 / RTX A4500 / RTX A5000 / RTX A6000
   Good workstation GPUs. Usually reliable for graphics workloads and enough
   VRAM for simulation/debugging.

3. NVIDIA A10 / A10G / L4
   Good cloud/datacenter choices. Prefer these over T4 if price is close.

4. RTX 3060 / 3070 / 3080 / 4060 / 4070
   Fine for smoke tests if VRAM and host reliability are acceptable.

5. T4
   Usable only as a budget smoke-test GPU. Expect weaker graphics/simulation
   performance and more patience.
```

Avoid for the first AvoidBench setup:

```text
A100 / H100
```

They are excellent compute GPUs but overkill and expensive for this task; the
main risk is Unity/graphics/display setup, not raw tensor throughput.

Practical first pick:

```text
RTX 3090 or RTX A5000 VM, Ubuntu desktop/noVNC if available,
80-120 GB disk, on-demand rental.
```

Avoid:

```text
CPU-only instance
tiny disk
unreliable/interruptible instance for setup
Docker image without SSH
headless-only setup if you have not debugged Unity on that host before
```

## Why noVNC/Desktop Helps

You can control everything from VS Code over SSH, but Unity still needs a
graphics/display path.  A noVNC/desktop template gives Unity something to bind
to and gives you a way to debug the window if rendering fails.

Pure SSH can work only if the image is already configured with one of:

```text
Xvfb
EGL/offscreen rendering
VirtualGL
noVNC/X11
```

## Vast.ai Template

First attempt:

```text
Use a CUDA/Ubuntu desktop or noVNC template, then install AvoidBench inside it.
```

Second attempt:

```text
Create a template from the official AvoidBench image:
hangyutud/noetic_avoidbench:latest
```

The official AvoidBench README uses that Docker image.  On Vast, you may still
need to add graphics/display support depending on the template and host.

## Ports

At minimum:

```text
22/tcp      SSH
6080/tcp    noVNC web, if template uses it
5901/tcp    VNC, if template uses it
8888/tcp    optional Jupyter
```

ROS ports usually do not need to be public if all nodes run inside the same
container/session.  Only expose what you need for SSH/noVNC/Jupyter.

Vast maps internal ports to external ports, so record the external host/port
shown in the Vast UI after launch.

## First Smoke Test

Inside the instance:

```bash
nvidia-smi
glxinfo -B || true
python3 - <<'PY'
import os
print("DISPLAY=", os.environ.get("DISPLAY"))
PY
```

Then test AvoidBench:

```bash
cd AvoidBench
catkin build
source devel/setup.bash
roslaunch avoid_manage rotors_gazebo.launch
```

If Unity/GUI does not show or depth topics do not publish, fix display/GPU
before debugging planners.

## Run Order

Recommended order:

```text
1. Launch AvoidBench official demo with no custom planner.
2. Confirm ROS topics publish: /depth, /rgb/left, /hummingbird/goal_point,
   /hummingbird/ground_truth/odometry.
3. Run a simple command publisher or official demo baseline.
4. Add this project's costmap planner bridge.
5. Record metrics and rosbag/video.
```

## Cost Control

Use the first hour only for environment validation:

```text
Can I SSH?
Does nvidia-smi work?
Does noVNC/desktop work?
Does AvoidBench launch?
Do /depth and odometry topics publish?
```

If any of those fail and fixing looks nontrivial, stop the instance and choose a
different host/template before spending more.
