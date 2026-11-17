# ROS 2 Jazzy stereo VIO with single-AprilTag initialization

This branch preserves the upstream SchurVINS source and builds the stereo VIO
core with a ROS 2 Jazzy adapter. The same Compose file builds a native
environment image on Ubuntu 24.04 Linux hosts with AMD64 or ARM64 CPUs.
The fork, calibration, launch settings, pinned dependencies and build
outputs are mounted as project files. The environment image contains dependencies.

## Build and run

From the root of the fork:

```bash
./ros2/env.sh fetch
./ros2/env.sh build-image
./ros2/env.sh build --packages-select schurvins_jazzy
```

Place the actual stereo VIO configuration, stereo/IMU calibration and the
edited `tag36h11.yaml` in `ros2/workspace/configs/board/`. Set `tag_id` to
the installed marker ID and `tag_size_m` to the measured black square side,
excluding the white margin. Then run:

```bash
./ros2/env.sh run ros2 run schurvins_jazzy stereo_vio --ros-args \
  --params-file /workspaces/schurvins_ws/configs/board/tag36h11.yaml \
  -p config_file:=/workspaces/schurvins_ws/configs/board/vio.yaml \
  -p calibration_file:=/workspaces/schurvins_ws/configs/board/calibration.yaml
```

The required inputs are exact-timestamp stereo images on
`/cam0/image_raw` and `/cam1/image_raw`, and monotonically timestamped IMU
samples on `/imu0`. They use best-effort subscriptions by default.
ROS remappings and `input_reliable:=true` are available. The calibration
is shared by VIO and Tag reprojection, including camera-to-body transforms
and the upstream camera projection model. A real underwater installation
requires calibration for its actual optical housing, a common acquisition
clock and a measured Tag size. Use `BUILD_JOBS=1` on a board with limited RAM.
The Compose default discovery range is `LOCALHOST` and uses host networking.

## Initialization and outputs

The initializer attempts at most five detections per second within the
stereo VIO worker, using the paired images for the same VIO pose. Both
cameras must see the configured Tag. It uses planar pose candidates
followed by joint stereo reprojection over at most eight recent views.
Near-duplicate viewpoints do not add confidence. Cheirality, reprojection
error and planar ambiguity checks precede a six-dimensional nominal 95%
alignment uncertainty gate. The gate evaluates the body pose uncertainty
at the current VIO position. The defaults are 0.06 m and 5 deg, with a
1 cm and 0.5 deg alignment covariance floor and 1 px corner noise.
Set these tolerances from the application. The default noise and floors are
engineering assumptions. Validate their statistical coverage for the
physical camera and short-window VIO errors before relying on them.

The first qualified estimate is accepted even if the Tag remains visible.
The alignment stays frozen for the current VIO epoch and Tag detection stops.
A VIO reset invalidates it and returns to the waiting state. On a temporary
tracking loss, consumers should check the native status and the freshness
of the pose messages. `tag_id: -1` (the default) runs the original stereo
adapter path with Tag initialization disabled.

| Output | Meaning |
|---|---|
| `/vio/pose`, `/vio/status` | Original SchurVINS pose and tracking state. |
| `/vio/tag_status` | Initialization state. The current state is latched with ROS 2 transient-local durability. |
| `/vio/tag_alignment` | One pose with covariance per successful alignment. The transform is `tag_frame -> vio_world`, describing the VIO origin expressed in Tag coordinates. Covariance order is translation in metres, then fixed-axis rotation in radians. |
| `/vio/tag_pose` | Body pose in the configured Tag frame, after a successful alignment. |
| `/tf` | Dynamic `tag_frame -> vio_world` while an accepted alignment and a tracking pose are available. |

The Tag frame is attached to the physical marker. The decoded top edge
points along +X, +Y points up in the printed pattern and +Z points out of
its face. Downstream users needing a gravity-aligned pool frame should
provide a separately defined frame transform. The Tag alignment covariance
describes the startup alignment. Subsequent global poses inherit VIO drift
after the one-time alignment.

## Verification

The included `ros2/tests/smoke.py` supports ROS 2 bag playback in the
native container with `--tag-params` plus `--expect-tag ready` or
`--expect-tag withheld`. Put replay inputs under `ros2/workspace/replay/`
and use paths relative to `/workspaces/schurvins_ws/replay` in the
container. It checks valid native poses and enforces one Tag alignment or
zero global output for the two expectation modes, respectively.
