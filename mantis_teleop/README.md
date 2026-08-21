# Mantis VR teleop (Meta Quest 2)

VR teleoperation of the PRL **mantis** (dual UR5, left arm + Weiss WSG50 gripper) over
ROS 2: a controller publisher turns Quest 2 (or HTC Vive) poses into `/vive/pose` +
`/vive/buttons`, and a bridge maps them onto the arm through Pink differential IK with a
collision barrier. Episodes can be recorded straight into a stock LeRobot v3 dataset and
replayed with the stock `lerobot-replay` CLI.

```
operator hand ──► quest_pub.py ──► /vive/pose, /vive/buttons ──► teleop_mantis.py ──► /forward_position_controller/commands
   (host, SteamVR/ALVR or adb)                                   (container, Pink IK + barrier)
```

## Files

| File | What it is |
|---|---|
| `teleop_mantis.py` | the teleop bridge: VR pose -> diff-IK -> arm + gripper commands |
| `config_teleop_mantis.yaml` | every tuned parameter (IK, teleop mapping, safety gates, gripper, recording) |
| `quest_pub.py` | Meta Quest 2 controller publisher (backends: ALVR/SteamVR or adb) |
| `vive_pub.py` | HTC Vive publisher, same message contract |
| `lerobot_recorder.py` | LeRobot episode recorder driven by the teleop's MENU button |
| `lerobot_robot_mantis/` | lerobot plugin registering `--robot.type=mantis_follower` for replay |
| `run_*.sh` | launch wrappers (see below) |
| `setup_workstation.sh` | one-shot setup of a new machine: share dir, workspace, patches, host prefix, URDF |
| `fastdds_udp_only.xml` | UDP-only DDS profile, needed for host <-> container traffic |
| `patches/` | changes made to the ROS workspace packages, mirrored by path |

## Setting up a new machine

```bash
sudo apt install -y git docker.io android-tools-adb python3-vcstool rsync
git clone git@github.com:khoviadin442/prl_ur5_ros2.git ~/rabota/prl_ur5_ros2
cd ~/rabota/prl_ur5_ros2 && git checkout mantis-teleop
./mantis_teleop/setup_workstation.sh
```

`setup_workstation.sh` copies the teleop files into the container share dir
(`~/rabota/docker_shared`), clones and patches `mantis_ws`, builds the host package
prefix and generates `mantis.urdf`. It never overwrites what is already there — pass
`--force-files` / `--force-patches` when that is what you want. Then build the image and
the workspace:

```bash
cd ~/rabota/prl_ur5_ros2/docker-ros2
./start_docker.bash mantis ~/rabota/docker_shared        # first run builds the image
# inside the container:
cd ~/share/mantis_ws && colcon build --symlink-install
pip install -e ~/share/lerobot_robot_mantis              # only needed for replay
```

The host publisher needs its own ROS environment (see below); with the SO-100 pixi
workspace that is `git clone` + `pixi install && pixi run build`. Re-run
`setup_workstation.sh` once the container is up if it reported `ur_description MISSING`.

## Requirements

* **Container side** (the docker image from `prl_ur5_ros2/docker-ros2`): ROS 2 Jazzy,
  the `mantis_ws` workspace built, `pinocchio`, `pin-pink`, `qpsolvers`, `scipy`;
  `pip install lerobot` only if episodes are recorded.
* **Host side**: a ROS 2 environment with `rclpy` plus either `openvr` (ALVR + SteamVR)
  or `oculus_reader` + `pure-python-adb` (USB backend).
* Quest 2 in developer mode, `android-tools-adb` installed.

The launch scripts source the host ROS environment from
`../SO-100-HTC-vive-teleop/.pixi/envs/default/setup.bash` by default. On a machine where
it lives elsewhere, point the env vars at it instead of editing the scripts:

```bash
export ROS_SETUP=/path/to/setup.bash          # host ROS env
export HOST_DEPS=/path/to/fakeprefix          # AMENT_PREFIX_PATH entry with the mesh packages
export OCULUS_READER=/path/to/oculus_reader   # adb backend only
export QUEST_DEPS=/path/to/quest_deps         # adb backend only (ppadb)
```

For the adb backend, set the two directories up once:

```bash
git clone https://github.com/rail-berkeley/oculus_reader "$OCULUS_READER"
pip3 install --target "$QUEST_DEPS" pure-python-adb
```

The APK inside that repo is stored in Git LFS — without `git-lfs` the clone gets a
132-byte stub and installation fails with `Failed to parse APK file`. Download it
directly instead:

```bash
cd "$OCULUS_READER/oculus_reader/APK"
curl -sL -o teleop-debug.apk \
  https://media.githubusercontent.com/media/rail-berkeley/oculus_reader/main/oculus_reader/APK/teleop-debug.apk
```

## Workspace patches

`patches/` mirrors the paths of the ROS packages that had to be changed; copy each file
over the corresponding one in `mantis_ws/src/` and rebuild. When this tree is published
inside the `prl_ur5_ros2` fork, its three `prl_ur5_ros2` patches are already applied in
that branch and the copies here are kept only for reference.

| Path | Change |
|---|---|
| `prl_ur5_robot_configuration/config/standard_setup.yaml` | left arm gets the WSG50 gripper and the teleop joint limits |
| `prl_ur5_robot_configuration/config/limits/teleop_joint_limits.yaml` | new file: +-270 deg wrists so a roll can complete |
| `prl_ur5_robot_configuration/config/controller_setup.yaml` | `forward_position_controller` active instead of the trajectory controllers |
| `prl_ur5_robot_configuration/config/fixed_cameras/cameras_config.yaml` | third camera (golf) disabled |
| `wsg50-ros-pkg/wsg_50_interface/**` | gripper commands moved off the ros2_control RT loop; open uses ACK + MOVE so a latched fast-stop cannot leave the fingers shut |
| `wsg50-ros-pkg/wsg_50_driver/config/wsg50_setup.yaml` | gripper IP, force and speed |
| `wsg50-ros-pkg/wsg_50_simulation/urdf/wsg_50.urdf.xacro` | wider finger collision meshes + the wire-box connector link |
| `prl_ur5_ros2/docker-ros2/Dockerfile` | teleop python stack (pinocchio 4 + pink + qpsolvers/daqp), CPU-only torch + lerobot, and the LD_LIBRARY_PATH that makes the pip pinocchio win |
| `prl_ur5_ros2/docker-ros2/start_docker.bash` | `--ipc=host` |
| `prl_ur5_ros2/prl_ur5_gazebo/launch/start_gazebo_sim.launch.py` | bullet-featherstone world for mimic-joint grippers |

After changing the robot configuration, regenerate the URDF the teleop loads
(`urdf:` in the config, by default `mantis_ws/mantis.urdf`):

```bash
source "$ROS_SETUP"
export AMENT_PREFIX_PATH="$HOST_DEPS:$AMENT_PREFIX_PATH"
python -c "import xacro; open('/tmp/m.urdf','w').write(
    xacro.process_file('<mantis_ws>/src/prl_ur5_ros2/prl_ur5_description/urdf/mantis.urdf.xacro',
                       mappings={'gz_sim':'true'}).toprettyxml(indent='  '))"
sed -e "s|file://$HOST_DEPS/share/ur_description|package://ur_description|g" \
    -e "s|file://$HOST_DEPS/share/prl_ur5_description|package://prl_ur5_description|g" \
    /tmp/m.urdf > <mantis_ws>/mantis.urdf
```

## Running

Canonical setup: robot stack and teleop in the container, controller publisher on the host.

```bash
# terminal 1 (container) — hardware; wait for "forward_position_controller ... activated"
ros2 launch prl_ur5_run real.launch.py            # add activate_cameras:=true to record
# terminal 2 (container) — teleop; wait for "Teleop ready"
python3 ~/share/teleop_mantis.py
# terminal 3 (host) — Quest publisher
./run_quest_pub.sh                                # QUEST_BACKEND=adb ./run_quest_pub.sh over USB
```

Everything on the host in one command (no container): `./run_teleop_quest.sh`.
The Vive equivalents are `run_vive_pub.sh` and `run_teleop_mantis.sh`; only ever run one
publisher at a time, they share the topics.

To check what the controller actually reports without publishing anything:
`./run_quest_pub.sh --scan`.

## Controls (right Touch controller)

| Button | Action |
|---|---|
| thumbstick click | engage / freeze |
| trigger, squeezed fully | gripper toggle: one click closes, the next opens |
| **B** | episode start / stop (start from frozen engages, stop freezes and drives HOME) |
| **A** | HOME ramp to the home pose; press again to cancel |
| **grip**, held | axis lock ("drawer mode"): orientation frozen, motion constrained to the gripper axis |

The Touch trigger has no mechanical click, so the click is derived from the analog value
with hysteresis (`QUEST_CLICK_ON` 0.90 / `QUEST_CLICK_OFF` 0.60).

Quest tracking is inside-out: the headset must be awake and looking at your hands. Park
it facing the workspace and defeat the proximity sensor, otherwise it sleeps and tracking
stops. A long press on the Meta button recenters the headset frame and the world under
the teleop jumps — freeze and engage again if it happens.

## Recording and replay

`record:` in the config controls the recorder (dataset root, fps, cameras). Press MENU/B
to start an episode and again to stop; frames are added at the primary camera's rate, so
`record.fps` must equal the camera stream rate. Exit the teleop with Ctrl-C — that is
what finalizes the dataset (and, with `batch_encoding_size > 1`, encodes the videos).

Replay an episode with the stock CLI through the plugin (teleop must be stopped, both
publish the arm command topic):

```bash
pip install -e lerobot_robot_mantis      # once, in the container
./run_replay.sh 0                        # episode 0
```

## Environment variables (quest_pub.py)

| Variable | Default | Meaning |
|---|---|---|
| `QUEST_BACKEND` | `openvr` | `openvr` (ALVR/SteamVR) or `adb` (oculus_reader over USB) |
| `QUEST_HAND` | `right` | which controller drives the arm |
| `QUEST_RATE` | `250` | publish rate [Hz] |
| `QUEST_POSE_PREDICTION` | `0.05` | s of pose prediction (openvr backend only) |
| `QUEST_ENGAGE_BUTTON` | `thumbstick` | engage / freeze button |
| `QUEST_MENU_BUTTON` | `b` | episode start / stop |
| `QUEST_HOME_BUTTON` | `a` | HOME ramp |
| `QUEST_AXISLOCK_BUTTON` | `grip` | axis lock |
| `QUEST_TRIGGER_CLICK` | `soft` | `soft` = analog hysteresis, `hw` = driver bit |
| `QUEST_CLICK_ON` / `QUEST_CLICK_OFF` | `0.90` / `0.60` | soft click thresholds |
| `QUEST_UNIVERSE` | `standing` | OpenVR universe (`standing` guarantees up = +Y) |
| `QUEST_ROT_OFFSET` | — | `rx,ry,rz` in degrees, extra rotation of the controller axes |
| `QUEST_BIT_*` | see `--scan` | override the button bits (`QUEST_BIT_A`, `QUEST_BIT_B`, …) |
| `QUEST_POSE_TOPIC` / `QUEST_BUTTONS_TOPIC` | `/vive/pose`, `/vive/buttons` | rename the topics (then also edit `topics:` in the config) |
| `QUEST_ADB_MODE` | `native` | `native` = one message per headset sample (~72 Hz); `extrapolate` resamples to 250 Hz and gets rejected by the glitch gate |
| `QUEST_ADB_IP` | — | adb over Wi-Fi instead of USB (the cable is ~3x faster) |
| `QUEST_ADB_STALE` | `0.15` | s without data before the pose stream goes silent |
| `QUEST_ADB_KEEPALIVE` | `0.05` | s between resends of an unchanged pose |

Two buttons must not share one physical control — mapping engage onto `grip` while axis
lock is also on `grip` fires both at once (quest_pub warns about it).
