# gnss_imu_prediction

3d position estimation and short term position prediction from a gnss
receiver and an imu, in simulation (cosys-airsim on unreal engine 5.2, ros 2
jazzy). gps fixes (1 hz) and imu acceleration (100 hz) are fused in a kalman
filter, which gives a smoother position and altitude than the raw receiver and
predicts where the drone will be a moment later.

the utm conversion, kalman core, measurement types and the simulator side come
from [localization_core](external/localization_core), included as a git
submodule. its sibling project, `drone_localization`, locates the same drone
from external sensors.

guides: [getting started](https://ulastanersoyak.github.io/gnss_imu_prediction/getting-started.html) ·
[how it works](https://ulastanersoyak.github.io/gnss_imu_prediction/how-it-works.html)

## how it works

```
airsim gps  (navsat fix, 1 hz) ──▶ receiver error model ──▶ utm ──┐
                                                                  ├──▶ kalman filter ──▶ estimate + prediction
airsim imu  (100 hz) ─────────▶ ahrs error model ─────────────────┘
```

- **gps**: each fix gives position, altitude, speed and course; the hdop sets
  how much it is trusted. positions are converted to utm metres.
- **imu**: specific force and orientation, rotated into the local
  east/north/up frame with gravity removed.
- **filter**: state is `[east, north, up, v_east, v_north, v_up]`. every imu
  sample predicts the state forward with the measured acceleration, every gps
  fix corrects it. nmea has no vertical velocity, so the climb rate comes from
  the accelerometer and the altitudes. fixes that disagree too much with the prediction are rejected
  (chi-square gate), and the filter restarts from gps if that keeps
  happening. without an imu it runs on gps alone.
- **prediction**: `position_estimator::predict_ahead(t)` extrapolates the
  current estimate `t` seconds ahead with the current acceleration.

## layout

| path | what |
|---|---|
| `includes/kalman/`, `src/kalman/` | `position_estimator`, the filter and the prediction (target `kalman`) |
| `sim/` | simulation: `positioning_sim` (a ros 2 node for cosys-airsim) and its parameters (`src/positioning_sim_parameters.yaml`), the launch file, run script and rviz2 view |
| `test/cpp/` | unit tests of the estimator (googletest) |
| `external/localization_core/` | submodule: utm, the kalman core, the measurement types, the simulator side (`airsim/`) |
| `docs/` | the html guides, the kalman paper used, images |

## building

needs cmake ≥ 3.22, a c++20 compiler, eigen, geographiclib and googletest
(fedora: `eigen3-devel GeographicLib-devel gtest-devel`, ubuntu:
`libeigen3-dev libgeographiclib-dev libgtest-dev`); the simulation builds in
the `ros2` distrobox set up by `external/localization_core/airsim/setup.sh`.

```
git clone --recurse-submodules <url>/gnss_imu_prediction
cd gnss_imu_prediction            # or, in an existing clone: git submodule update --init
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

for local clones side by side, git refuses `file://` submodules by default:
`git -c protocol.file.allow=always submodule update --init`.

## simulation

a quadrotor takes off and flies a figure-eight in
[Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim) (Unreal Engine)
while its speed swings between 1 and 4 m/s and its height between 5 and 11 m.
it carries a 100 hz imu (50 hz from airsim's ros 2 node) and a 1 hz gps;
`positioning_sim` adds 1.5 m of horizontal gps noise and imu bias and
orientation errors. it is a ROS 2 node: it feeds the sensors to the same
`position_estimator` the tests check, publishes the prediction (the "ghost"
in front of the drone), and compares every prediction with the true pose
once the simulation reaches that moment. the drone is flown by
localization_core's `airsim/autopilot.py`, so the program only measures and
estimates.

```
sim/run.sh                                         # the open field, unreal window
sim/run.sh --scenario antwerp                      # over groenplaats in antwerp's 3d city model
sim/run.sh --no-gui --duration 120                 # no window, stop after 120 simulated s
sim/run.sh --rain 0.6 --fog 0.2 --wind 6@270       # airsim's weather, wind from the west
sim/run.sh --horizon 0.5 --gps-noise 3 --gps-drift 2
sim/run.sh --rviz                                  # watch it in rviz2 (sim/rviz/positioning.rviz)
sim/run.sh --manual                                # no autopilot, fly it yourself: localization_core/airsim/keyboard.sh
```

the simulator side (unreal, cosys-airsim, its ros 2 node, the maps) lives in
localization_core: run `external/localization_core/airsim/setup.sh` once
(see [localization_core/airsim](external/localization_core/airsim/README.md)).
`run.sh` builds `positioning_sim` into `build-ros/` in the `ros2` distrobox
(ubuntu 24.04, ros 2 jazzy), starts unreal, then `sim/launch/positioning.launch.py`
(airsim's ros 2 node, the autopilot, the program and a rosbag2 recorder). the run is
recorded into `build-ros/runs/positioning-<scenario>-<time>` and, when it
ends (`--duration` or ctrl-c), scored against airsim's true pose by
localization_core's `airsim/evaluate.py` (evo); evo's own tools plot the same
recording (see localization_core/airsim). the options are ros 2
parameters (`--gps-noise 3` becomes `gps_noise:=3`): `positioning_sim`'s,
defined in `sim/src/positioning_sim_parameters.yaml` with
[generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library)
(`ros2 param describe /positioning_sim gps_noise` explains one), and the
autopilot's (`--radius`, `--altitude`, `--altitude-swing`):

| option | default | |
|---|---|---|
| `--scenario s` | field | `field` (an open field) or `antwerp` |
| `--no-gui` | | unreal without a window |
| `--rviz` | | rviz2 with `sim/rviz/positioning.rviz` |
| `--rain/--snow/--fog/--dust i` | | airsim weather, 0..1 |
| `--wind s[@from_deg]` | | wind speed m/s, from this direction (225 = south-west) |
| `--horizon s` | 1.0 | how far ahead the ghost predicts (0.01 = next imu sample) |
| `--gps-noise m` | 1.5 | white gps position noise |
| `--gps-drift m` | 0 | slowly wandering gps error, like multipath |
| `--gps-vfactor f` | 1.7 | altitude noise and drift relative to the horizontal |
| `--accel-bias a` | 0.02 | accelerometer bias per axis, m/s², fixed at startup |
| `--tilt-error deg` | 0.2 | slowly wandering roll/pitch error of the imu orientation |
| `--heading-error deg` | 1.0 | slowly wandering heading error of the imu orientation |
| `--duration s` | forever | stop after this much simulated time |
| `--radius m` | 12 | figure-eight loop radius |
| `--altitude m` | 8 | cruise height above the ground |
| `--altitude-swing m` | 3 | how far it climbs and descends around the cruise height |
| `--manual` | | no autopilot; publish `/cmd_vel` yourself (airsim/keyboard.sh) |

the estimates are published as `geometry_msgs/PoseStamped` in the `enu`
frame: `/positioning/estimate` (now), `/positioning/prediction` (the ghost,
stamped with the time it predicts), `/positioning/baseline` and
`/positioning/gps` (the noisy fixes), each pointing along its velocity. the
program prints the latest prediction error every second. `--rviz` shows
them in rviz2 (`sim/rviz/positioning.rviz`, fixed frame `enu`):

- **green arrow**: the ghost, the predicted pose
- **blue arrow**: the kalman estimate now
- **grey arrow**: the baseline, the last fix carried forward with its velocity
- **yellow arrow**: the latest noisy gps fix
- **the `drone` frame**: the true pose
