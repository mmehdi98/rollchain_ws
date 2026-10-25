# rollchain

ROS 2 workspace for **rollchain**: a planar continuum robot made of any number of sections of symmetric rolling links, driven by superelastic NiTi rods. Each rod runs from the base to the last link of its own section, so the links of section *s* carry the rods of sections *s..S*.

Give it rod tensions; it computes the static shape (one angle per joint) with the model in `docs/theory.md`, and publishes it as joint states for `robot_state_publisher` and RViz.

It scales the single-section asymmetric robot of [lopsy_ws](https://github.com/mmehdi98/lopsy_ws). With one section and symmetric links it gives the same angles as `lopsy_kinematics` (to 7e-13 rad).

## Packages

| Package | Contents |
|---|---|
| `rollchain_model` | The model as a pure C++17 library (Eigen, no ROS) and its gtests |
| `rollchain_kinematics` | Nodes: `rollchain_state_publisher`, `rod_tension_publisher`, `tension_slider_gui` |
| `rollchain_interfaces` | Messages `RodTensions` (in) and `RodState` (out) |
| `rollchain_description` | Robot configs, the URDF xacro, RViz config |
| `rollchain_bringup` | Launch file for the whole robot |

## Build and test

ROS 2 Humble or newer (tested on Jazzy).

```bash
cd rollchain_ws
rosdep install --from-paths src --ignore-src -y
colcon build
colcon test && colcon test-result --verbose
source install/setup.bash
```

## Run

```bash
ros2 launch rollchain_bringup rollchain.launch.xml
```

RViz, the model, and a slider window with one slider per rod. Arguments:

| Argument | Default | |
|---|---|---|
| `config` | `rollchain_description/config/reference_robot.yaml` | the robot config |
| `gui` | `true` | slider window; `false` starts `rod_tension_publisher` instead (set `tension.<rod>` parameters) |
| `rviz` | `true` | start RViz |
| `reconfigure` | `false` | start `rqt_reconfigure`, to tune physics, gravity and solver parameters live |
| `max_tension` | `50.0` | slider range [N] |

To check the URDF alone, with a slider per joint: `ros2 launch rollchain_description display.launch.xml`.

## The robot config

One YAML file describes the robot. The xacro reads it (`xacro rollchain.urdf.xacro config:=<file>`) and every node gets it as its parameter file, so the URDF and the model cannot disagree. See `rollchain_description/config/reference_robot.yaml` for a commented example (the reference robot of the theory: 3 sections of 10, 8 and 6 links).

- **Sections** are arrays with one entry per section: `sections.n`, `L`, `R`, `D`, `weight`, `y_cg`.
- **Rods** are arrays with one entry per rod, listed section by section: `rods.name`, `section`, `x` (lateral offset, + = left), `r`, `E`.
- **Holes and friction:** `holes.lambda0`, `holes.r_h`, `holes.wall`, `friction.mu`.
- **Units:** SI. Write exponents with a sign (`61.0e+9`): xacro reads the file with PyYAML, which reads `61.0e9` as a string.
- **Structure keys are fixed at start-up:** `base.length`, `sections.n/L/R/D`, `rods.name/section/x`. Everything else can be changed at runtime.

The node validates the config when it starts and lists every problem at once, for example a hole that does not fit in the link or a rod inside the contact path.

## rollchain_state_publisher

| | Name | Type |
|---|---|---|
| subscribes | `rod_tensions` | `rollchain_interfaces/RodTensions`: by name (partial updates allowed) or one value per rod |
| subscribes | `tip_wrench` | `geometry_msgs/WrenchStamped`: frame `base_link` = dead load, frame `tip` = follower load; force.z axial, force.x lateral, torque.y bends left |
| publishes | `joint_states` | `sensor_msgs/JointState`: `j1_roll` … `jN_roll` |
| publishes | `rod_state` | `rollchain_interfaces/RodState`: base and anchor tensions, pull-in, strain, stops, contact checks |

Parameters, besides the robot config:
- `gravity.enabled`, `gravity.psi0`: in-plane gravity; `psi0` 0 = upright, π = hanging.
- `solver.method`: `automatic` (sequential without loads, Newton with loads), `sequential`, `fixed_point` or `newton`.
- other `solver.*` tolerances and limits.
- `publish_rate`: how often the last joint states are re-published.

A parameter change builds a new model. If the new model is invalid, the change is rejected with the reason and the old model is kept.

## Using the library without ROS

```cpp
#include "rollchain_model/model.hpp"
using namespace rollchain_model;

RollchainModel model(params);                     // RobotParams; throws std::invalid_argument if invalid
State s = model.solve({0, 0, 0, 0, 10.0, 0});     // one base tension per rod [N]
Chain c = model.kinematics(s);                    // c.tip(), c.top[k], c.contact[k], ...
Loads upright; upright.gravity_in_plane = true;
State g = model.solve({0, 0, 0, 0, 10.0, 0}, upright);  // coupled solve (Newton)
```

## Tests

`rollchain_model` has 62 gtests, one file per layer:
- parameter validation;
- topology;
- joint geometry against the theory's joint ranges and stiffnesses;
- mechanics: lopsy's gap formulas, and every derivative against finite differences;
- statics: termination, exact decoupling, scale and mirror symmetry, the small-angle form, the one-section lopsy values, and reference cases A–D of theory §11.2;
- kinematics: rotation about the contact point, and work-consistent pull-in;
- loads: load moments against the work of the loads, the Jacobian against finite differences, and cases E–F by Newton and by fixed point.

`rollchain_kinematics` tests the parameter adapters.

## Notes

- **Planar only.** The statics already work with per-joint lever offsets, so the spatial version (theory §10) changes only the geometry builder and the kinematics.
- **Visuals** are cylinders coloured by section. Replace them with meshes of the real links in the xacro.
- **xacro builds the chain recursively.** That works up to about 150 links; beyond that it hits Python's recursion limit.
- **Docs:** `docs/theory.md` is the model; `docs/architecture.md` explains the design. The theory builds on the corrected 1-DOF model of lopsy.
- **License:** not chosen yet (`TODO` in every `package.xml`).
