---
title: "rollchain: package architecture"
purpose: >
  How to turn claude/rollchain_theory.md into a ROS 2 workspace, mapped onto the existing lopsy_ws code.
  Design only: packages, the robot config, layers, data, responsibilities, ROS interfaces, tests and milestones.
  No implementations; the code is meant to be written by hand.
status: "v1 (2026-10-25). Based on rollchain_theory.md v1 and lopsy_ws at commit 27496fb."
---

# rollchain: package architecture

## 0. Decisions that shape everything else

1. **One source of truth for the robot.** A single YAML config describes the sections, links and rods. Both the xacro (the URDF) and the nodes (the model) read it.
   - Why: in lopsy_ws the geometry lives in two places that already disagree. `lopsy.urdf.xacro` hard-codes A1 = 2.85 mm, A2 = 1.80 mm, R1 = 5.10 mm and R2 = 2.00 mm, while the node's defaults are 3.21, 1.0, 6.45 and 2.13 mm, so RViz draws a different robot from the one the model solves. With a variable number of sections this would get worse.
2. **The model library has no ROS in it.** Pure C++ (plus Eigen from milestone M6), tested with gtest. The ROS code only translates parameters and messages into library types and back. Same as lopsy, but enforced by putting the library in its own package (§1).
3. **Structure vs physics.**
   - *Structure:* sections, link counts, link geometry, rods and their offsets. It defines the URDF and the topology, so it is fixed at start-up.
   - *Physics and solver:* $E$, μ, $\lambda_0$, gravity orientation, solver settings. These can change at runtime by rebuilding the model.
4. **The statics never knows the robot is planar.** It works with the lever offset $u_{ij}$ of each rod at each joint (theory §5.1). A spatial version later changes only the geometry builder and the kinematics.
5. **Keep lopsy's good patterns:** layers that only depend downward, `ValueAndDerivative`, the immutable model class (build a new model instead of calling setters), and one test file per layer.

---

## 1. Workspace layout

| Package | lopsy counterpart | Contents | Depends on |
|---|---|---|---|
| `rollchain_interfaces` | lopsy_interfaces | messages (§4.1) | std_msgs |
| `rollchain_model` | the library half of lopsy_kinematics | pure C++ library and its gtests (§3) | Eigen (from M6) |
| `rollchain_kinematics` | the node half of lopsy_kinematics | state publisher, tension publisher, slider GUI, parameter/message adapters | rclcpp, rollchain_model, rollchain_interfaces, sensor_msgs |
| `rollchain_description` | lopsy_description | robot configs, xacro, meshes, RViz config, display launch | xacro, robot_state_publisher |
| `rollchain_bringup` | lopsy_bringup | launch files that pass one config to everything | all of the above |

- **Why split the library out:** the library's CMake then cannot pull in rclcpp by accident, and the tests build without ROS. The price is exporting a library target from an ament package (`ament_export_targets`), which is worth learning but is extra CMake.
- **If you'd rather not yet:** keep lopsy's single-package layout (library target plus nodes in `rollchain_kinematics`), and split later. The layers below are the same either way.

---

## 2. The robot config

### 2.1 Schema
One file per robot in `rollchain_description/config/`. The reference robot of theory §11.2 doubles as the first config and as the test fixture:

```yaml
/**:                      # wildcard: every node reads the same file
  ros__parameters:
    base:
      length: 0.015        # base height below its flat top [m]
    sections:              # one entry per section, base to tip
      n:      [10, 8, 6]
      L:      [0.015, 0.015, 0.015]
      R:      [0.004, 0.004, 0.004]
      D:      [0.0062, 0.0062, 0.0062]
      weight: [5.685e-3, 5.685e-3, 5.685e-3]   # one link [N]
      y_cg:   [7.35e-3, 7.35e-3, 7.35e-3]
    rods:                  # one entry per rod, listed section by section
      name:    ["1L", "1R", "2L", "2R", "3L", "3R"]
      section: [1, 1, 2, 2, 3, 3]
      x:       [1.6e-3, -1.6e-3, 2.0e-3, -2.0e-3, 2.4e-3, -2.4e-3]   # + = left
      r:       [0.3e-3, 0.3e-3, 0.3e-3, 0.3e-3, 0.3e-3, 0.3e-3]
      E:       [61.0e9, 61.0e9, 61.0e9, 61.0e9, 61.0e9, 61.0e9]
    holes:
      lambda0: 2.2e-3
      r_h: 0.4e-3
    friction:
      mu: 0.66
```

### 2.2 Rules
- **SI units everywhere**, in the file, the model and the URDF.
- **Arrays per section and per rod** rather than nested lists of structs, because ROS 2 parameters cannot hold arrays of structs. All section arrays must have length $S$, and all rod arrays the same length.
- **Rods listed section by section** (section numbers non-decreasing). Validate and reject rather than sort: the order is also the order of the tension message, so silently re-ordering would mis-assign tensions.
- **Rod names unique.** They label the GUI sliders and the message entries.
- **Structure parameters have no defaults** in the node. A missing or misspelled key then fails at start-up instead of silently using a built-in robot.
- **xacro reads the same file** through `xacro.load_yaml(...)`, indexing into `['/**']['ros__parameters']`.

### 2.3 Which keys are which

| Kind | Keys | Changeable at runtime |
|---|---|---|
| Structure | `base.length`, `sections.n/L/R/D`, `rods.name/section/x` | no: they describe the build. The URDF depends on the section keys, the topology and the message interface on the rod keys |
| Physics | `sections.weight/y_cg`, `rods.r/E`, `holes.*`, `friction.mu` | yes: rebuild the model |
| Mounting and loads | `gravity.enabled`, `gravity.psi0` (node parameters, §4.2) | yes: passed per solve |
| Solver | `solver.*` (node parameters) | yes: rebuild the model |

The rod offsets `rods.x` never appear in the URDF, but they are part of the physical build, so treat them as structure.

---

## 3. rollchain_model: the library

### 3.0 Layers

```
params ──► topology ──► joints ──► mechanics ──► statics ──► model
                          │                        ▲
                          └──► kinematics ──► loads┘
                                    │
                                    └──► outputs (pull-in, contact forces, diagnostics)
```

Each layer only uses the layers to its left. The model class owns the data and calls the solvers.

### 3.1 params: the input data
Plain structs, no logic beyond defaults.

**`SectionParams`**: `n`, `L`, `R`, `D`, `weight`, `y_cg` (one entry of the `sections` arrays).

**`RodParams`**: `name`, `section`, `x`, `r`, `E` (one entry of the `rods` arrays).

**`RobotParams`**

| Field | Meaning |
|---|---|
| `base_length` | height of the base below its flat top |
| `sections` | `std::vector<SectionParams>`, base to tip |
| `rods` | `std::vector<RodParams>`, section by section |
| `mu`, `lambda0`, `r_h` | friction, setback, hole radius |

**`Loads`** (per solve, not part of the robot)

| Field | Meaning |
|---|---|
| `gravity_in_plane`, `psi0` | whether gravity acts in the bending plane, and the base orientation (theory §6.6) |
| `F_tip`, `M_tip`, `follower` | tip force (axial, lateral), tip moment, and whether the force turns with the tip |

**`SolverSettings`**: lopsy's `abs_smoothing`, `bracket_margin`, `stop_stiffness`, `angle_tolerance`, `max_iterations`, without `blend_width`. New:
- `method`: automatic, sequential, fixed point or Newton. Automatic means sequential without loads, otherwise the best coupled solver you have implemented.
- for the coupled solvers: `residual_tolerance`, `max_coupled_iterations`, `load_steps`.

**`validate(const RobotParams&)`** collects every problem into one message and throws `std::invalid_argument` once. Checks:
- at least one section and one rod; every `n ≥ 1`; positive lengths, radii and moduli;
- rod sections in range and non-decreasing; rod names unique;
- $R_s \ge D_s/2$; $|x_j| < R_s$ for every section the rod crosses (theory §4.5);
- holes inside the link: $|x_j| + r_h$ plus a wall $\le D_s/2$;
- $|x_j| > r_h$, so the joint range (RC5) is positive (this also rejects a rod on the axis, theory §9.1);
- $\lambda_0 \le \tfrac12$ of the shortest hole length.

### 3.2 topology: who crosses what
Built once from `RobotParams`.

| Data | Meaning | Theory |
|---|---|---|
| `N`, `S` | number of joints and sections | §2.1 |
| `last_link[s]` | $N_s$, with `last_link[0] = 0` | §2.1 |
| `section_of[i]` | $\mathrm{sec}(i)$ for every joint | §2.1 |
| `first_rod[s]` | index of the first rod of section $s$ | — |

**The suffix trick.** Because rods are listed section by section, the rods crossing joint $i$, $\mathcal J_i = \{j : s_j \ge \mathrm{sec}(i)\}$ (RC1), are exactly the rods from `first_rod[sec(i)]` to the end of the list. No per-joint lists are needed: a joint's rods are an index range. A rod's anchor link is `last_link[s_j]`.

### 3.3 joints: per-joint geometry (replaces link_geometry)
**`RodAtJoint`**: one rod as seen by one joint.

| Field | Meaning | Theory |
|---|---|---|
| `rod` | index into the rod list (and the tension vector) | — |
| `u` | lever offset $u_{ij}$ (= $x_j$ in the planar robot) | §5.1 |
| `phi`, `h0` | hole-exit angle $\varphi_{ij}$ and straight gap height $h^0_{ij}$ | (RC3) |
| `EI`, `r` | bending stiffness and radius of the rod | — |

**`Joint`**: replaces lopsy's `Link`.

| Field | Meaning | Theory |
|---|---|---|
| `index`, `section` | $i$ and $\mathrm{sec}(i)$ | §2.1 |
| `R`, `L` | arc radius and length of link $i$ | §2.1 |
| `rods` | `std::vector<RodAtJoint>`, the rods of $\mathcal J_i$ | (RC1) |
| `theta_plus`, `theta_minus` | joint range | (RC5) |
| `k0` | stiffness at θ = 0 (for the Newton start) | (RC4) |
| `weight`, `y_cg` | link weight and CG depth | §6.6 |

**`build_joints(params, topology)`** returns `std::vector<Joint>`, base to tip.

In the planar robot all joints of a section are identical apart from their index. Storing them per joint anyway costs nothing (tens of joints, a handful of rods) and is what the spatial version needs, where $u_{ij}$ changes from joint to joint.

### 3.4 mechanics: one joint, pure functions of θ
Every function returns the value and its derivative, as in lopsy.

| Function | Does | Theory | From lopsy |
|---|---|---|---|
| `smooth_abs` | smooth $\lvert x\rvert$ for the capstan wrap | §6.3 | copy |
| `arc_factor` | ρ = θ / sin θ with its series near 0 | (RC10) | copy |
| `free_length` | $s = h\rho + 2\lambda_0$ | (RC10) | copy |
| `gap_height` | $h_{ij}(\theta) = R(1-\cos(\varphi_{ij}-\theta))$, one formula for every rod and both signs | (RC9) | **new**, replaces both branches and the blend |
| `joint_stop_moment` | stiff stop past `theta_plus` / `-theta_minus` | §5.3 | adapt (new limits) |
| `elastic_moment` | $\theta\sum E_jI_j/s_{ij}$ over the joint's rods, plus the stop | (RC10)–(RC11) | adapt (loop over rods) |
| `rod_strain` | largest $r_j\lvert\theta\rvert/s_{ij}$ over the joint's rods | §6.4 | adapt |

**Dropped:** `blend_weight`, `contact_radius` (R is constant), `gap_heights_exact` and the left/right branches.

### 3.5 tensions
- **`Tensions`**: one value per rod, in config order (`std::vector<double>`).
- **`attenuate(T, joint, theta, ...)`** applies the capstan factor of (RC7) across one joint to the joint's rods, with a sign κ per rod (baseline all +1). Rods whose anchor is below the joint are not touched.
- After a rod's anchor link, set its entry to 0 in the stored tension table. The anchor takes the rest of the tension, and outputs then read naturally.
- κ per joint and rod (theory §6.3) can come later. Pass κ as an argument from the start, so adding it does not change any interface.
- **Negative inputs** are clamped to 0 (rods cannot push), as in lopsy.

### 3.6 statics
| Function | Does | Theory |
|---|---|---|
| `joint_residual(joint, θ, T_below, M_load, ...)` | $g_i$ and $g'_i$; `M_load` is a known load moment (0 without loads) | (RC20) |
| `solve_joint(...)` | lopsy's bracketed Newton, with the bracket from `theta_plus/minus` and the start from (RC15) | §7.1 |
| `solve_sequential(joints, F, M_load)` | base to tip: solve joint $i$, then attenuate the tensions for joint $i+1$ | §7.1 |
| `solve_fixed_point(model, F, loads)` | repeat: kinematics → load moments → `solve_sequential` with them, until θ stops changing | §7.2 |
| `solve_newton(model, F, loads)` | residual vector (RC12), Jacobian (RC22), Eigen `PartialPivLU`, continuation in the load scale from 0 to 1 | §7.2, §8 |

**`State`** (the result of a solve)

| Field | Meaning |
|---|---|
| `theta` | $\theta_1..\theta_N$ |
| `tension` | table of $T_{j,i}$, row $i = 0..N$ (row 0 = base), column $j$ = rod |
| `converged`, `iterations`, `method` | what the solver did |

Notes:
- `solve_sequential` only needs $|\theta|$, not its derivative, so it can use the exact value. Smoothing matters only for the Jacobian (§8 of the theory).
- Without loads, every joint above the anchor of the last tensioned rod is exactly straight, so `solve_sequential` can stop there. Not with loads: then those joints carry load moments.
- Implement the fixed point first. It reuses `solve_sequential`, needs no linear algebra, and already handles gravity. Newton (M7) is the upgrade for heavy loads, where the fixed point converges slowly or not at all.
- Contact forces stay out of `State`, as in lopsy: compute them on demand (§3.8).

### 3.7 kinematics and loads (new)
- **2D vectors:** `Eigen::Vector2d` as (axial, lateral). Two free helpers: `moment(r, F)` (RC2) and `rot90(r)` (the $J$ of §3.2 of the theory).
- **`forward_kinematics(joints, theta)`** returns a **`Chain`**:
  - `top[0..N]` (top centre of each link, `top[0]` = base origin), `contact[1..N]` ($\mathbf O_i$), `cg[1..N]` and the absolute angles `Theta[0..N]`;
  - built joint by joint from (RC18) and the contact-point rule of §6.2.
- **`load_moments(chain, joints, loads)`**: $M^G_i + M^{ext}_i$ for every joint (RC16).
- **`load_arm_sums(chain, joints, loads)`**: $V_i$ for the Jacobian (RC21).

**Frames.** The theory's base frame is X axial, Y left. Keep the URDF consistent with lopsy's: z along the link axis, x to the left, joints rotating about +y (positive rotation tilts z toward x, so θ > 0 bends left). Then URDF (x, z) = theory (Y, X), and the URDF origin sits `base_length` below the theory's origin. The tip from `forward_kinematics` should match `ros2 run tf2_ros tf2_echo base_link tip` after this mapping, which is a good check at M5.

### 3.8 outputs, computed on demand
| Function | Does | Theory |
|---|---|---|
| `rod_pull_in(topology, joints, theta)` | geometric pull-in per rod; the stretch term is optional | (RC19) |
| `contact_forces(chain, state, loads, split)` | normal $F^R_i$ and tangential $H_i$; `split` chooses where the rods are turned (bisector by default) | (RC17) |
| `diagnostics(...)` | peak strain, joints at a stop, smallest $F^R_i$, largest $\lvert H_i\rvert/F^R_i$, residual norm | §6.4, §6.7 |

### 3.9 The model class
**`RollchainModel`**: same invariant as lopsy's `ContinuumRobotModel`.
- **Constructor:** takes `RobotParams` and `SolverSettings`, calls `validate()`, then builds the topology and the joints. Declare the members in that order, the joints last, since they are built from the others.
- **`solve(F, loads)`:** dispatches on `settings.method`.
- **Accessors** for params, settings, topology and joints. No setters: to change a parameter, build a new model.

---

## 4. ROS side

### 4.1 rollchain_interfaces
- **`RodTensions.msg`** (input): `std_msgs/Header header`, `string[] name`, `float64[] tension`.
  - Names make the message self-describing in `ros2 topic echo` and let the node check that it matches the config. lopsy's message has neither a header nor names.
- **`RodState.msg`** (output, M8): `header`, `name[]`, tension at the base, tension at the anchor, `pull_in[]`, `converged`, `max_strain`.

### 4.2 rollchain_state_publisher (generalises lopsy_state_publisher)
- **Start-up**
  - Declare the structure parameters without defaults and read-only; declare physics and solver parameters with defaults. Convert them to `RobotParams` and `SolverSettings` in one adapter function.
  - Build the model inside a try block. On `std::invalid_argument`, log the message as fatal and exit: a bad config must stop the launch.
  - Build the joint names once: `j1_roll … jN_roll`.
- **Subscription `rod_tensions`:** check that the names match the config, map them to the rod order, clamp negatives, solve, publish.
- **Publications:** `joint_states` with $N$ entries ($\theta_i$ directly; robot_state_publisher fills in the mimic slides), and `rod_state` from M8.
- **Runtime parameters:** in an on-set-parameters callback, build a new model from the changed physics or solver values.
  - If the new model fails validation, reject the change (result `successful = false`, with the validation message as the reason) and keep the old model.
  - Structure parameters are read-only, so they cannot change at runtime.
- **Lessons from lopsy**
  - Keep only the model. lopsy's node also stores `params_` and `solver_settings_`, which can drift from the model's own copies.
  - With the default single-threaded executor, parameter callbacks and subscriptions never run at the same time, so replacing the model in a callback is safe. A multi-threaded executor would need a mutex.
  - If a solve does not converge, do not publish it; warn, throttled.
- **Later:** a `tip_wrench` subscription (`geometry_msgs/WrenchStamped`, in the URDF base frame: axial = z, lateral = x, the moment promoting +θ = torque about +y).

### 4.3 rod_tension_publisher and tension_slider_gui
- Both read `rods.name` from the same config, so they show one slider or parameter per rod, whatever $S$ is.
- The GUI's `TensionControl` widget carries over unchanged; build the list of controls from the names. Replace "Zero both" with "Zero all".
- Optional later: a "decouple" helper that sets the antagonist of the section below from (RC14), so that only one section bends (theory §6.5).

### 4.4 rollchain_description
- **`config/`:** the reference robot (§2.1); later one file per prototype.
- **`urdf/rollchain.urdf.xacro`:** loads the config, then two nested recursive macros, one over sections and one over the links of a section, with a global link counter $i$. Per joint:
  1. `j{i}_slide` (prismatic along x, origin at the parent's flat top, mimic of `j{i}_roll` with multiplier $R_s$);
  2. `j{i}_roll` (revolute about +y, $R_s$ above the parent's top);
  3. `j{i}_to_link` (fixed, $-R_s$ along z), giving `link_{i}` with its origin at the crest and its flat top at $z = L_s$.

  Then a fixed `tip` frame on the last link.
  - The parent's flat top is at the parent's $L$, which changes at section boundaries; pass it down the recursion.
  - Give the revolutes generous limits (for example ±π/2). The model computes and enforces the real range (RC5), and duplicating that logic in xacro would create a second source of truth.
  - Visuals: a cylinder per link to start; per-section meshes once the CAD exists, named in the config if you like.
- **`launch/display.launch.xml`** with joint_state_publisher_gui, to check the chain before the model is connected.

### 4.5 rollchain_bringup
One launch argument `config` (default: the reference robot) goes to xacro (`config:=...`) and, as the parameter file, to every node. This is what makes §0 decision 1 hold.

---

## 5. Tests (gtest, one file per layer, in rollchain_model)

Put a `reference_robot()` builder in a shared test header: the §2.1 robot as `RobotParams`. Tests build params in C++ and do not parse YAML.

| File | Checks | Expected values |
|---|---|---|
| `test_params.cpp` | `validate()` rejects: unsorted rods, duplicate names, $\lvert x\rvert \ge R$, $\lvert x\rvert \le r_h$, a hole outside the link, an empty section, $\lambda_0$ too large | — |
| `test_topology.cpp` | $N$, `last_link`, `section_of`, rod ranges for theory §2.3 | $N = 24$; $N_s$ = 10, 18, 24; 6/4/2 rods per joint |
| `test_joints.cpp` | φ, $h^0$, range, $k^0$ | ranges 17.19°, 22.92°, 28.65°; $k^0$ = 0.4704, 0.3065, 0.1493 N·m/rad |
| `test_mechanics.cpp` | `gap_height` equals lopsy's (C9a/b) with $A_1 = A_2$, $R_1 = R_2$; every derivative against central differences; $M^{el}$ odd for the symmetric layout; $M^{el\prime} > 0$ over the range | min $M^{el\prime}$ = 0.132 N·m/rad (reference robot) |
| `test_statics.cpp` | zero tension → straight; negative → slack; only section-1 rods → joints 11–24 exactly 0; decoupling ($F_{2R} = 1.2\,F_{3L}$) → joints 1–18 exactly 0; scale $(F, E)\to\lambda(F, E)$; mirror L ↔ R; passive rods with $E = 0$ equal a single 24-link section; small-angle form (RC15) | theory §11.1 |
| `test_statics.cpp` | one section, lopsy equivalence (table below); reference cases A–D | theory §11.2 |
| `test_kinematics.cpp` | straight robot: tip at $\sum L$; one joint against (RC18); `contact` and `cg` against (RC6) by finite differences; joint-state names | — |
| `test_loads.cpp` | (RC16) equals the finite-difference derivative of the loads' work; Jacobian (RC22) against central differences; frictionless Jacobian symmetric; cases E–F by fixed point and by Newton | theory §11.1–11.2 |

**One section, lopsy equivalence.** $S = 1$, $n = 15$, $L$ = 15 mm, $R$ = 4 mm, rods at ±2.2 mm, $r$ = 0.3 mm, $E$ = 61 GPa, μ = 0.66, $\lambda_0$ = 2.2 mm. Range 25.78°, $k^0$ = 0.15341 N·m/rad. Angles in degrees:

| $F_L$, $F_R$ [N] | θ₁ | θ₁₅ (exact $\lvert\theta\rvert$) | θ₁₅ (smoothing ε = 1e-4 rad) |
|---|---|---|---|
| 10, 0 | 6.53138 | 3.39982 | 3.40198 |
| 0, 7 | −4.86976 | −2.85157 | −2.85350 |
| 12, 3 | 5.32352 | 3.15480 | 3.15676 |
| 25, 0 | 12.53536 | 4.83332 | 4.83596 |

Your lopsy_kinematics code, run with A1 = A2 = 2.2 mm and R1 = R2 = 4 mm, reproduces the smoothed column to 7e-13 rad, so it can serve as a second oracle.

**Tolerances.** The theory values are rounded to 5 decimals in degrees: compare to 1e-5°. If the sequential path uses smoothing, the angles move by up to 0.004°; either test with exact $|\theta|$ there or loosen the tolerance. If you allow ε = 0, guard the derivative of `smooth_abs` at 0 (it is 0/0).

---

## 6. Milestones

| # | Build | Done when |
|---|---|---|
| M0 | Workspace skeleton: the five packages, empty library, one passing gtest; pick a license | `colcon build` and `colcon test` pass |
| M1 | params, `validate()`, topology | `test_params`, `test_topology` pass |
| M2 | joints | `test_joints` pass |
| M3 | mechanics | `test_mechanics` pass |
| M4 | tensions and `solve_sequential` | `test_statics` pass, including cases A–D and the lopsy table |
| M5 | model class, interfaces, state publisher, slider GUI, xacro, bringup launch | RViz shows the reference robot moving with six sliders; tip from `forward_kinematics` matches TF |
| M6 | Eigen, kinematics, loads, fixed-point solver | cases E–F pass; gravity switchable at runtime |
| M7 | Jacobian and Newton | Jacobian tests pass; Newton and fixed point agree |
| M8 | pull-in, contact forces, diagnostics, `RodState`, runtime physics parameters | values published; an invalid parameter change is rejected and the old model kept |
| Later | κ per joint, displacement input (theory §7.3), spatial sections (theory §10) | — |

The library is usable after M4, and the robot is visible after M5. Everything after that adds loads and outputs.

---

## 7. From lopsy: copy, adapt, drop

| lopsy | rollchain |
|---|---|
| `ValueAndDerivative`, `smooth_abs`, `arc_factor`, `free_length` | copy |
| `solve_joint` (bracketed Newton with bisection fallback) | copy the algorithm; new bracket and start |
| `ContinuumRobotModel` (members built in the constructor, no setters) | same pattern in `RollchainModel`, plus `validate()` |
| `elastic_moment`, `joint_stop_moment`, `rod_strain`, `tension_after`, `solve` | adapt: loop over the joint's rods; tension vectors instead of `{left, right}` |
| `RobotParams` / `SolverSettings` | replace with the structs of §3.1 |
| `Link` (A_l, A_r, R_l, R_r, alpha, beta, Gamma, sigma …) | replace with `Joint` and `RodAtJoint` |
| `blend_weight`, `contact_radius`, gap branches and their blend | drop (single arc) |
| `contact_forces` (C11/C13) | drop; new version from (RC17) |
| `urdf_joint_names` / `urdf_joint_positions` (l/r split, 2n joints) | simplify: $N$ names, positions = θ |
| `RodTensions{left_tension, right_tension}` | replace with names and an array, plus a header |
| xacro with 15 explicit macro calls and hard-coded geometry | recursive macros reading the config |
| per-layer gtest files | same structure |
| `TensionControl` widget, launch structure | copy; build the controls from the config |

lopsy_ws itself can stay as it is: the published 1-DOF asymmetric reference. If you touch it again, the cheapest fix is to make the xacro read the node's geometry values, so the two cannot disagree.
