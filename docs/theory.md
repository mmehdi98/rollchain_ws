---
title: "rollchain: static model of a multi-section, symmetric, rod-driven rolling-joint continuum robot — theory reference"
purpose: >
  Theory-only reference for implementing the rollchain package (C++, ROS 2): a planar continuum robot made of
  S sections of symmetric rolling links, with any number of links per section, driven by NiTi rods that run from
  the base to the end of their own section. It generalises the corrected 1-DOF model and contains no code.
status: >
  v1 (2026-10-25). Built on the 1-DOF theory v2, whose joint balance (C16) and constant-setback free length (C17)
  are [PROPOSED] there and were identified on the single-section asymmetric prototype. Nothing here has been
  validated on a multi-section robot. The equations were checked numerically for internal consistency, and with
  one section the model reproduces the lopsy_kinematics code run with A1 = A2 and R1 = R2 (§11.1).
based_on: "claude/1dof_corrected_model_theory.md (v2), cited below as 1-DOF"
sources:
  thesis: "M. Lari, MSc thesis, Università di Roma Tor Vergata, A.Y. 2023/2024"
  paper: "M. Lari, M. Russo, IEEE RA-L 10(10), 2025, doi:10.1109/LRA.2025.3597860"
units: "SI (m, N, rad) in the model; mm and degrees for reporting"
---

# rollchain: static model of a multi-section symmetric rolling-joint robot

## 0. How to read this

**Parent model.** This document extends the 1-DOF theory. Where it says nothing else, the 1-DOF definitions hold. Equations taken over unchanged cite their 1-DOF number.

**Scope.** A planar robot: all joint axes are parallel. It has $S$ sections, $n_s$ links in section $s$, and in the baseline two rods per section. The spatial version (alternating joint axes) is outlined in §10.

**Status tags**

| Tag | Meaning |
|---|---|
| `[1-DOF]` | Carried over from the 1-DOF theory, with the status it has there |
| `[NEW]` | Derived here; checked numerically for internal consistency (§11.1), not against measurements |
| `[OPEN]` | Unresolved; see §12 |

New equations are numbered (RC1), (RC2), …

**What carries over and what changes**
- **Kept and generalised:** the joint balance (C16) and the setback free length (C17), now summed over every rod that crosses a joint.
- **Removed by the symmetric link:** the two arcs and their blend (C4), the closing/opening branches (C9a/b), the crest offset Γ, the mirror sign σ and the mirroring of 1-DOF §5.9.
- **Not carried over:** the per-link statics (C10)–(C13), which 1-DOF §10.1 showed to be inconsistent. Contact forces come from the same free-body cut as the angles (§6.7).
- **New:** section and rod bookkeeping, a friction direction per rod, gravity and external loads in the joint balance, the rod pull-in, and the full Jacobian with loads.

---

## 1. The robot

- **Links.** Every link is symmetric: flat on top, with a bottom profile that is one circular arc of radius $R$ whose lowest point (the crest) lies on the link axis. Each link rolls without slip on the flat top of the link below. Link 0 is the fixed base.
- **Sections.** The links form $S$ sections in series, numbered from the base. Section $s$ has $n_s$ links of length $L_s$ (crest to flat top) and arc radius $R_s$. $S$ and every $n_s$ are free parameters, and sections may differ in $L$, $R$ and diameter $D$.
- **Rods.** Superelastic NiTi rods, as in the 1-DOF robot. Each rod belongs to one section: it runs from the base through every link up to the last link of its section, where it is fixed.
  - The rods of section $s$ therefore pass through every link of sections $1..s$, and the links of section $s$ carry the rods of sections $s..S$.
  - Baseline: two rods per section, one on the left and one on the right (an antagonistic pair).
- **Actuators and backbone.** A rod's tension acts on every joint it crosses, which is both the actuation and the coupling between sections. Every rod crossing a joint adds bending stiffness there, pulled or not.
- **Actuation.** One motor per rod. The input is each rod's tension at the base, and rods cannot push.
- **Inputs and outputs.**
  - Inputs: the rod tensions $F_j$ at the base ($2S$ in the baseline), and optionally gravity and a tip load.
  - Outputs: the $N$ joint angles, the shape, the rod tensions along the robot, the contact forces, the rod pull-in at the base and the rod strains.

---

## 2. Topology and indexing

### 2.1 Sections and links
- Sections $s = 1..S$ from the base. Section $s$ has $n_s \ge 1$ links, and $N = \sum_s n_s$.
- Cumulative counts: $N_0 = 0$ and $N_s = \sum_{q\le s} n_q$. Link $N_s$ is the last link of section $s$, and $N_S = N$.
- Links and joints $i = 1..N$; joint $i$ lies between link $i-1$ and link $i$ (as in 1-DOF). $\mathrm{sec}(i)$ is the section $s$ with $N_{s-1} < i \le N_s$.
- Each link takes its section's geometry: $R_i = R_{\mathrm{sec}(i)}$ and $L_i = L_{\mathrm{sec}(i)}$. Joint $i$ rolls on link $i$'s arc, so the first joint of a section uses that section's radius.

### 2.2 Rods
- Rod $j$ has a section $s_j$ (it is fixed in link $N_{s_j}$), a signed lateral offset $x_j$ from the link axis (positive to the left), a radius $r_j$, an equivalent bending modulus $E_j$ and a base tension $F_j$.
- Section $s$ owns $m_s$ rods. Baseline $m_s = 2$: rod $(s,\mathrm L)$ at $x = +a_s$ and rod $(s,\mathrm R)$ at $x = -a_s$.
- A rod has the same offset in every link it crosses, because it is straight when the robot is straight.

**Rods crossing joint $i$:**

$$\mathcal J_i = \{\,j : s_j \ge \mathrm{sec}(i)\,\}\tag{RC1}$$

that is, every rod of section $\mathrm{sec}(i)$ and of every section after it. Rods of earlier sections are anchored below link $i$ and play no part at joint $i$.

**Holes per link.** A link of section $s$ carries $H_s = \sum_{q=s}^{S} m_q$ holes. With pairs, $H_s = 2(S-s+1)$: the base section carries $2S$ holes and the last section 2. This is how the number of sections sets the number of rods in each section.

### 2.3 Example: $S = 3$, $n = (10, 8, 6)$

| Section | Links | $N_s$ | Rods crossing its joints | Holes per link |
|---|---|---|---|---|
| 1 | 1–10 | 10 | 1L 1R 2L 2R 3L 3R | 6 |
| 2 | 11–18 | 18 | 2L 2R 3L 3R | 4 |
| 3 | 19–24 | 24 | 3L 3R | 2 |

This is also the reference robot of §11.2.

---

## 3. Conventions

### 3.1 Physical frame, no mirroring
- **Link frame** (the same for every link): origin at the crest, **y** along the link axis toward the tip, **x** toward the physical left.
- **$\theta_i$** is the rotation of link $i$ relative to link $i-1$, positive when link $i$ tilts to the left.
- Rod offsets $x_j$ are signed in this frame. Left and right rods differ only in the sign of $x_j$, and no geometry is relabelled with the bending direction.

In 1-DOF, x pointed to the actuated side and right-rod actuation was obtained by mirroring (1-DOF §5.9). Here every rod enters with its physical offset, so pulling a right rod needs no special case.

### 3.2 Moment sign
Vectors in the bending plane are written as (axial, lateral) components, lateral positive to the left. For a vector $\mathbf r = (r_a, r_\ell)$ from a pivot to the point where a force $\mathbf F = (F_a, F_\ell)$ acts:

$$m(\mathbf r,\mathbf F) = r_a F_\ell - r_\ell F_a\tag{RC2}$$

- $m > 0$ promotes positive θ (bending left). This is the 1-DOF convention $\mathbf F\times\mathbf r$ with x lateral and y axial [1-DOF §2.3].
- $m$ is unchanged by rotations in the plane, so it can be evaluated in any link frame or in the base frame.
- $J$ rotates a vector by +90° from axial toward lateral: $J(r_a, r_\ell) = (-r_\ell, r_a)$. Then $m(J\mathbf r, \mathbf F) = -\,\mathbf r\cdot\mathbf F$, which the Jacobian uses (§8).

### 3.3 Base (output) frame
- The origin is the top centre of the base. **X is axial** (the straight robot lies along +X) and **Y is lateral, positive to the left** [1-DOF §2.4, with "actuated side" replaced by "left"].
- Link $i$'s axis is at angle $\Theta_i = \sum_{k\le i}\theta_k$ from the base axis. Its axial and lateral unit vectors are $\hat{\mathbf a}_i = (\cos\Theta_i, \sin\Theta_i)$ and $\hat{\boldsymbol\ell}_i = J\hat{\mathbf a}_i$. $\Theta_N$ is the tip angle.
- The 1-DOF angle from the upward vertical is $\psi_i = \psi_0 + \Theta_i$ (C14).

---

## 4. Parameters

### 4.1 Per section

| Symbol | Meaning |
|---|---|
| $n_s$ | number of links (= joints) in section $s$ |
| $L_s$ | link length, crest to flat top |
| $R_s$ | bottom-arc radius |
| $D_s$ | link diameter (drawing; bounds the hole positions) |
| $G_s$, $y_{CG,s}$ | weight of one link and its CG depth below the flat top ($x_{CG} = 0$ by symmetry) |

### 4.2 Per rod

| Symbol | Meaning |
|---|---|
| $s_j$ | section; the rod is fixed in link $N_{s_j}$ |
| $x_j$ | signed lateral offset from the link axis, positive to the left |
| $w_j$ | depth along the joint axis; places the hole (fit, overlap), not used by the planar statics (§9.2) |
| $r_j$, $E_j$ | radius and equivalent bending modulus; $I_j = \pi r_j^4/4$ |

### 4.3 Global

| Symbol | Meaning |
|---|---|
| $\mu$ | effective rod–link friction coefficient for the $\lvert\theta\rvert$ wrap [1-DOF §5.3] |
| $\lambda_0$ | setback: depth to which a rod is effectively clamped inside each hole (C17) |
| $r_h$ | hole radius (joint range, §5.3) |
| $\psi_0$, $\mathbf F^{tip}$, $M^{tip}$ | gravity direction, tip force and tip moment (optional loads) |

### 4.4 Values to start from
- **From the 1-DOF identification** [1-DOF §10.5]: $E = 61$ GPa, $\lambda_0 = 2.2$ mm and $\mu = 0.66$, if the same rods ($r = 0.3$ mm), hole size and printing process are used.
  - These are effective values tied to the 1-DOF conventions (the $|\theta|$ wrap and the cut lever of §6.5). Re-identify them on the new robot.
  - $E$ was identified with $r = 0.3$ mm. What the data fixed is the joint stiffness, which scales with $E r^4$.
- **Not transferable:** the 1-DOF arc radii (6.45/2.13 mm). The symmetric link is a new design.
- **Link weight** 5.685 mN and $y_{CG} = 7.35$ mm are the 1-DOF link's values [1-DOF §3.1]; use them only for a similar link.

### 4.5 Geometric constraints
- $R_s \ge D_s/2$, so that the arc spans the link, and $|x_j| < R_s$ for every rod crossing section $s$, so that its hole exit lies on the arc.
- Every hole lies inside the link with a wall: $|x_j| + r_h + t_{wall} \le D_s/2$ (with rows, see §9.2).
- $\lambda_0 \le \tfrac12$ of the shortest hole length (1-DOF C17).

---

## 5. Derived quantities

### 5.1 Per joint and rod `[NEW]`
For every rod $j \in \mathcal J_i$:

$$\varphi_{ij}=\arcsin\frac{u_{ij}}{R_i},\qquad h^0_{ij}=R_i\,(1-\cos\varphi_{ij}),\qquad L^t_{ij}=L_i-h^0_{ij}\tag{RC3}$$

- $u_{ij}$ is the **lever offset** of rod $j$ at joint $i$. In the planar robot $u_{ij} = x_j$ at every joint. In the spatial version (§10) it depends on the joint's orientation, so storing $u_{ij}$ per joint and rod keeps the statics code the same for both.
- $\varphi_{ij}$ is the angular position of the rod's hole exit on link $i$'s arc, measured from the crest (signed). $h^0_{ij}$ is the gap height when straight, and $L^t_{ij}$ the hole length. These are the symmetric counterparts of (C1)–(C2).

### 5.2 Joint stiffness at θ = 0 `[NEW]`

$$k^0_i=\sum_{j\in\mathcal J_i}\frac{E_jI_j}{h^0_{ij}+2\lambda_0}\tag{RC4}$$

Proximal joints are stiffer, because more rods cross them. In the reference robot (§11.2), $k^0$ = 0.470, 0.307 and 0.149 N·m/rad in sections 1, 2 and 3.

### 5.3 Joint range `[NEW]`
The contact line must never reach a hole: the rod there would be pinched between the links. On link $i-1$'s top the contact sits $R_i\theta_i$ from the axis, so the innermost hole on each side sets the range:

$$\theta^+_i=\min_{j\in\mathcal J_i,\;u_{ij}>0}\frac{u_{ij}-r_h}{R_i},\qquad \theta^-_i=\min_{j\in\mathcal J_i,\;u_{ij}<0}\frac{\lvert u_{ij}\rvert-r_h}{R_i},\qquad -\theta^-_i<\theta_i<\theta^+_i\tag{RC5}$$

- The innermost holes of any section limit every joint they cross (§9.1).
- Add a stiff one-sided stop past $\theta^+_i$ and $-\theta^-_i$, as in 1-DOF §6.
  - In 1-DOF the stop was only a numerical guard: the pulling rod's lever $A - R\sin\theta$ vanishes exactly at the end of the range, so no tension can drive the joint past it.
  - Here a rod's lever vanishes at its own hole angle $\varphi_{ij}$, which for an outer rod lies beyond the range set by the inner holes. The stop can engage physically.
- 1-DOF took the range up to the hole-exit angle, where the gap height closes. The two definitions differ by about $r_h/R + (\varphi - \sin\varphi)$.

---

## 6. Model

### 6.1 Assumptions
1. **Planar.** All joint axes are parallel. Rods may sit at different depths along the joint axis (§9.2), which does not affect the planar statics.
2. **Links** are rigid and symmetric, and roll without slip.
3. **Rods** are linearly elastic with an equivalent bending modulus $E_j$. Superelasticity is neglected, and so is axial stretch in the statics; stretch enters only the pull-in (§6.9). [1-DOF assumption 2]
4. **Friction** follows the capstan law for every rod, with wrap $|\theta_i|$ at every joint it crosses. The loss at joint $i$ is booked to link $i$, so the tension at the cut through gap $i$ is the tension before that loss. [1-DOF §5.3, and the (C16) convention under which μ was identified]
5. **Sliding direction.** Each rod slides one way at each joint, given by $\kappa_{ij}$ (§6.3). Baseline $\kappa = +1$: every tensioned rod slides toward its actuator (loading). [generalises 1-DOF assumption 7]
6. **Free length** of each rod in each gap: the arc length of the gap plus $2\lambda_0$ (C17).
7. **Rod forces at a cut.** At the cut through gap $i$ each rod transmits its tension along link $i$'s axis at its offset, plus its bending moment, and no shear [as in (C16)]. `[OPEN]` (§12 item 3)
8. **Untensioned rods** are slack (zero tension) but still bend with the joint.
9. **Loads:** optional in-plane gravity and a tip force and moment. No other external loads (in particular no table friction).

### 6.2 Joint kinematics [1-DOF C5, symmetric]
- **Contact point.** $\mathbf O_i$ lies $R_i\sin\theta_i$ to the left of link $i$'s axis and $R_i(1-\cos\theta_i)$ above its crest. On link $i-1$'s flat top it lies $R_i\theta_i$ to the left of link $i-1$'s axis.
- With one arc there is no branch and no blend, and $R_i$ is constant, so every $dR/d\theta$ term of 1-DOF vanishes.

**Instantaneous centre** `[NEW]`. With the other joints fixed, link $i$ and everything above it rotate about the current contact point, while the contact point itself slides along link $i-1$'s top:

$$\frac{\partial\mathbf p}{\partial\theta_i}=J(\mathbf p-\mathbf O_i)\ \ \text{for any point }\mathbf p\text{ on links}\ge i,\qquad \frac{\partial\mathbf O_i}{\partial\theta_i}=R_i\,\hat{\boldsymbol\ell}_{i-1}\tag{RC6}$$

The load terms and their Jacobian are built on this.

### 6.3 Rod tensions and friction `[NEW]` (generalises C6–C7)
Every rod crossing joint $i$ turns through $\theta_i$ there, whatever its offset, so every rod sees the same wrap $|\theta_i|$:

$$T_{j,0}=F_j,\qquad T_{j,i}=T_{j,i-1}\,e^{-\kappa_{ij}\,\mu\,\lvert\theta_i\rvert}\quad(i\le N_{s_j}),\qquad\text{so}\quad T_{j,i-1}=F_j\,e^{-\mu\sum_{k<i}\kappa_{kj}\lvert\theta_k\rvert}\tag{RC7}$$

- $T_{j,i}$ is rod $j$'s tension just above link $i$, in gap $i+1$.
- $\kappa_{ij} = +1$: rod $j$ slides toward the base at joint $i$ (toward its actuator), so its tension decreases toward the tip. $\kappa_{ij} = -1$: it slides toward the tip (paid out under tension), so its tension increases toward the tip.
- **Baseline** ($\kappa = +1$ everywhere, common μ): all rods share one attenuation factor,

$$T_{j,i-1}=F_j\,\eta_{i-1},\qquad \eta_{i-1}=e^{-\mu\sum_{k<i}\lvert\theta_k\rvert}\tag{RC8}$$

- **When κ = +1 holds:** for a tensioned rod whose path shortens along its whole length during loading. It fails for an antagonist paid out under preload, and can fail for a distal rod crossing a proximal section that bends away from it. Then κ is a per-joint input, and a rod that sticks has an unknown tension within the capstan limits (§12 item 2).
- **Friction on link $i$**, needed only for a per-link force balance: $f_i = \sum_{j\in\mathcal J_i}(T_{j,i-1}-T_{j,i})$.

### 6.4 Elastic moment `[NEW]` (symmetric form of C8, C9, C17)
After link $i$ rolls by θ, the hole exit of rod $j$ sits at height

$$h_{ij}(\theta)=R_i\,\big(1-\cos(\varphi_{ij}-\theta)\big)\tag{RC9}$$

above link $i-1$'s flat top. The arc centre stays at height $R_i$ above the flat, and the hole exit is at angle $\varphi_{ij}-\theta$ from the lowest point of the rotated arc.
- **One formula** for every rod and both signs of θ. With $R_1 = R_2$ and $A_1 = A_2$ it equals (C9a) and (C9b) for both the closing and the opening gap (§11.1). The thesis's opening-gap approximation, which 1-DOF §5.4 corrected, is exact for a symmetric link.
- $h_{ij} \to 0$ when the contact reaches the hole exit ($\theta\to\varphi_{ij}$), which the range (RC5) excludes.

Free length, and the elastic moment of all rods crossing the joint:

$$s_{ij}(\theta)=h_{ij}(\theta)\,\frac{\theta}{\sin\theta}+2\lambda_0,\qquad M^{el}_i(\theta)=\theta\sum_{j\in\mathcal J_i}\frac{E_jI_j}{s_{ij}(\theta)}\ \ (+\ \text{stop})\tag{RC10}$$

- Rods of every section crossing joint $i$ contribute, pulled or not.
- $M^{el}_i$ is odd in θ when the rod layout is symmetric about the axis (pairs at $\pm a$).
- **Derivative** (symmetric form of C19): $h'_{ij}=-R_i\sin(\varphi_{ij}-\theta)$, $s'_{ij}=h'_{ij}\rho+h_{ij}\rho'$ with $\rho=\theta/\sin\theta$ and $\rho'=(\sin\theta-\theta\cos\theta)/\sin^2\theta$ (use $\theta/3$ near 0), and

$$M^{el\prime}_i=\sum_{j\in\mathcal J_i}E_jI_j\left[\frac{1}{s_{ij}}-\frac{\theta\,s'_{ij}}{s_{ij}^2}\right]\tag{RC11}$$

- **Strain diagnostic:** $\varepsilon_{ij} = r_j\,|\theta_i|/s_{ij}(\theta_i)$.

### 6.5 Joint balance `[NEW]` (generalises C16)
**Free body:** links $i..N$ together with the parts of all rods inside them, cut through gap $i$. Its external loads are:
1. for each rod $j\in\mathcal J_i$, the tension $T_{j,i-1}$ along link $i$'s axis toward the base, at offset $u_{ij}$;
2. the rods' bending moments at the cut, $M^{el}_i(\theta_i)$, opposing $+\theta_i$;
3. the contact force at $\mathbf O_i$, normal and tangential, which has no moment about $\mathbf O_i$;
4. the weights of links $i..N$ and the tip load.

Everything else is internal and needs no model: the rods' friction and normal forces on links $i..N$, their anchor forces in their end links, the rod coupling between neighbouring joints, and all contacts above joint $i$. Rods of earlier sections end below link $i$ and are not part of the free body.

**Moment about $\mathbf O_i$:**

$$M^{el}_i(\theta_i)=\sum_{j\in\mathcal J_i}T_{j,i-1}\,\big(u_{ij}-R_i\sin\theta_i\big)+M^G_i(\boldsymbol\theta)+M^{ext}_i(\boldsymbol\theta),\qquad i=1..N\tag{RC12}$$

Each rod pulls along link $i$'s axis at a lateral distance $u_{ij}-R_i\sin\theta_i$ from $\mathbf O_i$, so its moment does not depend on the height at which the force acts.

**Baseline** ($\kappa = +1$, common μ):

$$M^{el}_i(\theta_i)=\eta_{i-1}\big(Q_i-P_iR_i\sin\theta_i\big)+M^G_i+M^{ext}_i,\qquad P_i=\sum_{j\in\mathcal J_i}F_j,\quad Q_i=\sum_{j\in\mathcal J_i}F_ju_{ij}\tag{RC13}$$

$P_i$ and $Q_i$ change only at section boundaries, so write them $P^{(s)}$ and $Q^{(s)}$. With pairs at $\pm a_q$:

$$Q^{(s)}=\sum_{q=s}^{S}a_q\,(F_{q\mathrm L}-F_{q\mathrm R}),\qquad P^{(s)}=\sum_{q=s}^{S}(F_{q\mathrm L}+F_{q\mathrm R})\tag{RC14}$$

What (RC13) says:
- **Q bends, P stiffens.** $Q^{(s)}$, the first moment of the tensions, bends section $s$. $P^{(s)}$, the total tension, enters through $-PR\sin\theta$: rolling moves the contact toward the inside of the bend and shortens every lever. With $Q = 0$ a preload is a restoring moment that stiffens each joint by about $\eta PR\cos\theta$.
- **Coupling.** A rod of section $q$ bends every joint of sections $1..q$.
- **Exact decoupling.** If $Q^{(s)} = 0$ for every $s < q$, sections $1..q-1$ stay exactly straight (without loads). θ = 0 is then the unique root at those joints, and straight joints have no wrap, so friction cannot upset the balance.
  - Example: pull rod $(q,\mathrm L)$ with $F$ and rod $(q-1,\mathrm R)$ with $(a_q/a_{q-1})F$. Only section $q$ bends (reference case B, §11.2).
- **Section moments as actuation coordinates.** $F_{s\mathrm L}-F_{s\mathrm R}=(Q^{(s)}-Q^{(s+1)})/a_s$, with $Q^{(S+1)}=0$. Any set of section moments can be reached with non-negative tensions, and one preload per section is left free. Friction attenuates what reaches each section by η.
- **One section.** With $S = 1$ and rods at $\pm A$, (RC12) is (C16) with its passive-rod term (1-DOF §10.2) for a symmetric link. This was checked against the lopsy_kinematics code (§11.1).

**Small-angle form** (no loads): $k^0_i\theta_i \approx \eta_{i-1}(Q_i - P_iR_i\theta_i)$, so

$$\theta_i\approx\frac{\eta_{i-1}\,Q_i}{k^0_i+\eta_{i-1}P_iR_i}\tag{RC15}$$

Use it to start Newton. Within a section the angles fall toward the tip through η.

**Lever caveat** `[OPEN]`. The lever $u - R\sin\theta$ rests on assumption 7. Other rod-path models give other levers, and the spread grows with θ as the lever itself shrinks. For R = 4 mm and u = 2.2 mm:

| θ | $u - R\sin\theta$ (this model) | straight cable between the hole openings | tension along link $i-1$'s axis | tension along the bisector |
|---|---|---|---|---|
| 8.6° | 1.602 mm | 1.567 mm | 1.600 mm | 1.596 mm |
| 17.2° | 1.018 mm | 0.810 mm | 1.000 mm | 0.989 mm |

The 1-DOF identification absorbed this into its effective parameters at angles up to about 13°. It matters for designs that use most of the joint range (§12 item 3).

### 6.6 Gravity and external loads `[NEW]`
In the base frame:

$$M^G_i=\sum_{q=i}^{N}m\big(\mathbf p^G_q-\mathbf O_i,\ \mathbf W_q\big),\qquad M^{ext}_i=m\big(\mathbf p^{tip}-\mathbf O_i,\ \mathbf F^{tip}\big)+M^{tip}\tag{RC16}$$

- $\mathbf W_q = G_q\,\hat{\mathbf g}$ with $\hat{\mathbf g} = (-\cos\psi_0,\ \sin\psi_0)$ in (X, Y). $\psi_0$ is the angle of the base axis from the upward vertical, positive toward the left: upright $\psi_0 = 0$, hanging $\psi_0 = \pi$.
- Planar test on a table: there is no in-plane gravity, so set $G = 0$ (the table friction is not modelled).
- $\mathbf p^G_q$ is link $q$'s CG, on its axis at height $L_q - y_{CG}$ above its crest. $\mathbf p^{tip}$ is the top centre of link $N$.
- $\mathbf F^{tip}$ here is a dead load, fixed in the base frame. A follower load turns with $\Theta_N$ (Jacobian in §8).
- **Check:** (RC16) equals the derivative of the loads' work with respect to $\theta_i$, because link $i$ rotates about $\mathbf O_i$ (RC6) (§11.1).
- **Consequence:** these terms make joint $i$ depend on the joints above it, which removes the base-to-tip structure of the solution (§7.2).
- This replaces 1-DOF §5.7 (C14) for the joint balance; 1-DOF had not derived gravity for (C16).

### 6.7 Contact forces `[NEW]` (replaces C11, C13)
The force balance of the same free body gives the force from link $i-1$ on link $i$ (base frame):

$$\mathbf F^c_i=\Big(\sum_{j\in\mathcal J_i}T_{j,i-1}\Big)\,\hat{\mathbf t}_i-\sum_{q\ge i}\mathbf W_q-\mathbf F^{tip},\qquad F^R_i=\mathbf F^c_i\cdot\hat{\mathbf a}_{i-1},\qquad H_i=\mathbf F^c_i\cdot\hat{\boldsymbol\ell}_{i-1}\tag{RC17}$$

- $F^R_i$ is the normal force on link $i-1$'s flat top and $H_i$ the tangential (no-slip) force.
- $\hat{\mathbf t}_i$ is the direction of the rods' internal force at the cut. It lies between $\hat{\mathbf a}_{i-1}$ and $\hat{\mathbf a}_i$, depending on which hole edges turn the rods: along $\hat{\mathbf a}_{i-1}$ if link $i$ turns them (then $H = 0$ without loads), along $\hat{\mathbf a}_i$ if link $i-1$ does (then $|H|/F^R = \tan|\theta_i|$). `[OPEN]`
- **Default:** the bisector (shared turning). Without loads, $F^R_i = \sum T\cos(\theta_i/2)$ and $H_i = \sum T\sin(\theta_i/2)$.
- **Checks:** $F^R_i > 0$, and no slip needs a link–link friction coefficient of at least $|H_i|/F^R_i$ (about 0.15 at 17° without loads).
- Unlike (C11), these forces do not grow by $1/\cos\theta$ per joint (1-DOF §10.1).

### 6.8 Shape and URDF [1-DOF C15 with Γ = 0]
Link $i$'s top frame relative to link $i-1$'s top frame, with $X_i$ axial and $Y_i$ lateral:

$$X_i=R_i(1-\cos\theta_i)+L_i\cos\theta_i,\qquad Y_i=R_i(\theta_i-\sin\theta_i)+L_i\sin\theta_i,\qquad\text{rotation }\theta_i\tag{RC18}$$

- **Shape:** $T_1T_2\cdots T_N$. The $N+1$ points are the base origin and the top centre of each link.
- **Crest and CG:** link $i$'s crest is at $\big(R_i(1-\cos\theta_i),\ R_i(\theta_i-\sin\theta_i)\big)$ in link $i-1$'s top frame, and its CG lies $L_i - y_{CG}$ further along link $i$'s axis.
- **Section boundaries** need nothing special: link $N_s+1$ rolls on link $N_s$'s top with its own $R$ and $L$.
- **URDF.** Each rolling joint becomes three elements in series:
  1. a prismatic joint along link $i-1$'s lateral axis with value $R_i\theta_i$ (a mimic of the revolute, multiplier $R_i$);
  2. a revolute joint about the joint axis through the arc centre, $R_i$ above link $i-1$'s top, with value $\theta_i$;
  3. a fixed offset of $L_i - R_i$ along the new axis to link $i$'s top.

  That is $N$ revolute joints plus $N$ mimic prismatic joints. The joint state is θ itself, without lopsy's left/right split.

### 6.9 Rod pull-in `[NEW]`
The pull-in of rod $j$ at the base, positive when the rod is drawn toward the actuator, is

$$\delta_j=\sum_{i=1}^{N_{s_j}}\Big[u_{ij}\,\theta_i-R_i\,(1-\cos\theta_i)\Big]+\delta^{st}_j\tag{RC19}$$

- **Work-consistent with the joint balance.** Without friction and loads, the rod term of (RC12) is $\partial/\partial\theta_i$ of $\sum_j F_j\delta_j$. The statics are then the stationary points of $\Pi=\sum_i\int M^{el}_i\,d\theta_i-\sum_jF_j\delta_j-(\text{work of the loads})$ (§11.1).
- **Geometry.** It agrees with the length change of a straight chord between the hole openings to within $0.2\,R_i|\theta_i|^3$ per joint.
- The $-R(1-\cos\theta)$ term lengthens the path of every rod crossing the joint, on either side: an antagonist held at fixed length goes taut.
- **Stretch:** $\delta^{st}_j \approx \sum_i T_{j,i}L_i/(E^{ax}_jA_j)$ inside the robot, with $E^{ax}$ the axial modulus and $A_j = \pi r_j^2$. The rod between the base and the actuator adds its own.
- **First order:** $\delta_j \approx \sum_i u_{ij}\theta_i$.
- **Uses:** motor-angle feedback, and the displacement-input problem (§7.3).

---

## 7. Solution procedure

### 7.1 No in-plane gravity, no external loads: joint by joint `[NEW]` (generalises 1-DOF §6)
For $i = 1..N$, the tensions $T_{j,i-1}$ ($j\in\mathcal J_i$) are known from (RC7) and $\theta_1..\theta_{i-1}$. Solve the scalar equation

$$g_i(\theta)=M^{el}_i(\theta)-\sum_{j\in\mathcal J_i}T_{j,i-1}\big(u_{ij}-R_i\sin\theta\big)=0,\qquad g'_i(\theta)=M^{el\prime}_i(\theta)+R_i\cos\theta\sum_{j\in\mathcal J_i}T_{j,i-1}\tag{RC20}$$

- **Unique root.** $M^{el\prime}_i > 0$ over the joint range (minimum 0.132 N·m/rad in the reference robot, and 0.43 N·m/rad with $\lambda_0 = 0.5$ mm), so $g'_i > 0$ for any tensions. There is no branch to follow and no continuation is needed.
- **Solver:** bracketed Newton on $(-\theta^-_i-0.2,\ \theta^+_i+0.2)$, started from (RC15), as in 1-DOF §6.
- **Cost:** $N$ scalar solves, each summing over the $|\mathcal J_i|$ rods crossing the joint.
- Joints beyond the anchor of the last tensioned rod are exactly straight, so the solver can stop there.

### 7.2 With gravity or external loads: all joints together `[NEW]`
(RC16) couples each joint to all the joints above it, so solve the $N$ equations $g_i(\boldsymbol\theta) = 0$ together, with $g_i$ the left side minus the right side of (RC12).
- **Newton** on the analytic Jacobian (§8). Start from the §7.1 solution with the loads off, and continue in the load magnitude from 0 to 1.
- **Or a fixed-point iteration:** compute $M^G$ and $M^{ext}$ from the current shape, solve §7.1 with them as known moments, and repeat. It converges when the load stiffness is well below the joint stiffness. For the upright reference robot it needed 19 iterations to reach $10^{-14}$ rad.
- **Several equilibria** are possible (a tall upright robot, a large tip load). Without friction the Jacobian is the Hessian of Π, so it is symmetric, and an equilibrium is stable if it is positive definite. The upright reference robot at zero tension has a smallest eigenvalue of 0.146 N·m/rad: stable.

### 7.3 Displacement input `[NEW]` (outline)
If the motors command rod pull-in instead of tension:
- **Unknowns:** θ ($N$ angles) and the tensions $F$ (one per rod).
- **Equations:** (RC12), plus for each rod the complementarity $F_j \ge 0$, $c_j \ge 0$, $F_jc_j = 0$ with $c_j = \delta_j(\boldsymbol\theta, F) - \delta^{cmd}_j$. A taut rod has $c_j = 0$. A slack rod has $F_j = 0$ and $c_j > 0$: the motor drew in less than the shape needs.
- The problem is global rather than joint by joint, and κ follows from the motor motions. This is 1-DOF open item 5 in multi-rod form.

---

## 8. Jacobian `[NEW]` (generalises C18–C20)
The residuals are $g_i = M^{el}_i - \sum_{j\in\mathcal J_i}T_{j,i-1}(u_{ij}-R_i\sin\theta_i) - M^G_i - M^{ext}_i$. With $a'_k = \operatorname{sign}\theta_k$ (smoothed near 0, as in 1-DOF §10.6) and the load–arm sum

$$V_i=\sum_{q\ge i}\big(\mathbf p^G_q-\mathbf O_i\big)\cdot\mathbf W_q+\big(\mathbf p^{tip}-\mathbf O_i\big)\cdot\mathbf F^{tip}\tag{RC21}$$

the entries are

$$\frac{\partial g_i}{\partial\theta_k}=\begin{cases}\mu\,a'_k\displaystyle\sum_{j\in\mathcal J_i}\kappa_{kj}\,T_{j,i-1}\big(u_{ij}-R_i\sin\theta_i\big)+V_i & k<i\\[2ex] M^{el\prime}_i+R_i\cos\theta_i\displaystyle\sum_{j\in\mathcal J_i}T_{j,i-1}+V_i+R_i\,m\Big(\hat{\boldsymbol\ell}_{i-1},\ \sum_{q\ge i}\mathbf W_q+\mathbf F^{tip}\Big) & k=i\\[2ex] V_k & k>i\end{cases}\tag{RC22}$$

- Friction fills the lower triangle and the loads fill the rest. Without loads the matrix is lower triangular, so $d\boldsymbol\theta/dF$ costs one triangular solve.
- The load entries follow from (RC6) and $m(J\mathbf r, \mathbf F) = -\mathbf r\cdot\mathbf F$. Off the diagonal they depend only on $\max(i,k)$, so without friction the matrix is symmetric.
- **Follower tip load** (turning with $\Theta_N$): its contributions become 0 for $k < i$, $+R_i\,m(\hat{\boldsymbol\ell}_{i-1},\mathbf F^{tip})$ for $k = i$, and $(\mathbf O_i-\mathbf O_k)\cdot\mathbf F^{tip}$ for $k > i$.
- All entries were verified against central differences (§11.1).

---

## 9. Design notes `[NEW]`

### 9.1 Hole placement and joint range
- By (RC5), the innermost hole on each side limits every joint it crosses. Distal rods placed inside the proximal ones shrink the range of every proximal joint.
- The reference robot puts the proximal rods innermost ($a$ = 1.6, 2.0, 2.4 mm from section 1 to 3), so its range grows toward the tip: 17.2°, 22.9°, 28.6°.
- A rod on the axis ($x = 0$, such as a central backbone) is crossed by the contact line at θ = 0. It is incompatible with flat-top rolling unless the rolling surfaces are grooved.

### 9.2 Rows along the joint axis
- In the planar robot the rods can sit side by side along the joint axis (depth $w$) at the same $|x| = a$. All rods then share the lever, the gap height and the range: the statics depend on the tensions only through $P$ and $Q$, and inner holes no longer cut the range.
- They must fit in the link: $a^2 + w^2 \le (D/2 - r_h - t_{wall})^2$.
- The moment of the tensions about the lateral axis, $\sum T_jw_j$, is carried by the line contact. The contact resultant sits at $\bar w_i = \sum T_jw_j/\sum T_j$ along the contact line, which stays inside the link as long as every hole is inside the link and outside the contact path. The torsion $H_i\bar w_i$ is not modelled.

### 9.3 Preload
Antagonistic preload adds $P$ without $Q$ and stiffens each joint by about $\eta PR\cos\theta$. Reference case D (§11.2) has the same $Q$ as case A with twice the total tension, and bends less. The paid-out rod of a preloaded pair has $\kappa = -1$ (§6.3).

### 9.4 Friction between sections
Bending of the proximal sections attenuates, by η, every rod that reaches the distal sections, so these see less tension than commanded. In reference case A, 76 % of the base tension reaches section 2 and 59 % reaches section 3.

### 9.5 Strain
The distal joints carry the fewest rods and bend the most, so check $\varepsilon_{ij}$ there against the NiTi fatigue limit. The peak in the reference cases is 0.77 % (case B).

---

## 10. Spatial extension `[NEW]` (outline)
For sections that bend in 3D, for example alternating pitch and yaw joints with 3–4 rods per section:
- **Joint orientation.** Joint $i$ has a bending direction $\hat{\mathbf e}_i$ in the cross-section, at angle $\gamma_i$ about the link axis ($\gamma = 0$ is the left). Its arc is a cylinder about the rolling axis $\hat{\mathbf n}_i = \hat{\mathbf y}\times\hat{\mathbf e}_i$ (positive $\theta_i$ is a right-handed rotation about it), with the crest line through the link axis.
- **Rods.** Rod $j$ sits at cross-section position $\mathbf c_j$, the same in every link. Its lever offset at joint $i$ is $u_{ij} = \mathbf c_j\cdot\hat{\mathbf e}_i$. Its position along the rolling axis only enters moments carried by the line contact.
- **Unchanged:** (RC3)–(RC5), (RC7)–(RC15), (RC19) and (RC20). Every rod still turns through $|\theta_i|$ at joint $i$, and the joint balance is the moment about the rolling axis through $\mathbf O_i$.
- **Changed:** the shape and the loads become 3D.
  - The relative transform (RC18) acts in the plane of $\hat{\mathbf y}$ and $\hat{\mathbf e}_i$.
  - Load moments become $\hat{\mathbf n}_i\cdot\sum(\mathbf p-\mathbf O_i)\times\mathbf W$.
  - (RC6) becomes $\partial\mathbf p/\partial\theta_k = \hat{\mathbf n}_k\times(\mathbf p-\mathbf O_k)$.
- **Hole layout.** Rods at 45° to both joint directions (for example 4 rods on a circle of radius ρ at 45°, 135°, 225° and 315°) have $|u_{ij}| = \rho/\sqrt2$ at every joint. A rod on a joint's rolling axis ($u = 0$) breaks §9.1.
- Each section then has two section moments, one per joint direction.

---

## 11. Verification

### 11.1 Checks of this document
Done with an independent Python implementation of the equations above (2026-10-25).

| Check | Result |
|---|---|
| (RC9) vs (C9a/b) with $R_1 = R_2$, $A_1 = A_2$, 2000 random states | 3e-15 mm |
| $S = 1$, rods at ±2.2 mm, R = 4 mm, vs lopsy_kinematics (C++, A1 = A2, R1 = R2), 4 load cases | ≤ 7e-13 rad (the lopsy solver tolerance) |
| (RC6) vs finite differences | 8e-7 mm/rad |
| (RC16) vs the derivative of the loads' work (upright, hanging, tilted, with tip force and moment) | 3e-11 N·m |
| Rod term of (RC12) vs $\partial(\sum_j F_j\delta_j)/\partial\theta_i$, μ = 0 | 1e-11 N·m |
| Jacobian (RC22) vs central differences: friction; with gravity and tip load; mixed κ; follower tip load | ≤ 1e-9 relative |
| Frictionless Jacobian symmetric | exactly |
| Only section-1 rods pulled: joints beyond $N_1$ | exactly 0 |
| Decoupling $F_{2R} = (a_3/a_2)F_{3L}$: sections 1–2 | exactly 0 |
| Scale invariance $(F, E)\to\lambda(F, E)$; mirror L ↔ R | ≤ 3e-17 rad |
| Passive rods with $E = 0$ vs a single 24-link section | identical |
| Small-angle form (RC15) at 0.05 N | 3e-8 relative |
| Newton vs fixed point with gravity (upright) | 1e-15 rad |

### 11.2 Reference robot and values for testing the C++ code
- **Robot:** $S = 3$, $n = (10, 8, 6)$. Every section has $L$ = 15 mm and $R$ = 4 mm. Rods: $r$ = 0.3 mm, $E$ = 61 GPa, at $\pm a_s$ with $a$ = (1.6, 2.0, 2.4) mm, and at depths $w$ = (−1, +1, 0) mm along the joint axis so that the 0.8 mm holes do not overlap (§9.2; $w$ does not change any value below).
- **Model:** μ = 0.66, $\lambda_0$ = 2.2 mm, $r_h$ = 0.4 mm, stop stiffness 10 N·m/rad (never engaged in these cases). Exact $|\theta|$; smoothing it with ε = 1e-4 rad changes the angles by up to 0.004°.
- **Loads (cases E and F only):** link weight 5.685 mN, $y_{CG}$ = 7.35 mm.
- **Derived:** joint ranges 17.19°, 22.92° and 28.65°; $k^0$ = 0.4704, 0.3065 and 0.1493 N·m/rad.

Joint angles in degrees at the first and last joint of each section, the tip position and tip angle, and the geometric pull-in of rod 3L (no stretch):

| Case | Tensions [N] | Loads | θ₁ | θ₁₀ | θ₁₁ | θ₁₈ | θ₁₉ | θ₂₄ | tip X [mm] | tip Y [mm] | Θ_N [°] | δ_3L [mm] |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A | 3L = 10 | – | 2.69511 | 2.13295 | 3.09940 | 2.51305 | 4.67600 | 3.76474 | 282.8003 | 185.5121 | 71.4401 | 2.8546 |
| B | 3L = 10, 2R = 12 | – | 0 | 0 | 0 | 0 | 7.28124 | 5.37313 | 351.6305 | 34.6609 | 37.5446 | 1.4281 |
| C | 1L = 2L = 3L = 5 | – | 3.24251 | 2.47927 | 2.71338 | 2.25274 | 2.45850 | 2.16833 | 284.2163 | 192.8409 | 61.9428 | 2.4961 |
| D | 3L = 15, 3R = 5 | – | 2.49910 | 2.03531 | 2.88178 | 2.40236 | 4.21344 | 3.53529 | 291.5782 | 176.5813 | 66.7504 | 2.6765 |
| E | 3L = 10 | upright, ψ₀ = 0 | 3.97853 | 2.74886 | 3.89931 | 2.52491 | 4.59540 | 3.35670 | 242.8533 | 225.6911 | 82.5563 | 3.2814 |
| F | 3L = 10 | hanging, ψ₀ = π | 1.91687 | 1.72959 | 2.56808 | 2.48345 | 4.69323 | 4.04875 | 304.1758 | 156.3786 | 64.2942 | 2.5726 |

- **A:** coupling. One distal rod bends all three sections.
- **B:** the decoupling rule of §6.5 ($12 = 10\cdot a_3/a_2$).
- **C:** all three sections pulled the same way.
- **D:** preload inside section 3, with the same $Q$ as case A.
- **E, F:** gravity, solved as in §7.2.

---

## 12. Open items
1. **Validation.** There are no multi-section data. Suggested first tests on an $S = 2$ prototype: each rod alone; the decoupling pair; a preload sweep at constant $Q$; upright and hanging.
2. **Friction direction and sticking (κ).** Antagonists paid out under preload, and rods crossing sections that bend away from them. Unloading and hysteresis [1-DOF item 4].
3. **Rod forces at the cut** (assumption 7): the lever $u - R\sin\theta$ against the alternatives of §6.5, and the direction $\hat{\mathbf t}_i$ for the contact forces (§6.7). The complete per-link model of 1-DOF item 10 would settle both.
4. **Parameter transfer.** Whether $E$, $\lambda_0$ and μ carry over from the 1-DOF identification; $\lambda_0$ may depend on the hole clearance.
5. **Gravity** is derived here but not validated [1-DOF item 2].
6. **Displacement input and actuator model:** spool radius, and rod stretch outside the robot (§7.3) [1-DOF items 5–6].
7. **Passive rods** are assumed slack. A rod tensioned at assembly adds to $P$ and $Q$.
8. **Slip and table friction:** the link–link no-slip check (§6.7), and table friction in planar tests [1-DOF item 15].
9. **Spatial version** (§10): untested. Torsion and the out-of-plane contact moment are not modelled.
