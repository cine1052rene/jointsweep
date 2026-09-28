# jointsweep

Measure a robot description instead of reading it.

`check_urdf` checks syntax. Simulator test suites check that a model compiles and steps.
Neither tells you that a joint, moved on its own inside the limits the file declares, drives
the arm into its own shoulder — or that an actuator can command past the joint it drives, or
that a number changed when the model was converted from its source. jointsweep measures those
three things with MuJoCo, in a few seconds, and prints the numbers.

```
pip install "jointsweep[exact] @ git+https://github.com/cine1052rene/jointsweep"
jointsweep robot.urdf --exact
jointsweep derived/so101.xml --source upstream/so101_new_calib.xml
```

## What it checks

| check | question | severity |
|---|---|---|
| `home-collision` | do two non-adjacent links penetrate at the home pose? | error |
| `sweep-collision` | moving **one joint alone** from home through its declared range, does a link pair start to penetrate? where does it start, how deep at the limit? | error |
| `actuator-range` | can a position actuator's `ctrlrange` command past the joint's `range`? | error |
| `drift` | with `--source`, which joint limits differ from the model this one was derived from? | warning |

Every `sweep-collision` comes with a **measured remedy**: the first-contact angle (bisected to
0.002 rad) minus a margin, and how many degrees of travel that costs. `--write-fix` writes a copy
of the file with those limits (and the matching actuator `ctrlrange`) applied. The alternative —
a contact exclude — is named, not chosen for you.

`--exact` re-tests every hull contact with the actual collision meshes (boolean intersection
volume via trimesh + manifold3d), because MuJoCo collides meshes as convex hulls and a hull contact
is not always a real one. Primitive collision shapes are exact already.

## Why single-joint sweeps

A grid over every joint of a 6-DOF arm always finds self-collisions; real arms rely on the
controller to avoid them, so that is not a defect. What *is* a defect is a collision the file
allows with a single joint moving from the home pose: nothing else has to go wrong for the real
robot to hit itself.

## Measured on real models

TheRobotStudio SO-ARM100, `Simulation/SO101/so101_new_calib.urdf` (5 s, `--exact`):

```
- ERROR sweep-collision elbow_flex alone reaches self-collision inside its limits:
  base_link hits gripper_link from +1.5101 rad, 25.5 mm deep at the upper limit +1.6900
  - remedy (measured): new_limit 1.4901, travel_lost_deg 11.5
```

MuJoCo Menagerie `robotstudio_so101/so101.xml`, compared with its source (3 s):

```
- ERROR sweep-collision elbow_flex alone reaches self-collision inside its limits:
  shoulder hits wrist from +1.5794 rad, 24.6 mm deep at the upper limit +1.6900
  - remedy (measured): new_limit 1.5594, travel_lost_deg 7.5
- ERROR actuator-range actuator wrist_roll ctrlrange (-2.74385, 2.84121) commands past joint
  wrist_roll range (-2.7438473, 2.7438473): above by 0.0974; the source model has range
  (-2.7438473, 2.8412063), which the actuator matches: the joint range drifted, not the actuator
  - remedy (measured): range [-2.7438473, 2.8412063]
- WARNING drift joint wrist_roll range differs from the source (delta (-0.0, -0.097359))
```

Both were reported upstream: [SO-ARM100#162 (comment)](https://github.com/TheRobotStudio/SO-ARM100/pull/162#issuecomment-5857890644),
[mujoco_menagerie#325](https://github.com/google-deepmind/mujoco_menagerie/issues/325); the
wrist_roll drift was already being fixed in [mujoco_menagerie#324](https://github.com/google-deepmind/mujoco_menagerie/pull/324).
The last line of the actuator finding is there because of that case: the mismatch was first
read as a typo in the actuator, and it was the joint range that had lost its upper limit against
the source. A mismatch only says two numbers disagree; the source says which one moved.

## In CI

```yaml
# .github/workflows/jointsweep.yml
on: [pull_request]
jobs:
  jointsweep:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with: {fetch-depth: 0}
      - uses: cine1052rene/jointsweep@main
        with:
          exact: "true"       # optional
          fail-on: error      # error | warning | never
```

By default every `.urdf` / `.xml` changed in the pull request that contains `<robot` or
`<mujoco` is checked, and the findings land in the job summary. Pass `models:` to name files.

## Python

```python
from jointsweep import load, sweep, first_contact, exact_overlap, actuators, drift, check

m = load("so101.xml")
sweep(m, "elbow_flex")                          # [(angle, [(depth_mm, body_a, body_b), ...]), ...]
first_contact(m, "elbow_flex", toward="upper")  # 1.5794
exact_overlap(m, {"elbow_flex": 1.69}, "shoulder", "wrist")
findings, summary = check("so101.xml", source="so101_new_calib.xml", exact=True)
```

## Scope

URDF and MJCF, through MuJoCo's own loader (`mujoco>=3.1`). Mesh paths that do not resolve
(`package://…`) are looked up by file name under `--search`. No language model is involved
anywhere; the tool measures and reports, and the judgement of whether a collision is acceptable
stays with the person who knows the robot. Measurements on the two SO-101 models above were
first made inside [Polyjury](https://github.com/cine1052rene/polyjury), where this code started.

Apache-2.0.
