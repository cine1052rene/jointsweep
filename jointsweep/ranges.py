"""Numbers that have to agree with each other: actuator ranges with the joints they drive,
and a derived model with the model it was derived from."""
from __future__ import annotations

import mujoco

from jointsweep.sweep import joints

TOL = 1e-3  # rad or m; rounding in a derived file is not a finding


def actuators(m: mujoco.MjModel, tol: float = TOL) -> list[dict]:
    """Every position actuator on a limited joint: its ctrlrange next to the joint range, and
    by how much it can command the joint past its own limit."""
    out = []
    for i in range(m.nu):
        if int(m.actuator_trntype[i]) != int(mujoco.mjtTrn.mjTRN_JOINT):
            continue
        j = int(m.actuator_trnid[i][0])
        if not m.actuator_ctrllimited[i] or not m.jnt_limited[j]:
            continue
        # only a position servo commands a joint position (biasprm[1] = -kp); a motor's
        # ctrlrange is a force and has nothing to do with the joint range
        if float(m.actuator_biasprm[i][1]) == 0.0:
            continue
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) or f"actuator{i}"
        joint = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or f"joint{j}"
        lo, hi = (float(x) for x in m.actuator_ctrlrange[i])
        jlo, jhi = (float(x) for x in m.jnt_range[j])
        below, above = round(jlo - lo, 6), round(hi - jhi, 6)
        out.append({"actuator": name, "joint": joint, "ctrlrange": (lo, hi), "range": (jlo, jhi),
                    "below": below if below > tol else 0.0, "above": above if above > tol else 0.0})
    return out


def drift(derived: mujoco.MjModel, source: mujoco.MjModel, tol: float = TOL) -> list[dict]:
    """Joints whose limits in `derived` differ from `source`. The source is the reference by
    definition: when the two disagree, it is the derived file that moved."""
    a, b = joints(derived), joints(source)
    out = []
    for k in sorted(set(a) | set(b)):
        if k not in a:
            out.append({"joint": k, "missing_in": "derived"})
        elif k not in b:
            out.append({"joint": k, "missing_in": "source"})
        elif max(abs(a[k][0] - b[k][0]), abs(a[k][1] - b[k][1])) > tol:
            out.append({"joint": k, "derived": a[k], "source": b[k],
                        "delta": (round(a[k][0] - b[k][0], 6), round(a[k][1] - b[k][1], 6))})
    return out
