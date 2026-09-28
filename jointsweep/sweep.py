"""Contacts at a pose, and what one joint alone can make the robot hit.

Lessons measured on real arms (TheRobotStudio SO-ARM100, MuJoCo Menagerie so101, 2026-09):
- A full grid over every joint always finds self-collisions on a 6-DOF arm. That is normal;
  real arms rely on the controller. What is a defect is a collision at the home pose, or one
  that a SINGLE joint reaches from home inside its own declared limits.
- MuJoCo collides meshes as convex hulls. A hull contact between two mesh links may not be
  real; exact_overlap() re-tests the pose with the actual triangles when trimesh and
  manifold3d are installed. Primitive collision shapes (boxes, capsules) are exact already.
- Parent and child links touch at their joint by design and are ignored, as is anything
  shallower than DEPTH_MM.
"""
from __future__ import annotations

import mujoco
import numpy as np

DEPTH_MM = 3.0


def joints(m: mujoco.MjModel) -> dict[str, tuple[float, float]]:
    """Hinge and slide joints with their limits (inf when unlimited)."""
    out = {}
    for j in range(m.njnt):
        if int(m.jnt_type[j]) in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) or f"joint{j}"
            lim = (float(m.jnt_range[j][0]), float(m.jnt_range[j][1])) if m.jnt_limited[j] else (-np.inf, np.inf)
            out[name] = lim
    return out


def _jid(m, name: str) -> int:
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    if j < 0:
        raise KeyError(f"no joint {name!r}; joints are {list(joints(m))}")
    return j


def pose(m, d, values: dict[str, float]) -> None:
    d.qpos[:] = m.qpos0
    for name, v in values.items():
        d.qpos[m.jnt_qposadr[_jid(m, name)]] = v
    mujoco.mj_forward(m, d)


def body_name(m, b: int) -> str:
    return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or f"body{b}"


def contacts(m, values: dict[str, float] | None = None) -> list[tuple[float, str, str]]:
    """Penetrating contacts between links that are not parent and child, one entry per pair:
    [(depth_mm, body_a, body_b)], deepest first. `values` maps joint name -> position; every
    other joint stays at home."""
    d = mujoco.MjData(m)
    pose(m, d, values or {})
    out: dict[tuple[str, str], float] = {}
    for c in d.contact[:d.ncon]:
        b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
        if b1 == b2 or m.body_parentid[b1] == b2 or m.body_parentid[b2] == b1 or -c.dist * 1000 < DEPTH_MM:
            continue
        pair = tuple(sorted((body_name(m, b1), body_name(m, b2))))
        out[pair] = max(out.get(pair, 0.0), round(-float(c.dist) * 1000, 1))
    return sorted(((v, a, b) for (a, b), v in out.items()), reverse=True)


def home(m) -> list[tuple[float, str, str]]:
    return contacts(m, {})


def sweep(m, joint: str, steps: int = 41) -> list[tuple[float, list]]:
    """Move ONE joint through its limits from home. [(value, new contacts)] for every setting
    where it hits something that was not already touching at home."""
    lo, hi = joints(m)[joint]
    if not np.isfinite(lo) or not np.isfinite(hi):
        lo, hi = -np.pi, np.pi
    at_home = {(a, b) for _, a, b in home(m)}
    hits = []
    for v in np.linspace(lo, hi, steps):
        new = [c for c in contacts(m, {joint: float(v)}) if (c[1], c[2]) not in at_home]
        if new:
            hits.append((round(float(v), 4), new))
    return hits


def first_contact(m, joint: str, pair: tuple[str, str] | None = None, toward: str = "upper",
                  tol: float = 0.002) -> float | None:
    """Bisect for the joint position, moving from 0 toward its upper or lower limit, where a
    new contact first appears. None if that side stays clear."""
    lo, hi = joints(m)[joint]
    end = hi if toward == "upper" else lo
    if not np.isfinite(end):
        end = np.pi if toward == "upper" else -np.pi
    at_home = {(a, b) for _, a, b in home(m)}

    def hit(v: float) -> bool:
        return any((c[1], c[2]) not in at_home and (pair is None or {c[1], c[2]} == set(pair))
                   for c in contacts(m, {joint: v}))
    if not hit(end):
        return None
    a, b = 0.0, end
    while abs(b - a) > tol:
        mid = (a + b) / 2
        a, b = (a, mid) if hit(mid) else (mid, b)
    return round(b, 4)


def _body_mesh(m, d, name: str):
    import trimesh
    b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
    parts = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != b or int(m.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_MESH):
            continue
        if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
            continue
        k = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]]
        f = m.mesh_face[m.mesh_faceadr[k]:m.mesh_faceadr[k] + m.mesh_facenum[k]]
        parts.append(trimesh.Trimesh(v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g], f, process=True))
    return trimesh.util.concatenate(parts) if parts else None


def _solid(t):
    import manifold3d as mf
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32),
                               tri_verts=np.asarray(t.faces, np.uint32)))


def exact_overlap(m, values: dict[str, float], body_a: str, body_b: str) -> dict:
    """Is a hull contact real? Intersection volume (mm^3) of the two links' actual collision
    meshes at this pose, next to the volume of their convex hulls. Links built from primitive
    shapes are exact already and come back as {'primitive': True, 'real': True}."""
    try:
        import manifold3d  # noqa: F401
        import trimesh  # noqa: F401
    except ImportError:
        return {"error": "install jointsweep[exact] (trimesh + manifold3d) for exact mesh overlap"}
    d = mujoco.MjData(m)
    pose(m, d, values)
    A, B = _body_mesh(m, d, body_a), _body_mesh(m, d, body_b)
    if A is None or B is None:
        return {"primitive": True, "real": True}
    hull = (_solid(A.convex_hull) ^ _solid(B.convex_hull)).volume() * 1e9
    exact = (_solid(A) ^ _solid(B)).volume() * 1e9
    return {"hull_mm3": round(hull, 1), "exact_mm3": round(exact, 1), "real": exact > 1.0,
            "watertight": bool(A.is_watertight and B.is_watertight)}
