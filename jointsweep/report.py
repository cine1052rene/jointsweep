"""Run every check on one model and turn the numbers into findings with a measured remedy."""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

from jointsweep import model as _model
from jointsweep import ranges
from jointsweep import sweep as _sweep

MARGIN = 0.02  # rad kept between a proposed limit and the first contact


@dataclass
class Finding:
    kind: str            # home-collision | sweep-collision | actuator-range | drift
    severity: str        # error | warning
    joint: str
    message: str
    data: dict = field(default_factory=dict)
    remedy: dict = field(default_factory=dict)


def _collision_findings(m, name: str, lo: float, hi: float, at_home: set, exact: bool, margin: float) -> list[Finding]:
    out = []
    for side, limit in (("upper", hi), ("lower", lo)):
        if not math.isfinite(limit):
            continue
        end = [c for c in _sweep.contacts(m, {name: limit}) if (c[1], c[2]) not in at_home]
        if not end:
            continue
        onset = _sweep.first_contact(m, name, toward=side)
        pairs = []
        for depth, a, b in end:
            pair = {"pair": [a, b], "depth_mm": depth, "onset": _sweep.first_contact(m, name, (a, b), side)}
            if exact:
                pair["exact"] = _sweep.exact_overlap(m, {name: limit}, a, b)
            pairs.append(pair)
        real = [p for p in pairs if p.get("exact", {}).get("real", True)]
        if not real:
            out.append(Finding("sweep-collision", "warning", name,
                               f"{name} at {limit:+.4f}: convex hulls touch but the exact meshes do not", {"pairs": pairs}))
            continue
        new = round(onset - margin, 4) if side == "upper" else round(onset + margin, 4)
        worst = real[0]
        out.append(Finding(
            "sweep-collision", "error", name,
            f"{name} alone reaches self-collision inside its limits: {worst['pair'][0]} hits {worst['pair'][1]} "
            f"from {onset:+.4f} rad, {worst['depth_mm']} mm deep at the {side} limit {limit:+.4f}",
            {"side": side, "limit": limit, "onset": onset, "pairs": pairs},
            {"joint": name, "side": side, "new_limit": new, "travel_lost_deg": round(math.degrees(abs(limit - new)), 1),
             "or": "add a contact exclude for the pair if the collision shapes are deliberately conservative"}))
    return out


def check(path: str | Path, source: str | Path | None = None, steps: int = 41, exact: bool = False,
          search: str | Path | None = None, margin: float = MARGIN) -> tuple[list[Finding], dict]:
    """All findings for one description, plus a summary of what was measured."""
    m = _model.load(path, search)
    joints = _sweep.joints(m)
    findings: list[Finding] = []
    for depth, a, b in _sweep.home(m):
        findings.append(Finding("home-collision", "error", "-", f"{a} and {b} penetrate {depth} mm at the home pose",
                                {"pair": [a, b], "depth_mm": depth}))
    at_home = {(a, b) for _, a, b in _sweep.home(m)}
    clear = []
    for name, (lo, hi) in joints.items():
        if not _sweep.sweep(m, name, steps):
            clear.append(name)
            continue
        findings.extend(_collision_findings(m, name, lo, hi, at_home, exact, margin))
    for a in ranges.actuators(m):
        if a["below"] or a["above"]:
            past = " and ".join(f"{k} by {v:.4f}" for k, v in (("below", a["below"]), ("above", a["above"])) if v)
            findings.append(Finding("actuator-range", "error", a["joint"],
                                    f"actuator {a['actuator']} ctrlrange {a['ctrlrange']} commands past joint {a['joint']} "
                                    f"range {a['range']}: {past}", a,
                                    {"ctrlrange": list(a["range"]), "or": "widen the joint range if the actuator is right"}))
    if source:
        s = _model.load(source, search)
        for d in ranges.drift(m, s):
            if "missing_in" in d:
                findings.append(Finding("drift", "warning", d["joint"], f"joint {d['joint']} is missing in the {d['missing_in']} model", d))
                continue
            findings.append(Finding("drift", "warning", d["joint"],
                                    f"joint {d['joint']} range {d['derived']} differs from the source {d['source']} "
                                    f"(delta {d['delta']})", d, {"range": list(d["source"])}))
            # a mismatch is a number that disagrees with another; the source says which one moved.
            # (Menagerie so101 wrist_roll, 2026-09: the ctrlrange looked like the typo, but it was
            # the joint range that had lost its upper limit against the source.)
            for f in findings:
                if f.kind == "actuator-range" and f.joint == d["joint"] and \
                        max(abs(f.data["ctrlrange"][0] - d["source"][0]), abs(f.data["ctrlrange"][1] - d["source"][1])) <= ranges.TOL:
                    f.message += f"; the source model has range {d['source']}, which the actuator matches: the joint range drifted, not the actuator"
                    f.remedy = {"range": list(d["source"])}
    summary = {"model": str(path), "bodies": int(m.nbody), "geoms": int(m.ngeom),
               "joints": {k: list(v) for k, v in joints.items()}, "clear_joints": clear,
               "steps": steps, "exact": exact, "source": str(source) if source else None}
    return findings, summary


def to_markdown(findings: list[Finding], summary: dict) -> str:
    lines = [f"# jointsweep: {Path(summary['model']).name}", "",
             f"{summary['bodies']} bodies, {summary['geoms']} geoms, {len(summary['joints'])} joints; "
             f"{len(summary['clear_joints'])} joints clear over their full range."]
    if summary.get("source"):
        lines.append(f"Compared with source `{Path(summary['source']).name}`.")
    lines.append("")
    if not findings:
        lines.append("No findings.")
    for f in findings:
        lines.append(f"- **{f.severity.upper()}** `{f.kind}` {f.message}")
        if f.remedy:
            r = {k: v for k, v in f.remedy.items() if k != "or"}
            lines.append(f"  - remedy (measured): {r}" + (f"; or: {f.remedy['or']}" if "or" in f.remedy else ""))
    return "\n".join(lines) + "\n"


def to_dict(findings: list[Finding], summary: dict) -> dict:
    return {"summary": summary, "findings": [asdict(f) for f in findings]}
