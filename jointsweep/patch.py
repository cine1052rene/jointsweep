"""Rewrite joint limits in URDF (<limit lower= upper=>) or MJCF (range="lo hi") text, touching
only the values that change. A position actuator on the same joint follows the new limit."""
from __future__ import annotations

import re


def limits_in(text: str, joint: str) -> tuple[float, float] | None:
    m = re.search(rf'<joint\b[^>]*\bname="{re.escape(joint)}".*?</joint>', text, re.S)
    if m:
        lo = re.search(r'<limit\b[^>]*\blower="([^"]+)"', m.group(0))
        hi = re.search(r'<limit\b[^>]*\bupper="([^"]+)"', m.group(0))
        if lo and hi:
            return float(lo.group(1)), float(hi.group(1))
    m = re.search(rf'<joint\b[^>]*\bname="{re.escape(joint)}"[^>]*\brange="([^"]+)"', text)
    if m:
        lo, hi = m.group(1).split()[:2]
        return float(lo), float(hi)
    return None


def apply(text: str, limits: dict[str, tuple[float, float]]) -> tuple[str, list[str]]:
    changed = []
    for joint, (lo, hi) in limits.items():
        name = re.escape(joint)
        now = limits_in(text, joint)
        if now is None or (abs(now[0] - lo) <= 1e-6 and abs(now[1] - hi) <= 1e-6):
            continue
        urdf = re.compile(rf'(<joint\b[^>]*\bname="{name}".*?</joint>)', re.S)
        m = urdf.search(text)
        if m:
            block = new = m.group(1)
            if abs(now[0] - lo) > 1e-6:
                new = re.sub(r'(<limit\b[^>]*?\blower=")[^"]*(")', rf"\g<1>{lo:.4f}\g<2>", new, count=1)
            if abs(now[1] - hi) > 1e-6:
                new = re.sub(r'(<limit\b[^>]*?\bupper=")[^"]*(")', rf"\g<1>{hi:.4f}\g<2>", new, count=1)
            if new != block:
                text = text.replace(block, new, 1)
                changed.append(joint)
            continue
        mjcf = re.compile(rf'(<joint\b[^>]*\bname="{name}"[^>]*\brange=")[^"]*(")')
        if mjcf.search(text):
            text = mjcf.sub(rf"\g<1>{lo:.4f} {hi:.4f}\g<2>", text, count=1)
            changed.append(joint)
            act = re.compile(rf'(<(?:position|general|intvelocity)\b[^>]*\bjoint="{name}"[^>]*\bctrlrange=")[^"]*(")')
            text = act.sub(rf"\g<1>{lo:.4f} {hi:.4f}\g<2>", text, count=1)
    return text, changed
