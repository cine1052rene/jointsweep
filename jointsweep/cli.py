"""jointsweep MODEL [--source SOURCE] [--exact] [--json OUT] [--md OUT] [--write-fix OUT]"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from jointsweep import patch, report


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="jointsweep", description=__doc__)
    ap.add_argument("model", help="URDF or MJCF file")
    ap.add_argument("--source", help="the model this one was derived from (joint limits are compared)")
    ap.add_argument("--search", help="folder to look for mesh files by name (default: the model's folder)")
    ap.add_argument("--steps", type=int, default=41, help="positions per joint sweep (default 41)")
    ap.add_argument("--exact", action="store_true", help="confirm hull contacts with exact mesh booleans (needs jointsweep[exact])")
    ap.add_argument("--margin", type=float, default=report.MARGIN, help="rad kept between a proposed limit and first contact")
    ap.add_argument("--json", help="write findings as JSON here")
    ap.add_argument("--md", help="write findings as Markdown here")
    ap.add_argument("--write-fix", help="write a copy of the model with the measured limits applied")
    ap.add_argument("--fail-on", choices=["error", "warning", "never"], default="error",
                    help="exit 1 when a finding of this severity or worse exists (default error)")
    a = ap.parse_args(argv)

    findings, summary = report.check(a.model, a.source, a.steps, a.exact, a.search, a.margin)
    md = report.to_markdown(findings, summary)
    sys.stdout.write(md)
    if a.json:
        Path(a.json).write_text(json.dumps(report.to_dict(findings, summary), indent=2), encoding="utf-8")
    if a.md:
        Path(a.md).write_text(md, encoding="utf-8")
    if a.write_fix:
        text = Path(a.model).read_text(encoding="utf-8")
        limits: dict[str, tuple[float, float]] = {}
        for f in findings:
            if f.kind != "sweep-collision" or not f.remedy:
                continue
            now = limits.get(f.joint) or patch.limits_in(text, f.joint)
            if now is None:
                continue
            lo, hi = now
            if f.remedy["side"] == "upper":
                hi = min(hi, f.remedy["new_limit"])
            else:
                lo = max(lo, f.remedy["new_limit"])
            limits[f.joint] = (lo, hi)
        fixed, changed = patch.apply(text, limits)
        Path(a.write_fix).write_text(fixed, encoding="utf-8")
        sys.stdout.write(f"\nwrote {a.write_fix} with new limits for {changed or 'no joints'}\n")
    rank = {"error": 2, "warning": 1}
    worst = max((rank[f.severity] for f in findings), default=0)
    return 1 if a.fail_on != "never" and worst >= rank[a.fail_on] else 0


if __name__ == "__main__":
    sys.exit(main())
