from pathlib import Path

import pytest

from jointsweep import actuators, check, drift, first_contact, home, load, patch, sweep
from jointsweep.cli import main

FIX = Path(__file__).parent / "fixtures"
TOY = FIX / "toy_arm.xml"
SRC = FIX / "toy_arm_source.xml"


def test_home_is_clear():
    assert home(load(TOY)) == []


def test_elbow_alone_reaches_the_base():
    m = load(TOY)
    hits = sweep(m, "elbow")
    assert hits, "folding the forearm must hit the base box"
    assert all({a, b} == {"base", "fore"} for _, new in hits for _, a, b in new)
    assert sweep(m, "shoulder") == []
    onset = first_contact(m, "elbow", toward="upper")
    assert onset is not None and 2.0 < onset < 3.0
    assert first_contact(m, "elbow", toward="lower") is not None  # the arm is symmetric


def test_actuator_past_joint_range():
    out = {a["actuator"]: a for a in actuators(load(TOY))}
    assert out["elbow"]["above"] == pytest.approx(0.1, abs=1e-6)
    assert out["elbow"]["below"] == 0.0
    assert out["shoulder"]["above"] == 0.0


def test_drift_against_source():
    d = drift(load(TOY), load(SRC))
    assert [x["joint"] for x in d] == ["elbow"]
    assert d[0]["delta"] == pytest.approx((-0.1, 0.1))


def test_check_and_remedy():
    findings, summary = check(TOY, source=SRC)
    assert sorted(f.kind for f in findings) == ["actuator-range", "drift", "sweep-collision", "sweep-collision"]
    upper = [f for f in findings if f.kind == "sweep-collision" and f.data["side"] == "upper"][0]
    assert upper.remedy["new_limit"] < upper.data["onset"] < 3.0
    assert upper.remedy["travel_lost_deg"] > 0
    assert summary["clear_joints"] == ["shoulder"]


def test_write_fix_makes_it_clear(tmp_path):
    out = tmp_path / "fixed.xml"
    assert main([str(TOY), "--write-fix", str(out), "--fail-on", "never"]) == 0
    fixed = out.read_text(encoding="utf-8")
    lo, hi = patch.limits_in(fixed, "elbow")
    assert -3.0 < lo < 0 < hi < 3.0
    assert 'ctrlrange="-3.0 3.1"' not in fixed  # the actuator follows the joint
    findings, _ = check(out)
    assert all(f.kind != "sweep-collision" for f in findings)


def test_exit_code():
    assert main([str(TOY)]) == 1  # errors -> exit 1 by default
    assert main([str(SRC), "--fail-on", "never"]) == 0
