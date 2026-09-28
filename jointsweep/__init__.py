"""jointsweep: measure a robot description instead of reading it.

Three checks, all numbers, no language model involved:
- sweep:  move ONE joint from the home pose through its own limits; any link pair that starts
          to penetrate is a collision the description allows and the real robot cannot do
- ranges: actuator ctrlrange outside the joint range it drives
- drift:  joint limits that differ from the model this one was derived from
"""
from jointsweep.model import load
from jointsweep.ranges import actuators, drift
from jointsweep.report import Finding, check
from jointsweep.sweep import contacts, exact_overlap, first_contact, home, sweep

__all__ = ["load", "contacts", "home", "sweep", "first_contact", "exact_overlap",
           "actuators", "drift", "Finding", "check"]
__version__ = "0.1.0"
