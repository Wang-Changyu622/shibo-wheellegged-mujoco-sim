"""Explicit actuator assumptions, separated from immutable CAD source assets."""
from dataclasses import dataclass
from functools import lru_cache
import json
import math
from pathlib import Path


PARAMETERS_PATH = Path(__file__).resolve().parents[2] / "config" / "hardware_parameters.json"
PROFILE_NAMES = ("reference", "hardware_2s")
DEFAULT_PROFILE = "hardware_2s"


@dataclass(frozen=True)
class ActuatorProfile:
    name: str
    bus_voltage_v: float | None
    wheel_voltage_limit_v: float
    wheel_gain_nm_per_v: float
    wheel_bias_nm_per_rad_s: float
    wheel_torque_limit_nm: float
    hip_torque_limit_nm: float
    description: str


@lru_cache(maxsize=None)
def actuator_profile(name: str = DEFAULT_PROFILE) -> ActuatorProfile:
    if name not in PROFILE_NAMES:
        raise ValueError(f"Unknown hardware profile {name!r}; choose {PROFILE_NAMES}")
    document = json.loads(PARAMETERS_PATH.read_text(encoding="utf-8"))
    parameters = document["profiles"][name]
    fields = {key: parameters[key] for key in ActuatorProfile.__dataclass_fields__ if key != "name"}
    for key, value in fields.items():
        if key not in ("description", "bus_voltage_v") and not (math.isfinite(value) and value > 0):
            raise ValueError(f"Invalid positive actuator parameter {key}: {value}")
    return ActuatorProfile(name=name, **fields)
