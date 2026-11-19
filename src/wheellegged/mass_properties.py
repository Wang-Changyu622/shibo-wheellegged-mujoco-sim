"""Replace catalog component masses without renormalizing the whole robot.

Geometric moments are integrated from the original STEP BReps, not visual mesh
convex hulls or bounding boxes. The checked-in numeric table contains no mesh
geometry. Catalog mass alone does not identify a motor's rotor/stator split;
the reference CAD partition remains an explicit prior.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


PROPERTY_FILE = Path(__file__).resolve().parents[2] / "models" / "cad_mass_properties.json"
COMPONENT_SOLIDS = {
    "left_servo": tuple(range(9)),
    "right_servo": tuple(range(9, 18)),
    "left_wheel_motor": (18, 19),
    "right_wheel_motor": (20, 21),
}


@dataclass(frozen=True)
class InertialProperties:
    mass: float
    center: np.ndarray
    inertia: np.ndarray

    def xml_attributes(self) -> dict[str, str]:
        matrix = self.inertia
        values = [matrix[0, 0], matrix[1, 1], matrix[2, 2],
                  matrix[0, 1], matrix[0, 2], matrix[1, 2]]
        return {"mass": format(self.mass, ".17g"),
                "pos": " ".join(format(float(v), ".17g") for v in self.center),
                "fullinertia": " ".join(format(float(v), ".17g") for v in values)}


def _parallel_axis(center: np.ndarray) -> np.ndarray:
    return float(center @ center) * np.eye(3) - np.outer(center, center)


def _read_inertial(element: ET.Element) -> InertialProperties:
    # The locked CAD builder emits body-frame fullinertia, without inertial quat.
    if "fullinertia" not in element.attrib or "quat" in element.attrib:
        raise ValueError("Expected reference CAD body-frame fullinertia")
    raw = np.fromstring(element.attrib["fullinertia"], sep=" ")
    if raw.shape != (6,):
        raise ValueError("Invalid reference fullinertia")
    inertia = np.array([[raw[0], raw[3], raw[4]], [raw[3], raw[1], raw[5]],
                        [raw[4], raw[5], raw[2]]])
    return InertialProperties(float(element.attrib["mass"]),
                              np.fromstring(element.attrib["pos"], sep=" "), inertia)


@lru_cache(maxsize=4)
def _property_table(path: str, modification_ns: int) -> dict:
    del modification_ns  # It is part of the cache key, preventing stale reads.
    return json.loads(Path(path).read_text(encoding="utf-8"))


def component_inertial_overrides(
    cad_directory: Path,
    *,
    motor_mass_kg: float = .041,
    servo_mass_kg: float = .0206,
) -> dict[str, InertialProperties]:
    """Return revised mixed-body inertials; source assets remain untouched.

    The four catalog groups are disjoint. Independent tires, servo horns,
    encoders, lower-leg shells and the reference battery retain their existing
    estimated masses. A later measured total should be used as independent
    evidence, not enforced by scaling those unrelated components.
    """
    if not all(math.isfinite(v) and v > 0 for v in (motor_mass_kg, servo_mass_kg)):
        raise ValueError("Component masses must be finite and positive")
    cad_directory = Path(cad_directory)
    table = _property_table(str(PROPERTY_FILE), PROPERTY_FILE.stat().st_mtime_ns)
    for filename, key in (("cad-manifest.json", "manifest_sha256"),
                          ("mushibot.xml", "mjcf_sha256")):
        digest = hashlib.sha256((cad_directory / filename).read_bytes()).hexdigest()
        if digest != table["evidence"][key]:
            raise ValueError(f"Mass-property table does not match {filename}")
    parts = {part["solid"]: part for part in table["parts"]}
    source = ET.parse(cad_directory / "mushibot.xml").getroot()
    changes: dict[str, list[tuple[dict, float]]] = {}
    for name, ids in COMPONENT_SOLIDS.items():
        target = servo_mass_kg if name.endswith("servo") else motor_mass_kg
        old_mass = sum(parts[i]["reference_mass_kg"] for i in ids)
        for i in ids:
            part = parts[i]
            delta = part["reference_mass_kg"] * (target / old_mass - 1.0)
            changes.setdefault(part["body"], []).append((part, delta))
    replacements = {}
    for body, entries in changes.items():
        element = source.find(f".//body[@name='{body}']/inertial")
        if element is None:
            raise ValueError(f"Missing reference inertial for {body}")
        old = _read_inertial(element)
        mass = old.mass
        first_moment = old.mass * old.center
        origin_inertia = old.inertia + old.mass * _parallel_axis(old.center)
        for part, delta in entries:
            center = np.asarray(part["centroid_body_m"])
            unit_inertia = np.asarray(part["inertia_per_kg_body_m2"])
            mass += delta
            first_moment += delta * center
            origin_inertia += delta * (unit_inertia + _parallel_axis(center))
        center = first_moment / mass
        inertia = origin_inertia - mass * _parallel_axis(center)
        inertia = (inertia + inertia.T) / 2
        eigenvalues = np.linalg.eigvalsh(inertia)
        if mass <= 0 or eigenvalues[0] <= 0 or eigenvalues[-1] > eigenvalues[:2].sum() + 1e-12:
            raise ValueError(f"Component mass replacement produced invalid inertia for {body}")
        replacements[body] = InertialProperties(mass, center, inertia)
    return replacements


def apply_component_mass_profile(
    root: ET.Element,
    cad_directory: Path,
    *,
    motor_mass_kg: float = .041,
    servo_mass_kg: float = .0206,
) -> dict[str, InertialProperties]:
    """Apply replacements to an in-memory MJCF scene; safe to call twice."""
    replacements = component_inertial_overrides(
        cad_directory, motor_mass_kg=motor_mass_kg, servo_mass_kg=servo_mass_kg)
    for name, properties in replacements.items():
        inertial = root.find(f".//body[@name='{name}']/inertial")
        if inertial is None:
            raise ValueError(f"Scene is missing body inertial {name}")
        inertial.attrib.clear()
        inertial.attrib.update(properties.xml_attributes())
    return replacements
