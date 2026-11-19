"""Check catalog constraints against independent all-part mass aggregation."""
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from wheellegged.mass_properties import (
    COMPONENT_SOLIDS,
    PROPERTY_FILE,
    apply_component_mass_profile,
    component_inertial_overrides,
)


CAD = Path(__file__).resolve().parents[1] / "assets" / "cad"
pytestmark = pytest.mark.skipif(not (CAD / "mushibot.xml").exists(),
                                reason="Run scripts/fetch_assets.py first")


def _table():
    return json.loads(PROPERTY_FILE.read_text())


def _aggregate_body(parts, name, masses):
    rows = [part for part in parts if part["body"] == name]
    weights = np.array([masses[part["solid"]] for part in rows])
    centers = np.array([part["centroid_body_m"] for part in rows])
    mass = weights.sum()
    center = np.average(centers, weights=weights, axis=0)
    # Direct sum around the NEW common centroid, independent of runtime's
    # delta-mass update around the original body's coordinate origin.
    offsets = centers - center
    central = np.array([part["inertia_per_kg_body_m2"] for part in rows])
    displacement = (np.sum(offsets**2, axis=1)[:, None, None] * np.eye(3)
                    - offsets[:, :, None] * offsets[:, None, :])
    inertia = np.einsum("i,ijk->jk", weights, central + displacement)
    return mass, center, inertia


def test_component_partition_excludes_independent_tires_and_horns():
    table = _table()
    parts = {part["solid"]: part for part in table["parts"]}
    all_ids = [i for ids in COMPONENT_SOLIDS.values() for i in ids]
    assert len(all_ids) == len(set(all_ids)) == 22
    assert set(all_ids) == set(range(22))
    assert {parts[i]["body"] for i in range(18)} == {"chassis"}
    assert [parts[i]["body"] for i in (18, 19, 20, 21)] == [
        "L_lower", "L_wheel", "R_lower", "R_wheel"]
    assert {22, 23, 141, 142, 146, 147, 148, 149}.isdisjoint(all_ids)
    assert len(parts) == 156 and 126 not in parts  # Old CAD battery is omitted.


@pytest.mark.parametrize("motor,servo", [(0.041, 0.0206), (0.045, 0.0216)])
def test_replacement_matches_full_reaggregation_of_every_cad_part(motor, servo):
    parts = _table()["parts"]
    masses = {part["solid"]: part["reference_mass_kg"] for part in parts}
    for name, ids in COMPONENT_SOLIDS.items():
        target = servo if name.endswith("servo") else motor
        ratio = target / sum(masses[i] for i in ids)
        for i in ids:
            masses[i] *= ratio
        assert sum(masses[i] for i in ids) == pytest.approx(target, abs=1e-15)
    result = component_inertial_overrides(CAD, motor_mass_kg=motor, servo_mass_kg=servo)
    for name, actual in result.items():
        mass, center, inertia = _aggregate_body(parts, name, masses)
        assert actual.mass == pytest.approx(mass, abs=1e-12)
        assert actual.center == pytest.approx(center, abs=1e-10)
        assert actual.inertia == pytest.approx(inertia, rel=1e-8, abs=1e-14)


def test_compiled_catalog_profile_changes_only_five_bodies_and_is_idempotent():
    root = ET.parse(CAD / "mushibot.xml").getroot()
    old_inertials = {body.attrib["name"]: ET.tostring(body.find("inertial"))
                     for body in root.iter("body") if body.find("inertial") is not None}
    replacements = apply_component_mass_profile(root, CAD)
    assert set(replacements) == {"chassis", "L_lower", "L_wheel", "R_lower", "R_wheel"}
    once = ET.tostring(root)
    apply_component_mass_profile(root, CAD)
    assert ET.tostring(root) == once
    for body in root.iter("body"):
        name = body.attrib["name"]
        if name not in replacements and name in old_inertials:
            assert ET.tostring(body.find("inertial")) == old_inertials[name]
    root.find("compiler").set("meshdir", str(CAD / "meshes"))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    assert model.body_mass.sum() == pytest.approx(0.3176493810131978, abs=1e-12)
    assert model.body("battery_estimate").mass[0] == pytest.approx(.058, abs=1e-14)
    for name, properties in replacements.items():
        assert model.body(name).ipos == pytest.approx(properties.center, abs=1e-14)
        assert np.sort(model.body(name).inertia) == pytest.approx(
            np.linalg.eigvalsh(properties.inertia), rel=1e-10, abs=1e-14)


@pytest.mark.parametrize("filename", ["cad-manifest.json", "mushibot.xml"])
def test_changed_source_assets_require_regenerating_the_exact_table(tmp_path, filename):
    for asset in ("cad-manifest.json", "mushibot.xml"):
        shutil.copyfile(CAD / asset, tmp_path / asset)
    with (tmp_path / filename).open("a") as stream:
        stream.write("\n")
    with pytest.raises(ValueError, match="Mass-property table does not match"):
        component_inertial_overrides(tmp_path)


@pytest.mark.parametrize("mass", [0, -1, float("nan"), float("inf")])
def test_nonphysical_component_mass_is_rejected(mass):
    with pytest.raises(ValueError, match="finite and positive"):
        component_inertial_overrides(CAD, motor_mass_kg=mass)
