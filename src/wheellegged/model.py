"""Traceable CAD four-bar robot, task terrain, and consistent reset poses.

Geometry is in metres; joint angles are radians. The four-bar loop is solved
geometrically before MuJoCo receives a reset pose, so reset never injects an
artificial equality-constraint impulse. Hardware profiles explicitly constrain
the voltage motor/position servo proxies and selected component masses.
"""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .hardware import DEFAULT_PROFILE, actuator_profile
from .mass_properties import apply_component_mass_profile

ROOT = Path(__file__).resolve().parents[2]
CAD_PATH = ROOT / "assets" / "cad" / "mushibot.xml"
WHEEL_RADIUS = 0.0185
TRACK_WIDTH = 0.0873
NOMINAL_HIP = -0.20
SIDES = ("L", "R")
GROUND_MARGIN = 0.00005
HIP_CONTROL_RANGE = (-0.30, 0.0)

# Robot frame of the STEP-derived assembly: x forward, y left, z up.
_A = np.array([0.004793581489762, 0.0])
_B = np.array([-0.043843533312367, -0.018639502775678])
_C = np.array([-0.012901074593677, 0.016214164982780])
_D = np.array([-0.055059651162210, -0.014373509294863])
_W = np.array([0.002890291062, -0.036414475612])

# These are explicit small-robot baseline tasks, not full-size stair claims.
TERRAINS = {
    "flat": {"height": 0.0},
    "bridge": {"height": 0.006, "start": 0.15, "end": 0.45, "width": 0.025},
    "step": {"height": 0.006, "start": 0.20, "end": 1.20, "width": 0.40},
}


def _rotation(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, s], [-s, c]])


def _angle(reference: np.ndarray, target: np.ndarray) -> float:
    value = np.arctan2(reference[1], reference[0]) - np.arctan2(target[1], target[0])
    return float(np.arctan2(np.sin(value), np.cos(value)))


def _cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _fourbar(angle: float) -> tuple[float, float, np.ndarray]:
    b = _A + _rotation(angle) @ (_B - _A)
    v = _C - b
    length = float(np.linalg.norm(v))
    r1, r2 = np.linalg.norm(_D - _B), np.linalg.norm(_D - _C)
    along = (r1 * r1 - r2 * r2 + length * length) / (2 * length)
    discriminant = float(r1 * r1 - along * along)
    if discriminant < -1e-12:
        raise ValueError(f"Hip angle {angle:.6f} is outside the four-bar assembly branch")
    direction = v / length
    normal = np.array([-direction[1], direction[0]])
    choices = [b + along * direction + sign * np.sqrt(max(discriminant, 0)) * normal
               for sign in (-1, 1)]
    # The assembly branch at the CAD pose has D on the positive-cross side.
    original_cross = _cross2(_C - _B, _D - _B)
    d = next(p for p in choices if _cross2(_C - b, p - b) * original_cross >= 0)
    lower_angle = _angle(_D - _B, d - b)
    rod_angle = _angle(_D - _C, d - _C)
    wheel = b + _rotation(lower_angle) @ (_W - _B)
    return lower_angle - angle, rod_angle, wheel


def leg_geometry(hip_angle: float = NOMINAL_HIP) -> dict[str, float]:
    """Return CAD-derived height/dx and derivatives with respect to hip angle.

    height is hip-frame origin to axle vertically, excluding the wheel radius.
    dx is axle x relative to the chassis origin; neither includes body pitch.
    """
    knee, rod, wheel = _fourbar(float(hip_angle))
    eps = 1e-6
    derivative = (_fourbar(hip_angle + eps)[2] - _fourbar(hip_angle - eps)[2]) / (2 * eps)
    return {"height": float(-wheel[1]), "dx": float(wheel[0]),
            "dheight_dq": float(-derivative[1]), "ddx_dq": float(derivative[0]),
            "knee": knee, "rod": rod, "wheel_radius": WHEEL_RADIUS,
            "standing_height": float(WHEEL_RADIUS - wheel[1])}


def model_xml(terrain: str = "flat", timestep: float = 0.0005,
              profile: str = DEFAULT_PROFILE) -> str:
    """Create a scene without altering the vendored CAD source XML."""
    if terrain not in TERRAINS:
        raise ValueError(f"Unknown terrain {terrain!r}; choose {tuple(TERRAINS)}")
    if not (0 < timestep <= 0.005):
        raise ValueError("timestep must be in (0, 0.005] seconds")
    root = ET.parse(CAD_PATH).getroot()
    hardware = actuator_profile(profile)
    if profile == "hardware_2s":
        apply_component_mass_profile(root, CAD_PATH.parent)
    for side in SIDES:
        motor = root.find(f"./actuator/general[@name='{side}_wheel_voltage']")
        motor.set("gainprm", str(hardware.wheel_gain_nm_per_v))
        motor.set("biasprm", f"0 0 {-hardware.wheel_bias_nm_per_rad_s}")
        motor.set("ctrlrange", f"{-hardware.wheel_voltage_limit_v} {hardware.wheel_voltage_limit_v}")
        motor.set("forcerange", f"{-hardware.wheel_torque_limit_nm} {hardware.wheel_torque_limit_nm}")
        servo = root.find(f"./actuator/position[@name='{side}_servo']")
        servo.set("forcerange", f"{-hardware.hip_torque_limit_nm} {hardware.hip_torque_limit_nm}")
    root.find("compiler").set("meshdir", str(CAD_PATH.parent / "meshes"))
    root.find("option").set("timestep", str(timestep))
    world = root.find("worldbody")
    # The reference's 0.5 mm margin on both surfaces produces a 1 mm
    # pre-contact region, large relative to an 18.5 mm wheel. Retain the
    # contact stiffness but reduce each ground-contact margin to 50 microns.
    for geom in world.iter("geom"):
        if int(geom.get("contype", "0")) or int(geom.get("conaffinity", "0")):
            geom.set("margin", str(GROUND_MARGIN))
    for side in SIDES:
        root.find(f".//joint[@name='{side}_hip']").set("range", "-.35 .025")
        root.find(f"./actuator/position[@name='{side}_servo']").set("ctrlrange", "-.30 0")
    floor = world.find("geom[@name='floor']")
    floor.set("size", "2 1 .01")
    floor.set("rgba", ".19 .23 .29 1")
    asset = root.find("asset")
    ET.SubElement(asset, "texture", name="ground_grid", type="2d", builtin="checker",
                  rgb1=".17 .21 .27", rgb2=".24 .29 .35", width="512", height="512")
    ET.SubElement(asset, "material", name="ground_material", texture="ground_grid",
                  texrepeat="20 10", texuniform="true", reflectance=".05")
    floor.set("material", "ground_material")
    settings = TERRAINS[terrain]
    if terrain != "flat":
        start, end, height = (settings[k] for k in ("start", "end", "height"))
        y = TRACK_WIDTH / 2 if terrain == "bridge" else 0.0
        ET.SubElement(world, "geom", name=f"terrain_{terrain}", type="box",
                      pos=f"{(start + end) / 2} {y} {height / 2}",
                      size=f"{(end - start) / 2} {settings['width'] / 2} {height / 2}",
                      contype="2", conaffinity="1", friction=".8 .01 .001",
                      solref=".002 1", solimp=".99 .999 .0001", margin=str(GROUND_MARGIN),
                      rgba=".78 .49 .16 1")
    chassis = world.find("body[@name='chassis']")
    ET.SubElement(chassis, "site", name="imu", pos="0 0 .01", size=".001", rgba="1 0 0 1")
    ET.SubElement(chassis, "camera", name="follow", mode="trackcom", pos=".32 -.38 .22",
                  xyaxes=".765 .644 0 -.279 .331 .902")
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="imu_gyro", site="imu")
    ET.SubElement(sensors, "accelerometer", name="imu_accelerometer", site="imu")
    ET.SubElement(sensors, "framequat", name="imu_orientation", objtype="site", objname="imu")
    for side in SIDES:
        ET.SubElement(sensors, "contact", name=f"{side}_ground_contact",
                      geom1=f"{side}_tire_contact", body2="world", num="8",
                      data="found force normal", reduce="maxforce")
        for joint in ("hip", "wheel_joint"):
            ET.SubElement(sensors, "jointpos", name=f"{side}_{joint}_position", joint=f"{side}_{joint}")
            ET.SubElement(sensors, "jointvel", name=f"{side}_{joint}_velocity", joint=f"{side}_{joint}")
    return ET.tostring(root, encoding="unicode")


def load_model(terrain: str = "flat", timestep: float = 0.0005,
               profile: str = DEFAULT_PROFILE) -> mujoco.MjModel:
    """Load floating-base CAD dynamics; wheel commands are volts, servos radians."""
    return mujoco.MjModel.from_xml_string(model_xml(terrain, timestep, profile))


def make_data(model: mujoco.MjModel, hip_angle: float = NOMINAL_HIP,
              pitch: float = 0.0, x: float = 0.0) -> mujoco.MjData:
    """Initialize a mechanically consistent pose with both tires at ground level."""
    data = mujoco.MjData(model)
    geometry = leg_geometry(hip_angle)
    base = data.joint("floating_base").qpos
    base[0] = x
    base[3:7] = [np.cos(pitch / 2), 0, np.sin(pitch / 2), 0]
    for side in SIDES:
        data.joint(f"{side}_hip").qpos[0] = hip_angle
        data.joint(f"{side}_knee").qpos[0] = geometry["knee"]
        data.joint(f"{side}_rod_pivot").qpos[0] = geometry["rod"]
        data.actuator(f"{side}_servo").ctrl[0] = hip_angle
    mujoco.mj_forward(model, data)
    base[2] += WHEEL_RADIUS - float(np.min(wheel_centers(data)[:, 2]))
    mujoco.mj_forward(model, data)
    if closure_error(model, data) > 1e-8:
        raise RuntimeError("CAD reset failed the closed-chain geometric constraint")
    return data


def wheel_centers(data: mujoco.MjData) -> np.ndarray:
    return np.array([data.body(f"{side}_wheel").xpos.copy() for side in SIDES])


def closure_error(model: mujoco.MjModel, data: mujoco.MjData) -> float:
    """Largest four-bar connect-site gap in metres (call mj_forward first)."""
    return max(float(np.linalg.norm(data.site(f"{s}_closure_lower").xpos
                                    - data.site(f"{s}_closure_rod").xpos)) for s in SIDES)


def ground_forces(data: mujoco.MjData) -> np.ndarray:
    """Left/right sums of wheel-world normal contact pressure, in newtons.

    Values include an obstacle's vertical face if touched: this reports real
    world contact, not just vertical support. Robot self-contact is excluded.
    mj_forward or mj_step must have evaluated current force-stage sensors.
    """
    return np.array([np.maximum(data.sensor(f"{s}_ground_contact").data.reshape(8, 7)[:, 1], 0).sum()
                     for s in SIDES])
