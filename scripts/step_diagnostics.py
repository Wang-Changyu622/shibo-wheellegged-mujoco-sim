#!/usr/bin/env python3
"""Pinned, deterministic step sensitivity diagnostics; never modifies core models.

Geometry is recompiled per case, then read back from the actual MuJoCo geom.
Safety/crossing/settling checks and stricter motion-quality screens are separate.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, replace
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import mujoco
import numpy as np

from wheellegged.control import BalanceController, Command, balanced_data
from wheellegged.evaluate import _sample, continuous_duration, run_scenario, scenario_command
from wheellegged.hardware import PARAMETERS_PATH, actuator_profile
from wheellegged.mass_properties import PROPERTY_FILE
from wheellegged.model import CAD_PATH, SIDES, model_xml


DT = .0005
DURATION = 8.
PROFILE = "hardware_2s"
LIMITS = {
    "pitch_abs_rad": .45, "roll_abs_rad": .25, "closure_m": .001,
    "nonwheel_contact_force_n": .05, "support_vertical_n": .1,
    "platform_height_error_m": .001, "sustained_platform_s": .1,
    "tail_window_s": 1., "tail_speed_m_s": .03, "tail_pitch_rms_rad": .03,
    "limit_tolerance": 1e-8, "loaded_contact_n": .02,
    "edge_before_radii": 2., "edge_after_radii": 3.,
    "quality_speed_retention": .5, "quality_reverse_speed_m_s": -.02,
    "quality_slip_rms_m_s": .03, "quality_hip_saturation_s": .05,
    "quality_pitch_error_rad": .08, "saturation_fraction": .99,
}


@dataclass(frozen=True)
class Case:
    height_m: float
    hip_rad: float
    speed_m_s: float

    @property
    def name(self):
        return f"h{round(self.height_m * 1000):02d}_hip{round(-self.hip_rad * 100):02d}_v{round(self.speed_m_s * 100):02d}"


CASES = tuple(Case(.006, hip, speed) for hip in (-.2, -.3) for speed in (.10, .14, .18, .22)) + (
    Case(.004, -.3, .18), Case(.008, -.3, .14), Case(.008, -.3, .18), Case(.010, -.3, .18))
BASELINE = Case(.006, -.3, .18).name


def hashes():
    paths = list((ROOT / "src/wheellegged").glob("*.py")) + [
        PARAMETERS_PATH, PROPERTY_FILE, CAD_PATH, Path(__file__).resolve()]
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(paths)}


def make_model(case):
    root = ET.fromstring(model_xml("step", DT, PROFILE))
    step = root.find("./worldbody/geom[@name='terrain_step']")
    position = [float(x) for x in step.get("pos").split()]
    size = [float(x) for x in step.get("size").split()]
    position[2] = size[2] = case.height_m / 2
    step.set("pos", " ".join(format(value, ".17g") for value in position))
    step.set("size", " ".join(format(value, ".17g") for value in size))
    xml = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml)
    geom = model.geom("terrain_step")
    geometry = {
        "start_x_m": float(geom.pos[0] - geom.size[0]),
        "end_x_m": float(geom.pos[0] + geom.size[0]),
        "center_y_m": float(geom.pos[1]), "width_m": float(2 * geom.size[1]),
        "top_z_m": float(geom.pos[2] + geom.size[2]),
        "bottom_z_m": float(geom.pos[2] - geom.size[2]),
        "floor_z_m": float(model.geom("floor").pos[2]),
        "wheel_radii_m": [float(model.geom(f"{side}_tire_contact").size[0]) for side in SIDES],
    }
    assert np.isclose(geometry["top_z_m"] - geometry["floor_z_m"], case.height_m)
    assert geometry["bottom_z_m"] == geometry["floor_z_m"]
    return model, xml, geometry


def contacts(model, data, wheel_ids, step_id):
    """Contact-point tangential speed from the exact world-point Jacobian."""
    step_normal, sidewall, top_support = np.zeros(2), np.zeros(2), np.zeros(2)
    slip_squared_force, slip_force, slip_peak = np.zeros(2), np.zeros(2), np.zeros(2)
    friction_usage = np.zeros(2)
    force, jacobian = np.zeros(6), np.empty((3, model.nv))
    nonwheel = {}
    for index, contact in enumerate(data.contact):
        a, b = map(int, contact.geom)
        if model.geom_bodyid[a] != 0 and model.geom_bodyid[b] != 0:
            continue
        robot_geom, world_geom = (b, a) if model.geom_bodyid[a] == 0 else (a, b)
        mujoco.mj_contactForce(model, data, index, force)
        normal_force = max(0., float(force[0]))
        if robot_geom not in wheel_ids:
            name = model.geom(robot_geom).name
            nonwheel[name] = nonwheel.get(name, 0.) + normal_force
            continue
        side = wheel_ids.index(robot_geom)
        frame = np.asarray(contact.frame).reshape(3, 3)
        if world_geom == step_id:
            step_normal[side] += normal_force
            if abs(frame[0, 2]) < .5:
                sidewall[side] += normal_force
            else:
                top_support[side] += abs(frame[0, 2]) * normal_force
        if normal_force <= LIMITS["loaded_contact_n"]:
            continue
        mujoco.mj_jac(model, data, jacobian, None, contact.pos, int(model.geom_bodyid[robot_geom]))
        relative_velocity = frame @ (jacobian @ data.qvel)  # world geometry is static
        tangent_speed = float(np.linalg.norm(relative_velocity[1:]))
        slip_force[side] += normal_force
        slip_squared_force[side] += normal_force * tangent_speed ** 2
        slip_peak[side] = max(slip_peak[side], tangent_speed)
        sliding_mu = float(contact.friction[0])
        if sliding_mu > 0:
            friction_usage[side] = max(friction_usage[side], float(np.linalg.norm(force[1:3]))
                                      / (sliding_mu * normal_force))
    return {
        "step_wheel_normal_force": step_normal, "step_wall_normal_force": sidewall,
        "step_top_vertical_support": top_support,
        "slip_squared_force": slip_squared_force, "slip_force": slip_force,
        "loaded_contact_slip_peak": slip_peak, "sliding_friction_usage": friction_usage,
    }, nonwheel


def actual_ground_heights(wheels, geometry):
    inside = ((wheels[:, 0] >= geometry["start_x_m"]) & (wheels[:, 0] <= geometry["end_x_m"])
              & (abs(wheels[:, 1] - geometry["center_y_m"]) <= geometry["width_m"] / 2))
    return np.where(inside, geometry["top_z_m"], geometry["floor_z_m"])


def assess(case, model, trace, geometry, nonwheel_contacts):
    t, x = trace["time"], trace["axle_xyz"][:, 0]
    radius = np.asarray(geometry["wheel_radii_m"])
    wheels, bottom = trace["wheel_xyz"], trace["wheel_bottom_z"]
    top = ((wheels[:, :, 0] > geometry["start_x_m"] + radius)
           & (wheels[:, :, 0] < geometry["end_x_m"] - radius)
           & (abs(wheels[:, :, 1] - geometry["center_y_m"]) < geometry["width_m"] / 2)
           & (abs(bottom - geometry["top_z_m"]) < LIMITS["platform_height_error_m"])
           & (trace["step_top_vertical_support"] > LIMITS["support_vertical_n"]))
    both_top = np.all(top, axis=1)
    tail = t >= DURATION - LIMITS["tail_window_s"]
    edge = ((x >= geometry["start_x_m"] - LIMITS["edge_before_radii"] * radius.max())
            & (x <= geometry["start_x_m"] + LIMITS["edge_after_radii"] * radius.max())
            & (trace["command_velocity"] > 0))
    wheel_ids = [model.actuator(f"{side}_wheel_voltage").id for side in SIDES]
    hip_ids = [model.actuator(f"{side}_servo").id for side in SIDES]
    fraction = LIMITS["saturation_fraction"]
    wheel_saturated = np.any(abs(trace["ctrl"][:, wheel_ids])
                             >= fraction * model.actuator_ctrlrange[wheel_ids, 1], axis=1)
    hip_saturated = np.any(abs(trace["actuator_force"][:, hip_ids])
                           >= fraction * model.actuator_forcerange[hip_ids, 1], axis=1)
    pitch_error = trace["pitch"] - trace["pitch_reference"]
    contact_indices = np.flatnonzero(np.any(trace["step_wheel_normal_force"] > LIMITS["loaded_contact_n"], axis=1))
    first_contact = int(contact_indices[0]) if len(contact_indices) else None
    before = ((t >= t[first_contact] - .12) & (t <= t[first_contact] - .02)) if first_contact is not None else np.zeros(len(t), bool)
    pre_speed = float(np.mean(trace["speed"][before])) if np.any(before) else None
    edge_min = float(np.min(trace["speed"][edge])) if np.any(edge) else None
    edge_slip_force = float(np.sum(trace["slip_force"][edge]))
    edge_slip = (float(np.sqrt(np.sum(trace["slip_squared_force"][edge]) / edge_slip_force))
                 if edge_slip_force > 0 else None)
    retained = edge_min / pre_speed if edge_min is not None and pre_speed is not None and pre_speed > .03 else None
    settled = both_top & (abs(trace["speed"]) < LIMITS["tail_speed_m_s"]) & (abs(pitch_error) < LIMITS["tail_pitch_rms_rad"])
    settled_to_end = np.logical_and.accumulate(settled[::-1])[::-1]
    settle_indices = np.flatnonzero((t >= 4.) & settled_to_end)
    first_top = np.flatnonzero(both_top)
    displacement_error = np.linalg.norm(trace["axle_xyz"][:, :2] - trace["position_reference"], axis=1)
    metrics = {
        "elapsed_s": float(t[-1]), "first_tire_step_contact_s": float(t[first_contact]) if first_contact is not None else None,
        "first_tire_contact_axle_x_m": float(x[first_contact]) if first_contact is not None else None,
        "ideal_rigid_corner_contact_axle_x_m": float(geometry["start_x_m"]
            - np.sqrt(2 * radius.mean() * case.height_m - case.height_m ** 2)),
        "first_both_wheels_supported_on_top_s": float(t[first_top[0]]) if len(first_top) else None,
        "sustained_platform_support_s": continuous_duration(both_top, t),
        "final_axle_xyz_m": trace["axle_xyz"][-1].tolist(), "max_axle_x_m": float(np.max(x)),
        "settling_time_after_command_stop_s": float(t[settle_indices[0]] - 4.) if len(settle_indices) else None,
        "pre_contact_mean_speed_m_s": pre_speed, "edge_min_speed_m_s": edge_min,
        "edge_speed_retention_fraction": retained, "edge_speed_drop_fraction": 1. - retained if retained is not None else None,
        "pitch_peak_rad": float(np.max(abs(trace["pitch"]))), "roll_peak_rad": float(np.max(abs(trace["roll"]))),
        "edge_pitch_error_peak_rad": float(np.max(abs(pitch_error[edge]))) if np.any(edge) else None,
        "edge_speed_tracking_rmse_m_s": float(np.sqrt(np.mean((trace["speed"][edge] - trace["velocity_reference"][edge]) ** 2))) if np.any(edge) else None,
        "position_reference_error_peak_m": float(np.max(displacement_error)),
        "edge_position_reference_error_peak_m": float(np.max(displacement_error[edge])) if np.any(edge) else None,
        "edge_loaded_contact_slip_rms_m_s": edge_slip,
        "edge_loaded_contact_slip_peak_m_s": float(np.max(trace["loaded_contact_slip_peak"][edge])) if np.any(edge) else None,
        "edge_sliding_friction_usage_peak": float(np.max(trace["sliding_friction_usage"][edge])) if np.any(edge) else None,
        "step_wall_normal_peak_n": float(np.max(trace["step_wall_normal_force"])),
        "wheel_step_normal_peak_n": float(np.max(trace["step_wheel_normal_force"])),
        "nonwheel_world_contact_peak_n": float(np.max(trace["nonwheel_ground_force"])),
        "nonwheel_contacts": nonwheel_contacts,
        "wheel_voltage_saturation_duration_s": float(np.count_nonzero(wheel_saturated) * DT),
        "edge_wheel_voltage_saturation_duration_s": float(np.count_nonzero(wheel_saturated & edge) * DT),
        "hip_torque_saturation_duration_s": float(np.count_nonzero(hip_saturated) * DT),
        "edge_hip_torque_saturation_duration_s": float(np.count_nonzero(hip_saturated & edge) * DT),
        "wheel_voltage_peak_v": float(np.max(abs(trace["ctrl"][:, wheel_ids]))),
        "hip_torque_peak_nm": float(np.max(abs(trace["actuator_force"][:, hip_ids]))),
        "closure_peak_m": float(np.max(trace["closure_error"])),
        "tail_speed_peak_m_s": float(np.max(abs(trace["speed"][tail]))) if np.any(tail) else None,
        "tail_pitch_error_rms_rad": float(np.sqrt(np.mean(pitch_error[tail] ** 2))) if np.any(tail) else None,
        "warning_counts": np.max(trace["warning_count"], axis=0).tolist(),
    }
    finite = all(np.all(np.isfinite(v)) for v in trace.values() if np.issubdtype(v.dtype, np.number))
    tolerance = LIMITS["limit_tolerance"]
    safety = {
        "finite_state": bool(finite), "time_monotonic": bool(np.all(np.diff(t) > 0)),
        "completed_8s": bool(t[-1] >= DURATION - 1e-8),
        "no_solver_warnings": not any(metrics["warning_counts"]),
        "pitch_bounded": metrics["pitch_peak_rad"] < LIMITS["pitch_abs_rad"],
        "roll_bounded": metrics["roll_peak_rad"] < LIMITS["roll_abs_rad"],
        "closed_chain_bounded": metrics["closure_peak_m"] < LIMITS["closure_m"],
        "no_nonwheel_support": metrics["nonwheel_world_contact_peak_n"] < LIMITS["nonwheel_contact_force_n"],
        "control_limits": bool(np.all(trace["ctrl"] >= model.actuator_ctrlrange[:, 0] - tolerance)
                               and np.all(trace["ctrl"] <= model.actuator_ctrlrange[:, 1] + tolerance)),
        "actuator_force_limits": bool(np.all(trace["actuator_force"] >= model.actuator_forcerange[:, 0] - tolerance)
                                      and np.all(trace["actuator_force"] <= model.actuator_forcerange[:, 1] + tolerance)),
    }
    task = {
        "both_wheels_supported_on_actual_platform": metrics["sustained_platform_support_s"] >= LIMITS["sustained_platform_s"],
        "both_wheels_remain_on_actual_platform": bool(np.any(tail) and np.all(both_top[tail])),
        "tail_speed_stopped": metrics["tail_speed_peak_m_s"] is not None and metrics["tail_speed_peak_m_s"] < LIMITS["tail_speed_m_s"],
        "tail_pitch_recovered": metrics["tail_pitch_error_rms_rad"] is not None and metrics["tail_pitch_error_rms_rad"] < LIMITS["tail_pitch_rms_rad"],
    }
    quality = {
        "edge_speed_retains_at_least_half": retained is not None and retained >= LIMITS["quality_speed_retention"],
        "no_edge_reversal_over_2cm_s": edge_min is not None and edge_min >= LIMITS["quality_reverse_speed_m_s"],
        "edge_slip_rms_below_3cm_s": edge_slip is not None and edge_slip < LIMITS["quality_slip_rms_m_s"],
        "edge_hip_saturation_below_50ms": metrics["edge_hip_torque_saturation_duration_s"] < LIMITS["quality_hip_saturation_s"],
        "edge_pitch_error_below_008rad": metrics["edge_pitch_error_peak_rad"] is not None and metrics["edge_pitch_error_peak_rad"] < LIMITS["quality_pitch_error_rad"],
    }
    landmarks = {}
    for label, landmark_x in (("approach", geometry["start_x_m"] - .05),
                              ("ideal_corner", metrics["ideal_rigid_corner_contact_axle_x_m"]),
                              ("axle_at_edge", geometry["start_x_m"]),
                              ("on_platform", geometry["start_x_m"] + .07)):
        candidates = np.flatnonzero((x >= landmark_x) & (t >= 1.))
        if len(candidates):
            i = int(candidates[0])
            landmarks[label] = {"time_s": float(t[i]), "axle_x_m": float(x[i]),
                               "speed_m_s": float(trace["speed"][i]),
                               "pitch_deg": float(np.degrees(trace["pitch"][i])),
                               "wheel_step_force_n": trace["step_wheel_normal_force"][i].tolist()}
        else:
            landmarks[label] = None
    return {"metrics": metrics, "safety_checks": safety, "task_checks": task,
            "quality_checks": quality, "safe_crossing_and_settling": all(safety.values()) and all(task.values()),
            "motion_quality_screen_passed": all(quality.values()), "landmarks": landmarks}


def run_case(case, output):
    started = time.monotonic()
    directory = Path(output) / case.name
    directory.mkdir(parents=True, exist_ok=False)
    model, xml, geometry = make_model(case)
    (directory / "model.xml").write_text(xml, encoding="utf-8")
    data = balanced_data(model)
    controller = BalanceController(model, data)
    wheel_ids = [model.geom(f"{side}_tire_contact").id for side in SIDES]
    step_id = model.geom("terrain_step").id
    records, nonwheel_summary = [], {}

    def record(command):
        sample = _sample(model, data, controller, command, "step", wheel_ids)
        # _sample's stock clearance uses TERRAINS["step"]. Always replace it
        # before recording/scoring with this compiled case's real geometry.
        sample["wheel_clearance"] = sample["wheel_bottom_z"] - actual_ground_heights(sample["wheel_xyz"], geometry)
        extra, nonwheel = contacts(model, data, wheel_ids, step_id)
        sample.update(extra)
        records.append(sample)
        for name, force in nonwheel.items():
            entry = nonwheel_summary.setdefault(name, {"peak_n": 0., "normal_impulse_n_s": 0.})
            entry["peak_n"] = max(entry["peak_n"], force)
            entry["normal_impulse_n_s"] += force * DT

    record(Command())
    for index in range(round(DURATION / DT)):
        t = index * DT
        command = replace(scenario_command("step", t, DT), hip_angle=case.hip_rad,
                          velocity=case.speed_m_s if 1. <= t < 4. else 0.)
        data.ctrl[:] = controller.update(command)
        previous_time = float(data.time)
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        record(command)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)) or data.time <= previous_time:
            break
    trace = {key: np.asarray([sample[key] for sample in records]) for key in records[0]}
    result = assess(case, model, trace, geometry, nonwheel_summary)
    result.update({"case": asdict(case), "name": case.name, "geometry": geometry,
                   "runtime_xml_sha256": hashlib.sha256(xml.encode()).hexdigest(),
                   "total_mass_kg": float(model.body_mass.sum()),
                   "wall_time_s": time.monotonic() - started})
    np.savez_compressed(directory / "trace.npz", **trace)
    (directory / "report.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


def plot_case(case_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    directory = Path(case_dir)
    report = json.loads((directory / "report.json").read_text())
    trace = dict(np.load(directory / "trace.npz"))
    model = mujoco.MjModel.from_xml_path(str(directory / "model.xml"))
    wheel_ids = [model.actuator(f"{side}_wheel_voltage").id for side in SIDES]
    hip_ids = [model.actuator(f"{side}_servo").id for side in SIDES]
    t = trace["time"]
    first = report["metrics"]["first_tire_step_contact_s"]
    lo, hi = (max(0., first - .55), min(DURATION, first + 1.1)) if first is not None else (1., 4.)
    with plt.rc_context({"font.size": 9, "axes.grid": True, "grid.alpha": .25}):
        fig, axes = plt.subplots(6, 1, figsize=(10, 12), sharex=True, constrained_layout=True)
        axes[0].plot(t, trace["speed"], label="actual speed")
        axes[0].plot(t, trace["velocity_reference"], "--", label="slewed reference")
        axes[0].plot(t, trace["command_velocity"], ":", label="command")
        axes[0].set_ylabel("speed (m/s)")
        axes[1].plot(t, np.degrees(trace["pitch"]), label="pitch")
        axes[1].plot(t, np.degrees(trace["pitch_reference"]), "--", label="trim")
        axes[1].set_ylabel("pitch (deg)")
        axes[2].plot(t, trace["ground_support_vertical"].sum(axis=1), label="vertical wheel support")
        axes[2].plot(t, trace["step_wall_normal_force"].sum(axis=1), label="step wall pressure")
        axes[2].plot(t, trace["nonwheel_ground_force"], label="non-wheel world contact")
        axes[2].set_ylabel("contact (N)")
        axes[3].plot(t, trace["ctrl"][:, wheel_ids])
        axes[3].axhline(model.actuator_ctrlrange[wheel_ids[0], 1], color="black", ls="--", label="voltage limit")
        axes[3].axhline(model.actuator_ctrlrange[wheel_ids[0], 0], color="black", ls="--")
        axes[3].set_ylabel("wheel Uq (V)")
        axes[4].plot(t, trace["actuator_force"][:, hip_ids])
        cap = model.actuator_forcerange[hip_ids[0], 1]
        axes[4].axhline(cap, color="black", ls="--", label="rated hip cap")
        axes[4].axhline(-cap, color="black", ls="--")
        axes[4].set_ylabel("hip torque (Nm)")
        slip = np.sqrt(trace["slip_squared_force"].sum(axis=1) / np.maximum(trace["slip_force"].sum(axis=1), 1e-12))
        axes[5].plot(t, slip, label="loaded-contact slip RMS")
        axes[5].set_ylabel("slip (m/s)")
        axes[5].set_xlabel("simulation time (s)")
        for axis in axes:
            axis.set_xlim(lo, hi)
            if first is not None:
                axis.axvline(first, color="grey", ls=":", alpha=.7)
            axis.legend(loc="upper right", fontsize=8)
        case = report["case"]
        fig.suptitle(f"{report['name']}: {case['height_m']*1000:g} mm / hip {case['hip_rad']:.2f} rad / {case['speed_m_s']:.2f} m/s\n"
                     f"safe crossing+settling={report['safe_crossing_and_settling']}; motion-quality screen={report['motion_quality_screen_passed']}")
        fig.savefig(directory / "edge_diagnostics.png", dpi=150)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/step-review")
    parser.add_argument("--cases", nargs="+", choices=[case.name for case in CASES])
    parser.add_argument("--workers", type=int, default=2)
    parser.add_argument("--skip-reference", action="store_true", help="Skip fresh stock run_scenario parity audit")
    args = parser.parse_args()
    selected = [case for case in CASES if args.cases is None or case.name in args.cases]
    if args.workers < 1 or args.workers > 4:
        parser.error("workers must be between 1 and 4")
    args.output.mkdir(parents=True, exist_ok=True)
    if (args.output / "protocol.json").exists() or any((args.output / case.name).exists() for case in selected):
        raise FileExistsError("Choose a new output directory; recorded diagnostics are not overwritten")
    source_hashes = hashes()
    protocol = {
        "schema_version": 1, "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "cases": [{"name": case.name, **asdict(case)} for case in selected],
        "thresholds_fixed_before_rollouts": LIMITS, "source_sha256": source_hashes,
        "hardware_profile": asdict(actuator_profile(PROFILE)), "dt_s": DT, "duration_s": DURATION,
        "command_on_s": 1., "command_off_s": 4., "initial_hip_rad": -.2,
        "mujoco_version": mujoco.__version__, "numpy_version": np.__version__, "python_version": platform.python_version(),
        "randomization": "none; deterministic sensitivity cases are not independent statistical trials",
        "quality_screen_status": "prespecified exploratory engineering screens, not published standards or original PASS criteria",
    }
    (args.output / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    if args.workers == 1:
        results = [run_case(case, args.output) for case in selected]
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as pool:
            futures = [pool.submit(run_case, case, args.output) for case in selected]
            results = []
            for future in futures:
                result = future.result()
                results.append(result)
                print(result["name"], "safe_task=", result["safe_crossing_and_settling"],
                      "quality=", result["motion_quality_screen_passed"], flush=True)
    parity = None
    if not args.skip_reference and any(case.name == BASELINE for case in selected):
        reference = run_scenario("step", output_dir=args.output / "stock_reference", profile=PROFILE)
        measured = dict(np.load(args.output / BASELINE / "trace.npz"))
        keys = ("time", "qpos", "qvel", "ctrl", "actuator_force", "speed", "pitch")
        errors = {key: float(np.max(abs(measured[key] - reference["trace"][key]))) for key in keys}
        parity = {"stock_passed": reference["passed"], "max_absolute_errors": errors,
                  "identical_trajectory": all(value == 0. for value in errors.values())}
    for result in results:
        plot_case(args.output / result["name"])
    unchanged = source_hashes == hashes()
    summary = {"protocol": protocol, "source_unchanged_during_run": unchanged,
               "stock_baseline_parity": parity, "cases": results}
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"cases": len(results), "safe_task_passes": sum(r["safe_crossing_and_settling"] for r in results),
                      "quality_passes": sum(r["motion_quality_screen_passed"] for r in results),
                      "source_unchanged": unchanged, "stock_baseline_parity": parity}, indent=2))
    # Failed sensitivity cases are findings, not a failed diagnostic program.
    if not unchanged or (parity is not None and not parity["identical_trajectory"]):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
