"""Deterministic task rollouts with contact/geometry-based acceptance evidence.

``run_scenario`` returns ``model``, numpy ``trace``, JSON-safe ``metadata``,
``metrics``, ``checks`` and ``passed``. The model and trace are suitable for
offline rendering; replay never participates in closed-loop simulation.
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict
from pathlib import Path
import platform
import subprocess
import time

import mujoco
import numpy as np

from .control import BalanceController, Command, balanced_data
from .hardware import DEFAULT_PROFILE, PARAMETERS_PATH, actuator_profile
from .mass_properties import PROPERTY_FILE
from .model import (CAD_PATH, ROOT, SIDES, TERRAINS, TRACK_WIDTH, WHEEL_RADIUS,
                    closure_error, ground_forces, load_model, model_xml, wheel_centers)
from .state import observe


SCENARIOS = {"balance": 8.0, "move": 12.0, "step": 8.0, "bridge": 8.0, "jump": 5.0}
TERRAIN_APPROACH_SPEED = {"step": .18, "bridge": .25}
THRESHOLDS = {
    "pitch_peak_rad": .45,
    "roll_peak_rad": .25,
    "closure_peak_m": .001,
    "tail_window_s": 1.0,
    "tail_pitch_error_rms_rad": .03,
    "tail_speed_peak_m_s": .03,
    "balance_position_drift_m": .03,
    "move_velocity_rmse_m_s": .04,
    "move_min_directional_displacement_m": .15,
    "terrain_height_tolerance_m": .001,
    "support_force_n": .1,
    "bridge_support_duration_s": .1,
    "bridge_exit_margin_m": .05,
    "airborne_force_n": .02,
    "airborne_clearance_m": .001,
    "airborne_duration_s": .020,
    "nonwheel_ground_force_n": .05,
    "limit_tolerance": 1e-8,
}


def scenario_command(name: str, t: float, timestep: float = .0005) -> Command:
    """Fixed public command schedule; the jump is a single control-step pulse."""
    if name not in SCENARIOS:
        raise ValueError(f"Unknown scenario {name!r}; choose {tuple(SCENARIOS)}")
    velocity = 0.0
    if name == "move":
        velocity = .12 if 1.0 <= t < 4.0 else (-.12 if 5.0 <= t < 8.0 else 0.0)
    elif name in ("step", "bridge"):
        velocity = TERRAIN_APPROACH_SPEED[name] if 1.0 <= t < 4.0 else 0.0
    jump = name == "jump" and 1.5 - 1e-10 <= t < 1.5 + timestep - 1e-10
    # Prepare an extended leg posture before meeting the step. This prevents
    # the actual lower-leg shell from being used as an unintended foothold.
    return Command(velocity=velocity, hip_angle=-.30 if name == "step" else -.20, jump=jump)


def continuous_duration(mask: np.ndarray, times: np.ndarray) -> float:
    """Longest interval bounded by consecutive true samples (no one-frame credit)."""
    indices = np.flatnonzero(mask)
    if not len(indices):
        return 0.0
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    return max(float(times[group[-1]] - times[group[0]]) for group in groups)


def _ground_heights(wheel_xyz: np.ndarray, terrain: str) -> np.ndarray:
    heights = np.zeros(2)
    if terrain != "flat":
        settings = TERRAINS[terrain]
        middle_y = TRACK_WIDTH / 2 if terrain == "bridge" else 0.0
        on_top = ((wheel_xyz[:, 0] >= settings["start"])
                  & (wheel_xyz[:, 0] <= settings["end"])
                  & (abs(wheel_xyz[:, 1] - middle_y) <= settings["width"] / 2))
        heights[on_top] = settings["height"]
    return heights


def _contact_evidence(model: mujoco.MjModel, data: mujoco.MjData,
                      wheel_geom_ids: list[int]) -> tuple[np.ndarray, float]:
    support, nonwheel_force = np.zeros(2), 0.0
    force = np.zeros(6)
    for i, contact in enumerate(data.contact):
        a, b = (int(g) for g in contact.geom)
        if model.geom_bodyid[a] != 0 and model.geom_bodyid[b] != 0:
            continue  # self contact is not world support
        mujoco.mj_contactForce(model, data, i, force)
        normal_force = max(float(force[0]), 0.0)
        robot_geom = b if model.geom_bodyid[a] == 0 else a
        if robot_geom in wheel_geom_ids:
            # Projection of normal pressure, excluding shear; vertical walls give zero.
            support[wheel_geom_ids.index(robot_geom)] += normal_force * abs(contact.frame[2])
        else:
            nonwheel_force += normal_force
    return support, nonwheel_force


def _sample(model: mujoco.MjModel, data: mujoco.MjData, controller: BalanceController,
            command: Command, terrain: str, wheel_geom_ids: list[int]) -> dict:
    obs = observe(model, data)
    wheel_xyz = wheel_centers(data)
    bottom = np.empty(2)
    for i, geom_id in enumerate(wheel_geom_ids):
        # Exact vertical support extent of the collision cylinder, including roll.
        axis_z = data.geom_xmat[geom_id].reshape(3, 3)[2, 2]
        radius, half_width = model.geom_size[geom_id, :2]
        extent = radius * np.sqrt(max(0., 1. - axis_z * axis_z)) + half_width * abs(axis_z)
        bottom[i] = data.geom_xpos[geom_id, 2] - extent
    support, nonwheel_force = _contact_evidence(model, data, wheel_geom_ids)
    body_offset = data.body("chassis").xmat.reshape(3, 3).T @ (
        data.body("chassis").subtree_com - wheel_xyz.mean(axis=0))
    pitch_reference = float(-np.arctan2(body_offset[0], body_offset[2]))
    return {
        "time": obs.time, "xyz": data.body("chassis").xpos.copy(),
        "axle_xyz": wheel_xyz.mean(axis=0), "wheel_xyz": wheel_xyz,
        "wheel_z": obs.wheel_z, "wheel_bottom_z": bottom,
        "wheel_clearance": bottom - _ground_heights(wheel_xyz, terrain),
        "pitch": obs.pitch, "roll": obs.roll, "yaw": obs.yaw,
        "pitch_reference": pitch_reference,
        "pitch_rate": obs.pitch_rate, "roll_rate": obs.roll_rate, "yaw_rate": obs.yaw_rate,
        "speed": obs.speed, "hip": obs.hip, "ground_force": ground_forces(data),
        "ground_support_vertical": support, "nonwheel_ground_force": nonwheel_force,
        "closure_error": closure_error(model, data), "ctrl": data.ctrl.copy(),
        "actuator_force": data.actuator_force.copy(), "qpos": data.qpos.copy(),
        "qvel": data.qvel.copy(), "phase": controller.phase,
        "warning_count": np.array(data.warning.number, copy=True),
        "command_velocity": command.velocity, "command_jump": command.jump,
        "command_hip_angle": command.hip_angle, "command_yaw_rate": command.yaw_rate,
        "velocity_reference": controller.velocity_reference,
        "position_reference": controller.position_reference.copy(),
    }


def _rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values)))) if values.size else float("nan")


def _window(trace: dict, start: float, end: float) -> np.ndarray:
    return (trace["time"] >= start) & (trace["time"] < end)


def assess_trace(name: str, trace: dict, model: mujoco.MjModel,
                 trim_pitch: float, requested_duration: float) -> tuple[dict, dict]:
    """Assess physical evidence without reading the controller's claimed success."""
    limit = THRESHOLDS
    t = trace["time"]
    tail = t >= max(0., requested_duration - limit["tail_window_s"])
    finite = all(np.all(np.isfinite(values)) for values in trace.values()
                 if np.issubdtype(values.dtype, np.number))
    ctrl_ok = np.all((trace["ctrl"] >= model.actuator_ctrlrange[:, 0] - limit["limit_tolerance"])
                     & (trace["ctrl"] <= model.actuator_ctrlrange[:, 1] + limit["limit_tolerance"]))
    force_ok = np.all((trace["actuator_force"] >= model.actuator_forcerange[:, 0] - limit["limit_tolerance"])
                      & (trace["actuator_force"] <= model.actuator_forcerange[:, 1] + limit["limit_tolerance"]))
    metrics = {
        "elapsed_s": float(t[-1]),
        "pitch_peak_rad": float(np.max(abs(trace["pitch"]))),
        "roll_peak_rad": float(np.max(abs(trace["roll"]))),
        "closure_peak_m": float(np.max(trace["closure_error"])),
        "tail_pitch_error_rms_rad": _rms((trace["pitch"] - trace.get("pitch_reference", trim_pitch))[tail]),
        "tail_speed_peak_m_s": float(np.max(abs(trace["speed"][tail]))) if np.any(tail) else float("inf"),
        "final_axle_xyz_m": trace["axle_xyz"][-1].tolist(),
        "position_drift_m": float(np.linalg.norm(trace["axle_xyz"][-1, :2] - trace["axle_xyz"][0, :2])),
        "lateral_drift_peak_m": float(np.max(abs(trace["axle_xyz"][:, 1] - trace["axle_xyz"][0, 1]))),
        "nonwheel_ground_force_peak_n": float(np.max(trace["nonwheel_ground_force"])),
        "actuator_force_peak": np.max(abs(trace["actuator_force"]), axis=0).tolist(),
        "control_peak": np.max(abs(trace["ctrl"]), axis=0).tolist(),
        "warning_counts": np.max(trace["warning_count"], axis=0).tolist(),
    }
    checks = {
        "finite_state_and_telemetry": bool(finite),
        "no_solver_warnings": bool(np.all(trace["warning_count"] == 0)),
        "time_monotonic": bool(np.all(np.diff(t) > 0)),
        "completed_requested_duration": bool(t[-1] >= requested_duration - 1e-8),
        "full_task_duration": requested_duration >= SCENARIOS[name],
        "pitch_bounded": metrics["pitch_peak_rad"] < limit["pitch_peak_rad"],
        "roll_bounded": metrics["roll_peak_rad"] < limit["roll_peak_rad"],
        "closed_chain_bounded": metrics["closure_peak_m"] < limit["closure_peak_m"],
        "control_limits": bool(ctrl_ok), "actuator_force_limits": bool(force_ok),
        "no_nonwheel_ground_support": metrics["nonwheel_ground_force_peak_n"] < limit["nonwheel_ground_force_n"],
        "tail_pitch_recovered": metrics["tail_pitch_error_rms_rad"] < limit["tail_pitch_error_rms_rad"],
        "tail_speed_stopped": metrics["tail_speed_peak_m_s"] < limit["tail_speed_peak_m_s"],
    }
    if name == "balance":
        checks["position_held"] = metrics["position_drift_m"] < limit["balance_position_drift_m"]
    elif name == "move":
        for direction, start, end, speed in (("forward", 2., 4., .12), ("backward", 6., 8., -.12)):
            window = _window(trace, start, end)
            metrics[f"{direction}_speed_rmse_m_s"] = _rms(trace["speed"][window] - speed)
            # Displacement is measured within the full commanded interval, not inferred from wheel spin.
            full = _window(trace, start - 1., end)
            values = trace["axle_xyz"][full, 0]
            displacement = float(values[-1] - values[0]) if len(values) else 0.
            metrics[f"{direction}_displacement_m"] = displacement
            checks[f"{direction}_speed_tracked"] = metrics[f"{direction}_speed_rmse_m_s"] < limit["move_velocity_rmse_m_s"]
            checks[f"{direction}_translation"] = displacement * np.sign(speed) > limit["move_min_directional_displacement_m"]
    elif name in ("step", "bridge"):
        terrain = TERRAINS[name]
        wheels = trace["wheel_xyz"]
        bottom = trace["wheel_bottom_z"]
        force = trace["ground_support_vertical"]
        tolerance = limit["terrain_height_tolerance_m"]
        if name == "step":
            on_top = ((wheels[:, :, 0] > terrain["start"] + WHEEL_RADIUS)
                      & (wheels[:, :, 0] < terrain["end"] - WHEEL_RADIUS)
                      & (abs(wheels[:, :, 1]) < terrain["width"] / 2)
                      & (abs(bottom - terrain["height"]) < tolerance)
                      & (force > limit["support_force_n"]))
            checks["both_wheels_reached_platform"] = bool(np.any(np.all(on_top, axis=1)))
            checks["both_wheels_remain_on_platform"] = bool(np.any(tail) and np.all(on_top[tail]))
            metrics["platform_support_duration_s"] = continuous_duration(np.all(on_top, axis=1), t)
        else:
            central = ((wheels[:, 0, 0] > terrain["start"] + WHEEL_RADIUS)
                       & (wheels[:, 0, 0] < terrain["end"] - WHEEL_RADIUS))
            left_on_bridge = ((abs(wheels[:, 0, 1] - TRACK_WIDTH / 2) < terrain["width"] / 2)
                              & (abs(bottom[:, 0] - terrain["height"]) < tolerance)
                              & (force[:, 0] > limit["support_force_n"]))
            right_on_floor = ((wheels[:, 1, 1] < TRACK_WIDTH / 2 - terrain["width"] / 2)
                              & (abs(bottom[:, 1]) < tolerance)
                              & (force[:, 1] > limit["support_force_n"]))
            support_duration = continuous_duration(central & left_on_bridge & right_on_floor, t)
            metrics["asymmetric_support_duration_s"] = support_duration
            checks["real_asymmetric_bridge_support"] = support_duration >= limit["bridge_support_duration_s"]
            checks["bridge_path_not_bypassed"] = bool(np.any(central) and np.all(left_on_bridge[central] & right_on_floor[central]))
            checks["bridge_exited"] = bool(np.all(wheels[-1, :, 0] > terrain["end"] + limit["bridge_exit_margin_m"]))
            checks["both_wheels_back_on_floor"] = bool(np.any(tail) and np.all(abs(bottom[tail]) < tolerance)
                                                        and np.all(force[tail] > limit["support_force_n"]))
    elif name == "jump":
        airborne = ((trace["ground_force"] < limit["airborne_force_n"]).all(axis=1)
                    & (trace["wheel_clearance"] > limit["airborne_clearance_m"]).all(axis=1)
                    & (t > 1.5))
        duration = continuous_duration(airborne, t)
        metrics["airborne_duration_s"] = duration
        metrics["both_wheel_clearance_peak_m"] = float(np.max(np.min(trace["wheel_clearance"], axis=1)))
        metrics["chassis_height_peak_m"] = float(np.max(trace["xyz"][:, 2]))
        checks["sustained_physical_takeoff"] = duration >= limit["airborne_duration_s"]
        checks["landed_and_supported"] = bool(np.any(tail) and np.all(trace["ground_support_vertical"][tail] > limit["support_force_n"])
                                               and np.all(abs(trace["wheel_clearance"][tail]) < limit["terrain_height_tolerance_m"]))
        checks["returned_to_balance_phase"] = bool(np.any(tail) and np.all(trace["phase"][tail] == "balance"))
    return metrics, {name: bool(value) for name, value in checks.items()}


def _git_revision() -> str | None:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT,
                                       stderr=subprocess.DEVNULL, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _json_safe(value):
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def run_scenario(name: str, output_dir: str | Path | None = None, timestep: float = .0005,
                 seed: int = 0, pitch_offset: float | None = None,
                 duration: float | None = None, profile: str = DEFAULT_PROFILE) -> dict:
    """Run a fixed scenario, keeping every physical step and all failure evidence.

    ``seed`` is recorded for experiment indexing; these baseline scenarios have
    no randomization, so changing it alone does not create independent trials.
    A shortened ``duration`` is useful for previews but cannot pass a full task.
    Output directories containing existing trace/report files are not overwritten.
    """
    if name not in SCENARIOS:
        raise ValueError(f"Unknown scenario {name!r}; choose {tuple(SCENARIOS)}")
    duration = SCENARIOS[name] if duration is None else float(duration)
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("duration must be finite and positive")
    offset = (.05 if name == "balance" else 0.) if pitch_offset is None else float(pitch_offset)
    if not np.isfinite(offset):
        raise ValueError("pitch_offset must be finite")
    directory = Path(output_dir) if output_dir is not None else None
    if directory is not None and any((directory / filename).exists() for filename in ("trace.npz", "metrics.json")):
        raise FileExistsError(f"Refusing to overwrite existing experiment evidence: {directory}")
    terrain = name if name in ("step", "bridge") else "flat"
    source_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in sorted(Path(__file__).parent.glob("*.py"))}
    started = time.perf_counter()
    model = load_model(terrain, timestep, profile)
    data = balanced_data(model, pitch_offset=offset)
    controller = BalanceController(model, data)
    wheel_geom_ids = [model.geom(f"{side}_tire_contact").id for side in SIDES]
    mujoco.mj_forward(model, data)
    records = [_sample(model, data, controller, Command(), terrain, wheel_geom_ids)]
    steps = int(np.ceil(duration / timestep - 1e-10))
    for index in range(steps):
        command = scenario_command(name, index * timestep, timestep)
        data.ctrl[:] = controller.update(command)
        previous_time = float(data.time)
        mujoco.mj_step(model, data)
        # Synchronize geometry/contact observations with the integrated qpos/qvel.
        mujoco.mj_forward(model, data)
        records.append(_sample(model, data, controller, command, terrain, wheel_geom_ids))
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)) or data.time <= previous_time:
            break
    trace = {key: np.asarray([sample[key] for sample in records]) for key in records[0]}
    metrics, checks = assess_trace(name, trace, model, controller.design.pitch, duration)
    wall_time = time.perf_counter() - started
    metrics["wall_time_s"] = wall_time
    metrics["simulation_to_wall_time_ratio"] = float(data.time / wall_time)
    metadata = {
        "schema_version": 1, "scenario": name, "terrain": terrain,
        "hardware_profile": asdict(actuator_profile(profile)),
        "hardware_parameters_sha256": hashlib.sha256(PARAMETERS_PATH.read_bytes()).hexdigest(),
        "mass_properties_sha256": hashlib.sha256(PROPERTY_FILE.read_bytes()).hexdigest(),
        "total_mass_kg": float(model.body_mass.sum()),
        "terrain_geometry_m": dict(TERRAINS[terrain]), "track_width_m": TRACK_WIDTH,
        "wheel_radius_m": WHEEL_RADIUS, "timestep_s": timestep,
        "control_period_s": timestep, "requested_duration_s": duration,
        "seed": int(seed), "randomization": "none (deterministic baseline)",
        "initial_pitch_offset_rad": offset, "equilibrium_pitch_rad": controller.design.pitch,
        "mujoco_version": mujoco.__version__, "numpy_version": np.__version__,
        "python_version": platform.python_version(), "git_commit": _git_revision(),
        "source_xml_sha256": hashlib.sha256(CAD_PATH.read_bytes()).hexdigest(),
        "runtime_xml_sha256": hashlib.sha256(model_xml(terrain, timestep, profile).encode()).hexdigest(),
        "source_sha256": source_hashes,
        "command_schedule": {
            "hip_angle_rad": -.30 if name == "step" else -.20,
            "velocity_segments": ([{"start_s": 1., "end_s": 4., "m_s": .12},
                                   {"start_s": 5., "end_s": 8., "m_s": -.12}]
                                  if name == "move" else
                                  ([{"start_s": 1., "end_s": 4., "m_s": TERRAIN_APPROACH_SPEED[name]}]
                                   if name in ("step", "bridge") else [])),
            "velocity_otherwise_m_s": 0., "yaw_rate_rad_s": 0.,
            "jump_pulse_time_s": 1.5 if name == "jump" else None,
            "step_preparation": "extend hips during initial stationary second" if name == "step" else None,
        },
        "thresholds": dict(THRESHOLDS),
        "actuator_names": [model.actuator(i).name for i in range(model.nu)],
        "actuator_control_ranges": model.actuator_ctrlrange.tolist(),
        "actuator_force_ranges": model.actuator_forcerange.tolist(),
        "state_source": "MuJoCo ground truth; no noisy estimator or terrain perception claimed",
        "jump_events": [asdict(event) for event in controller.jump_supervisor.events],
        "jump_outcome": controller.jump_supervisor.last_outcome,
    }
    summary = _json_safe({"passed": all(checks.values()), "checks": checks,
                          "failed_checks": [key for key, value in checks.items() if not value],
                          "metrics": metrics, "metadata": metadata})
    if directory is not None:
        directory.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(directory / "trace.npz", **trace)
        (directory / "metrics.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False,
                                                          allow_nan=False) + "\n", encoding="utf-8")
    return {**summary, "model": model, "trace": trace}
