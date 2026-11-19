#!/usr/bin/env python3
"""Independent local dynamics, timestep, and contact-based jump checks.

Run from the repository root with:
    .venv/bin/python scripts/check_dynamics.py

This is a diagnostic, not a claim that four-state linear stability proves
stability of the complete contact-switching robot. No simulator state or
controller code is changed by the checks.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import mujoco
import numpy as np

from wheellegged.control import BalanceController, Command, balanced_data
from wheellegged.hardware import DEFAULT_PROFILE, PARAMETERS_PATH, actuator_profile
from wheellegged.lqr import design_lqr
from wheellegged.model import SIDES, WHEEL_RADIUS, ground_forces, load_model, make_data, wheel_centers
from wheellegged.state import observe


def balance_case(dt: float, refresh: bool = True) -> dict:
    model = load_model(timestep=dt)
    data = balanced_data(model, pitch_offset=0.04)
    controller = BalanceController(model, data)
    tail = []
    for _ in range(round(8 / dt)):
        data.ctrl[:] = controller.update()
        mujoco.mj_step(model, data)
        if refresh:
            # Keep the logged kinematics and the next feedback sample at the
            # same integrated state, as in the production rollout/ROS loop.
            mujoco.mj_forward(model, data)
        if data.time >= 6:
            o = observe(model, data)
            tail.append([o.x, o.speed, o.pitch, o.pitch_rate,
                         *[data.actuator(s + "_wheel_voltage").ctrl[0] for s in SIDES]])
    values = np.array(tail)
    return {"dt_s": dt, "refresh_after_step": refresh, "duration_s": 8,
            "tail_start_s": 6, "columns": ["x_m", "speed_m_s", "pitch_rad", "pitch_rate_rad_s",
                                              "left_voltage_V", "right_voltage_V"],
            "tail_mean": values.mean(axis=0).tolist(), "tail_std": values.std(axis=0).tolist(),
            "tail_absolute_peak": np.abs(values).max(axis=0).tolist(),
            "solver_warnings": [int(w.number) for w in data.warning]}


def tangent_and_measurement(model, data):
    """Independent lift/restriction for sagittal tangent states [x,theta,v,w]."""
    tangent = np.zeros((model.nv, 2))
    b = int(model.joint("floating_base").dofadr[0])
    lever = data.body("chassis").xpos - wheel_centers(data).mean(axis=0)
    tangent[b, 0] = 1
    tangent[b, 1], tangent[b + 2, 1], tangent[b + 4, 1] = lever[2], -lever[0], 1
    equal_voltage = np.zeros((model.nu, 1))
    axle_jacobian = np.zeros((3, model.nv))
    for side in SIDES:
        tangent[int(model.joint(side + "_wheel_joint").dofadr[0])] = [1 / WHEEL_RADIUS, -1]
        equal_voltage[model.actuator(side + "_wheel_voltage").id, 0] = 1
        jacobian = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jacobian, None, model.body(side + "_wheel").id)
        axle_jacobian += jacobian / 2
    size = 2 * model.nv + model.na
    lift, measure = np.zeros((size, 4)), np.zeros((4, size))
    lift[:model.nv, :2] = tangent
    lift[model.nv:2 * model.nv, 2:] = tangent
    measure[0, :model.nv], measure[1, b + 4] = axle_jacobian[0], 1
    measure[2, model.nv:2 * model.nv], measure[3, model.nv + b + 4] = axle_jacobian[0], 1
    return tangent, lift, measure, equal_voltage


def local_dynamics() -> dict:
    model = load_model(timestep=0.0005)
    data = balanced_data(model)
    controller = BalanceController(model, data)
    for _ in range(16000):
        mujoco.mj_forward(model, data)
        data.ctrl[:] = controller.update()
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    hip = float(np.mean([data.joint(s + "_hip").qpos[0] for s in SIDES]))
    design = design_lqr(model, hip)
    tangent, lift, measure, equal_voltage = tangent_and_measurement(model, data)
    size = 2 * model.nv + model.na
    fd_cases = []
    for eps in (1e-4, 1e-5, 1e-6, 1e-7, 1e-8):
        full_a, full_b = np.zeros((size, size)), np.zeros((size, model.nu))
        mujoco.mjd_transitionFD(model, data, eps, True, full_a, full_b, None, None)
        projected_a, projected_b = measure @ full_a @ lift, measure @ full_b @ equal_voltage
        poles = np.linalg.eigvals(projected_a - projected_b @ design.gain[None, :])
        fd_cases.append({"epsilon": eps, "projected_A": projected_a.tolist(),
                         "projected_B": projected_b.tolist(),
                         "closed_loop_poles_real_imag": [[float(p.real), float(p.imag)] for p in poles],
                         "closed_loop_spectral_radius": float(np.max(np.abs(poles)))})

    # Verify motor virtual work by changing the actual compiled actuator input.
    old_ctrl = data.ctrl.copy()
    data.ctrl[:] = 0
    mujoco.mj_forward(model, data)
    baseline = data.qfrc_actuator.copy()
    data.ctrl[:] = equal_voltage[:, 0]
    mujoco.mj_forward(model, data)
    generalized_force = tangent.T @ (data.qfrc_actuator - baseline)
    data.ctrl[:] = old_ctrl
    mechanical_damping = tangent.T @ (model.dof_damping[:, None] * tangent)

    # Potential curvature on a rolling, fixed-height axle. Wheels counterrotate
    # relative to the chassis, keeping their world orientation fixed as theta
    # changes. This isolates the small eccentric-wheel COM approximation.
    potentials = {}
    reference_data = make_data(model, hip, design.pitch)
    com_height = reference_data.body("chassis").subtree_com[2] - WHEEL_RADIUS
    analytic_stiffness = np.sum(model.body_mass) * 9.81 * com_height
    for eps in (1e-3, 1e-4):
        values = []
        for delta in (-eps, 0, eps):
            d = make_data(model, hip, design.pitch + delta)
            for side in SIDES:
                d.joint(side + "_wheel_joint").qpos[0] = -delta
            mujoco.mj_forward(model, d)
            values.append(float(9.81 * (model.body_mass @ d.xipos[:, 2])))
        curvature = -(values[0] - 2 * values[1] + values[2]) / eps ** 2
        potentials[str(eps)] = {"restoring_instability_Nm_per_rad": curvature,
                               "relative_difference": float((analytic_stiffness - curvature) / curvature)}

    return {"dt_s": 0.0005, "settled_actual_hip_rad": hip,
            "state_order": ["axle_x", "pitch", "axle_vx", "pitch_rate"],
            "lift_measure_identity_error": float(np.linalg.norm(measure @ lift - np.eye(4))),
            "design_A": design.A.tolist(), "design_B": design.B.tolist(),
            "fd_epsilon_sweep": fd_cases,
            "design_spectral_radius": float(np.max(np.abs(design.poles))),
            "equal_1V_generalized_force_N_Nm": generalized_force.tolist(),
            "expected_equal_1V_force_N_Nm": [0.02 / WHEEL_RADIUS, -0.02],
            "projected_mechanical_damping": mechanical_damping.tolist(),
            "analytic_gravity_stiffness_Nm_per_rad": float(analytic_stiffness),
            "potential_curvature": potentials,
            "scope": "Instantaneous 4-state sagittal projection of mjd_transitionFD; not full-state or hybrid proof."}


def minimum_wheel_clearance(model, data) -> float:
    """Exact cylinder support extent above the flat plane, including roll."""
    bottoms = []
    for side in SIDES:
        geom = model.geom(side + "_tire_contact")
        pose = data.geom(side + "_tire_contact")
        vertical_axis = float(pose.xmat.reshape(3, 3)[2, 2])
        radius, half_width = geom.size[:2]
        extent = radius * np.sqrt(max(0., 1. - vertical_axis ** 2)) + half_width * abs(vertical_axis)
        bottoms.append(float(pose.xpos[2] - extent))
    return min(bottoms)


def jump_case(dt: float) -> dict:
    model = load_model(timestep=dt)
    data = balanced_data(model)
    controller = BalanceController(model, data)
    phase_events, raw_intervals, qualified_flights = [], [], []
    previous_phase, flight_start = None, None
    max_clearance, first_recontact = 0.0, None
    flight_samples, support_start = [], None
    peak_torque = np.zeros(2)
    for index in range(round(6 / dt)):
        mujoco.mj_forward(model, data)
        data.ctrl[:] = controller.update(Command(jump=index == round(1.5 / dt)))
        if controller.phase != previous_phase:
            phase_events.append({"phase": controller.phase, "time_s": float(data.time)})
            previous_phase = controller.phase
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        forces = ground_forces(data)
        clearance = minimum_wheel_clearance(model, data)
        airborne = bool(np.all(forces < 0.05) and clearance > 0.0001)
        if data.time > 1.5:
            max_clearance = max(max_clearance, clearance)
            if airborne and flight_start is None:
                flight_start = float(data.time)
                flight_samples = []
            if airborne:
                mujoco.mj_subtreeVel(model, data)
                flight_samples.append([float(data.time), float(data.body("chassis").subtree_com[2]),
                                       float(data.body("chassis").subtree_linvel[2])])
            if not airborne and flight_start is not None:
                end = float(data.time)
                raw_intervals.append([flight_start, end])
                samples = np.array(flight_samples)
                sample_duration = float(samples[-1, 0] - samples[0, 0])
                if sample_duration >= 0.010:
                    slope = np.polyfit(samples[:, 0] - samples[0, 0], samples[:, 2], 1)[0]
                    qualified_flights.append({"start_s": flight_start, "end_s": float(samples[-1, 0]),
                                             "first_nonairborne_sample_s": end,
                                             "duration_s": sample_duration,
                                             "takeoff_com_vz_m_s": float(samples[0, 2]),
                                             "com_rise_after_takeoff_m": float(max(samples[:, 1]) - samples[0, 1]),
                                             "fitted_com_vertical_acceleration_m_s2": float(slope)})
                flight_start = None
            if qualified_flights and first_recontact is None:
                if np.all(forces > 0.10):
                    support_start = float(data.time) if support_start is None else support_start
                    if data.time - support_start >= 0.010:
                        first_recontact = {"contact_start_s": support_start,
                                           "confirmed_after_10ms_s": float(data.time)}
                else:
                    support_start = None
        peak_torque = np.maximum(peak_torque, [abs(data.actuator(s + "_servo").force[0]) for s in SIDES])
    if flight_start is not None:
        raw_intervals.append([flight_start, float(data.time)])
    observation = observe(model, data)
    return {"dt_s": dt, "phase_events": phase_events, "raw_airborne_intervals_s": raw_intervals,
            "supervisor_events": [asdict(event) for event in controller.jump_supervisor.events],
            "supervisor_last_outcome": controller.jump_supervisor.last_outcome,
            "flight_criteria": {"each_wheel_force_less_than_N": .05,
                                "each_wheel_geometric_clearance_above_m": .0001,
                                "minimum_continuous_interval_s": .010},
            "qualified_flights_at_least_10ms": qualified_flights,
            "airborne_duration_s": sum(f["duration_s"] for f in qualified_flights),
            "max_minimum_wheel_clearance_m": max_clearance,
            "first_bilateral_recontact_after_qualified_flight": first_recontact,
            "peak_servo_torque_Nm": peak_torque.tolist(),
            "final_pitch_rad": observation.pitch, "final_speed_m_s": observation.speed,
            "final_phase": controller.phase,
            "solver_warnings": [int(w.number) for w in data.warning]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "docs/evidence/dynamics_review.json")
    args = parser.parse_args()
    result = {"mujoco": mujoco.__version__, "profile": asdict(actuator_profile(DEFAULT_PROFILE)),
              "total_mass_kg": float(load_model().body_mass.sum()),
              "hardware_parameters_sha256": hashlib.sha256(PARAMETERS_PATH.read_bytes()).hexdigest(),
              "cad_mass_properties_sha256": hashlib.sha256((ROOT / "models/cad_mass_properties.json").read_bytes()).hexdigest(),
              "source_sha256": {}, "balance_cases": [], "jump_cases": [],
              "diagnostic_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    for name in ("model.py", "lqr.py", "control.py", "state.py", "jump.py", "hardware.py", "mass_properties.py"):
        result["source_sha256"][name] = hashlib.sha256((ROOT / "src/wheellegged" / name).read_bytes()).hexdigest()
    result["local_dynamics"] = local_dynamics()
    print("MuJoCo finite-difference projection completed", flush=True)
    for dt in (0.001, 0.0005, 0.00025):
        result["balance_cases"].append(balance_case(dt))
        result["jump_cases"].append(jump_case(dt))
        print(f"Completed balance/jump at dt={dt} s", flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(args.output, flush=True)


if __name__ == "__main__":
    main()
