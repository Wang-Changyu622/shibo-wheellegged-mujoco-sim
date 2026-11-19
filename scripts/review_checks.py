#!/usr/bin/env python3
"""Record isolated code-review probes; these are observations, not task scores.

Known limitations and injected numerical faults are recorded without asserting
that they are acceptance-test failures. ROS is optional and imported only with
--ros. MuJoCo warning logs stay alongside the chosen JSON output.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

import mujoco
import numpy as np

from wheellegged.control import BalanceController, Command, balanced_data
from wheellegged.evaluate import _ground_heights, _sample
from wheellegged.lqr import design_lqr
from wheellegged.model import load_model
from wheellegged.viewer import _Session


ROOT = Path(__file__).resolve().parents[1]


def source_hashes() -> dict[str, str]:
    paths = [ROOT / "src/wheellegged" / f"{name}.py" for name in (
        "control", "evaluate", "model", "jump", "lqr", "viewer", "state",
        "hardware", "mass_properties")]
    paths += [ROOT / path for path in (
        "scripts/review_checks.py", "ros2/wheellegged_ros/wheellegged_ros/sim_node.py",
        "config/hardware_parameters.json", "models/cad_mass_properties.json")]
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in paths}


def ros_fault_probe() -> dict:
    """Exercise the real ROS callback and physics, without a DDS transport test."""
    old_path = sys.path.copy()
    old_environment = {key: os.environ.get(key) for key in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")}
    node = None
    initialized = False
    try:
        sys.path.insert(0, str(ROOT / "ros2/wheellegged_ros"))
        os.environ.update(ROS_DOMAIN_ID="76", ROS_LOCALHOST_ONLY="1")
        import rclpy
        from wheellegged_ros.sim_node import SimulationNode

        rclpy.init()
        initialized = True
        node = SimulationNode()
        for _ in range(3):
            node._advance()
        before = float(node.data.time)
        node.data.joint("floating_base").qvel[0] = 1e30
        failure = None
        try:
            node._advance()
        except Exception as error:
            failure = f"{type(error).__name__}: {error}"
        return {
            "fault": "injected qvel=1e30; real MuJoCo step inside ROS callback",
            "transport_scope": "direct callback probe, not a DDS integration test",
            "time_before": before, "time_after": float(node.data.time),
            "callback_exception": failure,
        }
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if initialized and rclpy.ok():
                rclpy.shutdown()
        finally:
            sys.path[:] = old_path
            for key, value in old_environment.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value


def run_checks(include_ros: bool = False) -> dict:
    model = load_model()
    data = balanced_data(model)
    controller = BalanceController(model, data)
    wheel_ids = [model.geom(f"{side}_tire_contact").id for side in ("L", "R")]
    commands = [Command(), Command(left_hip_angle=-.3, right_hip_angle=0.)]
    samples = [_sample(model, data, controller, command, "flat", wheel_ids) for command in commands]
    probes = {"independent_command_log_collision": {
        "commands_hip_targets": [list(command.hip_targets) for command in commands],
        "recorded_command_hip_angle": [sample["command_hip_angle"] for sample in samples],
        "all_sample_fields_identical": all(np.array_equal(samples[0][key], samples[1][key])
                                            for key in samples[0]),
        "missing_fields": [key for key in ("command_left_hip_angle", "command_right_hip_angle", "roll_reference")
                           if key not in samples[0]],
    }}

    # Frozen observations emulate persistent stall. This is a controller-contract
    # fixture, not a free-base physical rollout or a claim about any one step.
    controller = BalanceController(model, balanced_data(model))
    for index in range(8000):
        controller.data.time = index * controller.dt
        control = controller.update(Command(velocity=.2 if index < 6000 else 0.))
    probes["stalled_reference_after_stop"] = {
        "fixture": "fixed robot observations, 3 s at +0.2 m/s then 1 s at zero command",
        "physics_integrated": False,
        "velocity_reference_m_s": float(controller.velocity_reference),
        "position_reference_minus_fixed_axle_m": float(controller.position_reference[0]
                                                      - controller.data.body("L_wheel").xpos[0]),
        "wheel_voltage_commands_v": [float(control[model.actuator(f"{side}_wheel_voltage").id])
                                     for side in ("L", "R")],
    }

    designs = {str(dt): design_lqr(load_model(timestep=dt)) for dt in (.0005, .00025)}
    probes["coupled_physics_and_control_period"] = {
        "gain_by_timestep": {dt: design.gain.tolist() for dt, design in designs.items()},
        "gains_identical": bool(np.array_equal(designs["0.0005"].gain, designs["0.00025"].gain)),
        "control_updates_per_simulation_second": [2000, 4000],
    }

    step_model = load_model("step")
    step_model.geom("terrain_step").size[2] = .006
    step_model.geom("terrain_step").pos[2] = .006
    probes["metric_terrain_not_read_from_model"] = {
        "fixture": "compiled platform raised from 6 mm to 12 mm as a terrain-randomization prototype",
        "compiled_top_m": float(step_model.geom("terrain_step").pos[2] + step_model.geom("terrain_step").size[2]),
        "metric_ground_heights_m": _ground_heights(np.array([[.5, .04365, .03], [.5, -.04365, .03]]), "step").tolist(),
        "nominal_fixed_scene_affected": False,
    }

    session = _Session("flat", "hardware_2s", "balance", automatic=False)
    for _ in range(12):
        session.step()
    before = float(session.data.time)
    session.data.joint("floating_base").qvel[0] = 1e30
    session.step()
    probes["viewer_bad_velocity_guard"] = {
        "fault": "injected qvel=1e30 to trigger MuJoCo's numerical reset",
        "time_before": before, "time_after": float(session.data.time),
        "paused": session.paused, "notice": session.notice,
    }
    probes["ros_bad_velocity_guard"] = ros_fault_probe() if include_ros else {
        "status": "not_run", "reason": "--ros was not requested; ROS dependencies are optional"}
    return {
        "status": "observations_recorded",
        "scope": "Code-review probes, not acceptance pass/fail. Known limitations and injected faults are observations; nominal task success is evaluated separately.",
        "mujoco_version": mujoco.__version__, "ros_probe_enabled": include_ros,
        "probes": probes, "source_sha256": source_hashes(),
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "outputs/review-repro/review_checks.json")
    parser.add_argument("--ros", action="store_true", help="include the optional ROS adapter fault probe")
    args = parser.parse_args(argv)
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({"status": "running", "scope": "code-review probes"}) + "\n")
    previous_directory = Path.cwd()
    try:
        # MuJoCo's warning log is relative to cwd. Fault-injection evidence must
        # not leave MUJOCO_LOG.TXT in the repository root or caller's directory.
        os.chdir(output.parent)
        report = run_checks(include_ros=args.ros)
        output.write_text(json.dumps(report, indent=2) + "\n")
    except Exception as error:
        output.write_text(json.dumps({
            "status": "probe_execution_error", "error": f"{type(error).__name__}: {error}",
            "source_sha256": source_hashes(),
        }, indent=2) + "\n")
        raise
    finally:
        os.chdir(previous_directory)
    print(json.dumps({"status": report["status"], "output": str(output),
                      "ros_probe_enabled": args.ros}, indent=2))


if __name__ == "__main__":
    main()
