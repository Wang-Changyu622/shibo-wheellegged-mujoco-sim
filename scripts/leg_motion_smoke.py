#!/usr/bin/env python3
"""Measure keyboard-driven leg motion and standing jumps using actual physics.

The application's key handlers receive PRESS/RELEASE events; no OS keys, pose
animation or controller phase labels stand in for physical motion. All checks
sample every 0.5 ms integration step. This script needs no display.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import glfw
import mujoco
import numpy as np

from wheellegged.evaluate import THRESHOLDS, _sample, continuous_duration
from wheellegged.model import SIDES
from wheellegged.viewer import PHYSICS_TIMESTEP, _Session


class Trial:
    """One genuine control/physics rollout with an auditable key event ledger."""

    def __init__(self, *, roll_limit: float = .25):
        self.session = _Session("flat", "hardware_2s", "balance", False)
        self.wheel_ids = [self.session.model.geom(f"{side}_tire_contact").id for side in SIDES]
        self.records: list[dict] = []
        self.events: list[dict] = []
        self.checks: dict[str, bool] = {}
        self.metrics: dict = {}
        self.roll_limit = roll_limit
        self.sample()

    def sample(self):
        s = self.session
        sample = _sample(s.model, s.data, s.controller, s.last_command, "flat", self.wheel_ids)
        chassis = s.data.body("chassis")
        # Independent MuJoCo kinematics: do not evaluate the commanded-angle
        # four-bar function to decide whether the actual legs moved.
        rotation = chassis.xmat.reshape(3, 3)
        sample["leg_extension"] = np.asarray([
            -(rotation.T @ (s.data.body(f"{side}_wheel").xpos - chassis.xpos))[2]
            for side in SIDES
        ])
        sample["requested_hips"] = np.asarray(s.command.hip_targets)
        self.records.append(sample)

    def advance(self, seconds: float):
        for _ in range(round(seconds / PHYSICS_TIMESTEP)):
            self.session.step()
            self.sample()
            if self.session.paused:
                raise RuntimeError("physics paused during leg motion acceptance")

    def key(self, key: int, action: int):
        data = self.session.data
        qpos, qvel, now = data.qpos.copy(), data.qvel.copy(), float(data.time)
        self.session.handle_key(key, action)
        untouched = (np.array_equal(qpos, data.qpos) and np.array_equal(qvel, data.qvel)
                     and now == data.time)
        self.events.append({"time_s": now, "key": chr(key), "action": int(action),
                            "physics_unchanged": bool(untouched)})

    def tap(self, key: int):
        self.key(key, glfw.PRESS)
        self.key(key, glfw.RELEASE)

    def hold(self, keys: tuple[int, ...], seconds: float):
        for key in keys:
            self.key(key, glfw.PRESS)
        self.advance(seconds)
        for key in keys:
            self.key(key, glfw.RELEASE)

    def tail(self, key: str, seconds: float = .5):
        since = self.session.data.time - seconds + 1e-10
        return np.asarray([row[key] for row in self.records if row["time"] >= since])

    def posture(self):
        return {"extension_m": self.tail("leg_extension").mean(axis=0).tolist(),
                "hip_rad": self.tail("hip").mean(axis=0).tolist(),
                "roll_rad": float(self.tail("roll").mean()),
                "height_m": float(self.tail("xyz")[:, 2].mean()),
                "requested_hip_rad": list(self.session.command.hip_targets)}

    def finish(self):
        s = self.session
        trace = {key: np.asarray([record[key] for record in self.records])
                 for key in self.records[0]}
        self.metrics.update({
            "elapsed_s": float(s.data.time),
            "pitch_peak_rad": float(np.max(abs(trace["pitch"]))),
            "roll_peak_rad": float(np.max(abs(trace["roll"]))),
            "closure_peak_m": float(np.max(trace["closure_error"])),
            "nonwheel_support_peak_n": float(np.max(trace["nonwheel_ground_force"])),
            "tail_speed_peak_m_s": float(np.max(abs(self.tail("speed")))),
            "warning_counts": s.data.warning.number.tolist(),
            "allowed_roll_peak_rad": self.roll_limit,
        })
        self.checks.update({
            "events_never_write_physics": all(event["physics_unchanged"] for event in self.events),
            "finite_state_and_telemetry": all(np.all(np.isfinite(value)) for value in trace.values()
                                              if np.issubdtype(value.dtype, np.number)),
            "time_monotonic": bool(np.all(np.diff(trace["time"]) > 0)),
            "closed_chain_bounded": self.metrics["closure_peak_m"] < THRESHOLDS["closure_peak_m"],
            "pitch_bounded": self.metrics["pitch_peak_rad"] < THRESHOLDS["pitch_peak_rad"],
            "roll_bounded": self.metrics["roll_peak_rad"] < self.roll_limit,
            "no_nonwheel_ground_support": self.metrics["nonwheel_support_peak_n"] < .05,
            "no_solver_warnings": not any(self.metrics["warning_counts"]),
            "requested_hips_in_bounds": bool(np.all((trace["requested_hips"] >= -.3 - 1e-10)
                                                    & (trace["requested_hips"] <= 1e-10))),
            "control_limits": bool(np.all((trace["ctrl"] >= s.model.actuator_ctrlrange[:, 0] - 1e-8)
                                          & (trace["ctrl"] <= s.model.actuator_ctrlrange[:, 1] + 1e-8))),
            "actuator_force_limits": bool(np.all((trace["actuator_force"] >= s.model.actuator_forcerange[:, 0] - 1e-8)
                                                 & (trace["actuator_force"] <= s.model.actuator_forcerange[:, 1] + 1e-8))),
            "tail_stopped": self.metrics["tail_speed_peak_m_s"] < .03,
        })
        checks = {key: bool(value) for key, value in self.checks.items()}
        return {"passed": all(checks.values()), "checks": checks,
                "failed_checks": [key for key, value in checks.items() if not value],
                "metrics": self.metrics, "events": self.events,
                "jump_events": [asdict(event) for event in s.controller.jump_supervisor.events],
                "trace": trace}


def height_trial():
    """Each side in both directions, release hold, bilateral travel and reset."""
    trial = Trial()
    trial.advance(1.)
    baseline = trial.posture()
    baseline_extensions = np.asarray(baseline["extension_m"])
    trial.metrics["nominal"] = baseline
    for side, up, down, sign in ((0, glfw.KEY_Q, glfw.KEY_Z, 1.),
                                 (1, glfw.KEY_E, glfw.KEY_C, -1.)):
        label = SIDES[side]
        trial.hold((up,), .6)
        released = tuple(trial.session.command.hip_targets)
        trial.advance(1.5)
        raised = trial.posture()
        change = np.asarray(raised["extension_m"]) - baseline_extensions
        trial.metrics[f"{label}_raised"] = raised
        trial.checks[f"{label}_release_holds_target"] = released == tuple(trial.session.command.hip_targets)
        trial.checks[f"{label}_extension_really_increases"] = change[side] > .003
        trial.checks[f"{label}_other_leg_remains_near_nominal"] = abs(change[1 - side]) < .002
        trial.checks[f"{label}_roll_follows_raised_side"] = sign * raised["roll_rad"] > .02
        trial.hold((down,), 1.2)
        trial.advance(1.5)
        lowered = trial.posture()
        trial.metrics[f"{label}_lowered"] = lowered
        change = np.asarray(lowered["extension_m"]) - baseline_extensions
        trial.checks[f"{label}_extension_really_decreases"] = change[side] < -.003
        trial.checks[f"{label}_roll_follows_lowered_side"] = sign * lowered["roll_rad"] < -.02
        trial.tap(glfw.KEY_H)
        trial.advance(1.5)
    trial.hold((glfw.KEY_Q, glfw.KEY_E), 3.)
    trial.advance(1.5)
    raised = trial.posture()
    trial.metrics["both_raised_to_limit"] = raised
    trial.checks["bilateral_upper_target_limit"] = np.allclose(raised["requested_hip_rad"], [-.3, -.3], atol=1e-10)
    trial.checks["bilateral_actual_raise"] = np.all(np.asarray(raised["extension_m"]) > baseline_extensions + .006)
    trial.hold((glfw.KEY_Z, glfw.KEY_C), 4.)
    trial.advance(1.5)
    lowered = trial.posture()
    trial.metrics["both_lowered_to_limit"] = lowered
    trial.checks["bilateral_lower_target_limit"] = np.allclose(lowered["requested_hip_rad"], [0., 0.], atol=1e-10)
    trial.checks["bilateral_actual_lower"] = np.all(np.asarray(lowered["extension_m"]) < baseline_extensions - .015)
    trial.tap(glfw.KEY_H)
    trial.advance(2.)
    restored = trial.posture()
    trial.metrics["restored"] = restored
    trial.checks["neutral_height_restored"] = np.max(abs(np.asarray(restored["extension_m"]) - baseline_extensions)) < .0015
    trial.checks["neutral_roll_restored"] = abs(restored["roll_rad"]) < .02
    # One continuous recorded sequence is suitable for visual review: actual
    # left/right changes, full bilateral travel, neutral reset, then the J key.
    jump_time = float(trial.session.data.time)
    trial.tap(glfw.KEY_J)
    trial.advance(4.5)
    airtime, clearance = physical_flight(trial.records, jump_time)
    trial.metrics["final_jump_airborne_duration_s"] = airtime
    trial.metrics["final_jump_minimum_wheel_clearance_peak_m"] = clearance
    trial.checks["final_neutral_jump_really_airborne"] = airtime >= .020
    trial.checks["final_neutral_jump_landed"] = trial.session.controller.jump_supervisor.last_outcome == "landed"
    return trial.finish()


def physical_flight(records: list[dict], after: float):
    times = np.asarray([row["time"] for row in records])
    force = np.asarray([row["ground_force"] for row in records])
    clearance = np.asarray([row["wheel_clearance"] for row in records])
    airborne = ((times > after) & np.all(force < .02, axis=1)
                & np.all(clearance > .001, axis=1))
    return continuous_duration(airborne, times), float(np.max(np.min(clearance, axis=1)))


def jump_trial(asymmetric: bool = False, extreme: bool = False):
    """The J key must cause true free flight, touchdown and posture recovery."""
    # A deliberate full-travel split is about 0.31 rad on flat ground. Its
    # separately declared 0.35 rad bound must not relax the normal cases.
    trial = Trial(roll_limit=.35 if extreme else .25)
    trial.advance(1.5)
    if extreme:
        trial.hold((glfw.KEY_Q, glfw.KEY_C), 3.)
        trial.advance(2.)
    elif asymmetric:
        trial.hold((glfw.KEY_Q,), .6)
        trial.advance(1.5)
    before = trial.posture()
    requested = tuple(trial.session.command.hip_targets)
    jump_time = float(trial.session.data.time)
    trial.tap(glfw.KEY_J)
    trial.advance(4.5)
    trace = {key: np.asarray([record[key] for record in trial.records])
             for key in trial.records[0]}
    flight_duration, clearance = physical_flight(trial.records, jump_time)
    after = trial.posture()
    events = trial.session.controller.jump_supervisor.events
    trial.metrics.update({"before_jump": before, "after_jump": after,
                          "jump_requested_at_s": jump_time,
                          "airborne_duration_s": flight_duration,
                          "minimum_wheel_clearance_peak_m": clearance,
                          "jump_outcome": trial.session.controller.jump_supervisor.last_outcome})
    trial.checks.update({
        "one_accepted_jump": sum(event.reason == "request_accepted" for event in events) == 1,
        "sustained_physical_takeoff": flight_duration >= .020,
        "landed_on_both_wheels": bool(np.all(trial.tail("ground_support_vertical") > .1)
                                      and np.all(abs(trial.tail("wheel_clearance")) < .001)),
        "returned_to_balance": bool(np.all(trial.tail("phase") == "balance")),
        "jump_outcome_landed": trial.session.controller.jump_supervisor.last_outcome == "landed",
        "requested_posture_preserved": tuple(trial.session.command.hip_targets) == requested,
        "physical_leg_posture_restored": np.max(abs(np.asarray(after["extension_m"]) - before["extension_m"])) < .0015,
        "physical_roll_restored": abs(after["roll_rad"] - before["roll_rad"]) < .02,
    })
    pushing = np.flatnonzero(trace["phase"] == "push")
    trial.checks["push_begins_with_near_level_symmetric_actual_posture"] = bool(
        len(pushing) and abs(trace["roll"][pushing[0]]) < .08
        and abs(np.diff(trace["hip"][pushing[0]])[0]) < .02)
    if extreme:
        trial.checks["full_independent_travel_was_requested"] = np.allclose(before["requested_hip_rad"], [-.3, 0.], atol=1e-10)
        trial.checks["full_independent_travel_was_physical"] = (
            before["extension_m"][0] - before["extension_m"][1] > .024
            and .25 < before["roll_rad"] < .35)
    return trial.finish()


def drive_trial():
    """Raise one leg during forward motion, then retain height while braking."""
    trial = Trial()
    trial.advance(1.)
    start = trial.records[-1]["axle_xyz"].copy()
    trial.key(glfw.KEY_W, glfw.PRESS)
    trial.hold((glfw.KEY_Q,), .4)
    requested = tuple(trial.session.command.hip_targets)
    trial.advance(2.)
    release = trial.records[-1]["axle_xyz"].copy()
    trial.key(glfw.KEY_W, glfw.RELEASE)
    trial.advance(4.)
    posture = trial.posture()
    trial.metrics.update({"held_forward_duration_s": 2.4,
                          "forward_displacement_m": float(release[0] - start[0]),
                          "posture_after_braking": posture})
    trial.checks.update({
        "forward_motion_with_asymmetric_height": release[0] - start[0] > .15,
        "release_zeros_movement_target": trial.session.command.velocity == 0. and trial.session.command.yaw_rate == 0.,
        "release_preserves_height_target": tuple(trial.session.command.hip_targets) == requested,
        "asymmetric_height_remains_physical": posture["extension_m"][0] - posture["extension_m"][1] > .002,
        "asymmetric_roll_remains_controlled": .01 < posture["roll_rad"] < .12,
    })
    return trial.finish()


TRIALS = {"independent_heights": height_trial, "neutral_jump": jump_trial,
          "asymmetric_jump": lambda: jump_trial(True),
          "full_split_jump": lambda: jump_trial(extreme=True), "drive_and_height": drive_trial}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "leg_motion.json")
    parser.add_argument("--trace-dir", type=Path)
    parser.add_argument("--case", choices=("all", *TRIALS), default="all")
    args = parser.parse_args()
    cases = TRIALS if args.case == "all" else {args.case: TRIALS[args.case]}
    files = [Path(__file__), *(ROOT / "src" / "wheellegged").glob("*.py")]
    source_hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in files}
    results = {}
    started = time.perf_counter()
    for name, run in cases.items():
        result = run()
        trace = result.pop("trace")
        if args.trace_dir is not None:
            args.trace_dir.mkdir(parents=True, exist_ok=True)
            path = args.trace_dir / f"{name}.npz"
            if path.exists():
                raise FileExistsError(f"Refusing to overwrite trial trace: {path}")
            np.savez_compressed(path, **trace)
        results[name] = result
        print(f"{name}: {'PASS' if result['passed'] else 'FAIL'} {result['failed_checks']}", flush=True)
    sources_unchanged = all(hashlib.sha256(path.read_bytes()).hexdigest()
                            == source_hashes[str(path.relative_to(ROOT))] for path in files)
    report = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
              "passed": sources_unchanged and all(case["passed"] for case in results.values()),
              "sources_unchanged_during_run": sources_unchanged,
              "profile": "hardware_2s", "terrain": "flat", "timestep_s": PHYSICS_TIMESTEP,
              "input_delivery": "Application key-handler PRESS/RELEASE injection; no operating-system keyboard automation",
              "height_measurement": "Chassis-frame vertical distance from chassis origin to each wheel axle, from MuJoCo kinematics",
              "jump_acceptance": "Both contact forces <0.02 N and both exact cylinder bottoms >1 mm above floor for >=20 ms, then supported recovery",
              "mujoco_version": mujoco.__version__, "python_version": platform.python_version(),
              "wall_seconds": time.perf_counter() - started,
              "source_sha256": source_hashes,
              "cases": results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
