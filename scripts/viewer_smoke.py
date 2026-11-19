#!/usr/bin/env python3
"""Check real GLFW windows, press/release/focus input and closed-loop turns.

Events enter this application's registered callbacks, never the operating system.
Run in a graphical desktop:
    .venv/bin/python scripts/viewer_smoke.py --output outputs/viewer_smoke.json
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import sys
import threading
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import glfw
import mujoco
import numpy as np

from wheellegged.state import observe
from wheellegged.viewer import (GlfwViewer, MANUAL_SPEED, MANUAL_YAW_RATE,
                               PHYSICS_TIMESTEP, _Session, run_viewer)


def closed_loop_turn(turn_key: int) -> dict:
    """Hold W plus A/D for 3 s, release both, then let physics brake for 6 s."""
    session = _Session("flat", "hardware_2s", "balance", False)
    for _ in range(round(1. / PHYSICS_TIMESTEP)):
        session.step()
    initial = observe(session.model, session.data)
    session.handle_key(glfw.KEY_W, glfw.PRESS)
    session.handle_key(turn_key, glfw.PRESS)
    records = []

    def advance(duration):
        for _ in range(round(duration / PHYSICS_TIMESTEP)):
            session.step()
            state = observe(session.model, session.data)
            records.append((state.time, state.speed, state.yaw_rate, state.pitch, state.roll))

    advance(3.)
    at_release = observe(session.model, session.data)
    qpos, qvel, release_time = session.data.qpos.copy(), session.data.qvel.copy(), session.data.time
    session.handle_key(glfw.KEY_W, glfw.RELEASE)
    session.handle_key(turn_key, glfw.RELEASE)
    target_zero = session.command.velocity == 0. and session.command.yaw_rate == 0.
    untouched = (np.array_equal(qpos, session.data.qpos)
                 and np.array_equal(qvel, session.data.qvel) and session.data.time == release_time)
    advance(6.)
    trace = np.asarray(records)
    tail = trace[trace[:, 0] >= session.data.time - 1.]
    displacement = np.array([at_release.x - initial.x, at_release.y - initial.y])
    yaw_change = float(np.arctan2(np.sin(at_release.yaw - initial.yaw),
                                 np.cos(at_release.yaw - initial.yaw)))
    sign = 1. if turn_key == glfw.KEY_A else -1.
    metrics = {
        "held_seconds": 3., "release_recovery_seconds": 6.,
        "moving_displacement_xy_m": displacement.tolist(),
        "moving_translation_m": float(np.linalg.norm(displacement)),
        "yaw_change_rad": yaw_change, "speed_at_release_m_s": at_release.speed,
        "qvel_norm_at_release": float(np.linalg.norm(qvel)),
        "pitch_peak_rad": float(np.max(np.abs(trace[:, 3]))),
        "roll_peak_rad": float(np.max(np.abs(trace[:, 4]))),
        "tail_speed_peak_m_s": float(np.max(np.abs(tail[:, 1]))),
        "tail_yaw_rate_peak_rad_s": float(np.max(np.abs(tail[:, 2]))),
        "warning_counts": session.data.warning.number.tolist(),
    }
    checks = {
        "target_zero_on_release": target_zero,
        "release_does_not_write_physics_state": bool(untouched),
        "actual_motion_still_present_at_release": abs(at_release.speed) > .03,
        "real_forward_translation": bool(metrics["moving_translation_m"] > .15 and displacement[0] > .1),
        "turns_in_requested_direction": sign * yaw_change > .5,
        "pitch_bounded": metrics["pitch_peak_rad"] < .45,
        "roll_bounded": metrics["roll_peak_rad"] < .25,
        "released_speed_converges": metrics["tail_speed_peak_m_s"] < .03,
        "released_yaw_rate_converges": metrics["tail_yaw_rate_peak_rad_s"] < .05,
        "no_solver_warnings": not any(metrics["warning_counts"]),
    }
    return {"keys": "W+" + chr(turn_key), "metrics": metrics, "checks": checks,
            "passed": bool(all(checks.values()))}


@dataclass
class Stage:
    name: str
    events: tuple
    predicate: object
    hold_seconds: float = 0.


class CallbackProbe:
    """Drive registered callbacks on the same thread that renders physics."""

    def __init__(self):
        self.original_key = glfw.set_key_callback
        self.original_focus = glfw.set_window_focus_callback
        self.original_create = glfw.create_window
        self.original_destroy = glfw.destroy_window
        self.original_render = GlfwViewer.render
        self.callbacks = {}
        self.windows_created = self.windows_destroyed = 0
        self.active = self.finished = False
        self.checks, self.snapshots, self.injected_events = {}, [], []
        self.frames = 0
        self.current_stage = None
        self.stage_started = 0.
        self.stage_frame = 0
        self.state = {}
        self.plan = self.stages()

    def register_key(self, window, callback):
        self.callbacks["key"] = (window, callback)
        return self.original_key(window, callback)

    def register_focus(self, window, callback):
        self.callbacks["focus"] = (window, callback)
        return self.original_focus(window, callback)

    def create(self, *args, **kwargs):
        window = self.original_create(*args, **kwargs)
        self.windows_created += int(bool(window))
        return window

    def destroy(self, window):
        self.original_destroy(window)
        self.windows_destroyed += 1

    @staticmethod
    def press(key):
        return ("key", key, glfw.PRESS)

    @staticmethod
    def release(key):
        return ("key", key, glfw.RELEASE)

    @staticmethod
    def tap(key):
        return (("key", key, glfw.PRESS), ("key", key, glfw.RELEASE))

    def inject(self, event):
        kind = event[0]
        window, callback = self.callbacks[kind]
        if kind == "key":
            callback(window, event[1], 0, event[2], 0)
            self.injected_events.append({"kind": kind, "key": event[1], "action": event[2]})
        else:
            callback(window, event[1])
            self.injected_events.append({"kind": kind, "focused": event[1]})

    @staticmethod
    def target(state, velocity, yaw):
        return state["velocity_target"] == velocity and state["yaw_target"] == yaw

    def stages(self):
        p, r, tap, target = self.press, self.release, self.tap, self.target
        speed, yaw = MANUAL_SPEED, MANUAL_YAW_RATE
        yield Stage("focus_callback_ready", (("focus", True),), lambda s: s["focused"])
        yield Stage("w_press_moves_forward", (p(glfw.KEY_W),), lambda s: target(s, speed, 0.))
        # No repeats: each rendered frame must retain the held target for 1.1 s.
        yield Stage("held_without_repeat_for_one_second", (), lambda s: target(s, speed, 0.), 1.1)
        yield Stage("w_a_combination", (p(glfw.KEY_A),), lambda s: target(s, speed, yaw))
        yield Stage("release_w_preserves_held_turn", (r(glfw.KEY_W),), lambda s: target(s, 0., yaw))
        yield Stage("release_a_zeros_axes", (r(glfw.KEY_A),), lambda s: target(s, 0., 0.))
        yield Stage("s_press_moves_backward", (p(glfw.KEY_S),), lambda s: target(s, -speed, 0.))
        yield Stage("opposite_w_s_cancel", (p(glfw.KEY_W),), lambda s: target(s, 0., 0.))
        yield Stage("release_s_restores_held_w", (r(glfw.KEY_S),), lambda s: target(s, speed, 0.))
        yield Stage("release_last_motion_key_zeros_target", (r(glfw.KEY_W),), lambda s: target(s, 0., 0.))
        yield Stage("d_press_turns_right", (p(glfw.KEY_D),), lambda s: target(s, 0., -yaw))
        yield Stage("release_d_stops_turn_target", (r(glfw.KEY_D),), lambda s: target(s, 0., 0.))
        yield Stage("moving_before_focus_loss", (p(glfw.KEY_W), p(glfw.KEY_A)), lambda s: target(s, speed, yaw))
        yield Stage("focus_loss_clears_held_and_targets", (("focus", False),),
                    lambda s: not s["focused"] and not s["held_keys"] and target(s, 0., 0.))
        yield Stage("unfocused_press_and_repeat_ignored", (p(glfw.KEY_W), ("key", glfw.KEY_A, glfw.REPEAT)),
                    lambda s: not s["held_keys"] and target(s, 0., 0.))
        yield Stage("refocus_does_not_resume_movement", (("focus", True),), lambda s: target(s, 0., 0.))
        yield Stage("fresh_press_after_refocus", (p(glfw.KEY_W),), lambda s: target(s, speed, 0.))
        yield Stage("x_explicit_stop", tap(glfw.KEY_X), lambda s: not s["held_keys"] and target(s, 0., 0.))
        yield Stage("repeat_does_not_undo_x_stop", (("key", glfw.KEY_W, glfw.REPEAT),), lambda s: target(s, 0., 0.))
        yield Stage("fresh_press_after_x", (r(glfw.KEY_W), p(glfw.KEY_W)), lambda s: target(s, speed, 0.))
        yield Stage("pause_clears_movement", tap(glfw.KEY_P), lambda s: s["paused"] and target(s, 0., 0.))
        frozen_time = self.state["simulation_time_s"]
        yield Stage("pause_freezes_physics", (), lambda s: s["simulation_time_s"] == frozen_time, .25)
        yield Stage("release_and_repeat_while_paused", (r(glfw.KEY_W), ("key", glfw.KEY_P, glfw.REPEAT)),
                    lambda s: s["paused"] and target(s, 0., 0.))
        yield Stage("reset_while_paused", tap(glfw.KEY_R),
                    lambda s: s["paused"] and s["simulation_time_s"] == 0. and not s["held_keys"])
        yield Stage("space_resumes_without_stale_keys", tap(glfw.KEY_SPACE), lambda s: not s["paused"] and target(s, 0., 0.))
        yield Stage("jump_press_is_one_pulse", (p(glfw.KEY_J),), lambda s: s["jump_request_count"] == 1)
        yield Stage("jump_repeat_does_not_retrigger", (("key", glfw.KEY_J, glfw.REPEAT),) * 3,
                    lambda s: s["jump_request_count"] == 1, .15)
        yield Stage("jump_release_does_not_retrigger", (r(glfw.KEY_J),), lambda s: s["jump_request_count"] == 1)
        yield Stage("free_camera_toggle", tap(glfw.KEY_F), lambda s: not s["camera_follow"])
        yield Stage("automatic_mode_toggle", tap(glfw.KEY_M), lambda s: s["automatic"])
        yield Stage("manual_mode_toggle_stops", tap(glfw.KEY_M), lambda s: not s["automatic"] and target(s, 0., 0.))
        for key, name in ((glfw.KEY_3, "step"), (glfw.KEY_4, "bridge"), (glfw.KEY_5, "jump"),
                          (glfw.KEY_1, "balance"), (glfw.KEY_2, "move")):
            yield Stage("scene_" + name, tap(key), lambda s, scene=name: s["scenario"] == scene and s["automatic"])

    def render(self, viewer):
        rendered = self.original_render(viewer)
        if not self.active or self.finished:
            return rendered
        self.frames += 1
        session = viewer.session
        self.state = {
            "simulation_time_s": float(session.data.time), "scenario": session.scenario,
            "velocity_target": session.command.velocity, "yaw_target": session.command.yaw_rate,
            "focused": session.focused, "held_keys": sorted(session.held_keys),
            "paused": session.paused, "automatic": session.automatic, "camera_follow": session.follow,
            "jump_request_count": session.jump_request_count,
        }
        if self.current_stage is not None and self.frames > self.stage_frame:
            stage = self.current_stage
            if not stage.predicate(self.state):
                self.checks[stage.name] = False
                raise AssertionError(f"{stage.name}: {self.state}")
            if time.monotonic() - self.stage_started < stage.hold_seconds:
                return rendered
            self.checks[stage.name] = True
            self.snapshots.append({"label": stage.name, **self.state,
                                   "observed_frames": self.frames - self.stage_frame,
                                   "observed_wall_seconds": time.monotonic() - self.stage_started})
            self.current_stage = None
        if self.current_stage is None:
            try:
                self.current_stage = next(self.plan)
            except StopIteration:
                self.finished = True
                self.inject(self.press(glfw.KEY_ESCAPE))
                return rendered
            self.stage_started, self.stage_frame = time.monotonic(), self.frames
            for event in self.current_stage.events:
                self.inject(event)
        return rendered


def run_smoke() -> dict:
    probe = CallbackProbe()
    errors, automatic, interactive = [], {}, {}
    threads_before = set(threading.enumerate())
    motion = {name: closed_loop_turn(key) for name, key in (("left", glfw.KEY_A), ("right", glfw.KEY_D))}
    try:
        with ExitStack() as stack:
            stack.enter_context(patch.object(glfw, "set_key_callback", probe.register_key))
            stack.enter_context(patch.object(glfw, "set_window_focus_callback", probe.register_focus))
            stack.enter_context(patch.object(glfw, "create_window", probe.create))
            stack.enter_context(patch.object(glfw, "destroy_window", probe.destroy))

            def render(viewer):
                return probe.render(viewer)

            stack.enter_context(patch.object(GlfwViewer, "render", render))
            automatic = run_viewer(scenario="move", automatic=True, duration=3.)
            probe.checks["wall_timeout_exit"] = automatic["reason"] == "duration_elapsed" and automatic["wall_time_s"] < 5.
            probe.checks["timeout_fixed_step"] = automatic["physics_timestep_s"] == .0005
            probe.checks["timeout_render_and_physics"] = automatic["frames"] > 30 and automatic["physics_steps"] > automatic["frames"]
            probe.active = True
            interactive = run_viewer(duration=20.)
            probe.checks["same_window_scene_switches"] = interactive["scene_visits"] == [
                "balance", "step", "bridge", "jump", "balance", "move"] and probe.windows_created == 2
            expected_keys = [{"key": event["key"], "action": event["action"]}
                             for event in probe.injected_events if event["kind"] == "key"]
            probe.checks["all_injected_key_actions_received"] = interactive["received_key_events"] == expected_keys
            probe.checks["genuine_release_and_repeat_actions_exercised"] = (
                any(event["action"] == glfw.RELEASE for event in expected_keys)
                and any(event["action"] == glfw.REPEAT for event in expected_keys))
            probe.checks["escape_exits_window"] = interactive["reason"] == "escape"
            probe.checks["glfw_input_api"] = interactive["input_api"] == "glfw_press_release_focus"
            probe.checks["interactive_fixed_step"] = interactive["physics_timestep_s"] == .0005
            probe.checks["interactive_no_solver_warnings"] = not any(interactive["warning_counts"])
            probe.checks["both_windows_closed"] = automatic["window_closed"] and interactive["window_closed"]
    except Exception as error:
        errors.append(repr(error))
    remaining = set(threading.enumerate()) - threads_before
    probe.checks["all_created_windows_destroyed"] = probe.windows_created == probe.windows_destroyed == 2
    probe.checks["no_new_threads_left"] = not remaining
    probe.checks["probe_completed_without_error"] = probe.finished and not errors
    probe.checks["closed_loop_left_and_right_motion"] = all(case["passed"] for case in motion.values())
    return {
        "schema_version": 2, "test_date": datetime.now().date().isoformat(),
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "mujoco_version": mujoco.__version__, "glfw_python_version": glfw.__version__,
        "python_version": platform.python_version(), "platform": platform.system(),
        "display": os.environ.get("DISPLAY"),
        "method": "Real application-owned GLFW windows and MuJoCo mjv/mjr rendering. PRESS, "
                  "RELEASE, REPEAT and focus events are injected into the registered GLFW "
                  "callbacks from the render thread, then consumed by the production input queue. "
                  "No OS-level input automation and no manual keyboard claim.",
        "scope": "Window rendering/lifecycle, action-state input semantics, no-repeat hold, "
                 "release/focus/stop/pause/reset, one-shot jump and scene changes. Headless motion "
                 "cases separately measure true closed-loop turning and braking; not hardware validation.",
        "source_sha256": hashlib.sha256((ROOT / "src/wheellegged/viewer.py").read_bytes()).hexdigest(),
        "probe_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "automatic_timeout": automatic, "callback_integration": interactive,
        "headless_motion": motion, "injected_events": probe.injected_events,
        "snapshots": probe.snapshots, "checks": probe.checks, "errors": errors,
        "windows_created": probe.windows_created, "windows_destroyed": probe.windows_destroyed,
        "remaining_threads": sorted(thread.name for thread in remaining),
        "passed": bool(probe.checks) and all(probe.checks.values()) and not errors,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/viewer_smoke.json",
                        help="JSON report path (default: repository outputs/viewer_smoke.json)")
    args = parser.parse_args()
    report = run_smoke()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "checks": report["checks"],
                      "errors": report["errors"], "report": str(args.output)}, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
