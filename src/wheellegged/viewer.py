"""MuJoCo rendering with real window-local key press/release control.

GLFW owns the interactive window and delivers PRESS/RELEASE/focus events.
No key-repeat timing, global hooks or deadman timeout infer key release.
The passive native_viewer helper remains available for the ROS display.
"""
from __future__ import annotations

from collections import deque
from contextlib import contextmanager
from dataclasses import replace
import math
from queue import Empty, SimpleQueue
import threading
import time

import glfw
import mujoco
import numpy as np

from .control import BalanceController, Command, balanced_data
from .evaluate import SCENARIOS, scenario_command
from .hardware import DEFAULT_PROFILE, actuator_profile
from .model import TERRAINS, ground_forces, load_model
from .state import observe


PHYSICS_TIMESTEP = 0.0005
MANUAL_SPEED = 0.12
MANUAL_YAW_RATE = 0.4
MOVEMENT_KEYS = frozenset((glfw.KEY_W, glfw.KEY_S, glfw.KEY_A, glfw.KEY_D))
DEMO_KEYS = dict(zip("12345", ("balance", "move", "step", "bridge", "jump")))
HELP = ("Hold W/S: forward/back | Hold A/D: turn | Release: stop target | X stop | J jump\n"
        "1 balance | 2 forward/back | 3 step | 4 bridge | 5 jump\n"
        "M auto/manual | R reset | P/Space pause | F follow camera | Esc quit")


@contextmanager
def native_viewer(model: mujoco.MjModel, data: mujoco.MjData, *, key_callback=None):
    """Own a passive window and finish its renderer before interpreter teardown.

    MuJoCo 3.17 Handle.close() only requests an exit. Its public is_running()
    becomes false before the daemon render thread destroys GLFW resources.
    Join threads created during this launch before another window or atexit.
    """
    import mujoco.viewer

    existing_threads = set(threading.enumerate())
    handle = mujoco.viewer.launch_passive(model, data, key_callback=key_callback,
                                         show_left_ui=False, show_right_ui=False)
    window_threads = set(threading.enumerate()) - existing_threads
    try:
        yield handle
    finally:
        handle.close()
        stop_deadline = time.monotonic() + 5.
        for thread in window_threads:
            if thread is not threading.current_thread():
                thread.join(timeout=max(0., stop_deadline - time.monotonic()))
        if any(thread.is_alive() for thread in window_threads):
            raise RuntimeError("MuJoCo viewer threads did not finish within 5 seconds")


class _Session:
    """Single scene and its commands; also usable without a display for checks."""

    def __init__(self, terrain: str, profile: str, scenario: str, automatic: bool):
        self.terrain, self.profile, self.scenario = terrain, profile, scenario
        self.automatic, self.paused, self.follow = automatic, False, True
        self.focused = True
        self.held_keys: set[int] = set()
        self.model = load_model(terrain, PHYSICS_TIMESTEP, profile)
        self.data = mujoco.MjData(self.model)
        self.command = Command()
        self.last_command = Command()
        self.step_count = 0
        self.reset_count = 0
        self.jump_request_count = 0
        self.notice = "Automatic demo" if automatic else "Manual control"
        self.history: deque[tuple[float, float, float, float, float]] = deque(maxlen=360)
        self.reset()

    def reset(self) -> None:
        # Keep the MjData object bound to the viewer; reset controller references too.
        offset = .05 if self.automatic and self.scenario == "balance" else 0.0
        mujoco.mj_copyData(self.data, self.model,
                         balanced_data(self.model, pitch_offset=offset))
        self.controller = BalanceController(self.model, self.data)
        self.command = Command(hip_angle=-.30 if self.scenario == "step" else -.20)
        self.last_command = self.command
        self.held_keys.clear()
        self.jump_request_count = 0
        self.step_count = 0
        self.history.clear()
        self.reset_count += 1
        mujoco.mj_forward(self.model, self.data)

    def _movement_target(self) -> None:
        """Compute both axes from current pressed state; opposite keys cancel."""
        forward = int(glfw.KEY_W in self.held_keys) - int(glfw.KEY_S in self.held_keys)
        turn = int(glfw.KEY_A in self.held_keys) - int(glfw.KEY_D in self.held_keys)
        self.command = replace(self.command, velocity=MANUAL_SPEED * forward,
                               yaw_rate=MANUAL_YAW_RATE * turn)

    def _stop(self) -> None:
        self.held_keys.clear()
        self.command = replace(self.command, velocity=0., yaw_rate=0., jump=False)

    def handle_focus(self, focused: bool) -> None:
        was_focused = self.focused
        self.focused = bool(focused)
        if not focused:
            self.automatic = False
            self._stop()
            self.notice = "Unfocused: motion targets zero"
        elif not was_focused:
            self.notice = "Focused: press WASD to drive"

    def handle_key(self, key: int, action: int) -> str | None:
        """Consume genuine GLFW action events; return a scene name or ``quit``."""
        if action == glfw.RELEASE:
            self.held_keys.discard(key)
            if key in MOVEMENT_KEYS and not self.automatic:
                self._movement_target()
            return None
        if action != glfw.PRESS or not self.focused or key in self.held_keys:
            return None  # REPEAT never maintains movement or retriggers actions.
        self.held_keys.add(key)
        if key == glfw.KEY_ESCAPE:
            return "quit"
        char = chr(key).upper() if 0 <= key < 128 else ""
        if char in DEMO_KEYS:
            return DEMO_KEYS[char]
        if char in ("P", " "):
            self.paused = not self.paused
            self._stop()
            self.notice = "Paused" if self.paused else "Running"
        elif char == "F":
            self.follow = not self.follow
            self.notice = "Camera follows chassis" if self.follow else "Free camera: drag/scroll"
        elif char == "R":
            self.reset()
            self.notice = "Reset pose and controller"
        elif char == "M":
            self.automatic = not self.automatic
            self._stop()
            if self.automatic:
                self.reset()
                self.notice = "Automatic demo restarted"
            else:
                self.notice = "Manual control; speed and yaw targets zeroed"
        elif char in ("W", "S", "A", "D", "X", "J"):
            if self.automatic:
                self.command = replace(self.last_command, jump=False)
                self.automatic = False
            self._movement_target()
            if char == "X":
                self._stop()
            elif char == "J":
                self.command = replace(self.command, jump=True)
            self.notice = ("Jump requested; readiness guards apply" if char == "J"
                           else "Hold WASD; release to stop target")
        return None

    def step(self) -> None:
        command = (scenario_command(self.scenario, self.step_count * PHYSICS_TIMESTEP,
                                    PHYSICS_TIMESTEP)
                   if self.automatic else self.command)
        self.last_command = command
        self.jump_request_count += int(command.jump)
        self.data.ctrl[:] = self.controller.update(command)
        self.command = replace(self.command, jump=False)  # exactly one control-step pulse
        previous_time = float(self.data.time)
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.step_count += 1
        if (not np.all(np.isfinite(self.data.qpos))
                or not np.all(np.isfinite(self.data.qvel)) or self.data.time <= previous_time):
            self.paused = True
            self.notice = "Non-finite/reset physics state: paused; R resets"


class GlfwViewer:
    """One application-owned window, public MuJoCo rendering, local input only.

    All GLFW/OpenGL work runs on the calling thread. Callbacks enqueue events;
    they do not mutate physics. Scene switches reuse the window and its input
    callbacks, while rebuilding the model-specific rendering resources.
    """

    def __init__(self, session: _Session, event_callback):
        self.session = session
        self.event_callback = event_callback
        self.window = self.context = self.scene = None
        self._glfw_initialized = False
        self._texts, self._figures = [], []
        self._mouse_buttons: set[int] = set()
        self._cursor = (0., 0.)
        self.cam, self.opt = mujoco.MjvCamera(), mujoco.MjvOption()
        mujoco.mjv_defaultOption(self.opt)
        if not glfw.init():
            raise RuntimeError("GLFW could not initialize; run the viewer in a graphical desktop")
        self._glfw_initialized = True
        try:
            glfw.default_window_hints()
            glfw.window_hint(glfw.SAMPLES, 4)
            self.window = glfw.create_window(1600, 960, "Wheel-legged robot | hold WASD to move", None, None)
            if not self.window:
                raise RuntimeError("GLFW could not create the MuJoCo window")
            glfw.make_context_current(self.window)
            glfw.swap_interval(0)  # run_viewer owns the rendering cadence
            self.set_session(session)
            glfw.set_key_callback(self.window, self._key_callback)
            glfw.set_window_focus_callback(self.window, self._focus_callback)
            glfw.set_mouse_button_callback(self.window, self._mouse_button)
            glfw.set_cursor_pos_callback(self.window, self._cursor_position)
            glfw.set_scroll_callback(self.window, self._scroll)
            self._cursor = glfw.get_cursor_pos(self.window)
            self.session.handle_focus(bool(glfw.get_window_attrib(self.window, glfw.FOCUSED)))
        except BaseException:
            self.close()
            raise

    def set_session(self, session: _Session) -> None:
        glfw.make_context_current(self.window)
        if self.context is not None:
            self.context.free()
            self.context = None
        self.session = session
        self.scene = mujoco.MjvScene(session.model, maxgeom=10000)
        self.context = mujoco.MjrContext(session.model, mujoco.mjtFontScale.mjFONTSCALE_150)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.cam.distance, self.cam.azimuth, self.cam.elevation = .46, 125, -24
        self.cam.lookat[:] = session.data.body("chassis").xpos
        self._texts, self._figures = [], []
        self._mouse_buttons.clear()

    def _key_callback(self, window, key, scancode, action, mods) -> None:
        self.event_callback(("key", int(key), int(action)))

    def _focus_callback(self, window, focused) -> None:
        if not focused:
            self._mouse_buttons.clear()
        self.event_callback(("focus", bool(focused)))

    def _mouse_button(self, window, button, action, mods) -> None:
        if action == glfw.PRESS:
            self._mouse_buttons.add(button)
            if button == glfw.MOUSE_BUTTON_RIGHT:
                self.session.follow = False
                self.session.notice = "Free camera: drag/scroll; F follows the chassis"
        elif action == glfw.RELEASE:
            self._mouse_buttons.discard(button)
        self._cursor = glfw.get_cursor_pos(window)

    def _cursor_position(self, window, x, y) -> None:
        dx, dy = x - self._cursor[0], y - self._cursor[1]
        self._cursor = (x, y)
        if not self._mouse_buttons or not self.session.focused:
            return
        height = max(glfw.get_window_size(window)[1], 1)
        shift = (glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                 or glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS)
        if glfw.MOUSE_BUTTON_RIGHT in self._mouse_buttons:
            action = mujoco.mjtMouse.mjMOUSE_MOVE_H if shift else mujoco.mjtMouse.mjMOUSE_MOVE_V
        elif glfw.MOUSE_BUTTON_LEFT in self._mouse_buttons:
            action = mujoco.mjtMouse.mjMOUSE_ROTATE_H if shift else mujoco.mjtMouse.mjMOUSE_ROTATE_V
        else:
            action = mujoco.mjtMouse.mjMOUSE_ZOOM
        mujoco.mjv_moveCamera(self.session.model, action, dx / height, dy / height, self.cam)

    def _scroll(self, window, x_offset, y_offset) -> None:
        mujoco.mjv_moveCamera(self.session.model, mujoco.mjtMouse.mjMOUSE_ZOOM,
                              0., -.05 * y_offset, self.cam)

    @property
    def viewport(self):
        width, height = glfw.get_framebuffer_size(self.window) if self.window else (0, 0)
        return mujoco.MjrRect(0, 0, width, height)

    def set_texts(self, texts) -> None:
        self._texts = texts

    def set_figures(self, figures) -> None:
        self._figures = figures

    def clear_figures(self) -> None:
        self._figures = []

    def is_running(self) -> bool:
        return bool(self.window and not glfw.window_should_close(self.window))

    def render(self) -> bool:
        viewport = self.viewport
        if viewport.width <= 0 or viewport.height <= 0:
            return False
        mujoco.mjv_updateScene(self.session.model, self.session.data, self.opt, None,
                               self.cam, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        mujoco.mjr_render(viewport, self.scene, self.context)
        for font, position, labels, values in self._texts:
            mujoco.mjr_overlay(font, position, viewport, labels, values, self.context)
        for rectangle, figure in self._figures:
            mujoco.mjr_figure(rectangle, figure, self.context)
        glfw.swap_buffers(self.window)
        return True

    def close(self) -> None:
        if self.window:
            glfw.make_context_current(self.window)
            if self.context is not None:
                self.context.free()
                self.context = None
            self.scene = None
            glfw.destroy_window(self.window)
            self.window = None
        if self._glfw_initialized:
            glfw.terminate()
            self._glfw_initialized = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


def _figure(title: str, ylabel: str, names: tuple[str, str]) -> mujoco.MjvFigure:
    figure = mujoco.MjvFigure()
    mujoco.mjv_defaultFigure(figure)
    figure.title, figure.xlabel = title, "simulation time (s)"
    figure.yformat = "%.2f"
    figure.linename[0], figure.linename[1] = names
    figure.linergb[0], figure.linergb[1] = (.28, .8, 1.), (1., .66, .25)
    figure.figurergba[:] = (.055, .075, .10, .9)
    figure.panergba[:] = (.08, .10, .14, 1.)
    figure.gridrgb[:] = (.25, .28, .32)
    figure.flg_legend = True
    figure.flg_extend = False
    # Units in the title avoid requiring an unsupported vertical label property.
    figure.title = f"{title} ({ylabel})"
    return figure


def _overlays(viewer, session: _Session, figures: tuple, realtime_ratio: float) -> None:
    observation = observe(session.model, session.data)
    command = session.last_command if session.automatic else session.command
    data = session.data
    force = ground_forces(data)
    status = "PAUSED" if session.paused else "RUNNING"
    mode = "AUTO" if session.automatic else "MANUAL"
    complete = session.automatic and data.time >= SCENARIOS[session.scenario]
    labels = ("Wheel-legged robot\nProfile\nMode / scene\nSim time / real-time\n"
              "Speed target / actual\nYaw target / actual\nPitch / roll\n"
              "Wheel contact L / R\nWheel Uq L / R\nWheel torque L / R\n"
              "Hip target L / R\nHip torque L / R\nJump phase / outcome\nCamera\nKeys / focus\nStatus")
    pair = lambda suffix, field: [float(getattr(data.actuator(f"{side}_{suffix}"), field)[0])
                                 for side in ("L", "R")]
    values = (f"{status} | 0.5 ms physics\n{session.profile}\n"
              f"{mode} / {session.scenario}\n{data.time:.2f} s / {realtime_ratio:.2f}x\n"
              f"{command.velocity:+.3f} / {observation.speed:+.3f} m/s\n"
              f"{command.yaw_rate:+.2f} / {observation.yaw_rate:+.2f} rad/s\n"
              f"{np.degrees(observation.pitch):+.2f} / {np.degrees(observation.roll):+.2f} deg\n"
              f"{force[0]:.3f} / {force[1]:.3f} N\n"
              + "{:+.3f} / {:+.3f} V\n".format(*pair("wheel_voltage", "ctrl"))
              + "{:+.4f} / {:+.4f} Nm\n".format(*pair("wheel_voltage", "force"))
              + "{:+.3f} / {:+.3f} rad\n".format(*pair("servo", "ctrl"))
              + "{:+.4f} / {:+.4f} Nm\n".format(*pair("servo", "force"))
              + f"{session.controller.phase} / {session.controller.jump_supervisor.last_outcome}\n"
              + ("FOLLOW\n" if session.follow else "FREE\n")
              + f"{''.join(chr(k) for k in sorted(session.held_keys & MOVEMENT_KEYS)) or '-'} / "
              + ("FOCUSED\n" if session.focused else "UNFOCUSED\n")
              + ("Demo complete; holding. R replays." if complete else session.notice))
    viewer.set_texts([
        (mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, labels, values),
        (mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
         HELP + "\nLive simulation preview; use 'run' for measured acceptance.", ""),
    ])
    if not session.history or data.time - session.history[-1][0] >= 1. / 30. - 1e-10:
        session.history.append((float(data.time), command.velocity, observation.speed,
                                np.degrees(observation.pitch), np.degrees(observation.roll)))
    samples = np.asarray(session.history)
    for figure, columns in zip(figures, ((1, 2), (3, 4))):
        for line, column in enumerate(columns):
            figure.linepnt[line] = len(samples)
            figure.linedata[line, :2 * len(samples)] = samples[:, (0, column)].reshape(-1)
        figure.range[0] = (max(0., data.time - 12.), max(12., data.time))
        minimum_span = .2 if columns == (1, 2) else 5.
        extent = max(minimum_span, float(np.max(np.abs(samples[:, columns]))) * 1.2)
        figure.range[1] = (-extent, extent)
    viewport = viewer.viewport
    if viewport is not None and viewport.width >= 1050 and viewport.height >= 650:
        width, height = min(430, viewport.width // 3), min(250, viewport.height // 3)
        left = viewport.left + viewport.width - width - 12
        viewer.set_figures([
            (mujoco.MjrRect(left, viewport.bottom + height + 24, width, height), figures[0]),
            (mujoco.MjrRect(left, viewport.bottom + 12, width, height), figures[1]),
        ])
    else:
        viewer.clear_figures()


def run_viewer(terrain: str = "flat", profile: str = DEFAULT_PROFILE, *,
               scenario: str | None = None, automatic: bool = False,
               duration: float | None = None, render_hz: float = 60.) -> dict:
    """Run the GLFW/MuJoCo view; duration bounds wall time including pauses.

    A supplied scenario selects its matching terrain. Keys 1--5 restart demos
    inside the same window. Key releases zero the corresponding target before
    the next physics batch; controller acceleration limits still apply.
    """
    if terrain not in TERRAINS:
        raise ValueError(f"Unknown terrain {terrain!r}; choose {tuple(TERRAINS)}")
    if scenario is not None and scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario {scenario!r}; choose {tuple(SCENARIOS)}")
    if duration is not None and (not math.isfinite(duration) or duration <= 0):
        raise ValueError("duration must be finite and positive, or None")
    if not math.isfinite(render_hz) or not 1. <= render_hz <= 240.:
        raise ValueError("render_hz must be finite and between 1 and 240")
    actuator_profile(profile)
    if scenario is None:
        scenario = terrain if terrain in ("step", "bridge") else "balance"
    else:
        terrain = scenario if scenario in ("step", "bridge") else "flat"
    events: SimpleQueue[tuple] = SimpleQueue()
    started = time.monotonic()
    deadline = math.inf if duration is None else started + duration
    frames, physics_steps = 0, 0
    seen_keys, key_events, focus_events = [], [], []
    scene_visits = [scenario]
    end_reason = "window_closed"
    session = _Session(terrain, profile, scenario, automatic)
    warning_counts = np.zeros(len(session.data.warning.number), dtype=int)
    figures = (_figure("Speed", "m/s", ("target", "actual")),
               _figure("Attitude", "deg", ("pitch", "roll")))
    print(HELP, flush=True)
    with GlfwViewer(session, events.put) as viewer:
        previous_wall = frame_deadline = time.monotonic()
        accumulator, ratio = 0., 0.
        quit_requested = False
        while viewer.is_running() and time.monotonic() < deadline:
            frame_start = time.monotonic()
            elapsed = frame_start - previous_wall
            previous_wall = frame_start
            count_before = physics_steps
            glfw.poll_events()
            while True:
                try:
                    event = events.get_nowait()
                except Empty:
                    break
                if event[0] == "focus":
                    focus_events.append(event[1])
                    session.handle_focus(event[1])
                    continue
                _, key, action = event
                key_events.append({"key": key, "action": action})
                if action == glfw.PRESS:
                    seen_keys.append(key)
                next_scene = session.handle_key(key, action)
                if next_scene == "quit":
                    end_reason, quit_requested = "escape", True
                    break
                if next_scene is not None:
                    terrain = next_scene if next_scene in ("step", "bridge") else "flat"
                    session = _Session(terrain, profile, next_scene, automatic=True)
                    viewer.set_session(session)
                    scene_visits.append(next_scene)
                    accumulator = 0.
            if quit_requested:
                break
            if session.paused:
                accumulator = 0.
            else:
                # Bound wall-clock catch-up, never skip a physics state or infer
                # key release from elapsed time. Events are handled above first.
                accumulator = min(accumulator + elapsed, .05)
                while accumulator >= PHYSICS_TIMESTEP and not session.paused:
                    session.step()
                    physics_steps += 1
                    accumulator -= PHYSICS_TIMESTEP
            warning_counts = np.maximum(warning_counts, session.data.warning.number)
            if session.follow:
                viewer.cam.lookat[:] = session.data.body("chassis").xpos
            if elapsed > 0 and not session.paused:
                current_ratio = (physics_steps - count_before) * PHYSICS_TIMESTEP / elapsed
                ratio = .9 * ratio + .1 * current_ratio
            elif session.paused:
                ratio = 0.
            _overlays(viewer, session, figures, ratio)
            frames += int(viewer.render())
            frame_deadline = max(frame_deadline + 1. / render_hz, frame_start)
            remaining = min(frame_deadline, deadline) - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
    if time.monotonic() >= deadline:
        end_reason = "duration_elapsed"
    final_command = session.last_command if session.automatic else session.command
    return {
        "reason": end_reason, "wall_time_s": time.monotonic() - started,
        "frames": frames, "physics_steps": physics_steps,
        "physics_timestep_s": PHYSICS_TIMESTEP, "render_hz_target": render_hz,
        "input_api": "glfw_press_release_focus", "window_closed": viewer.window is None,
        "profile": profile, "scenario": session.scenario, "terrain": session.terrain,
        "automatic": session.automatic, "paused": session.paused, "focused": session.focused,
        "held_keys": sorted(session.held_keys), "camera_follow": session.follow,
        "scene_visits": scene_visits, "received_keys": seen_keys,
        "received_key_events": key_events, "received_focus_events": focus_events,
        "final_simulation_time_s": float(session.data.time),
        "final_command": {"velocity": final_command.velocity, "yaw_rate": final_command.yaw_rate},
        "jump_phase": session.controller.phase, "jump_request_count": session.jump_request_count,
        "warning_counts": warning_counts.tolist(),
    }
