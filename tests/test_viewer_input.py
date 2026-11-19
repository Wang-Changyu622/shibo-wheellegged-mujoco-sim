"""Game-style movement uses actual press/release/focus state, without a window."""
import glfw
from dataclasses import replace
import numpy as np
import pytest

from wheellegged.viewer import (HEIGHT_KEYS, MANUAL_HIP_RATE, MANUAL_SPEED,
                               MANUAL_YAW_RATE, PHYSICS_TIMESTEP, _Session)


@pytest.fixture
def session():
    return _Session("flat", "hardware_2s", "balance", False)


@pytest.mark.parametrize("key,target", [
    (glfw.KEY_W, (MANUAL_SPEED, 0.)),
    (glfw.KEY_S, (-MANUAL_SPEED, 0.)),
    (glfw.KEY_A, (0., MANUAL_YAW_RATE)),
    (glfw.KEY_D, (0., -MANUAL_YAW_RATE)),
])
def test_press_sets_axis_and_release_zeros_without_changing_physics(session, key, target):
    session.handle_key(key, glfw.PRESS)
    assert (session.command.velocity, session.command.yaw_rate) == pytest.approx(target)
    for _ in range(40):
        session.step()
    position, velocity = session.data.qpos.copy(), session.data.qvel.copy()
    now = session.data.time
    session.handle_key(key, glfw.RELEASE)
    assert (session.command.velocity, session.command.yaw_rate) == (0., 0.)
    assert np.array_equal(position, session.data.qpos)
    assert np.array_equal(velocity, session.data.qvel)
    assert now == session.data.time


def test_combined_axes_and_opposite_keys_are_independent(session):
    for key in (glfw.KEY_W, glfw.KEY_A):
        session.handle_key(key, glfw.PRESS)
    assert (session.command.velocity, session.command.yaw_rate) == (MANUAL_SPEED, MANUAL_YAW_RATE)
    session.handle_key(glfw.KEY_W, glfw.RELEASE)
    assert (session.command.velocity, session.command.yaw_rate) == (0., MANUAL_YAW_RATE)
    for key in (glfw.KEY_S, glfw.KEY_D):
        session.handle_key(key, glfw.PRESS)
    assert (session.command.velocity, session.command.yaw_rate) == (-MANUAL_SPEED, 0.)
    session.handle_key(glfw.KEY_W, glfw.PRESS)
    assert (session.command.velocity, session.command.yaw_rate) == (0., 0.)
    session.handle_key(glfw.KEY_S, glfw.RELEASE)
    session.handle_key(glfw.KEY_A, glfw.RELEASE)
    assert (session.command.velocity, session.command.yaw_rate) == (MANUAL_SPEED, -MANUAL_YAW_RATE)


def test_repeat_neither_drives_an_unpressed_key_nor_resurrects_after_stop(session):
    session.handle_key(glfw.KEY_W, glfw.REPEAT)
    assert session.command.velocity == 0.
    session.handle_key(glfw.KEY_W, glfw.PRESS)
    for _ in range(400):
        session.step()  # no repeats are needed to maintain a held target
    assert session.command.velocity == MANUAL_SPEED
    session.handle_key(glfw.KEY_X, glfw.PRESS)
    session.handle_key(glfw.KEY_W, glfw.REPEAT)
    assert session.command.velocity == 0. and not session.held_keys
    session.handle_key(glfw.KEY_W, glfw.RELEASE)
    session.handle_key(glfw.KEY_W, glfw.PRESS)
    assert session.command.velocity == MANUAL_SPEED


def test_focus_loss_clears_axes_and_refocus_requires_fresh_press(session):
    session.handle_key(glfw.KEY_W, glfw.PRESS)
    session.handle_key(glfw.KEY_D, glfw.PRESS)
    session.handle_focus(False)
    assert not session.focused and not session.held_keys and not session.automatic
    assert (session.command.velocity, session.command.yaw_rate) == (0., 0.)
    session.handle_key(glfw.KEY_W, glfw.PRESS)
    session.handle_key(glfw.KEY_D, glfw.REPEAT)
    assert not session.held_keys and session.command.velocity == 0.
    session.handle_focus(True)
    assert session.command.velocity == 0.
    session.handle_key(glfw.KEY_W, glfw.PRESS)
    assert session.command.velocity == MANUAL_SPEED


def test_release_still_works_while_paused_and_repeat_does_not_resume(session):
    session.handle_key(glfw.KEY_W, glfw.PRESS)
    session.handle_key(glfw.KEY_P, glfw.PRESS)
    assert session.paused
    session.handle_key(glfw.KEY_W, glfw.RELEASE)
    session.handle_key(glfw.KEY_P, glfw.REPEAT)
    assert session.paused and session.command.velocity == 0.
    session.handle_key(glfw.KEY_P, glfw.RELEASE)
    session.handle_key(glfw.KEY_P, glfw.PRESS)
    assert not session.paused and session.command.velocity == 0.


def test_jump_is_one_control_step_per_press_edge(session):
    session.handle_key(glfw.KEY_J, glfw.PRESS)
    session.step()
    assert session.jump_request_count == 1 and not session.command.jump
    for _ in range(4):
        session.handle_key(glfw.KEY_J, glfw.REPEAT)
        session.step()
    assert session.jump_request_count == 1
    session.handle_key(glfw.KEY_J, glfw.RELEASE)
    session.handle_key(glfw.KEY_J, glfw.PRESS)
    session.step()
    assert session.jump_request_count == 2


def test_scene_shortcuts_require_press_and_reset_clears_pressed_state(session):
    assert session.handle_key(glfw.KEY_3, glfw.RELEASE) is None
    assert session.handle_key(glfw.KEY_3, glfw.REPEAT) is None
    assert session.handle_key(glfw.KEY_3, glfw.PRESS) == "step"
    session.handle_key(glfw.KEY_W, glfw.PRESS)
    session.handle_key(glfw.KEY_R, glfw.PRESS)
    assert not session.held_keys and session.data.time == 0.
    assert (session.command.velocity, session.command.yaw_rate) == (0., 0.)


@pytest.mark.parametrize("key,direction", [
    (glfw.KEY_Q, (-1., 0.)), (glfw.KEY_Z, (1., 0.)),
    (glfw.KEY_E, (0., -1.)), (glfw.KEY_C, (0., 1.)),
    (glfw.KEY_T, (-1., -1.)), (glfw.KEY_G, (1., 1.)),
])
def test_height_keys_integrate_at_fixed_rate_then_hold_without_teleport(session, key, direction):
    qpos, qvel, now = session.data.qpos.copy(), session.data.qvel.copy(), session.data.time
    session.handle_key(key, glfw.PRESS)
    assert np.array_equal(qpos, session.data.qpos) and np.array_equal(qvel, session.data.qvel)
    assert now == session.data.time
    for _ in range(40):
        session.step()
    expected = -.2 + np.asarray(direction) * MANUAL_HIP_RATE * PHYSICS_TIMESTEP * 40
    assert session.command.hip_targets == pytest.approx(expected)
    qpos, qvel = session.data.qpos.copy(), session.data.qvel.copy()
    session.handle_key(key, glfw.RELEASE)
    assert np.array_equal(qpos, session.data.qpos) and np.array_equal(qvel, session.data.qvel)
    for _ in range(40):
        session.step()
    assert session.command.hip_targets == pytest.approx(expected)


def test_height_opposites_cancel_and_shared_keys_do_not_double_rate(session):
    for key in (glfw.KEY_Q, glfw.KEY_Z, glfw.KEY_E, glfw.KEY_C):
        session.handle_key(key, glfw.PRESS)
    for _ in range(20):
        session.step()
    assert session.command.hip_targets == (-.2, -.2)
    session.handle_key(glfw.KEY_Z, glfw.RELEASE)
    session.handle_key(glfw.KEY_C, glfw.RELEASE)
    session.handle_key(glfw.KEY_T, glfw.PRESS)
    for _ in range(20):
        session.step()
    assert session.command.hip_targets == pytest.approx((-.201, -.201))


def test_height_targets_bound_and_nominal_key_changes_no_physical_state(session):
    session.command = replace(session.command, left_hip_angle=-.2999, right_hip_angle=-.0001)
    session.handle_key(glfw.KEY_Q, glfw.PRESS)
    session.handle_key(glfw.KEY_C, glfw.PRESS)
    for _ in range(10):
        session.step()
    assert session.command.hip_targets == (-.3, 0.)
    qpos, qvel, now = session.data.qpos.copy(), session.data.qvel.copy(), session.data.time
    session.handle_key(glfw.KEY_H, glfw.PRESS)
    assert session.command.hip_targets == (-.2, -.2)
    assert not (session.held_keys & HEIGHT_KEYS)
    assert np.array_equal(qpos, session.data.qpos) and np.array_equal(qvel, session.data.qvel)
    assert now == session.data.time


def test_height_focus_and_pause_preserve_request_but_clear_adjustment(session):
    session.handle_key(glfw.KEY_Q, glfw.PRESS)
    for _ in range(40):
        session.step()
    requested = session.command.hip_targets
    session.handle_focus(False)
    session.handle_key(glfw.KEY_E, glfw.PRESS)
    for _ in range(20):
        session.step()
    assert session.command.hip_targets == requested and not session.held_keys
    session.handle_focus(True)
    session.handle_key(glfw.KEY_P, glfw.PRESS)
    session.handle_key(glfw.KEY_Q, glfw.PRESS)
    session.handle_key(glfw.KEY_H, glfw.PRESS)
    assert not (session.held_keys & HEIGHT_KEYS)
    assert session.command.hip_targets == requested
    session.handle_key(glfw.KEY_P, glfw.RELEASE)
    session.handle_key(glfw.KEY_P, glfw.PRESS)
    for _ in range(20):
        session.step()
    assert session.command.hip_targets == requested


def test_jump_stops_motion_and_freezes_requested_heights_without_latent_keys(session):
    for _ in range(200):
        session.step()
    session.handle_key(glfw.KEY_Q, glfw.PRESS)
    session.handle_key(glfw.KEY_W, glfw.PRESS)
    for _ in range(20):
        session.step()
    requested = session.command.hip_targets
    session.handle_key(glfw.KEY_J, glfw.PRESS)
    assert (session.command.velocity, session.command.yaw_rate) == (0., 0.)
    assert session.held_keys == {glfw.KEY_J}
    session.handle_key(glfw.KEY_E, glfw.PRESS)  # ignored while request is pending
    session.handle_key(glfw.KEY_H, glfw.PRESS)
    session.step()
    assert session.jump_feedback == "ACCEPTED" and session.controller.phase == "crouch"
    session.handle_key(glfw.KEY_Q, glfw.PRESS)  # ignored during jump, never queued
    session.handle_key(glfw.KEY_H, glfw.PRESS)
    session.handle_key(glfw.KEY_J, glfw.REPEAT)
    for _ in range(100):
        session.step()
    assert session.command.hip_targets == requested
    assert not (session.held_keys & HEIGHT_KEYS)
    assert session.jump_request_count == 1


def test_rejected_jump_reports_reason_without_delayed_retry(session):
    session.handle_key(glfw.KEY_J, glfw.PRESS)  # no support dwell yet
    session.step()
    assert session.jump_feedback == "REJECTED: unsupported"
    for _ in range(100):
        session.step()
    assert session.jump_request_count == 1 and session.controller.phase == "balance"
