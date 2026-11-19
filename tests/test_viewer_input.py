"""Game-style movement uses actual press/release/focus state, without a window."""
import glfw
import numpy as np
import pytest

from wheellegged.viewer import MANUAL_SPEED, MANUAL_YAW_RATE, _Session


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
