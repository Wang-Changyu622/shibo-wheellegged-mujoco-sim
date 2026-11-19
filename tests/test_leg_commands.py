"""Independent leg commands, constrained allocation and jump preparation.

Kinematic fixtures below isolate controller contracts without claiming physical
motion. Separate free-base motion trials validate support, flight and recovery.
"""
import mujoco
import numpy as np
import pytest

from wheellegged.control import BalanceController, Command, balanced_data
from wheellegged.jump import JumpSupervisor
from wheellegged.model import HIP_CONTROL_RANGE, load_model


@pytest.fixture(scope="module")
def model():
    return load_model()


@pytest.mark.parametrize("field", ("left_hip_angle", "right_hip_angle"))
@pytest.mark.parametrize("value", (np.nan, np.inf, -np.inf))
def test_independent_targets_reject_nonfinite_values(field, value):
    with pytest.raises(ValueError, match="finite"):
        Command(**{field: value})


@pytest.mark.parametrize("command,expected", [
    (Command(), (-.2, -.2)),
    (Command(hip_angle=-.1, left_hip_angle=-.26), (-.26, -.1)),
    (Command(hip_angle=-.1, right_hip_angle=-.26), (-.1, -.26)),
    (Command(left_hip_angle=-100, right_hip_angle=100), (-.3, 0)),
    (Command(hip_angle=100, right_hip_angle=-.26), (0, -.26)),
])
def test_targets_inherit_and_clip_independently(command, expected):
    assert command.hip_targets == expected


def reference_pair(controller):
    return np.array([controller.hip_reference + controller.hip_differential_reference,
                     controller.hip_reference - controller.hip_differential_reference])


def set_roll(model, data, roll):
    mujoco.mju_axisAngle2Quat(data.joint("floating_base").qpos[3:7],
                             np.array([1., 0., 0.]), roll)
    mujoco.mj_forward(model, data)


@pytest.mark.parametrize("differential", (-.15, .15))
def test_extreme_split_crouch_obeys_each_legs_slew_limit(model, differential):
    controller = BalanceController(model, balanced_data(model))
    controller.hip_reference = -.15
    controller.hip_differential_reference = differential
    command = Command(left_hip_angle=-.15 + differential, right_hip_angle=-.15 - differential)
    # Start an accepted supervisor episode independently of the kinematic fixture.
    controller.jump_supervisor.update(0., False, [1., 1.])
    controller.jump_supervisor.update(.02, True, [1., 1.])
    previous = reference_pair(controller)
    for index in range(1300):
        controller.data.time = .02 + index * controller.dt
        controller.update(command)
        current = reference_pair(controller)
        assert np.max(np.abs(current - previous)) <= .5 * controller.dt + 1e-15
        assert np.all(current >= HIP_CONTROL_RANGE[0] - 1e-15)
        assert np.all(current <= HIP_CONTROL_RANGE[1] + 1e-15)
        assert controller.phase == "crouch"
        previous = current
    assert current == pytest.approx([-.04, -.04], abs=1e-12)


@pytest.mark.parametrize("left,right,sign", [(-.26, -.2, 1), (-.2, -.26, -1)])
def test_left_extension_requests_positive_roll_and_right_negative(model, left, right, sign):
    controller = BalanceController(model, balanced_data(model))
    command = Command(left_hip_angle=left, right_hip_angle=right)
    for _ in range(500):
        controller.update(command)
    assert reference_pair(controller) == pytest.approx([left, right], abs=1e-12)
    assert .05 < sign * controller.roll_reference < .07


@pytest.mark.parametrize("targets,roll", [((-.3, 0.), .6), ((0., -.3), -.6)])
def test_residual_roll_preserves_mean_and_does_not_wind_up_at_limits(model, targets, roll):
    controller = BalanceController(model, balanced_data(model))
    controller.hip_reference = np.mean(targets)
    controller.hip_differential_reference = (targets[0] - targets[1]) / 2
    set_roll(model, controller.data, roll)
    command = Command(left_hip_angle=targets[0], right_hip_angle=targets[1])
    for _ in range(100):
        control = controller.update(command)
    servo_targets = np.array([control[model.actuator(side + "_servo").id] for side in ("L", "R")])
    assert np.mean(servo_targets) == pytest.approx(np.mean(targets), abs=1e-15)
    assert np.all(servo_targets >= -.3) and np.all(servo_targets <= 0.)
    assert controller._roll_integral == 0.


def test_saturated_integral_can_unwind_when_error_changes_sign(model):
    controller = BalanceController(model, balanced_data(model))
    controller.hip_reference, controller.hip_differential_reference = -.15, .15
    controller.roll_reference = controller._geometric_roll_reference()
    controller._roll_integral = .04
    set_roll(model, controller.data, controller.roll_reference - .005)
    controller.update(Command(left_hip_angle=0., right_hip_angle=-.3))
    assert controller._roll_integral < .04


def test_push_aborts_if_crouch_has_not_reached_safe_posture():
    supervisor = JumpSupervisor()
    supervisor.update(0., False, [1., 1.])
    assert supervisor.update(.02, True, [1., 1.]).phase == "crouch"
    assert supervisor.update(.82, False, [1., 1.], push_ready=False).phase == "recover"
    assert supervisor.events[-1].reason == "posture_not_ready_before_push"
    assert supervisor.last_outcome == "aborted"


@pytest.mark.parametrize("command", [Command(jump=True, yaw_rate=.2), Command(jump=True, velocity=.2)])
def test_active_motion_requests_do_not_authorize_standing_jump(model, command):
    controller = BalanceController(model, balanced_data(model))
    controller.update(Command())
    controller.data.time = .02
    controller.update(command)
    assert controller.phase == "balance"
    assert controller.jump_supervisor.events[-1].event == "request_rejected"
    assert controller.jump_supervisor.events[-1].reason == "posture_or_speed"


@pytest.mark.parametrize("velocity_index,value", [(0, .2), (5, .5)])
def test_measured_motion_rejects_jump_even_with_zero_command(model, velocity_index, value):
    controller = BalanceController(model, balanced_data(model))
    controller.data.joint("floating_base").qvel[velocity_index] = value
    mujoco.mj_forward(model, controller.data)
    controller.update(Command())
    controller.data.time = .02
    controller.update(Command(jump=True))
    assert controller.phase == "balance"
    assert controller.jump_supervisor.events[-1].reason == "posture_or_speed"


def test_explicit_symmetric_targets_match_inherited_baseline_bit_for_bit(model):
    first = BalanceController(model, balanced_data(model, pitch_offset=.05))
    second = BalanceController(model, balanced_data(model, pitch_offset=.05))
    for index in range(1200):
        hip = -.2 if index < 400 else (-.3 if index < 800 else 0.)
        velocity = .05 if index < 600 else -.04
        inherited = Command(hip_angle=hip, velocity=velocity)
        explicit = Command(left_hip_angle=hip, right_hip_angle=hip, velocity=velocity)
        controls = (first.update(inherited), second.update(explicit))
        np.testing.assert_array_equal(*controls)
        for controller, control in zip((first, second), controls):
            controller.data.ctrl[:] = control
            mujoco.mj_step(model, controller.data)
            mujoco.mj_forward(model, controller.data)
        np.testing.assert_array_equal(first.data.qpos, second.data.qpos)
