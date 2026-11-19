"""Controller boundary contracts prevent inconsistent saturated commands."""
import numpy as np
import pytest

from wheellegged.control import BalanceController, Command, balanced_data
from wheellegged.model import load_model


@pytest.mark.parametrize("field", ("velocity", "yaw_rate", "hip_angle"))
@pytest.mark.parametrize("value", (float("nan"), float("inf"), -float("inf")))
def test_command_rejects_nonfinite_fields(field, value):
    with pytest.raises(ValueError, match="finite"):
        Command(**{field: value})


def test_large_yaw_request_matches_documented_bounded_command():
    model = load_model()
    first = BalanceController(model, balanced_data(model))
    second = BalanceController(model, balanced_data(model))
    assert first.update(Command(yaw_rate=100.0)) == pytest.approx(second.update(Command(yaw_rate=1.0)))
    assert first.yaw_reference == pytest.approx(second.yaw_reference)


def test_velocity_reference_remains_inside_baseline_envelope():
    model = load_model()
    controller = BalanceController(model, balanced_data(model))
    for _ in range(3000):
        controller.update(Command(velocity=100.0))
    assert controller.velocity_reference == pytest.approx(.3)


@pytest.mark.parametrize("hip", (-.3, 0.0))
def test_roll_differential_preserves_mean_leg_target_at_joint_limits(hip):
    model = load_model()
    data = balanced_data(model, hip=hip)
    # Inject a measured roll offset without advancing dynamics: this isolates the
    # allocator boundary from the robot's ability to recover the perturbation.
    quat = data.joint("floating_base").qpos[3:7].copy()
    roll = np.array([np.cos(.05 / 2), np.sin(.05 / 2), 0.0, 0.0])
    import mujoco
    mujoco.mju_mulQuat(data.joint("floating_base").qpos[3:7], roll, quat)
    mujoco.mj_forward(model, data)
    controller = BalanceController(model, data)
    assert controller.hip_reference == pytest.approx(hip)
    controls = controller.update(Command(hip_angle=hip))
    targets = [controls[model.actuator(side + "_servo").id] for side in ("L", "R")]
    assert targets == pytest.approx([hip, hip])
