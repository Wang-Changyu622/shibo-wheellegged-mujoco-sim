"""Physical acceptance tests for free-base control, not animation state labels."""
import numpy as np
import pytest

from wheellegged.evaluate import assess_trace, continuous_duration, run_scenario


@pytest.fixture(scope="module")
def rollouts():
    # Cache full physical trials so metric assertions do not rerun the simulator.
    return {name: run_scenario(name) for name in ("balance", "move", "step", "bridge", "jump")}


@pytest.mark.parametrize("name", ("balance", "move", "step", "bridge", "jump"))
def test_nominal_tasks_satisfy_physical_acceptance(rollouts, name):
    result = rollouts[name]
    assert result["passed"], {key: result["metrics"] for key in result["failed_checks"]}
    assert len(result["trace"]["qpos"]) == len(result["trace"]["time"])
    assert result["trace"]["wheel_xyz"].shape[1:] == (2, 3)


def test_balance_recovers_opposite_pitch_perturbation():
    result = run_scenario("balance", pitch_offset=-.05)
    assert result["passed"], result["failed_checks"]


def test_jump_survives_halved_integration_step(rollouts):
    coarse = rollouts["jump"]
    fine = run_scenario("jump", timestep=.00025)
    assert fine["passed"], fine["failed_checks"]
    # These are physical tolerances: 5 ms flight, 1 mm clearance, 5 mm final pose.
    assert abs(coarse["metrics"]["airborne_duration_s"] - fine["metrics"]["airborne_duration_s"]) < .005
    assert abs(coarse["metrics"]["both_wheel_clearance_peak_m"] - fine["metrics"]["both_wheel_clearance_peak_m"]) < .001
    assert np.linalg.norm(coarse["trace"]["axle_xyz"][-1] - fine["trace"]["axle_xyz"][-1]) < .005


def test_jump_cannot_pass_by_claimed_phase_or_contact_dropout(rollouts):
    baseline = rollouts["balance"]
    fake = {key: value.copy() for key, value in baseline["trace"].items()}
    fake["phase"][:] = "flight"
    fake["ground_force"][:] = 0.0
    _, checks = assess_trace("jump", fake, baseline["model"],
                             baseline["metadata"]["equilibrium_pitch_rad"], 8.0)
    assert not checks["sustained_physical_takeoff"]  # no geometric clearance
    fake["ground_force"][:] = baseline["trace"]["ground_force"]
    fake["wheel_clearance"][:] = .01
    _, checks = assess_trace("jump", fake, baseline["model"],
                             baseline["metadata"]["equilibrium_pitch_rad"], 8.0)
    assert not checks["sustained_physical_takeoff"]  # still physically loaded


def test_one_frame_or_disjoint_airborne_samples_are_not_a_jump():
    t = np.arange(6) * .001
    assert continuous_duration(np.array([0, 1, 0, 1, 0, 1], dtype=bool), t) == 0.0
    assert continuous_duration(np.array([0, 1, 1, 1, 0, 0], dtype=bool), t) == pytest.approx(.002)


def test_short_preview_is_not_reported_as_full_task(tmp_path):
    result = run_scenario("balance", duration=.02, output_dir=tmp_path)
    assert not result["passed"]
    assert not result["checks"]["full_task_duration"]
    with np.load(tmp_path / "trace.npz", allow_pickle=False) as trace:
        assert np.array_equal(trace["qpos"], result["trace"]["qpos"])
    assert (tmp_path / "metrics.json").exists()
    with pytest.raises(FileExistsError):
        run_scenario("balance", duration=.02, output_dir=tmp_path)
