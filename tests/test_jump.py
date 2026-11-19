"""Jump supervision must follow sustained contact, not command timestamps."""
import pytest

from wheellegged.jump import JumpSupervisor


def ready_supervisor():
    supervisor = JumpSupervisor()
    supervisor.update(0.0, False, [1.0, 1.0])
    assert supervisor.update(.020, True, [1.0, 1.0]).phase == "crouch"
    assert supervisor.update(.820, False, [1.0, 1.0]).phase == "push"
    assert supervisor.update(.900, False, [1.0, 1.0]).phase == "tuck"
    return supervisor


def test_takeoff_requires_continuous_two_wheel_unloading():
    supervisor = ready_supervisor()
    assert supervisor.update(.901, False, [0.0, 0.0]).phase == "tuck"
    assert supervisor.update(.902, False, [1.0, 0.0]).phase == "tuck"
    assert supervisor.update(.910, False, [0.0, 0.0]).phase == "tuck"
    assert supervisor.update(.917, False, [0.0, 0.0]).phase == "tuck"
    assert supervisor.update(.918, False, [0.0, 0.0]).phase == "flight"
    events = [event for event in supervisor.events if event.event == "takeoff_confirmed"]
    assert len(events) == 1 and events[0].time == pytest.approx(.918)


def test_confirmed_early_takeoff_preserves_minimum_push_duration():
    supervisor = JumpSupervisor()
    supervisor.update(0.0, False, [1.0, 1.0])
    supervisor.update(.020, True, [1.0, 1.0])
    supervisor.update(.820, False, [1.0, 1.0])
    supervisor.update(.830, False, [0.0, 0.0])
    assert supervisor.update(.838, False, [0.0, 0.0]).phase == "push"
    assert supervisor.update(.899, False, [0.0, 0.0]).hip_target == -.3
    assert supervisor.update(.900, False, [0.0, 0.0]).phase == "flight"


def test_landing_requires_continuous_two_wheel_support_then_smooth_recovery():
    supervisor = ready_supervisor()
    supervisor.update(.901, False, [0.0, 0.0])
    supervisor.update(.909, False, [0.0, 0.0])
    assert supervisor.update(.940, False, [1.0, 1.0]).phase == "flight"
    assert supervisor.update(.945, False, [1.0, 0.0]).phase == "flight"
    assert supervisor.update(.950, False, [1.0, 1.0]).phase == "flight"
    assert supervisor.update(.959, False, [1.0, 1.0]).phase == "flight"
    landed = supervisor.update(.960, False, [1.0, 1.0])
    assert landed.phase == "land" and landed.hip_target == -.2 and not landed.direct_target
    assert supervisor.update(1.459, False, [1.0, 1.0]).phase == "land"
    assert supervisor.update(1.460, False, [1.0, 1.0]).phase == "balance"
    assert supervisor.last_outcome == "landed"


@pytest.mark.parametrize("airborne,reason", [(False, "takeoff_timeout"), (True, "flight_timeout")])
def test_timeouts_record_abort_and_require_supported_recovery(airborne, reason):
    supervisor = ready_supervisor()
    if airborne:
        supervisor.update(.901, False, [0.0, 0.0])
        supervisor.update(.909, False, [0.0, 0.0])
        now, force = 1.409, [0.0, 0.0]
    else:
        now, force = 1.070, [1.0, 1.0]
    output = supervisor.update(now, False, force)
    assert output.phase == "recover" and not output.direct_target
    assert supervisor.last_outcome == "aborted"
    assert supervisor.events[-1].reason == reason
    assert supervisor.update(now + .6, False, [0.0, 0.0]).phase == "recover"
    supervisor.update(now + .7, False, [1.0, 1.0])
    assert supervisor.update(now + .710, False, [1.0, 1.0]).phase == "balance"
    assert supervisor.last_outcome == "aborted"


def test_request_guard_and_edge_trigger_do_not_queue_an_unsafe_jump():
    supervisor = JumpSupervisor()
    supervisor.update(0.0, False, [1.0, 1.0])
    assert supervisor.update(.020, True, [1.0, 1.0], ready=False).phase == "balance"
    assert supervisor.events[-1].reason == "posture_or_speed"
    assert supervisor.update(.030, True, [1.0, 1.0], ready=True).phase == "balance"
    supervisor.update(.040, False, [1.0, 1.0])
    assert supervisor.update(.050, True, [1.0, 1.0]).phase == "crouch"
