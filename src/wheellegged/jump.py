"""Contact-driven supervisor for the hardware-limited standing jump.

Only hip targets are selected here; MuJoCo and the torque-limited actuators own
the motion. A phase label is never evidence of a physical jump: evaluation must
also verify sustained geometric clearance and return to supported balance.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from collections.abc import Sequence


@dataclass(frozen=True)
class JumpOutput:
    phase: str
    hip_target: float | None
    direct_target: bool


@dataclass(frozen=True)
class JumpEvent:
    time: float
    event: str
    phase: str
    reason: str = ""


class JumpSupervisor:
    """Debounced two-wheel contact transitions, with bounded timing fallbacks.

    Requests are rising-edge triggered and require 10 ms of two-wheel contact
    plus the controller's posture/speed readiness guard. ``push`` lasts at least
    80 ms. ``tuck`` awaits takeoff confirmation; ``flight`` requires both contact
    forces below 0.02 N continuously for 8 ms. Subsequent two-wheel support above
    0.1 N for 10 ms triggers ``land``. One-frame dropout and one-wheel contact
    cannot advance those confirmations.

    No takeoff within 250 ms of push, or no landing within 500 ms of confirmed
    flight, enters ``recover`` with an explicit aborted outcome. Both ``land``
    and ``recover`` restore the nominal hip target smoothly in the controller;
    balance resumes only after 500 ms and confirmed support. A failed request
    does not retry while its input remains high.
    """

    def __init__(self):
        self.phase = "balance"
        self.events: list[JumpEvent] = []
        self.last_outcome: str | None = None
        self._phase_start = 0.0
        self._push_start: float | None = None
        self._takeoff_confirmed_at: float | None = None
        self._support_since: float | None = None
        self._air_since: float | None = None
        self._previous_request = False
        self._previous_time: float | None = None

    @staticmethod
    def _elapsed(time: float, since: float | None, duration: float) -> bool:
        return since is not None and time - since >= duration - 1e-12

    def _transition(self, time: float, phase: str, reason: str) -> None:
        self.phase, self._phase_start = phase, time
        self.events.append(JumpEvent(time, "phase_changed", phase, reason))

    def _abort(self, time: float, reason: str) -> None:
        self.last_outcome = "aborted"
        self._transition(time, "recover", reason)

    def update(self, time: float, request: bool, ground_forces: Sequence[float],
               *, ready: bool = True) -> JumpOutput:
        """Advance from current contact samples; repeated times add no dwell.

        ``ready`` is a posture/rate/speed gate supplied by the controller. Ground
        forces are left/right wheel-world normal forces; obstacle side contacts
        count as contact, so this baseline only authorizes standing jumps from
        the controller's supported near-upright state, not arbitrary wall pushes.
        """
        if not math.isfinite(time) or (self._previous_time is not None and time < self._previous_time):
            raise ValueError("jump supervisor time must be finite and monotonic; reset with a new instance")
        if len(ground_forces) != 2 or not all(math.isfinite(float(force)) for force in ground_forces):
            raise ValueError("jump supervisor requires two finite wheel contact forces")
        self._previous_time = time
        supported = all(force > 0.1 for force in ground_forces)
        airborne = all(force < 0.02 for force in ground_forces)
        self._support_since = (time if self._support_since is None else self._support_since) if supported else None
        self._air_since = (time if self._air_since is None else self._air_since) if airborne else None
        support_confirmed = self._elapsed(time, self._support_since, .010)
        rising_request = bool(request) and not self._previous_request
        self._previous_request = bool(request)

        if rising_request:
            if self.phase == "balance" and support_confirmed and ready:
                self.last_outcome = None
                self._takeoff_confirmed_at = self._push_start = None
                self._air_since = None
                self._transition(time, "crouch", "request_accepted")
            else:
                reason = "busy" if self.phase != "balance" else (
                    "unsupported" if not support_confirmed else "posture_or_speed")
                self.events.append(JumpEvent(time, "request_rejected", self.phase, reason))

        if self.phase == "crouch" and self._elapsed(time, self._phase_start, .8):
            if support_confirmed:
                self._push_start = time
                self._air_since = None
                self._transition(time, "push", "crouch_complete")
            else:
                self._abort(time, "support_lost_before_push")

        if self.phase in ("push", "tuck", "flight"):
            if self._takeoff_confirmed_at is None and self._elapsed(time, self._air_since, .008):
                self._takeoff_confirmed_at = time
                self.events.append(JumpEvent(time, "takeoff_confirmed", self.phase,
                                             "both_wheels_unloaded_8ms"))
            push_complete = self._elapsed(time, self._push_start, .080)
            if push_complete:
                if self._takeoff_confirmed_at is not None and support_confirmed:
                    self._transition(time, "land", "both_wheels_supported_10ms")
                elif self._takeoff_confirmed_at is not None:
                    if self._elapsed(time, self._takeoff_confirmed_at, .5):
                        self._abort(time, "flight_timeout")
                    elif self.phase != "flight":
                        self._transition(time, "flight", "takeoff_confirmed")
                elif self._elapsed(time, self._push_start, .25):
                    self._abort(time, "takeoff_timeout")
                elif self.phase == "push":
                    self._transition(time, "tuck", "push_minimum_complete")

        if self.phase in ("land", "recover"):
            if self._elapsed(time, self._phase_start, .5) and support_confirmed:
                if self.phase == "land":
                    self.last_outcome = "landed"
                self._transition(time, "balance", "supported_recovery_complete")

        hip_target = {
            "balance": None, "crouch": -.04, "push": -.3,
            "tuck": -.04, "flight": -.04, "land": -.2, "recover": -.2,
        }[self.phase]
        return JumpOutput(self.phase, hip_target, self.phase in ("push", "tuck", "flight"))
