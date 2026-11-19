"""Physical acceptance through the same key handlers as the interactive viewer."""
import importlib.util
from pathlib import Path

import pytest


path = Path(__file__).resolve().parents[1] / "scripts" / "leg_motion_smoke.py"
spec = importlib.util.spec_from_file_location("leg_motion_smoke", path)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


@pytest.fixture(scope="module", params=tuple(probe.TRIALS))
def trial(request):
    return request.param, probe.TRIALS[request.param]()


def test_keyboard_leg_motion_satisfies_physical_acceptance(trial):
    name, result = trial
    assert result["passed"], {"case": name, "failed": result["failed_checks"],
                              "metrics": result["metrics"]}
