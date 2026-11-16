"""One request in flight per API client; queued calls are bounded by their own deadline."""
import os
import subprocess

import pytest


@pytest.mark.parametrize('client', ['UNITREE_SINGLE_FLIGHT_TEST', 'UNITREE_B2_SINGLE_FLIGHT_TEST'])
@pytest.mark.parametrize('mode', ['in-flight', 'queued', 'no-loss'])
def test_single_flight(client, mode):
    result = subprocess.run([os.environ[client], mode], capture_output=True, text=True,
                            timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr


def test_copied_generic_handles_share_single_flight():
    result = subprocess.run([os.environ['UNITREE_SINGLE_FLIGHT_TEST'], 'copies'],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
