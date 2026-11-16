"""Legacy sport query paths use private remapped software peers only."""
import os
import subprocess
import uuid

import pytest


@pytest.mark.parametrize("binary", ["UNITREE_LEGACY_GO2_TEST", "UNITREE_LEGACY_B2_TEST"])
@pytest.mark.parametrize("mode", ["correlation", "timeout", "status", "no-server"])
def test_legacy_sport_query(binary, mode):
    topic = "/unitree_test/legacy/run_" + uuid.uuid4().hex
    result = subprocess.run(
        [os.environ[binary], mode, "--ros-args",
         "-r", "/api/sport/request:=" + topic + "/request",
         "-r", "/api/sport/response:=" + topic + "/response"],
        timeout=9, text=True, capture_output=True)
    assert result.returncode == 0, result.stdout + result.stderr
