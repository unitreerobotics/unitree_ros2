"""Check return-code getters preserve outputs on malformed application responses."""
import os
import subprocess
import uuid

import pytest


CASES = [
    ("generic", mode) for mode in
    ("volume", "fsm", "phase", "mode", "silent", "robot-list", "robot-switch")
] + [
    ("b2", mode) for mode in ("fsm", "phase", "mode", "silent", "ids", "b2-mode")
]


def run_getter(client, mode, good=False):
    topic = "/unitree_test/response_shape/run_" + uuid.uuid4().hex
    command = [os.environ["UNITREE_SHAPE_" + client.upper()], mode]
    if good:
        command.append("good")
    command.append("--ros-args")
    for api in ("voice", "sport", "motion_switcher", "robot_state"):
        for direction in ("request", "response"):
            original = "/api/" + api + "/" + direction
            command += ["-r", original + ":=" + topic + "/" + api + "/" + direction]
    result = subprocess.run(command, timeout=9, text=True, capture_output=True)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("client,mode", CASES)
@pytest.mark.parametrize("good", [False, True], ids=["invalid-response", "valid-response"])
def test_getter_response_shape(client, mode, good):
    run_getter(client, mode, good)


@pytest.mark.parametrize("mode", ["robot-good", "robot-status", "robot-timeout", "robot-no-peer"])
def test_robot_state_query_lifecycle(mode):
    run_getter("generic", mode)
