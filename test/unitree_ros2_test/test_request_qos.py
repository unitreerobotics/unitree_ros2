"""API clients advertise a reliable, volatile request writer with a five-second lifespan."""
import os
import subprocess
import uuid

import pytest


@pytest.mark.parametrize('client', ['UNITREE_REQUEST_QOS_TEST', 'UNITREE_B2_REQUEST_QOS_TEST',
                                    'UNITREE_LEGACY_GO2_REQUEST_QOS_TEST',
                                    'UNITREE_LEGACY_B2_REQUEST_QOS_TEST',
                                    'UNITREE_ROBOT_STATE_REQUEST_QOS_TEST'])
def test_request_writer_qos(client):
    prefix = '/unitree_test/qos_' + uuid.uuid4().hex
    result = subprocess.run([os.environ[client], '--ros-args',
                             '-r', '/api/sport/request:=' + prefix + '/request',
                             '-r', '/api/sport/response:=' + prefix + '/response',
                             '-r', '/api/robot_state/request:=' + prefix + '/request',
                             '-r', '/api/robot_state/response:=' + prefix + '/response'],
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
