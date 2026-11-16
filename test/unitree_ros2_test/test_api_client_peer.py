"""API clients against a separate plain Cyclone DDS API server (no ROS on the server)."""
import os
import subprocess
import time
import uuid

import pytest

CLIENTS = {
    'generic': 'UNITREE_API_CLIENT_PEER_TEST',
    'b2': 'UNITREE_B2_API_CLIENT_PEER_TEST',
}


def environments(namespace, **server_extra):
    server_env = os.environ.copy()
    server_env['UNITREE_DDS_TEST_NAMESPACE'] = namespace
    server_env.update(server_extra)
    if os.environ.get('UNITREE_DDS_PEER_LIBRARY_PATH'):
        server_env['LD_LIBRARY_PATH'] = os.environ['UNITREE_DDS_PEER_LIBRARY_PATH']
    client_env = os.environ.copy()
    client_env['UNITREE_DDS_TEST_NAMESPACE'] = namespace
    return server_env, client_env


def start_server(env):
    return subprocess.Popen([os.environ['UNITREE_DDS_API_SERVER']], env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def stop(process):
    if process.poll() is None:
        process.kill()
    return process.communicate()


@pytest.mark.parametrize('client', sorted(CLIENTS))
def test_api_client_against_plain_dds_server(client):
    namespace = 'unitree_test/api_' + uuid.uuid4().hex
    # Both clients serialize calls, so neither sends the concurrent out-of-order
    # pair (2002/2003) that the server can still exercise.
    server_env, client_env = environments(namespace, UNITREE_DDS_API_SERVER_EXPECT='19')
    server = start_server(server_env)
    try:
        # Started together on purpose: the client must not depend on settle time.
        result = subprocess.run([os.environ[CLIENTS[client]], 'scenario'], env=client_env,
                                capture_output=True, text=True, timeout=30)
        server.wait(timeout=30)
    finally:
        _, server_error = stop(server)
    assert result.returncode == 0, result.stdout + result.stderr + server_error
    assert server.returncode == 0, server_error


@pytest.mark.parametrize('client', sorted(CLIENTS))
def test_api_client_reconnects_to_restarted_server(client):
    namespace = 'unitree_test/api_' + uuid.uuid4().hex
    server_env, client_env = environments(namespace, UNITREE_DDS_API_SERVER_REPLIES='1')
    # The restarted server keeps answering, like firmware: matching is asymmetric,
    # so on a slow link its first reply can precede its writer matching the client.
    second_env = dict(server_env, UNITREE_DDS_API_SERVER_REPLIES='1000')
    first = start_server(server_env)
    second = None
    process = subprocess.Popen([os.environ[CLIENTS[client]], 'reconnect'], env=client_env,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        first.wait(timeout=30)
        time.sleep(1.0)  # Client calls in this gap must time out, not hang.
        second = start_server(second_env)
        output, error = process.communicate(timeout=40)
    finally:
        stop(process)
        first_error = stop(first)[1]
        second_error = stop(second)[1] if second else ''
    assert process.returncode == 0, output + error
    assert first.returncode == 0, first_error
    assert second is not None and 'request api=2001' in second_error, second_error
