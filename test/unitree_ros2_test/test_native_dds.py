"""ROS RMW against a separate plain Cyclone DDS process, using XCDR1."""
import os
import subprocess
import time
import uuid

import rclpy
from unitree_api.msg import Request, Response
from unitree_go.msg import WirelessController, LowState, SportModeState
from unitree_hg.msg import LowState as HGLowState
import pytest

# Loopback configuration of the pre-Jazzy setup_local.sh. unitree_sdk2 processes
# (for example a local simulator) bind "lo" the same way, without unicast peers
# or fixed participant indexes, so the current loopback setup must still reach them.
LEGACY_LOOPBACK_URI = (
    '<CycloneDDS><Domain><General><Interfaces>'
    '<NetworkInterface name="lo" priority="default" multicast="default" />'
    '</Interfaces></General></Domain></CycloneDDS>')


@pytest.mark.parametrize('peer_config', ['same-as-ros', 'legacy-loopback'])
def test_plain_dds_request_response_and_telemetry(peer_config):
    peer = os.environ['UNITREE_DDS_PEER']
    namespace = '/unitree_test/native_' + uuid.uuid4().hex
    rclpy.init()
    node = rclpy.create_node('native_dds_regression')
    responses, telemetry = [], []
    go_states, hg_states, sport_states = [], [], []
    subscriptions = [
        node.create_subscription(Response, namespace + '/response', responses.append, 10),
        node.create_subscription(
            WirelessController, namespace + '/wireless', telemetry.append, 10),
        node.create_subscription(LowState, namespace + '/go_lowstate', go_states.append, 10),
        node.create_subscription(HGLowState, namespace + '/hg_lowstate', hg_states.append, 10),
        node.create_subscription(SportModeState, namespace + '/sport', sport_states.append, 10),
    ]
    publisher = node.create_publisher(Request, namespace + '/request', 10)
    request = Request()
    request.header.identity.id = 0x1234567887654321
    request.header.identity.api_id = 1001
    request.header.lease.id = 42
    request.header.policy.priority = -7
    request.parameter = '{"probe":true}'
    request.binary = [0, 127, 128, 255]
    peer_env = os.environ.copy()
    peer_env['UNITREE_DDS_TEST_NAMESPACE'] = namespace.lstrip('/')
    if peer_config == 'legacy-loopback':
        peer_env['CYCLONEDDS_URI'] = LEGACY_LOOPBACK_URI
    if os.environ.get('UNITREE_DDS_PEER_LIBRARY_PATH'):
        peer_env['LD_LIBRARY_PATH'] = os.environ['UNITREE_DDS_PEER_LIBRARY_PATH']
    process = subprocess.Popen(
        [peer], env=peer_env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        deadline = time.monotonic() + 20
        while (time.monotonic() < deadline
               and not all((responses, telemetry, go_states, hg_states, sport_states))):
            if publisher.get_subscription_count():
                publisher.publish(request)
            rclpy.spin_once(node, timeout_sec=0.05)
        output, error = process.communicate(timeout=5)
        assert process.returncode == 0, (output, error)
        assert responses and telemetry, (len(responses), len(telemetry))
        response = responses[0]
        assert response.header.identity.id == request.header.identity.id
        assert response.header.identity.api_id == 1001
        assert response.header.status.code == -23
        assert response.data == '{"native_dds":true}'
        assert list(response.binary) == [-128, -1, 0, 127]
        state = telemetry[0]
        assert [state.lx, state.ly, state.rx, state.ry] == [0.25, -0.5, 0.75, 1.0]
        assert state.keys == 0xabcd
        go = go_states[0]
        assert list(go.head) == [0xfe, 0xef] and go.tick == 0xfedcba98
        assert go.motor_state[19].q == -1.25 and go.foot_force[3] == -1234
        assert go.wireless_remote[39] == 0xa5 and go.crc == 0x89abcdef
        hg = hg_states[0]
        assert hg.tick == 0x12345678 and hg.motor_state[34].q == 2.5
        assert hg.motor_state[34].temperature[1] == -2345
        assert hg.reserve[3] == 0xdeadbeef and hg.crc == 0x76543210
        sport = sport_states[0]
        assert sport.stamp.sec == 123456 and sport.stamp.nanosec == 789012345
        assert sport.position[2] == -3.5 and sport.foot_speed_body[11] == 6.25
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
