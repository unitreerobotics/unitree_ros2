"""The GO2 telemetry preflight fails closed and writes nothing.

Everything runs in private loopback domains. Peers create endpoints and never
write a sample, and the control examples are only named, never executed.
"""
import itertools
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from rcl_interfaces.msg import Log, ParameterEvent
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from unitree_api.msg import Request
from unitree_go.msg import LowCmd, WirelessController

SCRIPT = Path(__file__).resolve().parents[2] / 'scripts/go2_telemetry_preflight.py'
EXAMPLES = Path(get_package_prefix('unitree_ros2_example')) / 'lib/unitree_ros2_example'
APPROVED = {'read_low_state', 'read_motion_state', 'read_wireless_controller'}
LOOPBACK_URI = (
    '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="true"/>'
    '</Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery>'
    '<ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers>'
    '</Discovery></Domain></CycloneDDS>')
DOMAINS = itertools.cycle(range(180, 220))


@pytest.fixture
def domain(monkeypatch):
    number = next(DOMAINS)
    monkeypatch.setenv('ROS_DOMAIN_ID', str(number))
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('CYCLONEDDS_URI', LOOPBACK_URI)
    monkeypatch.delenv('ROS_AUTOMATIC_DISCOVERY_RANGE', raising=False)
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    return number


def preflight(domain, *extra, timeout=10, settle=1):
    return subprocess.run(
        ['python3', str(SCRIPT), '--domain', str(domain), '--interface', 'lo',
         '--timeout', str(timeout), '--settle', str(settle), *extra],
        capture_output=True, text=True, timeout=timeout + settle + 15)


class Peer:

    def __init__(self, *args):
        self.args = [os.environ['UNITREE_TELEMETRY_ENDPOINT_PEER'], *args]

    def __enter__(self):
        self.process = subprocess.Popen(self.args, stdout=subprocess.PIPE, text=True)
        assert self.process.stdout.readline().strip() == 'ready'
        return self

    def __exit__(self, *_):
        self.process.send_signal(signal.SIGTERM)
        self.process.wait(timeout=5)


class Observer:
    """A ROS node in the test domain. Its endpoints are not those of the check."""

    def __enter__(self):
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node('preflight_test_observer', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        return self

    def __exit__(self, *_):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


@pytest.mark.parametrize('name', sorted(
    path.name for path in EXAMPLES.iterdir()
    if path.is_file() and os.access(path, os.X_OK) and path.name not in APPROVED))
def test_every_other_example_is_rejected_before_dds_starts(name, domain):
    result = preflight(domain, 'read_low_state', name)
    assert result.returncode == 1, result.stdout
    assert 'rejected: ' + name in result.stdout, result.stdout
    assert 'telemetry /lowstate' not in result.stdout, result.stdout


@pytest.mark.parametrize('variable,value,expected', [
    ('ROS_DOMAIN_ID', '42', 'ROS_DOMAIN_ID set explicitly'),
    ('ROS_DOMAIN_ID', None, 'ROS_DOMAIN_ID set explicitly'),
    ('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp', 'RMW is'),
    ('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST', 'discovery is not restricted'),
    ('CYCLONEDDS_URI', None, 'Cyclone DDS is bound'),
    ('CYCLONEDDS_URI', 'file:///cannot/be/verified.xml', 'Cyclone DDS is bound'),
])
def test_environment_uncertainty_fails(variable, value, expected, domain, monkeypatch):
    if value is None:
        monkeypatch.delenv(variable)
    else:
        monkeypatch.setenv(variable, value)
    result = preflight(domain)
    assert result.returncode == 1, result.stdout
    assert '[FAIL] ' + expected in result.stdout, result.stdout
    assert 'telemetry /lowstate' not in result.stdout, result.stdout


def test_no_robot_fails_within_its_bound(domain):
    started = time.monotonic()
    result = preflight(domain, timeout=2, settle=1)
    assert time.monotonic() - started < 10
    assert result.returncode == 1, result.stdout
    assert 'no publisher discovered' in result.stdout, result.stdout


def test_firmware_like_peer_passes_and_the_check_writes_nothing(domain, tmp_path):
    seen, own_writers = [], set()
    report = tmp_path / 'report.json'
    with Peer(), Observer() as observer:
        node = observer.node
        node.create_subscription(
            ParameterEvent, '/parameter_events', lambda m: seen.append(m.node), 100)
        node.create_subscription(Log, '/rosout', lambda m: seen.append(m.name), 100)
        # Prove this subscription would see a node writing a parameter event.
        canary = rclpy.create_node('preflight_canary', context=observer.context)
        deadline = time.monotonic() + 5
        while not any('preflight_canary' in name for name in seen):
            assert time.monotonic() < deadline
            canary.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])
            observer.executor.spin_once(timeout_sec=0.05)
        canary.destroy_node()
        command = ['python3', str(SCRIPT), '--domain', str(domain), '--interface', 'lo',
                   '--timeout', '10', '--settle', '1', '--report', str(report)]
        process = subprocess.Popen(command, stdout=subprocess.PIPE, text=True)
        while process.poll() is None:
            observer.executor.spin_once(timeout_sec=0.02)
            for topic in ('/parameter_events', '/rosout'):
                own_writers |= {info.node_name for info in node.get_publishers_info_by_topic(topic)
                                if info.node_name.startswith('go2_telemetry_preflight')}
        output = process.stdout.read()
    assert process.returncode == 0, output
    assert not own_writers
    assert not [name for name in seen if 'go2_telemetry_preflight' in name]
    data = json.loads(report.read_text())
    assert data['result'] == 'PASS' and data['domain'] == domain
    assert set(data['binaries']) == APPROVED and not data['command_writers']
    assert data['telemetry_publishers']['/lowstate'][0]['node'] == '_CREATED_BY_BARE_DDS_APP_'


def test_nonros_command_writer_fails_unless_the_exact_count_is_acknowledged(domain):
    with Peer('with-lowcmd'):
        result = preflight(domain)
        assert result.returncode == 1, result.stdout
        assert '[FAIL] no unacknowledged non-ROS writer' in result.stdout, result.stdout
        result = preflight(domain, '--allow-nonros-writer', '/lowcmd=2')
        assert result.returncode == 1, result.stdout
        result = preflight(domain, '--allow-nonros-writer', '/lowcmd=1')
        assert result.returncode == 0, result.stdout


def test_withdrawn_baseline_writer_is_reported_apart_and_still_fails(domain, tmp_path):
    baseline, after = tmp_path / 'before.json', tmp_path / 'after.json'
    with Peer('with-lowcmd'):
        result = preflight(domain, '--allow-nonros-writer', '/lowcmd=1', '--report', str(baseline))
        assert result.returncode == 0, result.stdout
    with Peer():
        result = preflight(domain, '--allow-nonros-writer', '/lowcmd=1',
                           '--baseline', str(baseline), '--report', str(after))
    assert result.returncode == 3, result.stdout
    assert '[FAIL] no baseline command writer withdrawn' in result.stdout, result.stdout
    assert '[ok] no command writer appeared since the baseline' in result.stdout, result.stdout
    data = json.loads(after.read_text())
    assert data['result'] == 'FAIL' and data['failure'] == 'baseline writers withdrawn only'
    withdrawn = data['baseline_comparison']['withdrawn']
    assert [item['topic'] for item in withdrawn] == ['/lowcmd']
    assert withdrawn[0]['participant_present'] is False


def test_replaced_writer_fails_although_the_count_matches(domain, tmp_path):
    baseline = tmp_path / 'before.json'
    with Peer('with-lowcmd'):
        result = preflight(domain, '--allow-nonros-writer', '/lowcmd=1', '--report', str(baseline))
        assert result.returncode == 0, result.stdout
    with Peer('with-lowcmd'):
        result = preflight(domain, '--allow-nonros-writer', '/lowcmd=1',
                           '--baseline', str(baseline))
    assert result.returncode == 1, result.stdout
    assert '[ok] no unacknowledged non-ROS writer' in result.stdout, result.stdout
    assert '[FAIL] no command writer appeared since the baseline' in result.stdout, result.stdout
    assert 'unidentified participant' in result.stdout, result.stdout


def test_failing_baseline_is_not_accepted(domain, tmp_path):
    baseline = tmp_path / 'before.json'
    with Peer('with-lowcmd'):
        result = preflight(domain, '--report', str(baseline))
        assert result.returncode == 1, result.stdout
        result = preflight(domain, '--allow-nonros-writer', '/lowcmd=1',
                           '--baseline', str(baseline))
    assert result.returncode == 1, result.stdout
    assert '[FAIL] baseline is a passing report' in result.stdout, result.stdout


def test_monitor_reports_a_command_writer_that_is_gone_by_the_end(domain, tmp_path):
    baseline, report = tmp_path / 'before.json', tmp_path / 'during.json'
    with Peer(), Observer() as observer:
        result = preflight(domain, '--report', str(baseline))
        assert result.returncode == 0, result.stdout
        process = subprocess.Popen(
            ['python3', str(SCRIPT), '--domain', str(domain), '--interface', 'lo',
             '--timeout', '10', '--settle', '1', '--monitor', '6',
             '--baseline', str(baseline), '--report', str(report)],
            stdout=subprocess.PIPE, text=True)
        time.sleep(1.5)
        # Named like an approved example, with a writer it never publishes to.
        example = rclpy.create_node('low_state_suber', context=observer.context)
        example.create_publisher(LowCmd, '/lowcmd', 10)
        time.sleep(2)
        example.destroy_node()
        output = process.communicate(timeout=30)[0]
    assert process.returncode == 1, output
    assert '[FAIL] no ROS node publishes a command topic' in output, output
    assert '/lowcmd' in output and 'telemetry example node low_state_suber' in output, output
    data = json.loads(report.read_text())
    assert data['example_nodes_seen'] == ['low_state_suber']
    assert [item['node'] for item in data['command_writers']] == ['low_state_suber']


@pytest.mark.parametrize('kind,topic,allow', [
    (LowCmd, '/lowcmd', '/lowcmd=1'),
    (Request, '/api/robot_state/request', '/api/robot_state/request=1'),
    (Request, '/api/sport/request', '/api/sport/request=1'),
], ids=['lowcmd', 'robot-state', 'sport'])
def test_ros_node_command_writer_always_fails(kind, topic, allow, domain):
    with Peer(), Observer() as observer:
        observer.node.create_publisher(kind, topic, 10)  # Never published to.
        result = preflight(domain, '--allow-nonros-writer', allow)
    assert result.returncode == 1, result.stdout
    assert '[FAIL] no ROS node publishes a command topic' in result.stdout, result.stdout


def test_ros_node_cannot_stand_in_for_firmware_telemetry(domain):
    with Observer() as observer:
        observer.node.create_publisher(WirelessController, '/wirelesscontroller', 10)
        result = preflight(domain, 'read_wireless_controller', timeout=2, settle=1)
    assert result.returncode == 1, result.stdout
    assert 'published by ROS node' in result.stdout, result.stdout


@pytest.mark.parametrize('executable,node_name,topic', [
    ('read_low_state', 'low_state_suber', '/lowstate'),
    ('read_motion_state', 'motion_state_suber', '/lf/sportmodestate'),
    ('read_wireless_controller', 'wireless_controller_suber', '/wirelesscontroller'),
])
def test_approved_examples_are_subscriber_only(executable, node_name, topic, domain):
    process = subprocess.Popen([str(EXAMPLES / executable)], stdout=subprocess.DEVNULL)
    try:
        with Observer() as observer:
            node = observer.node
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and node_name not in node.get_node_names():
                time.sleep(0.02)
            assert node_name in node.get_node_names()
            time.sleep(0.5)
            publishers = {name for name, _ in
                          node.get_publisher_names_and_types_by_node(node_name, '/')}
            subscribers = {name for name, _ in
                           node.get_subscriber_names_and_types_by_node(node_name, '/')}
            clients = node.get_client_names_and_types_by_node(node_name, '/')
            services = {name.rsplit('/', 1)[1] for name, _ in
                        node.get_service_names_and_types_by_node(node_name, '/')}
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
    assert publishers == {'/rosout', '/parameter_events'}
    assert subscribers == {topic, '/parameter_events'}
    assert not clients
    assert services <= {'describe_parameters', 'get_parameter_types', 'get_parameters',
                        'list_parameters', 'set_parameters', 'set_parameters_atomically',
                        'get_type_description'}
