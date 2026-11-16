"""Synthetic regression fixtures for the read-only Unitree bag checker."""
import importlib.util
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import sys

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

SPEC = importlib.util.spec_from_file_location(
    'check_bags', Path(__file__).with_name('check_bags.py'))
check_bags = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check_bags)

TYPES = (
    'unitree_go/msg/LowState',
    'unitree_go/msg/SportModeState',
    'unitree_go/msg/WirelessController',
)


def telemetry(typename):
    message = get_message(typename)()
    if typename.endswith('/LowState'):
        message.head = [0xFE, 0xEF]
        message.tick = 0x12345678
        message.crc = 0x89ABCDEF
        message.imu_state.quaternion = [1.0, 0.125, -0.25, 0.5]
        message.motor_state[0].q = -1.25
        message.motor_state[-1].temperature = -3
    elif typename.endswith('/SportModeState'):
        message.stamp.sec = 123
        message.stamp.nanosec = 456789
        message.error_code = 17
        message.mode = 2
        message.progress = 0.5
        message.gait_type = 3
        message.position = [1.0, -2.0, 0.25]
        message.velocity = [-0.125, 0.5, 0.0]
        message.foot_force = [-123, 234, -345, 456]
        message.foot_speed_body[-1] = 0.25
    else:
        message.lx = -0.5
        message.ly = 0.25
        message.rx = 0.125
        message.ry = -0.0625
        message.keys = 0xF123
    return message


def recording(tmp_path, rows):
    path = tmp_path / 'synthetic.db3'
    with sqlite3.connect(path) as connection:
        connection.executescript('''
            CREATE TABLE schema(schema_version INTEGER, ros_distro TEXT);
            INSERT INTO schema VALUES(3, 'synthetic');
            CREATE TABLE topics(
                id INTEGER PRIMARY KEY, name TEXT, type TEXT,
                serialization_format TEXT, offered_qos_profiles TEXT);
            CREATE TABLE messages(
                id INTEGER PRIMARY KEY, topic_id INTEGER,
                timestamp INTEGER, data BLOB);
        ''')
        for index, (typename, raw) in enumerate(rows, 1):
            connection.execute(
                'INSERT INTO topics VALUES(?, ?, ?, ?, ?)',
                (index, '/telemetry/sample_{}'.format(index),
                 typename, 'cdr', ''))
            connection.execute(
                'INSERT INTO messages VALUES(?, ?, ?, ?)',
                (index, index, 1234567890000000000 + index, raw))
    return path


def validate(path, resolver=get_message):
    return check_bags.validate(
        path, resolver, deserialize_message, serialize_message, 0)


@pytest.mark.parametrize('typename', TYPES)
def test_all_fields_roundtrip_and_source_preserved(tmp_path, typename):
    message = telemetry(typename)
    raw = serialize_message(message)
    path = recording(tmp_path, [(typename, raw)])
    before = path.read_bytes()
    result, samples = validate(path)
    assert result['passed'], result['errors']
    assert result['validated_messages'] == 1
    assert result['sha256_before'] == result['sha256_after']
    assert path.read_bytes() == before
    topic = result['topics'][0]
    assert topic['first_timestamp_ns'] == 1234567890000000001
    assert topic['last_timestamp_ns'] == 1234567890000000001
    assert topic['timestamp_and_payload_sha256']
    check_bags.equal(check_bags.plain(message),
                     check_bags.plain(samples[typename]))


@pytest.mark.parametrize('alias', ['direct', 'symlink', 'hardlink'])
def test_report_refuses_source_recording_alias(tmp_path, alias):
    typename = TYPES[0]
    path = recording(tmp_path, [
        (typename, serialize_message(telemetry(typename)))])
    report = path if alias == 'direct' else tmp_path / 'report.json'
    if alias == 'symlink':
        report.symlink_to(path)
    elif alias == 'hardlink':
        os.link(path, report)
    before = path.read_bytes()
    result = subprocess.run(
        [sys.executable, str(Path(check_bags.__file__)), str(path),
         '--report', str(report)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 2, result.stdout + result.stderr
    assert 'must not overwrite a source recording' in result.stderr
    assert path.read_bytes() == before


def test_report_refuses_source_recording_directory(tmp_path):
    typename = TYPES[0]
    path = recording(tmp_path, [
        (typename, serialize_message(telemetry(typename)))])
    metadata = tmp_path / 'metadata.yaml'
    metadata.write_text('original metadata\n')
    before = path.read_bytes()
    result = subprocess.run(
        [sys.executable, str(Path(check_bags.__file__)), str(tmp_path),
         '--report', str(metadata)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 2, result.stdout + result.stderr
    assert 'outside source recording directories' in result.stderr
    assert path.read_bytes() == before
    assert metadata.read_text() == 'original metadata\n'


@pytest.mark.parametrize('alias', ['direct', 'hardlink'])
def test_file_input_report_cannot_overwrite_source_metadata(tmp_path, alias):
    typename = TYPES[0]
    path = recording(tmp_path, [
        (typename, serialize_message(telemetry(typename)))])
    metadata = tmp_path / 'metadata.yaml'
    metadata.write_text('original metadata\n')
    report = metadata if alias == 'direct' else tmp_path / 'report.json'
    if alias == 'hardlink':
        os.link(metadata, report)
    before = path.read_bytes()
    result = subprocess.run(
        [sys.executable, str(Path(check_bags.__file__)), str(path),
         '--report', str(report)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 2, result.stdout + result.stderr
    assert 'must not overwrite a source recording' in result.stderr
    assert path.read_bytes() == before
    assert metadata.read_text() == 'original metadata\n'


def test_report_outside_source_directory_preserves_recording(tmp_path):
    directory = tmp_path / 'source'
    directory.mkdir()
    typename = TYPES[0]
    path = recording(directory, [
        (typename, serialize_message(telemetry(typename)))])
    before = path.read_bytes()
    report = tmp_path / 'report.json'
    result = subprocess.run(
        [sys.executable, str(Path(check_bags.__file__)), str(directory),
         '--report', str(report)], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    data = json.loads(report.read_text())
    assert data['passed']
    assert data['recordings'][0]['validated_messages'] == 1
    assert path.read_bytes() == before


@pytest.mark.parametrize('typename', TYPES)
def test_truncated_payload_rejected(tmp_path, typename):
    raw = serialize_message(telemetry(typename))
    path = recording(tmp_path, [(typename, raw[:-8])])
    result, _ = validate(path)
    assert not result['passed']
    assert any('truncated CDR' in error for error in result['errors'])


@pytest.mark.parametrize('typename', TYPES)
def test_unknown_nonzero_trailer_rejected(tmp_path, typename):
    raw = serialize_message(telemetry(typename)) + b'\x01\x00\x00\x00'
    path = recording(tmp_path, [(typename, raw)])
    result, _ = validate(path)
    assert not result['passed']
    assert any('unexplained CDR trailer' in error
               for error in result['errors'])


def test_observed_sport_padding_explicitly_reported(tmp_path):
    typename = 'unitree_go/msg/SportModeState'
    raw = serialize_message(telemetry(typename))
    decoder = check_bags.CDR(raw, get_message)
    decoder.message(get_message(typename))
    raw = raw[:decoder.pos] + bytes(280)
    path = recording(tmp_path, [(typename, raw)])
    result, _ = validate(path)
    assert result['passed'], result['errors']
    assert result['topics'][0]['trailing_byte_counts'] == {'280': 1}


def test_undocumented_sport_padding_rejected(tmp_path):
    typename = 'unitree_go/msg/SportModeState'
    raw = serialize_message(telemetry(typename)) + bytes(279)
    path = recording(tmp_path, [(typename, raw)])
    result, _ = validate(path)
    assert not result['passed']
    assert any('unexplained CDR trailer' in error
               for error in result['errors'])


def test_missing_required_telemetry_definition_fails(tmp_path):
    typename = 'unitree_go/msg/LowState'
    path = recording(tmp_path, [
        (typename, serialize_message(telemetry(typename)))])

    def unavailable(name):
        raise ImportError('synthetic missing definition: ' + name)

    result, _ = validate(path, unavailable)
    assert not result['passed']
    assert not result['topics'][0]['definition_available']
    assert any('missing required definition: ' + typename in error
               for error in result['errors'])


def test_unrelated_missing_definition_reported(tmp_path):
    typename = 'unitree_go/msg/LowState'
    path = recording(tmp_path, [
        (typename, serialize_message(telemetry(typename))),
        ('unavailable_msgs/msg/Telemetry', b'\x00\x01\x00\x00')])
    result, _ = validate(path)
    assert result['passed'], result['errors']
    unavailable = result['topics'][1]
    assert not unavailable['definition_available']
    assert unavailable['unresolved_definition']
    assert not unavailable['required_telemetry']


def test_ros_validation_refuses_non_loopback_configuration(monkeypatch):
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv(
        'CYCLONEDDS_URI',
        '<CycloneDDS><Domain><General><Interfaces>'
        '<NetworkInterface name="eth0"/>'
        '</Interfaces></General></Domain></CycloneDDS>')
    with pytest.raises(RuntimeError, match='loopback-only'):
        check_bags.validate_ros({}, get_message)


def test_mcap_telemetry_roundtrip(tmp_path):
    import rosbag2_py
    directory = tmp_path / 'mcap'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(directory), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for index, typename in enumerate(TYPES, 1):
        name = '/telemetry/sample_{}'.format(index)
        attributes = dict(name=name, type=typename, serialization_format='cdr')
        try:
            metadata = rosbag2_py.TopicMetadata(id=index, **attributes)
        except TypeError:
            metadata = rosbag2_py.TopicMetadata(**attributes)
        writer.create_topic(metadata)
        writer.write(name, serialize_message(telemetry(typename)),
                     1234567890000000000 + index)
    del writer
    path = next(directory.glob('*.mcap'))
    result, samples = validate(path)
    assert result['passed'], result['errors']
    assert result['storage'] == 'mcap'
    assert result['validated_messages'] == 3
    assert result['sha256_before'] == result['sha256_after']
    for typename in TYPES:
        check_bags.equal(check_bags.plain(telemetry(typename)),
                         check_bags.plain(samples[typename]))


def test_unspecified_final_cdr_alignment_bytes(tmp_path):
    typename = 'unitree_go/msg/WirelessController'
    raw = serialize_message(telemetry(typename))
    # The final uint16 ends at byte 22. Some serializers align the payload
    # to four bytes without clearing the two padding bytes.
    raw = raw[:22] + b'\xa5\x5a'
    path = recording(tmp_path, [(typename, raw)])
    result, samples = validate(path)
    assert result['passed'], result['errors']
    assert result['topics'][0]['nonzero_alignment_padding_count'] == 1
    check_bags.equal(check_bags.plain(telemetry(typename)),
                     check_bags.plain(samples[typename]))
