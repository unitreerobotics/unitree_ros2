#!/usr/bin/env python3
"""Verify timed SQLite ROS bag telemetry playback on remapped loopback topics.

An explicit dataset directory is required. The original files are opened
read-only. Supported telemetry types are selected, and each selected topic is
remapped away from robot topic names. No robot command type is published.
"""
import argparse
from contextlib import ExitStack
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import resource
import signal
import statistics
import struct
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

SPEC = importlib.util.spec_from_file_location(
    'check_bags', Path(__file__).with_name('check_bags.py'))
check_bags = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(check_bags)


def verify_isolation(get_message):
    """Require a dedicated domain, explicit loopback and unicast discovery."""
    if os.environ.get('ROS_DOMAIN_ID') != '173':
        raise RuntimeError('timed playback requires ROS_DOMAIN_ID=173')
    try:
        config = ET.fromstring(os.environ.get('CYCLONEDDS_URI', ''))
    except ET.ParseError as error:
        raise RuntimeError(
            'expected inline loopback Cyclone DDS XML') from error
    interfaces = list(config.iter('NetworkInterface'))
    if len(interfaces) != 1 or interfaces[0].get('name') != 'lo':
        raise RuntimeError('timed playback requires only the lo interface')
    multicast = list(config.iter('AllowMulticast'))
    if (len(multicast) != 1
            or (multicast[0].text or '').strip().lower() != 'false'):
        raise RuntimeError('timed playback requires AllowMulticast=false')
    peers = list(config.iter('Peer'))
    if any(peer.get('Address') != '127.0.0.1' for peer in peers):
        raise RuntimeError('timed playback refuses non-loopback peers')
    # Reuse the bag checker's transport guard before any replay nodes exist.
    # An empty mapping creates no publishers and immediately shuts down.
    check_bags.validate_ros({}, get_message)


def normalized(value):
    """Encode semantic fields deterministically, preserving sequence order."""
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      allow_nan=True).encode('utf-8')


def update_digest(digest, data):
    digest.update(struct.pack('<Q', len(data)))
    digest.update(data)


def rss_kib(pid):
    """Read resident memory without adding a platform-specific dependency."""
    try:
        for line in Path('/proc/{}/status'.format(pid)).read_text().splitlines():
            if line.startswith('VmRSS:'):
                return int(line.split()[1])
    except (OSError, ValueError):
        pass
    return None


class Timing:
    """Bound timing storage while retaining exact extrema and mean."""

    def __init__(self):
        self.count = 0
        self.total = 0
        self.minimum = None
        self.maximum = None
        self.samples = []

    def add(self, nanoseconds):
        self.count += 1
        self.total += nanoseconds
        self.minimum = (nanoseconds if self.minimum is None
                        else min(self.minimum, nanoseconds))
        self.maximum = (nanoseconds if self.maximum is None
                        else max(self.maximum, nanoseconds))
        if len(self.samples) < 100000:
            self.samples.append(nanoseconds)

    def report(self):
        if not self.count:
            return {'count': 0, 'available': False}
        values = sorted(self.samples)
        return {
            'available': True, 'count': self.count,
            'minimum_ms': self.minimum / 1e6,
            'maximum_ms': self.maximum / 1e6,
            'mean_ms': self.total / self.count / 1e6,
            'p50_ms': statistics.median(values) / 1e6,
            'p95_ms': values[math.ceil(0.95 * len(values)) - 1] / 1e6,
            'p99_ms': values[math.ceil(0.99 * len(values)) - 1] / 1e6,
            'percentile_samples': len(values),
            'percentile_policy': 'first 100000 observations, all when fewer',
        }


def source_files(directory):
    """Follow the rosbag metadata split order and reject external paths."""
    import yaml
    metadata = directory / 'metadata.yaml'
    if not metadata.is_file():
        raise ValueError('bag directory must contain metadata.yaml')
    info = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
    if info['storage_identifier'] != 'sqlite3':
        raise ValueError('timed verifier currently supports SQLite3 bags only')
    files = []
    for relative in info['relative_file_paths']:
        path = (directory / relative).resolve()
        if path.parent != directory.resolve() or path.suffix != '.db3':
            raise ValueError('unexpected or external SQLite bag path')
        if not path.is_file():
            raise ValueError('missing SQLite bag file: ' + str(path))
        files.append(path)
    if not files:
        raise ValueError('bag metadata contains no SQLite files')
    return info, files


def verify_output_paths(directory, *outputs):
    """Reject source-directory outputs and aliases of original recording files."""
    source = directory.resolve()
    originals = [source / 'metadata.yaml', *source.glob('*.db3'), *source.glob('*.mcap')]
    for output in outputs:
        destination = output.resolve()
        if destination == source or source in destination.parents:
            raise ValueError('outputs must be outside the source bag directory')
        if destination.exists() and any(
                path.exists() and path.samefile(destination) for path in originals):
            raise ValueError('outputs must not overwrite a source recording')


def expected_rows(files, topic_name):
    """Read a topic lazily, preserving file and stored timestamp order."""
    for path in files:
        with check_bags.open_sqlite(path) as connection:
            for stamp, data in connection.execute(
                    'SELECT messages.timestamp, messages.data FROM messages '
                    'JOIN topics ON topics.id=messages.topic_id '
                    'WHERE topics.name=? '
                    'ORDER BY messages.timestamp, messages.id',
                    (topic_name,)):
                yield stamp, bytes(data)


def inventory(files, get_message, deserialize_message):
    """Resolve selected types and compute complete expected field digests."""
    topics = {}
    for path in files:
        for topic in check_bags.sqlite_inventory(path)['topics']:
            typename = check_bags.canonical_type(topic['type'])
            if typename not in check_bags.TELEMETRY:
                continue
            name = topic['name']
            if name in topics and topics[name]['type'] != typename:
                raise ValueError(
                    'topic type changes between bag files: ' + name)
            if topic['serialization_format'] != 'cdr':
                raise ValueError('only CDR telemetry can be replayed')
            if name not in topics:
                topics[name] = {'name': name, 'type': typename, 'count': 0}
            topics[name]['count'] += topic['count']
    topics = {name: topic for name, topic in topics.items() if topic['count']}
    if not topics:
        raise ValueError('no supported Unitree telemetry in this recording')
    start = end = None
    run_name = 'run_' + uuid.uuid4().hex[:12]
    for index, (name, topic) in enumerate(topics.items()):
        cls = get_message(topic['type'])
        fields_digest = hashlib.sha256()
        storage_digest = hashlib.sha256()
        for stamp, raw in expected_rows(files, name):
            # Some distro RMW deserializers access undersized encapsulations
            # before checking bounds. Reject malformed recording data first.
            decoder = check_bags.CDR(raw, get_message)
            decoder.message(cls)
            check_bags.check_trailer(topic['type'], raw, decoder.pos)
            message = deserialize_message(raw, cls)
            update_digest(fields_digest, normalized(check_bags.plain(message)))
            storage_digest.update(struct.pack('<q', stamp) + raw)
            start = stamp if start is None else min(start, stamp)
            end = stamp if end is None else max(end, stamp)
        topic['expected_fields_sha256'] = fields_digest.hexdigest()
        topic['source_timestamp_and_payload_sha256'] = (
            storage_digest.hexdigest())
        topic['remapped_name'] = (
            '/unitree_regression/replay/{}/topic_{}'.format(run_name, index))
        topic['received_count'] = 0
        topic['errors'] = []
    return topics, (end - start) / 1e9


def stop_player(player):
    """Stop only the process group created for this playback invocation."""
    if player is not None and player.poll() is None:
        os.killpg(player.pid, signal.SIGTERM)
        try:
            player.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(player.pid, signal.SIGKILL)
            player.wait(timeout=5)


def build_player_command(directory, rate, qos_path, topics):
    """Build a telemetry whitelist with a private remap for each topic."""
    if not topics:
        raise ValueError('playback requires at least one telemetry topic')
    for name, topic in topics.items():
        if topic['type'] not in check_bags.TELEMETRY:
            raise ValueError('playback refuses a non-telemetry message type')
        prefix = '/unitree_regression/replay/'
        if not topic['remapped_name'].startswith(prefix):
            raise ValueError('playback requires a private telemetry remap')
        if not name.startswith('/') or ':=' in name:
            raise ValueError('invalid source telemetry topic name')
    input_arguments = ([str(directory), '--storage', 'sqlite3']
                       if os.environ.get('ROS_DISTRO') in ('foxy', 'humble')
                       else ['--input', str(directory), 'sqlite3'])
    return [
        'ros2', 'bag', 'play', *input_arguments,
        '--rate', str(rate), '--delay', '2', '--disable-keyboard-controls',
        '--read-ahead-queue-size', '10000', '--wait-for-all-acked', '10000',
        '--qos-profile-overrides-path', str(qos_path),
        '--topics', *topics,
        '--remap', *['{}:={}'.format(name, topic['remapped_name'])
                     for name, topic in topics.items()],
    ]


def replay(directory, rate, workdir, get_message, deserialize_message,
           reliability='reliable'):
    verify_output_paths(
        directory, workdir, workdir / 'playback-qos.yaml', workdir / 'rosbag-play.log')
    if reliability not in ('reliable', 'best_effort'):
        raise ValueError('reliability must be reliable or best_effort')
    import rclpy
    from rclpy.subscription import Subscription
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )
    import yaml
    verify_isolation(get_message)
    metadata, files = source_files(directory)
    original_files = [directory / 'metadata.yaml'] + files
    original_hashes = {str(p): check_bags.digest(p) for p in original_files}
    topics, duration = inventory(files, get_message, deserialize_message)
    selected_duration = duration
    # Playback retains time gaps before/after selected topics. Include the full
    # recording duration when unrelated topics start earlier or finish later.
    duration = max(duration, metadata['duration']['nanoseconds'] / 1e9)
    report = {
        'bag_directory': str(directory), 'storage': 'sqlite3',
        'ros_distro': os.environ.get('ROS_DISTRO'), 'rate': rate,
        'playback_qos': {'reliability': reliability, 'durability': 'volatile',
                         'history': 'keep_last', 'depth': 10000},
        'source_duration_seconds': duration,
        'selected_telemetry_duration_seconds': selected_duration,
        'topics': list(topics.values()),
        'original_sha256_before': original_hashes,
        'command_publication': False,
        'hardware_interoperability_verified': False,
        'source_metadata_version': metadata.get('version'), 'errors': [],
    }
    overrides = {
        name: {'reliability': reliability, 'durability': 'volatile',
               'history': 'keep_last', 'depth': 10000}
        for name in topics
    }
    qos_path = workdir / 'playback-qos.yaml'
    qos_path.write_text(yaml.safe_dump(overrides))
    command = build_player_command(directory, rate, qos_path, topics)
    report['player_command'] = command
    player = None
    node = None
    timing = {name: {key: Timing() for key in (
        'publisher_schedule_drift', 'middleware_receive_delay', 'executor_dispatch')}
        for name in topics}
    anchors = {}
    started = time.monotonic()
    cpu_start = time.process_time()
    child_start = resource.getrusage(resource.RUSAGE_CHILDREN)
    own_rss = rss_kib(os.getpid())
    peak_rss = own_rss
    player_peak_rss = None
    rclpy.init()
    try:
        node = rclpy.create_node('unitree_timed_bag_regression')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10000,
            reliability=(ReliabilityPolicy.RELIABLE if reliability == 'reliable'
                         else ReliabilityPolicy.BEST_EFFORT),
            durability=DurabilityPolicy.VOLATILE)
        actual_digests = {name: hashlib.sha256() for name in topics}
        subscriptions = []
        with ExitStack() as stack:
            iterators = {}
            for name in topics:
                generator = expected_rows(files, name)
                stack.callback(generator.close)
                iterators[name] = generator

            last_message_at = time.monotonic()

            def callback(name, message, info):
                nonlocal last_message_at
                callback_wall = time.time_ns()
                topic = topics[name]
                topic['received_count'] += 1
                actual = check_bags.plain(message)
                update_digest(actual_digests[name], normalized(actual))
                try:
                    stamp, raw = next(iterators[name])
                    if isinstance(info, dict):
                        source = info.get('source_timestamp', 0)
                        received = info.get('received_timestamp', 0)
                    else:
                        source = getattr(info, 'source_timestamp', 0)
                        received = getattr(info, 'received_timestamp', 0)
                    # Bag epochs are historical and are never used as latency
                    # clocks. DDS timestamps are valid only when both endpoints
                    # share this host clock and the RMW provides them.
                    if source > 0 and abs(callback_wall - source) < 60 * 10**9:
                        anchor_stamp, anchor_source = anchors.setdefault(
                            name, (stamp, source))
                        timing[name]['publisher_schedule_drift'].add(
                            source - anchor_source
                            - round((stamp - anchor_stamp) / rate))
                        if received >= source and received <= callback_wall:
                            timing[name]['middleware_receive_delay'].add(received - source)
                            timing[name]['executor_dispatch'].add(
                                callback_wall - received)
                    expected = deserialize_message(
                        raw, get_message(topic['type']))
                    check_bags.equal(check_bags.plain(expected), actual)
                except StopIteration:
                    if len(topic['errors']) < 5:
                        topic['errors'].append('unexpected extra message')
                except Exception as error:
                    if len(topic['errors']) < 5:
                        topic['errors'].append(str(error))
                last_message_at = time.monotonic()

            def subscription_callback(name):
                if hasattr(Subscription, 'CallbackType'):
                    def receive(message, info):
                        callback(name, message, info)
                else:
                    # Humble callbacks receive only the message. An optional
                    # second argument selects message-only callbacks on Jazzy,
                    # so use distinct signatures rather than a default value.
                    def receive(message):
                        callback(name, message, None)
                return receive

            for name, topic in topics.items():
                subscriptions.append(node.create_subscription(
                    get_message(topic['type']), topic['remapped_name'],
                    subscription_callback(name), qos))
            log_path = workdir / 'rosbag-play.log'
            with log_path.open('w') as player_log:
                player = subprocess.Popen(
                    command, stdout=player_log, stderr=subprocess.STDOUT,
                    start_new_session=True)
                deadline = time.monotonic() + duration / rate + 30
                completed = None
                next_resource_sample = 0
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if time.monotonic() >= next_resource_sample:
                        current_rss = rss_kib(os.getpid())
                        current_player_rss = rss_kib(player.pid)
                        if current_rss is not None:
                            peak_rss = max(peak_rss or 0, current_rss)
                        if current_player_rss is not None:
                            player_peak_rss = max(
                                player_peak_rss or 0, current_player_rss)
                        next_resource_sample = time.monotonic() + 0.1
                    if player.poll() is not None:
                        if completed is None:
                            completed = time.monotonic()
                        quiet_seconds = time.monotonic() - max(
                            completed, last_message_at)
                        all_received = all(
                            topic['received_count'] >= topic['count']
                            for topic in topics.values())
                        # A fast player can exit while a slower checker still
                        # has queued samples. Drain while callbacks progress;
                        # a fixed delay after process exit falsely reports loss.
                        if ((all_received and quiet_seconds >= 0.2)
                                or quiet_seconds >= 2):
                            break
                else:
                    report['errors'].append(
                        'playback exceeded duration/rate + 30s')
                stop_player(player)
            report['player_exit_code'] = player.returncode
            report['player_log'] = str(log_path)
            player_output = log_path.read_text()
            report['player_log_tail'] = player_output[-6000:]
            player_errors = [line for line in player_output.splitlines()
                             if re.search(r'\[(?:ERROR|FATAL)\]', line)]
            report['player_errors'] = player_errors[:20]
            if player_errors:
                report['errors'].append('rosbag player logged an error')
            if player.returncode:
                report['errors'].append('ros2 bag play returned nonzero')
            for name, topic in topics.items():
                topic['timing'] = {
                    key: values.report() for key, values in timing[name].items()}
                topic['received_fields_sha256'] = (
                    actual_digests[name].hexdigest())
                if topic['received_count'] != topic['count']:
                    topic['errors'].append(
                        'received {} of {} expected messages'.format(
                            topic['received_count'], topic['count']))
                if (topic['received_fields_sha256']
                        != topic['expected_fields_sha256']):
                    topic['errors'].append(
                        'ordered semantic field digest differs')
                if topic['errors']:
                    report['errors'].append(
                        'topic verification failed: ' + name)
    except Exception as error:
        report['errors'].append(str(error))
    finally:
        stop_player(player)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        elapsed = time.monotonic() - started
        child_end = resource.getrusage(resource.RUSAGE_CHILDREN)
        report['runtime'] = {
            'wall_seconds': elapsed,
            'subscriber_cpu_seconds': time.process_time() - cpu_start,
            'player_cpu_seconds': (
                child_end.ru_utime + child_end.ru_stime
                - child_start.ru_utime - child_start.ru_stime),
            'subscriber_rss_before_kib': own_rss,
            'subscriber_rss_peak_kib': peak_rss,
            'subscriber_rss_after_cleanup_kib': rss_kib(os.getpid()),
            'player_rss_peak_kib': player_peak_rss,
            'rss_sample_period_seconds': 0.1,
            'throughput_scope': 'whole run including 2s startup and drain',
            'messages_per_wall_second': (
                sum(topic['received_count'] for topic in topics.values()) / elapsed),
        }
        report['timing_policy'] = {
            'publisher_schedule_drift':
                'DDS publication interval minus bag interval/rate, anchored per topic',
            'middleware_receive_delay':
                'RMW receive minus DDS source on this host; includes middleware queues',
            'executor_dispatch':
                'callback wall time minus RMW receive timestamp on this shared host clock',
            'unavailable': 'nonpositive or incompatible RMW clocks are excluded',
            'historical_bag_timestamp_used_for_latency': False,
            'completion_drain':
                'until all expected samples and 0.2s quiet, or 2s inactivity; bounded by deadline',
        }
        report['original_sha256_after'] = {
            str(p): check_bags.digest(p) for p in original_files}
        if report['original_sha256_after'] != original_hashes:
            report['errors'].append('an original bag file changed')
    report['passed'] = not report['errors']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_directory', type=Path)
    parser.add_argument('--rate', type=float, default=0.25)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--reliability', choices=('reliable', 'best_effort'),
                        default='reliable')
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('--rate must be positive and finite')
    directory = args.bag_directory.resolve()
    # Keep logs beside the report, never inside the source recording.
    workdir = args.report.parent / (args.report.stem + '-playback')
    try:
        verify_output_paths(
            directory, args.report, workdir,
            workdir / 'playback-qos.yaml', workdir / 'rosbag-play.log')
    except ValueError as error:
        parser.error(str(error))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message
    workdir.mkdir(parents=True, exist_ok=True)
    try:
        report = replay(directory, args.rate, workdir, get_message,
                        deserialize_message, reliability=args.reliability)
    except Exception as error:
        report = {'bag_directory': str(directory), 'passed': False,
                  'errors': [str(error)],
                  'hardware_interoperability_verified': False}
    encoded = json.dumps(report, indent=2) + '\n'
    args.report.write_text(encoded)
    print(encoded)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
