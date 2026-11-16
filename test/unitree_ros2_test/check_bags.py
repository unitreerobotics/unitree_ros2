#!/usr/bin/env python3
"""Validate external Unitree telemetry recordings without changing or replaying originals.

Run in a sourced ROS environment. Defaults to offline reads. --ros additionally
publishes only telemetry on private remapped topics and requires loopback-only
Cyclone DDS configuration. No robot command topic is published.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sqlite3
import struct
import sys
import time

TELEMETRY = {
    'unitree_go/msg/LowState',
    'unitree_go/msg/SportModeState',
    'unitree_go/msg/WirelessController',
    'unitree_hg/msg/LowState',
}
PRIMITIVES = {
    'boolean': ('?', 1), 'bool': ('?', 1),
    'byte': ('B', 1), 'octet': ('B', 1), 'char': ('B', 1),
    'int8': ('b', 1), 'uint8': ('B', 1),
    'int16': ('h', 2), 'uint16': ('H', 2),
    'int32': ('i', 4), 'uint32': ('I', 4),
    'int64': ('q', 8), 'uint64': ('Q', 8),
    'float': ('f', 4), 'float32': ('f', 4),
    'double': ('d', 8), 'float64': ('d', 8),
}


def canonical_type(name):
    parts = name.split('/')
    return '/'.join((parts[0], 'msg', parts[1])) if len(parts) == 2 else name


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


class CDR:
    """Independent, bounds-checked XCDR1 decoder for ROS message field metadata."""

    def __init__(self, data, get_message):
        self.data = bytes(data)
        if len(data) < 4 or data[:2] not in (b'\x00\x00', b'\x00\x01'):
            raise ValueError('expected XCDR1 CDR encapsulation (big/little endian)')
        if data[2] != 0 or data[3] & 0xfc:
            raise ValueError('unsupported CDR encapsulation options')
        self.order = '<' if data[1] else '>'
        self.pos = 4
        self.get_message = get_message

    def primitive(self, kind):
        fmt, width = PRIMITIVES[kind]
        self.pos += (-(self.pos - 4)) % width
        if self.pos + width > len(self.data):
            raise ValueError('truncated CDR at byte {}'.format(self.pos))
        result = struct.unpack_from(self.order + fmt, self.data, self.pos)[0]
        if kind in ('boolean', 'bool') and self.data[self.pos] not in (0, 1):
            raise ValueError('invalid CDR boolean at byte {}'.format(self.pos))
        self.pos += width
        return result

    def field(self, kind):
        array = re.fullmatch(r'(.+)\[(\d+)\]', kind)
        if array:
            if array[1] in PRIMITIVES and array[1] not in ('boolean', 'bool'):
                fmt, width = PRIMITIVES[array[1]]
                self.pos += (-(self.pos - 4)) % width
                count = int(array[2])
                if self.pos + width * count > len(self.data):
                    raise ValueError('truncated CDR at byte {}'.format(self.pos))
                result = struct.unpack_from(self.order + str(count) + fmt,
                                            self.data, self.pos)
                self.pos += width * count
                return list(result)
            return [self.field(array[1]) for _ in range(int(array[2]))]
        sequence = re.fullmatch(r'(?:bounded_)?sequence<(.+)>', kind)
        if sequence:
            element, _, bound = sequence[1].partition(',')
            count = self.primitive('uint32')
            if count > len(self.data) or (bound and count > int(bound.strip())):
                raise ValueError('invalid CDR sequence length {}'.format(count))
            return [self.field(element.strip()) for _ in range(count)]
        if kind.startswith('string'):
            count = self.primitive('uint32')
            if count < 1 or self.pos + count > len(self.data):
                raise ValueError('invalid CDR string length')
            raw = self.data[self.pos:self.pos + count]
            self.pos += count
            if raw[-1] != 0:
                raise ValueError('CDR string lacks null terminator')
            bound = re.fullmatch(r'string<=(\d+)', kind)
            if bound and count - 1 > int(bound[1]):
                raise ValueError('CDR string exceeds declared bound')
            return raw[:-1].decode('utf-8')
        if kind in PRIMITIVES:
            return self.primitive(kind)
        return self.message(self.get_message(canonical_type(kind)))

    def message(self, cls):
        return {name: self.field(kind)
                for name, kind in cls.get_fields_and_field_types().items()}


def plain(value):
    if hasattr(value, 'get_fields_and_field_types'):
        return {name: plain(getattr(value, name))
                for name in value.get_fields_and_field_types()}
    if isinstance(value, (str, bool, int, float)):
        return value
    if hasattr(value, 'tolist'):
        return plain(value.tolist())
    return [plain(item) for item in value]


def equal(left, right, location='message'):
    if isinstance(left, dict):
        if left.keys() != right.keys():
            raise AssertionError(location + ': fields differ')
        for key in left:
            equal(left[key], right[key], location + '.' + key)
    elif isinstance(left, list):
        if len(left) != len(right):
            raise AssertionError(location + ': lengths differ')
        for i, (a, b) in enumerate(zip(left, right)):
            equal(a, b, '{}[{}]'.format(location, i))
    elif isinstance(left, float) and math.isnan(left):
        if not math.isnan(right):
            raise AssertionError(location + ': NaN changed')
    elif isinstance(left, float) and left == 0 and right == 0:
        if math.copysign(1, left) != math.copysign(1, right):
            raise AssertionError(location + ': zero sign changed')
    elif left != right:
        raise AssertionError('{}: {!r} != {!r}'.format(location, left, right))


def open_sqlite(path):
    for suffix in ('-wal', '-journal'):
        sidecar = Path(str(path) + suffix)
        if sidecar.exists() and sidecar.stat().st_size:
            raise ValueError('requires a finalized SQLite bag without active WAL/journal')
    connection = sqlite3.connect(path.resolve().as_uri() + '?mode=ro&immutable=1', uri=True)
    connection.row_factory = sqlite3.Row
    return connection


def sqlite_inventory(path):
    with open_sqlite(path) as connection:
        tables = {row[0] for row in connection.execute(
            "SELECT name FROM sqlite_master WHERE type='table'")}
        schema = dict(connection.execute('SELECT * FROM schema').fetchone()) \
            if 'schema' in tables else {}
        counts = {row[0]: row[1:] for row in connection.execute(
            'SELECT topic_id, count(*), min(timestamp), max(timestamp) '
            'FROM messages GROUP BY topic_id')}
        topics = []
        for row in connection.execute('SELECT * FROM topics'):
            topic = dict(row)
            topic.pop('offered_qos_profiles', None)
            count, start, end = counts.get(topic['id'], (0, None, None))
            topic.update(count=count, first_timestamp_ns=start,
                         last_timestamp_ns=end)
            topics.append(topic)
        starts = [values[1] for values in counts.values() if values[1] is not None]
        ends = [values[2] for values in counts.values() if values[2] is not None]
        start, end = (min(starts), max(ends)) if starts else (None, None)
    return {'schema': schema, 'topics': topics,
            'duration_ns': end - start if start is not None else 0}


def rows_sqlite(path, topics):
    selected = {t['id']: t for t in topics if canonical_type(t['type']) in TELEMETRY}
    with open_sqlite(path) as connection:
        # Read only selected telemetry. Request/command payloads are never replayed.
        for topic_id in selected:
            for stamp, data in connection.execute(
                    'SELECT timestamp, data FROM messages WHERE topic_id=? '
                    'ORDER BY timestamp, id', (topic_id,)):
                yield selected[topic_id], stamp, data


def open_rosbag(path):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    return reader


def mcap_inventory(path):
    reader = open_rosbag(path)
    topics = {
        t.name: {
            'name': t.name, 'type': t.type,
            'serialization_format': t.serialization_format,
            'count': 0, 'first_timestamp_ns': None, 'last_timestamp_ns': None,
        }
        for t in reader.get_all_topics_and_types()
    }
    start = end = None
    while reader.has_next():
        name, _, stamp = reader.read_next()
        topic = topics[name]
        topic['count'] += 1
        topic['first_timestamp_ns'] = stamp if topic['first_timestamp_ns'] is None \
            else min(stamp, topic['first_timestamp_ns'])
        topic['last_timestamp_ns'] = stamp if topic['last_timestamp_ns'] is None \
            else max(stamp, topic['last_timestamp_ns'])
        start = stamp if start is None else min(start, stamp)
        end = stamp if end is None else max(end, stamp)
    return {'schema': {}, 'topics': list(topics.values()),
            'duration_ns': end - start if start is not None else 0}


def rows_mcap(path, topics):
    selected = {t['name']: t for t in topics if canonical_type(t['type']) in TELEMETRY}
    reader = open_rosbag(path)
    while reader.has_next():
        name, data, stamp = reader.read_next()
        if name in selected:
            yield selected[name], stamp, data


def validate_ros(samples, get_message):
    import rclpy
    from rclpy.qos import QoSProfile
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
        raise RuntimeError('--ros requires RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')
    uri = os.environ.get('CYCLONEDDS_URI', '')
    if not re.search(r'<NetworkInterface\b[^>]*\bname=[\"\']lo[\"\']', uri):
        raise RuntimeError('--ros requires explicit inline loopback-only Cyclone DDS URI')
    if re.search(r'<NetworkInterface\b[^>]*\bname=[\"\'](?!lo[\"\'])', uri):
        raise RuntimeError('--ros refuses a non-loopback interface')
    rclpy.init()
    node = rclpy.create_node('unitree_bag_regression')
    checked = 0
    try:
        for index, (typename, message) in enumerate(samples.items()):
            received = []
            topic = '/unitree_regression/telemetry/sample_{}'.format(index)
            cls = get_message(typename)
            subscription = node.create_subscription(
                cls, topic, lambda msg: received.append(plain(msg)), QoSProfile(depth=10))
            publisher = node.create_publisher(cls, topic, QoSProfile(depth=10))
            deadline = time.monotonic() + 10
            while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if publisher.get_subscription_count() < 1:
                raise AssertionError('loopback subscriber discovery timed out: ' + typename)
            publisher.publish(message)
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not received:
                raise AssertionError('loopback subscriber received no sample: ' + typename)
            equal(plain(message), received[0])
            checked += 1
            node.destroy_subscription(subscription)
            node.destroy_publisher(publisher)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return checked


def check_trailer(typename, raw, consumed):
    """Accept only known XCDR1 alignment or the observed legacy sport trailer."""
    trailing = bytes(raw[consumed:])
    alignment = 0 < len(trailing) <= 3 and len(raw) % 4 == 0
    legacy = (typename == 'unitree_go/msg/SportModeState'
              and len(trailing) == 280 and not any(trailing))
    if trailing and not (alignment or legacy):
        raise ValueError('unexplained CDR trailer: {} bytes'.format(len(trailing)))
    return trailing, alignment


def validate(path, get_message, deserialize_message, serialize_message, limit):
    storage = 'sqlite3' if path.suffix == '.db3' else 'mcap'
    result = {'path': str(path), 'storage': storage,
              'sha256_before': digest(path), 'errors': [], 'validated_messages': 0}
    result.update(sqlite_inventory(path) if storage == 'sqlite3' else mcap_inventory(path))
    metadata = path.parent / 'metadata.yaml'
    if metadata.exists():
        import yaml
        result['metadata'] = yaml.safe_load(metadata.read_text())
    classes = {}
    for topic in result['topics']:
        typename = canonical_type(topic['type'])
        topic['required_telemetry'] = typename in TELEMETRY
        try:
            classes[typename] = get_message(typename)
            topic['definition_available'] = True
        except (ImportError, AttributeError, ValueError) as error:
            topic['definition_available'] = False
            topic['unresolved_definition'] = str(error)
            if topic['required_telemetry'] and topic['count']:
                result['errors'].append('missing required definition: ' + typename)
        topic['validated_messages'] = 0
        topic['trailing_byte_counts'] = {}
    iterator = rows_sqlite(path, result['topics']) if storage == 'sqlite3' \
        else rows_mcap(path, result['topics'])
    samples = {}
    for topic, stamp, raw in iterator:
        typename = canonical_type(topic['type'])
        if typename not in classes or (limit and topic['validated_messages'] >= limit):
            continue
        try:
            if topic['serialization_format'] != 'cdr':
                raise ValueError('unsupported serialization: ' + topic['serialization_format'])
            cls = classes[typename]
            decoder = CDR(raw, get_message)
            expected = decoder.message(cls)
            # XCDR1 alignment bytes are unspecified, including Fast DDS's
            # final alignment padding in some Humble recordings.
            trailing, alignment_padding = check_trailer(typename, raw, decoder.pos)
            message = deserialize_message(bytes(raw), cls)
            equal(expected, plain(message))
            serialized = serialize_message(message)
            again = CDR(serialized, get_message)
            equal(expected, again.message(cls))
            equal(expected, plain(deserialize_message(serialized, cls)))
            ros_padding = serialized[again.pos:]
            if ros_padding and not (len(ros_padding) <= 3
                                    and len(serialized) % 4 == 0):
                raise AssertionError('ROS serialization emitted unexplained trailer')
            topic['trailing_byte_counts'][str(len(trailing))] = \
                topic['trailing_byte_counts'].get(str(len(trailing)), 0) + 1
            if alignment_padding and any(trailing):
                topic['nonzero_alignment_padding_count'] = \
                    topic.get('nonzero_alignment_padding_count', 0) + 1
            topic['validated_messages'] += 1
            result['validated_messages'] += 1
            samples.setdefault(typename, message)
            # Original bag timestamps remain untouched; maintain a reproducible digest
            # of timestamps and full serialized payloads in source order per topic.
            if '_digest' not in topic:
                topic['_digest'] = hashlib.sha256()
            topic['_digest'].update(struct.pack('<q', stamp) + bytes(raw))
            if '_ros_digest' not in topic:
                topic['_ros_digest'] = hashlib.sha256()
            topic['_ros_digest'].update(struct.pack('<q', stamp) + bytes(serialized))
            if '_fields_digest' not in topic:
                topic['_fields_digest'] = hashlib.sha256()
            semantic = json.dumps(plain(message), sort_keys=True,
                                  separators=(',', ':'), allow_nan=True).encode('utf-8')
            topic['_fields_digest'].update(struct.pack('<qQ', stamp, len(semantic)) + semantic)
        except Exception as error:
            result['errors'].append('{} at {}: {}'.format(topic['name'], stamp, error))
            if len(result['errors']) >= 20:
                break
    for topic in result['topics']:
        if '_digest' in topic:
            topic['timestamp_and_payload_sha256'] = topic.pop('_digest').hexdigest()
            topic['reserialized_cdr_sha256'] = topic.pop('_ros_digest').hexdigest()
            topic['semantic_fields_sha256'] = topic.pop('_fields_digest').hexdigest()
    if not result['validated_messages']:
        result['errors'].append('no supported Unitree telemetry samples validated')
    result['sha256_after'] = digest(path)
    if result['sha256_before'] != result['sha256_after']:
        result['errors'].append('source recording changed during validation')
    result['passed'] = not result['errors']
    return result, samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--max-messages-per-topic', type=int, default=0,
                        help='0 validates every selected telemetry sample')
    parser.add_argument('--ros', action='store_true',
                        help='also test remapped loopback telemetry pub/sub')
    args = parser.parse_args()
    paths = []
    for bag in args.bags:
        candidates = sorted(bag.glob('*.db3')) + sorted(bag.glob('*.mcap')) \
            if bag.is_dir() else [bag]
        if not candidates:
            parser.error('no sqlite3/MCAP recording found: ' + str(bag))
        paths.extend(candidates)
    if args.report:
        destination = args.report.resolve()
        for bag in args.bags:
            if bag.is_dir() and (bag.resolve() == destination
                                 or bag.resolve() in destination.parents):
                parser.error('--report must be outside source recording directories')
        protected = paths + [path.parent / 'metadata.yaml' for path in paths
                             if (path.parent / 'metadata.yaml').is_file()]
        for path in protected:
            if (destination == path.resolve()
                    or (destination.exists() and path.samefile(destination))):
                parser.error('--report must not overwrite a source recording')
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message, serialize_message
    report = {'ros_distro': os.environ.get('ROS_DISTRO'), 'recordings': [],
              'ros_subscriber_types_verified': 0,
              'hardware_interoperability_verified': False,
              'cdr_trailer_policy': {
                  'alignment': 'up to three unspecified bytes to a 4-byte boundary',
                  'unitree_go/msg/SportModeState':
                      '280 zero bytes observed in firmware telemetry recordings',
                  'other_trailers': 'rejected as unknown schema data'}}
    samples = {}
    for path in paths:
        try:
            result, messages = validate(path, get_message, deserialize_message,
                                        serialize_message, args.max_messages_per_topic)
            report['recordings'].append(result)
            samples.update(messages)
        except Exception as error:
            report['recordings'].append({'path': str(path), 'passed': False,
                                         'errors': [str(error)]})
    if args.ros:
        try:
            report['ros_subscriber_types_verified'] = validate_ros(samples, get_message)
        except Exception as error:
            report['ros_error'] = str(error)
    report['passed'] = all(r['passed'] for r in report['recordings']) \
        and 'ros_error' not in report
    encoded = json.dumps(report, indent=2, allow_nan=False) + '\n'
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(encoded)
    print(encoded)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
