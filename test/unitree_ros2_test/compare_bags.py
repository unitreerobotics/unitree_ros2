#!/usr/bin/env python3
"""Compare recorded CDR with ROS using independently parsed installed .msg files.

Offline only. Reads SQLite bags in stored record order, never publishes messages.
Run once per distribution and compare the per-topic field and timestamp digests.
Original files are opened immutable/read-only. Output must be outside all sources.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import time

import check_bags as bags


def normalized_kind(kind):
    kind = re.sub(r'\bfloat\b', 'float32', kind)
    kind = re.sub(r'\bdouble\b', 'float64', kind)
    kind = re.sub(r'\bboolean\b', 'bool', kind)
    return re.sub(r'(\w+)/(?:msg/)?(\w+)', r'\1/msg/\2', kind)


class Definition:
    def __init__(self, fields):
        self.fields = fields

    def get_fields_and_field_types(self):
        return self.fields


class Definitions:
    """Read the source contract, independently of generated SLOT_TYPES metadata."""

    def __init__(self):
        self.cache = {}
        self.hashes = {}
        self.layouts = {}
        self.raw_fields = {}

    def get(self, typename):
        typename = bags.canonical_type(typename)
        if typename not in self.cache:
            from ament_index_python.packages import get_package_share_directory
            package, _, name = typename.split('/')
            path = Path(get_package_share_directory(package)) / 'msg' / (name + '.msg')
            raw = path.read_bytes()
            self.hashes[typename] = hashlib.sha256(raw).hexdigest()
            fields = {}
            for line in raw.decode().splitlines():
                line = line.split('#', 1)[0].strip()
                if not line:
                    continue
                kind, remainder = line.split(None, 1)
                if '=' in remainder.split()[0]:
                    continue  # Constants are not serialized fields.
                field = remainder.split()[0]
                array = re.fullmatch(r'(.+)\[(.*)\]', kind)
                element = array[1] if array else kind
                if element not in bags.PRIMITIVES and not element.startswith('string'):
                    element = bags.canonical_type(
                        element if '/' in element else package + '/' + element)
                if array:
                    if not array[2] or array[2].startswith('<='):
                        bound = ', ' + array[2][2:] if array[2] else ''
                        kind = 'sequence<' + element + bound + '>'
                    else:
                        kind = element + '[' + array[2] + ']'
                else:
                    kind = element
                fields[field] = kind
            self.cache[typename] = Definition(fields)
        return self.cache[typename]

    def fixed_layout(self, typename):
        """Compile fixed schemas into independent struct layouts and field accessors."""
        if typename in self.layouts:
            return self.layouts[typename]
        formats = []
        expressions = []
        names = []
        position = 0
        wire = ''
        segments = []

        def visit(kind, expression):
            nonlocal position, wire
            array = re.fullmatch(r'(.+)\[(\d+)\]', kind)
            if array:
                for i in range(int(array[2])):
                    visit(array[1], expression + '[{}]'.format(i))
            elif kind in bags.PRIMITIVES:
                fmt, width = bags.PRIMITIVES[kind]
                if fmt == '?':
                    raise ValueError('boolean needs strict byte validation')
                padding = -position % width
                wire += 'x' * padding + fmt
                begin = position + padding
                if segments and segments[-1][1] == begin:
                    segments[-1] = (segments[-1][0], begin + width)
                else:
                    segments.append((begin, begin + width))
                position += padding + width
                formats.append(fmt)
                expressions.append(expression)
                names.append(expression.replace('message.', '', 1))
            elif kind.startswith(('sequence<', 'string')):
                raise ValueError('variable schema')
            else:
                for name, field_type in self.get(kind).fields.items():
                    visit(field_type, expression + '.' + name)

        try:
            visit(typename, 'message')
            namespace = {}
            exec('def values(message):\n    return (' + ','.join(expressions) + ',)\n',
                 {}, namespace)
            slices = ['data[{}:{}]'.format(4 + a, 4 + b) for a, b in segments]
            exec('def raw_fields(data):\n    return ' + '+'.join(slices) + '\n',
                 {}, namespace)
            self.raw_fields[typename] = namespace['raw_fields']
            result = (struct.Struct('<' + wire), struct.Struct('>' + wire),
                      struct.Struct('<' + ''.join(formats)), namespace['values'], names)
        except ValueError:
            result = None
        self.layouts[typename] = result
        return result

    def canonical(self, kind, value):
        """Encode typed fields, including float bits and lengths, without CDR padding."""
        array = re.fullmatch(r'(.+)\[(\d+)\]', kind)
        sequence = re.fullmatch(r'sequence<(.+)>', kind)
        if array or sequence:
            element = array[1] if array else sequence[1].split(',')[0].strip()
            if array and len(value) != int(array[2]):
                raise ValueError('array dimension differs: ' + kind)
            prefix = b'' if array else struct.pack('<I', len(value))
            if element in bags.PRIMITIVES:
                return prefix + struct.pack(
                    '<' + str(len(value)) + bags.PRIMITIVES[element][0], *value)
            return prefix + b''.join(self.canonical(element, v) for v in value)
        if kind in bags.PRIMITIVES:
            return struct.pack('<' + bags.PRIMITIVES[kind][0], value)
        if kind.startswith('string'):
            raw = value.encode('utf-8')
            return struct.pack('<I', len(raw)) + raw
        fields = self.get(kind).fields
        if isinstance(value, dict):
            return b''.join(self.canonical(t, value[n]) for n, t in fields.items())
        return b''.join(self.canonical(t, getattr(value, n)) for n, t in fields.items())


def inspect(path, definitions, get_message, deserialize, serialize, types, limit,
            source_verification='sha256', source_hash=None):
    started = time.monotonic()
    before = path.stat()
    report = {'source': str(path), 'sha256': source_hash,
              'source_verification': source_verification, 'topics': [],
              'errors': [], 'checked': 0}
    if source_verification == 'sha256' and source_hash is None:
        report['sha256'] = bags.digest(path)
    inventory = bags.sqlite_inventory(path)
    for topic in inventory['topics']:
        typename = bags.canonical_type(topic['type'])
        if not (typename.startswith(('unitree_go/', 'unitree_hg/', 'unitree_api/'))
                or typename in types):
            continue
        topic.update(checked=0, errors=[], trailers={}, byte_identical=0,
                     serialization_field_equivalent=0, timestamp_reversals=0,
                     repeated_timestamps=0, adjacent_identical_payloads=0)
        report['topics'].append(topic)
        fields_digest = hashlib.sha256()
        source_digest = hashlib.sha256()
        serialized_digest = hashlib.sha256()
        try:
            cls = get_message(typename)
            definition = definitions.get(typename)
            schema = {n: normalized_kind(t)
                      for n, t in cls.get_fields_and_field_types().items()}
            if schema != definition.fields:
                raise ValueError('installed .msg and generated schema disagree: ' + typename)
            layout = definitions.fixed_layout(typename)
            with bags.open_sqlite(path) as connection:
                previous_stamp = previous_raw = None
                for row_id, stamp, raw in connection.execute(
                        'SELECT id, timestamp, data FROM messages WHERE topic_id=? ORDER BY id',
                        (topic['id'],)):
                    if limit and topic['checked'] >= limit:
                        break
                    try:
                        if topic['serialization_format'] != 'cdr':
                            raise ValueError('expected CDR serialization')
                        decoder = bags.CDR(raw, definitions.get)
                        if layout:
                            little, big, packed, values, names = layout
                            wire = little if decoder.order == '<' else big
                            reference_values = wire.unpack_from(raw, 4)
                            expected = (definitions.raw_fields[typename](raw)
                                        if decoder.order == '<'
                                        else packed.pack(*reference_values))
                            consumed = 4 + wire.size
                        else:
                            reference = decoder.message(definition)
                            expected = definitions.canonical(typename, reference)
                            consumed = decoder.pos
                        bags.check_trailer(typename, raw, consumed)
                        message = deserialize(raw, cls)
                        actual_values = values(message) if layout else None
                        actual = (packed.pack(*actual_values) if layout
                                  else definitions.canonical(typename, message))
                        if actual != expected:
                            if layout:
                                offset = 0
                                for name, fmt in zip(names, packed.format[1:]):
                                    end = offset + struct.calcsize(fmt)
                                    if expected[offset:end] != actual[offset:end]:
                                        raise AssertionError(name + ': typed field bits differ')
                                    offset = end
                            else:
                                bags.equal(reference, bags.plain(message))
                            raise AssertionError('typed field bits differ')
                        encoded = serialize(message)
                        roundtrip = bags.CDR(encoded, definitions.get)
                        if layout:
                            wire = little if roundtrip.order == '<' else big
                            again = (definitions.raw_fields[typename](encoded)
                                     if roundtrip.order == '<'
                                     else packed.pack(*wire.unpack_from(encoded, 4)))
                            end = 4 + wire.size
                        else:
                            again = definitions.canonical(typename, roundtrip.message(definition))
                            end = roundtrip.pos
                        if again != expected:
                            raise AssertionError('reserialized field bits differ')
                        bags.check_trailer(typename, encoded, end)
                        topic['byte_identical'] += int(raw == encoded)
                        topic['serialization_field_equivalent'] += 1
                        trailer = str(len(raw) - consumed)
                        topic['trailers'][trailer] = topic['trailers'].get(trailer, 0) + 1
                        fields_digest.update(struct.pack('<qQ', stamp, len(expected)) + expected)
                        source_digest.update(struct.pack('<qQ', stamp, len(raw)) + raw)
                        serialized_digest.update(struct.pack('<qQ', stamp, len(encoded)) + encoded)
                        topic['timestamp_reversals'] += int(
                            previous_stamp is not None and stamp < previous_stamp)
                        topic['repeated_timestamps'] += int(stamp == previous_stamp)
                        topic['adjacent_identical_payloads'] += int(raw == previous_raw)
                        previous_stamp, previous_raw = stamp, raw
                        topic['checked'] += 1
                        report['checked'] += 1
                    except Exception as error:
                        topic['errors'].append({'row_id': row_id, 'timestamp_ns': stamp,
                                                'error': str(error)})
                        if len(topic['errors']) >= 5:
                            break
        except Exception as error:
            topic['errors'].append({'error': str(error)})
        topic['fields_sha256'] = fields_digest.hexdigest()
        topic['source_records_sha256'] = source_digest.hexdigest()
        topic['serialized_records_sha256'] = serialized_digest.hexdigest()
        topic['complete'] = topic['checked'] == topic['count']
        if topic['errors']:
            report['errors'].append(topic['name'])
    after = path.stat()
    report['source_stats_unchanged'] = (
        (before.st_size, before.st_mtime_ns, before.st_ino)
        == (after.st_size, after.st_mtime_ns, after.st_ino))
    report['source_unchanged'] = report['source_stats_unchanged'] and (
        source_verification == 'stat' or report['sha256'] == bags.digest(path))
    if not report['checked']:
        report['errors'].append('no supported samples validated')
    report['elapsed_seconds'] = time.monotonic() - started
    report['passed'] = not report['errors'] and report['source_unchanged']
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', type=Path, nargs='+')
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--standard-type', action='append', default=[])
    parser.add_argument('--limit', type=int, default=0,
                        help='0 checks all selected messages')
    parser.add_argument('--source-verification', choices=['sha256', 'stat'], default='sha256',
                        help='stat avoids hashing large files; use a read-only mount and '
                        'a separately hashed dataset manifest')
    args = parser.parse_args()
    if args.limit < 0:
        parser.error('--limit must be nonnegative')
    destination = args.report.resolve()
    for path in args.bags:
        if not path.is_file() or path.suffix != '.db3':
            parser.error('expected an existing SQLite .db3 file: ' + str(path))
        originals = [path, path.parent / 'metadata.yaml']
        if (path.parent.resolve() in destination.parents
                or path.resolve() == destination
                or (destination.exists() and any(
                    original.exists() and original.samefile(destination)
                    for original in originals))):
            parser.error('report must be outside source recording directories')
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    from rclpy.utilities import get_rmw_implementation_identifier
    import os
    definitions = Definitions()
    result = {'ros_distro': os.environ.get('ROS_DISTRO'), 'recordings': [],
              'rmw_implementation': get_rmw_implementation_identifier(),
              'hardware_verified': False, 'limit_per_topic': args.limit}
    seen = set()
    for path in args.bags:
        info = path.stat()
        checksum = (bags.digest(path) if args.source_verification == 'sha256'
                    else (info.st_dev, info.st_ino))
        if checksum in seen:
            continue
        seen.add(checksum)
        try:
            item = inspect(path, definitions, get_message, deserialize_message,
                           serialize_message, args.standard_type, args.limit,
                           args.source_verification,
                           checksum if args.source_verification == 'sha256' else None)
        except Exception as error:
            item = {'source': str(path), 'passed': False, 'errors': [str(error)]}
        result['recordings'].append(item)
        print(json.dumps({'source': str(path), 'checked': item.get('checked', 0),
                          'passed': item['passed']}), flush=True)
        result['definition_sha256'] = definitions.hashes
        result['passed'] = all(r['passed'] for r in result['recordings'])
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2) + '\n')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
