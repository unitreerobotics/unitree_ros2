"""Nonzero CDR roundtrips for every installed official Unitree interface."""
import importlib

import pytest
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_parser.definition import (
    AbstractSequence, Array, BasicType, NamespacedType,
)
from rosidl_runtime_py.utilities import get_message


def interfaces():
    for package in ('unitree_api', 'unitree_go', 'unitree_hg'):
        module = importlib.import_module(package + '.msg')
        for name in sorted(dir(module)):
            value = getattr(module, name)
            if isinstance(value, type) and hasattr(value, 'SLOT_TYPES'):
                yield package + '/msg/' + name


def sample_value(slot, seed=1):
    if isinstance(slot, (Array, AbstractSequence)):
        length = slot.size if isinstance(slot, Array) else 3
        return [sample_value(slot.value_type, seed + i) for i in range(length)]
    if isinstance(slot, NamespacedType):
        cls = get_message('/'.join(slot.namespaces) + '/' + slot.name)
        return populate(cls(), seed)
    if isinstance(slot, BasicType):
        if slot.typename == 'boolean':
            return bool(seed % 2)
        if slot.typename in ('float', 'double', 'long double'):
            return seed * 0.25
        if slot.typename.startswith('uint') or slot.typename in ('octet', 'char', 'byte'):
            return seed % 100 + 1
        return -(seed % 100 + 1)
    return 'unitree-test-' + str(seed)


def populate(message, seed=1):
    slots = zip(message.get_fields_and_field_types(), message.SLOT_TYPES)
    for i, (name, slot) in enumerate(slots):
        setattr(message, name, sample_value(slot, seed + i))
    return message


def fields(message):
    if hasattr(message, 'get_fields_and_field_types'):
        return {name: fields(getattr(message, name))
                for name in message.get_fields_and_field_types()}
    if hasattr(message, 'tolist'):
        return fields(message.tolist())
    if isinstance(message, (list, tuple)):
        return [fields(value) for value in message]
    return message


@pytest.mark.parametrize('type_name', list(interfaces()))
def test_cdr_roundtrip(type_name):
    cls = get_message(type_name)
    original = populate(cls())
    encoded = serialize_message(original)
    decoded = deserialize_message(encoded, cls)
    assert fields(decoded) == fields(original)
    # CDR alignment bytes are not semantic fields and differ across serializers.
    assert fields(deserialize_message(serialize_message(decoded), cls)) == fields(original)
