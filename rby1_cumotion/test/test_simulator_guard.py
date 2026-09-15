"""Simulator initialization must never accept a different model or endpoint."""

from copy import deepcopy

import pytest

from rby1_cumotion.prepare_sim import validate_simulator


@pytest.fixture
def simulator():
    return {'State': {'Running': True},
            'Config': {'Image': 'rby1_cumotion_sim:m-v1.2'},
            'NetworkSettings': {'Ports': {
                '50051/tcp': [{'HostIp': '127.0.0.1', 'HostPort': '50051'}]}}}


def test_running_local_simulator(simulator):
    validate_simulator(simulator, 'm_1_2')


@pytest.mark.parametrize('model', ['a_1_2', 'm_1_3'])
def test_model_mismatch(simulator, model):
    with pytest.raises(ValueError):
        validate_simulator(simulator, model)


@pytest.mark.parametrize('bindings', [None, [],
    [{'HostIp': '0.0.0.0', 'HostPort': '50051'}],
    [{'HostIp': '127.0.0.1', 'HostPort': '50052'}],
    [{'HostIp': '127.0.0.1', 'HostPort': '50051'}, {'HostIp': '::', 'HostPort': '50051'}],
])
def test_incorrect_sdk_binding(simulator, bindings):
    simulator['NetworkSettings']['Ports']['50051/tcp'] = bindings
    with pytest.raises(ValueError):
        validate_simulator(simulator, 'm_1_2')


def test_stopped_or_wrong_image(simulator):
    stopped = deepcopy(simulator)
    stopped['State']['Running'] = False
    simulator['Config']['Image'] = 'another_robot:latest'
    for info in (stopped, simulator):
        with pytest.raises(ValueError):
            validate_simulator(info, 'm_1_2')
