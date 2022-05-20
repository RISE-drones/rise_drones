import dss.auxiliaries
import pytest


def test_ack():
  answer = {'fcn': 'ack', 'call': 'heart_beat'}
  assert dss.auxiliaries.zmq.is_ack(answer)
  assert not dss.auxiliaries.zmq.is_nack(answer)

def test_valid_ip():
  assert dss.auxiliaries.zmq.valid_ip('localhost', localhost=True)
  assert not dss.auxiliaries.zmq.valid_ip('localhost', localhost=False)
  assert dss.auxiliaries.zmq.valid_ip('*', asterisk=True)
  assert not dss.auxiliaries.zmq.valid_ip('*', asterisk=False)

def test_get_nack_reason():
  with pytest.raises(dss.auxiliaries.exception.InputError):
    dss.auxiliaries.zmq.get_nack_reason({'fcn': 'ack', 'call': 'heart_beat'})

  answer = {'fcn': 'nack', 'call': 'heart_beat'}
  assert dss.auxiliaries.zmq.get_nack_reason(answer) == 'unknown nack reason'
  answer = {'fcn': 'nack', 'call': 'heart_beat', 'description': 'some reason'}
  assert dss.auxiliaries.zmq.get_nack_reason(answer) == answer['description']
