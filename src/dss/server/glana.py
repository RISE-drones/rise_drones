'''GLANA Control Service'''

import datetime
import json
import logging
import threading
import time

import zmq

import dss.auxiliaries

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.2.0'
__copyright__ = 'Copyright (c) 2019-2021, RISE'
__status__ = 'development'

class Glana:
  def __init__(self, context, address):
    self._logger = logging.getLogger(__name__)

    self._address = address
    self._context = context
    self._mutex = threading.Lock()
    self._socket = self._context.socket(zmq.REQ)
    self._connected = False
    self._recording = False

  @property
  def connected(self):
    '''Checks if the glana service is connected'''
    return self._connected

  @property
  def recording(self):
    '''Checks if the glana service is recording'''
    return self._recording

  def connect(self):
    '''Connects to GLANA'''
    if self._connected:
      self._logger.info('GLANA is already connected on %s', self._address)
      return True

    # connect to GLANA Control
    self._socket.connect(self._address)
    self._socket.RCVTIMEO = 500 # in milliseconds

    if self.up():
      self._logger.info('Connection to GLANA established on %s', self._address)
      self._connected = True
    else:
      self._logger.error('Connection to GLANA failed on %s', self._address)
    return self._connected

  def disconnect(self):
    '''Disconnects from GLANA'''
    self._connected = False

  def up(self):
    '''Making sure GLANA control is alive'''
    self._logger.info("Making sure GLANA Control is alive...")
    call = 'up'
    msg = {'fcn': call, 'arg': ''}
    answer = self.send_and_receive(msg)
    return self.is_ack(answer, call)

  def rec_ok(self):
    '''Making sure GLANA control is recording'''
    self._logger.info("Making sure GLANA Control is recording...")
    call = 'rec_ok'
    msg = {'fcn': call, 'arg': ''}
    answer = self.send_and_receive(msg)
    return self.is_ack(answer, call)

  def autogain(self):
    '''Asking GLANA control to adjust the gain'''
    self._logger.info("Asking GLANA control to adjust the gain...")
    call = 'autogain'
    msg = {'fcn': call, 'arg': ''}
    answer = self.send_and_receive(msg)
    return self.is_ack(answer, call)

  def start_rec(self):
    '''Start recording and wait for first immage'''
    if self.recording:
      self._logger.error('Connection to GLANA failed on %s', self._address)
      return True

    path_str = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    call = 'start_rec'
    msg = {'fcn': call, 'arg': path_str}
    answer = self.send_and_receive(msg)
    if not self.is_ack(answer, call):
      raise dss.auxiliaries.exception.AbortTask()

    attempt = 10
    while not self.recording and attempt > 0:
      attempt = attempt - 1
      time.sleep(0.2)
      if self.rec_ok():
        self._recording = True
        self._logger.info("GLANA control is recording")
    return self.recording

  def stop_rec(self):
    if not self.recording:
      return True

    call = 'stop_rec'
    msg = {'fcn': call, 'arg': ''}
    answer = self.send_and_receive(msg)
    if not self.is_ack(answer, call):
      raise dss.auxiliaries.exception.AbortTask()

  def is_ack(self, answer, call):
    if answer and 'fcn' in answer and 'arg' in answer:
      return answer['fcn'] == 'ack' and answer['arg'] == call
    return False

  def send_and_receive(self, msg: dict):
    '''Sends a request message to GLANA and returns the answer.'''
    with self._mutex:
      json_msg = json.dumps(msg)

      try:
        self._socket.send_json(json_msg)
        json_reply = self._socket.recv_json()
      except zmq.error.Again:
        logging.warning("glana-service isn't responding to request %s", str(msg))
        reply = None
      except zmq.error.ZMQError:
        logging.error("Lost connection to glana-service")
        reply = None
      else:
        reply = json.loads(json_reply)

    return reply
