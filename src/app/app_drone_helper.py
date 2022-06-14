#!/usr/bin python3

import argparse
import json
import logging
import threading
import time
import traceback

import zmq

import dss.auxiliaries
import dss.client

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.0.0'
__copyright__ = 'Copyright (c) 2021, RISE'
__status__ = 'development'

class DroneHelper:
  def __init__(self, id, crm_ip, crm_port):
    self._logger = logging.getLogger(__name__)

    self._context = zmq.Context()
    self._id = id
    self._crm_ip = crm_ip
    self._crm_socket = dss.auxiliaries.zmq.Req(self._context, crm_ip, crm_port)
    self._rep_socket = dss.auxiliaries.zmq.Rep(self._context)
    self._pub_data = dss.auxiliaries.zmq.Pub(self._context)
    self._pub_info = dss.auxiliaries.zmq.Pub(self._context)
    self._alive = True

    self._commands = {'ping':            self._request_ping,
                      'release_dss':     self._request_release_dss,
                      'set_pattern':     self._request_set_pattern,
                      'follow_me':       self._request_follow_me,
                      'photo_stream':    self._request_photo_stream}

    # start main thread
    self._mutex = threading.Lock()
    self._main_thread = threading.Thread(target=self._main, daemon=False)
    self._main_thread.start()

  @property
  def alive(self):
    '''checks if CRM is alive'''
    return self._alive

  def register(self):
    self._crm_socket.send_and_receive({'fcn': 'register', 'name': 'DroneHelper', 'desc': 'this app takes care of active drones on behalf of crm', 'type': 'da', 'id': self._id, 'ip': self._crm_ip, 'port': self._rep_socket.port})

  def unregister(self):
    self._crm_socket.send_and_receive({'fcn': 'unregister', 'id': self._id})

  def _main(self):
    self._logger.info('DroneHelper {id} is listening on {ip}:{port}'.format(id=self._id, ip=self._crm_ip, port=self._rep_socket.port))
    self.register()

    while self._alive:
      try:
        msg = self._rep_socket.recv_json()
      except zmq.error.Again:
        continue # timeout: no message received; try again

      msg = json.loads(msg)

      fcn = dss.auxiliaries.zmq.get_fcn(msg)
      if fcn in self._commands:
        try:
          with self._mutex:
            answer = self._commands[fcn](msg)
        except:
          self._logger.error(f'unexpected exception\n{traceback.format_exc()}')
          answer = dss.auxiliaries.zmq.nack(fcn, 'unexpected exception')
      else:
        answer = dss.auxiliaries.zmq.nack(fcn, 'request is not supported')

      answer = json.dumps(answer)
      self._rep_socket.send_json(answer)

    self._main_thread = None

  def _request_ping(self, msg: dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)

    # check arguments
    if 'id' not in msg:
      return dss.auxiliaries.zmq.nack(fcn, 'bad arguments: {id} is mandatory')

    return {'fcn': 'ack', 'call': fcn, 'id': self._id}

  def _request_release_dss(self, msg:dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    return dss.auxiliaries.zmq.nack(fcn, 'not implemented')

  def _request_set_pattern(self, msg:dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    return dss.auxiliaries.zmq.nack(fcn, 'not implemented')

  def _request_follow_me(self, msg:dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    return dss.auxiliaries.zmq.nack(fcn, 'not implemented')

  def _request_photo_stream(self, msg:dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    return dss.auxiliaries.zmq.nack(fcn, 'not implemented')

def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='DSS-APP "DroneHelper"', allow_abbrev=False)
  parser.add_argument('--id', type=str, help='id of the DroneHelper instance provided by CRM')
  parser.add_argument('--ip', type=str, help='ip of the CRM')
  parser.add_argument('--port', type=int, help='port of the CRM')
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout')
  args = parser.parse_args()

  if not all(arg for arg in [args.id, args.ip, args.port]):
    raise Exception('bad arguments')

  dss.auxiliaries.logging.configure(f'app-tyrapp-{args.id}', args.stdout)

  try:
    client = DroneHelper(args.id, args.ip, args.port)
    while client.alive:
      time.sleep(3.0)
  except KeyboardInterrupt:
    logging.warning('shutdown due to keyboard interrupt')
  except:
    logging.error('unexpected exception\n{}'.format(traceback.format_exc()))


if __name__ == '__main__':
  _main()
