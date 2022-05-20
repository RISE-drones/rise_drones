#!/usr/bin/env python3
'''SRTL'''

import argparse
import json
import logging
import time
import traceback

import zmq

import dss.auxiliaries

#--------------------------------------------------------------------#
__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.0'
__copyright__ = 'Copyright (c) 2020-2021, RISE'
__status__ = 'development'


#--------------------------------------------------------------------#
_logger = logging.getLogger('dss.app')


#--------------------------------------------------------------------#
class AppClient:
  def __init__(self, app_id, crm_ip, crm_port, dss_id):
    self._app_id = app_id
    self._crm_ip = crm_ip
    self._crm_port = crm_port
    self._dss_id = dss_id

    self._app_ip = crm_ip # HACK? APP is running on the same system as CRM

    self._alive = True
    self._context = zmq.Context()

    # all sockets
    self._app_socket = None #Rep: ANY -> APP
    self._crm_socket = None #Req: APP -> CRM
    self._dss_socket = None #Req: APP -> DSS
    self._info_socket = None #Pub: APP -> ANY

    # commands from ANY to APP
    self._commands = {'heart_beat': self._request_heart_beat,
                      'ping':       self._request_ping}

    # task queue for all scheduled tasks
    self._task_queue = dss.auxiliaries.TaskQueue()
    self._task_queue.start()

#----                                                            ----#
  def main(self):
    self._crm_socket = dss.auxiliaries.zmq.Req(self._context, self._crm_ip, self._crm_port, label='crm')
    self._crm_socket.start_heartbeat(self._app_id)
    self._app_socket = dss.auxiliaries.zmq.Rep(self._context, label=f'app {self._app_id}')
    self._info_socket = dss.auxiliaries.zmq.Pub(self._context, label=f'info {self._app_id}')

    _logger.info('App {app_id} is listening on {ip}:{port}'.format(app_id=self._app_id, ip=self._app_socket.ip, port=self._app_socket.port))

    # register APP
    answer = self._crm_socket.send_and_receive({'fcn': 'register', 'name': 'SRTL', 'desc': 'app_srtl', 'type': 'da', 'id': self._app_id, 'ip': self._app_ip, 'port': self._app_socket.port})
    if dss.auxiliaries.zmq.is_ack(answer):
      # predefined mission
      self._task_queue.add(self.task_getDrone)
      self._task_queue.add(self.task_srtl)
      self._task_queue.add(self.task_releaseDrone)
      self._task_queue.add(self.task_quit)
    else:
      _logger.error(f'register failed: {answer}')
      self._alive = False

    while self._alive:
      try:
        msg = self._app_socket.recv_json()
      except zmq.error.Again:
        continue # timeout: no message received; try again

      msg = json.loads(msg)

      fcn = dss.auxiliaries.zmq.get_fcn(msg)
      if fcn in self._commands:
        try:
          answer = self._commands[fcn](msg)
        except:
          _logger.error(f'unexpected exception\n{traceback.format_exc()}')
          answer = dss.auxiliaries.zmq.nack(fcn, 'unexpected exception')
      else:
        answer = dss.auxiliaries.zmq.nack(fcn, 'request is not supported')

      answer = json.dumps(answer)
      self._app_socket.send_json(answer)

    # unregister APP from CRM
    answer = self._crm_socket.send_and_receive({'fcn': 'unregister', 'id': self._app_id})
    if not dss.auxiliaries.zmq.is_ack(answer):
      _logger.error(f'unregister failed: {answer}')

    # stop task queue
    _logger.debug('waiting for task queue to stop')
    self._task_queue.stop()

    self._app_socket.close()
    self._info_socket.close()
    self._crm_socket.close()
    _logger.debug('~ THE END ~')

#---- APP -> DSS ----------------------------------------------------#
  def is_armed(self, default=True):
    try:
      return self.get_armed()
    except:
      return default

#----                                                            ----#
  def get_armed(self):
    '''The function get_armed requests the current armed state. The
    Drone Safety System replies with a bool indicating the armed
    state.'''
    call = 'get_armed'
    answer = self._dss_socket.send_and_receive({'fcn': call, 'id': self._app_id})
    if not dss.auxiliaries.zmq.is_ack(answer):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    return bool(answer['armed'])

#----                                                            ----#
  def is_idling(self):
    call = 'get_idle'
    answer = self._dss_socket.send_and_receive({'fcn': call, 'id': self._app_id})
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      #raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
      return False
    return answer['idle']

#----                                                            ----#
  def task_srtl(self):
    if not self._alive:
      return

    answer = self._dss_socket.send_and_receive({'fcn': 'photo', 'id': self._app_id, 'cmd': 'continous_photo', 'enable': False, 'publish': 'low', 'period': 0.0})

    answer = self._dss_socket.send_and_receive({'fcn': 'dss_srtl', 'id': self._app_id, 'hover_time': 5})
    if not dss.auxiliaries.zmq.is_ack(answer):
      _logger.error(f'dss_srtl failed: {answer}')
      self._alive = False

    while self.is_armed() and self._alive:
      time.sleep(1.0)
    while not self.is_idling() and self._alive:
      time.sleep(1.0)

#---- APP -> CRM ----------------------------------------------------#
  def task_getDrone(self):
    if not self._alive:
      return

    answer = self._crm_socket.send_and_receive({'fcn': 'get_drone', 'id': self._app_id, 'force': self._dss_id})
    if not dss.auxiliaries.zmq.is_ack(answer):
      _logger.error(f'get_drone failed: {answer}')
      self._alive = False
    else:
      self._dss_socket = dss.auxiliaries.zmq.Req(self._context, answer['ip'], answer['port'], label=f'dss {self._dss_id}')
      self._dss_socket.start_heartbeat(self._app_id)
      self._info_socket.publish('clients', {'clients': [self._dss_id]})

#----                                                            ----#
  def task_releaseDrone(self):
    if not self._alive:
      return

    answer = self._crm_socket.send_and_receive({'fcn': 'release_drone', 'id': self._app_id, 'id_released': self._dss_id})
    if not dss.auxiliaries.zmq.is_ack(answer):
      _logger.error(f'release_drone failed: {answer}')
      self._alive = False

    self._info_socket.publish('clients', {'clients': []})
    if self._dss_socket:
      self._dss_socket.close()
      self._dss_socket = None

#----                                                            ----#
  def task_quit(self):
    self._alive = False

#---- ANYONE TO APP -------------------------------------------------#
  def _request_heart_beat(self, msg:dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)

    # check arguments
    if not all(key in msg for key in ['id']):
      return dss.auxiliaries.zmq.nack(fcn, 'bad arguments: {id} is mandatory')

    id_ = msg['id']
    if id_ != self._app_id:
      return dss.auxiliaries.zmq.nack(fcn, 'wrong id')

    return dss.auxiliaries.zmq.ack(fcn)

#----                                                            ----#
  def _request_ping(self, msg: dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)

    # check arguments
    if not all(key in msg for key in ['id']):
      return dss.auxiliaries.zmq.nack(fcn, 'bad arguments: {id} is mandatory')

    id_ = msg['id']
    if id_ != self._app_id:
      return dss.auxiliaries.zmq.nack(fcn, 'wrong id')

    return dss.auxiliaries.zmq.ack(fcn, {'id': self._app_id, 'info_pub_port': self._info_socket.port})


#--------------------------------------------------------------------#
def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='DSS-APP "SRTL"', allow_abbrev=False, add_help=False)
  parser.add_argument('-h', '--help', action='help', help=argparse.SUPPRESS)
  parser.add_argument('--dss', type=str, help='id of the dss to fly home', required=True)
  parser.add_argument('--id', type=str, help='id of the app instance provided by CRM', required=True)
  parser.add_argument('--ip', type=str, help='ip of the CRM', required=True)
  parser.add_argument('--log', type=str, default='debug', help='logging threshold')
  parser.add_argument('--port', type=int, help='port of the CRM', required=True)
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout')
  args = parser.parse_args()

  subnet = dss.auxiliaries.zmq.get_subnet(port=args.port)
  dss.auxiliaries.logging.configure(f'{args.id}_app_srtl', stdout=args.stdout, rotating=True, loglevel=args.log, subdir=subnet)

  client = AppClient(args.id, args.ip, args.port, args.dss)
  try:
    client.main()
  except KeyboardInterrupt:
    logging.warning('shutdown due to keyboard interrupt')
  except:
    logging.error('unexpected exception\n{}'.format(traceback.format_exc()))


#--------------------------------------------------------------------#
if __name__ == '__main__':
  _main()
