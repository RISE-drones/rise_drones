#!/usr/bin/env python3

'''
APP "app_selfie"

This app connects to CRM and receives an app_id.
'''

import argparse
import json
import logging
import sys
import threading
import time
import traceback

import zmq

import dss.auxiliaries
import dss.client

#--------------------------------------------------------------------#

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '0.2.0'
__copyright__ = 'Copyright (c) 2022, RISE'
__status__ = 'development'

#--------------------------------------------------------------------#

_logger = logging.getLogger('dss.selfie')
_context = zmq.Context()

#--------------------------------------------------------------------#
class Selfie():
  def __init__(self, app_ip, app_id, crm, drone_id, owner):
    # Create Client object
    self.drone = dss.client.Client(timeout=2000, exception_handler=None, context=_context)
    self._drone_id_arg = drone_id

    # Create CRM object
    self.crm = dss.client.CRM(_context, crm, app_name='app_selfie.py', desc='Selfie application', app_id=app_id)

    self._alive = True
    self._dss_data_thread = None
    self._dss_data_thread_active = False
    self._dss_info_thread = None
    self._dss_info_thread_active = False

    self._task_queue = dss.auxiliaries.TaskQueue(exception_handler=None)
    self._task_queue.start()

    # Find the VPN ip of host machine
    self._app_ip = app_ip
    auto_ip = dss.auxiliaries.zmq.get_ip()
    if auto_ip != app_ip:
      _logger.warning('Automatic get ip function and given ip does not agree: %s vs %s', auto_ip, app_ip)

    # Set the owner, it shall be the process who launched app_selfie
    self._owner = owner
    # The application sockets
    # Use ports depending on subnet used to pass RISE firewall
    # Rep: ANY -> APP
    self._app_socket = dss.auxiliaries.zmq.Rep(_context, label='app', min_port=self.crm.port, max_port=self.crm.port+50)
    # Pub: APP -> ANY
    self._info_socket = dss.auxiliaries.zmq.Pub(_context, label='info', min_port=self.crm.port, max_port=self.crm.port+50)

    # Start the app reply thread
    self._app_reply_thread = threading.Thread(target=self._main_app_reply, daemon=True)
    self._app_reply_thread.start()

    # Register with CRM (self.crm.app_id is first available after the register call)
    answer = self.crm.register(self._app_ip, self._app_socket.port)
    self._app_id = answer['id']

    # All nack reasons raises exception, registreation is successful
    _logger.info('App %s listening on %s:%s', self.crm.app_id, self._app_socket.ip, self._app_socket.port)
    _logger.info('App_selfie registered with CRM: %s', self.crm.app_id)

    # Update socket labels with received id
    self._app_socket.add_id_to_label(self.crm.app_id)
    self._info_socket.add_id_to_label(self.crm.app_id)

    # Supported commands from ANY to APP
    self._commands = {'push_dss':     {'request': self._request_push_dss}, # Not implemented
                      'get_info':     {'request': self._request_get_info},
                      'follow_her':   {'request': self._request_follow_her},
                      'set_pattern':  {'request': self._request_set_pattern}}

    # Default flight pattern
    self._pattern = {'pattern': 'above', 'rel_alt': 15, "heading": 'course'}

#--------------------------------------------------------------------#
  @property
  def alive(self):
    '''checks if application is alive'''
    return self._alive

  @property
  def app_id(self):
    '''application id'''
    return self._app_id


#--------------------------------------------------------------------#
  # This method runs on KeyBoardInterrupt, time to release resources and clean up.
  # Disconnect connected drones and unregister from crm, close ports etc..
  def kill(self):
    _logger.info('Closing down...')
    self._alive = False
    # Kill info and data thread
    self._dss_info_thread_active = False
    self._dss_data_thread_active = False
    self._info_socket.close()
    self._task_queue.stop()

    # Unregister APP from CRM
    _logger.info('Unregister from CRM')
    answer = self.crm.unregister()
    if not dss.auxiliaries.zmq.is_ack(answer):
      _logger.error('Unregister failed: %s', answer)
    _logger.info('CRM socket closed')

    # Disconnect drone if drone is alive
    if self.drone.alive:
      #wait until other DSS threads finished
      time.sleep(0.5)
      _logger.info('Closing socket to DSS')
      self.drone.close_dss_socket()

    _logger.debug('~ THE END ~')

#--------------------------------------------------------------------#
# Application reply thread
  def _main_app_reply(self):
    _logger.info('Reply socket is listening on: %s', self._app_socket.port)
    while self.alive:
      try:
        msg = self._app_socket.recv_json()
        msg = json.loads(msg)
        fcn = msg['fcn'] if 'fcn' in msg else ''

        if fcn in self._commands:
          request = self._commands[fcn]['request']
          answer = request(msg)
        else :
          answer = dss.auxiliaries.zmq.nack(msg['fcn'], 'Request not supported')
        answer = json.dumps(answer)
        self._app_socket.send_json(answer)
      except:
        pass
    self._app_socket.close()
    _logger.info('Reply socket closed, thread exit')

#--------------------------------------------------------------------#
# Ack nack helper
	# Is message from owner?
  def from_owner(self, msg) -> bool:
    return msg['id'] == self._owner
#--------------------------------------------------------------------#
# Genereal application reply API
#--------------------------------------------------------------------#
# Request: 'push_dss'
  def _request_push_dss(self, msg):
    answer = dss.auxiliaries.zmq.nack(msg['fcn'], 'Not implemented')
    return answer

#--------------------------------------------------------------------#
# Request: 'get_info'
  def _request_get_info(self, msg):
    answer = dss.auxiliaries.zmq.ack(msg['fcn'])
    answer['id'] = self.crm.app_id
    answer['info_pub_port'] = self._info_socket.port
    answer['data_pub_port'] = None
    return answer

#--------------------------------------------------------------------#
# Specific APP Selfie reply API
#--------------------------------------------------------------------#
  # Request: 'follow_her'
  def _request_follow_her(self, msg):
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Check nack reasons
    if not self.from_owner(msg) and msg['id'] != "GUI":
      descr = 'Requester ({}) is not the APP owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    # Accept
    else:
      enable = msg['enable']
      if not enable:
        self.drone.disable_follow_stream()
        self.drone.abort()
        self._alive = False
      else:
        target_drone = msg['target_id']
        self._task_queue.add(self._task_follow_her, target_drone)
      answer = dss.auxiliaries.zmq.ack(fcn)
    return answer

#--------------------------------------------------------------------#
  # task follow her
  def _task_follow_her(self, her):
    her_ip = ""
    her_pub_port = 0
    # Get rep port of target drone

    answer = self.crm.clients(filter=her)
    client = answer['clients'][0]
    her_ip = client['ip']
    her_rep_port = client['port']

    # Connect to her, the target drone
    drone_her = dss.client.Client(timeout=2000, exception_handler=None, context=_context)
    drone_her.connect_as_guest(ip=her_ip, port=her_rep_port, app_id=self.app_id)

    # Enable LLA stream
    drone_her.enable_data_stream('LLA')

    # Get lla port of target drone
    her_pub_port = drone_her.get_port('info_pub_port')
    drone_her.close_dss_socket()

    # Clean up
    del drone_her

    # We are already connected to a drone, take off and follow stream
    self.drone.await_controls()
    self.drone.try_set_init_point()
    self.drone.arm_and_takeoff(height=15)

    # Trigger drone follow stream, and retrigger if PILOT hands over controls again
    while self.alive:
      # Add task to drone task que
      self.drone.enable_follow_stream(ip=her_ip, port=her_pub_port)
      try:
        self.drone.photo_rec(True)
      except dss.auxiliaries.exception.Nack as error:
        _logger(f'Nacked when starting recording, {error.msg}')

      # Monitor if PILOT takes the controls
      no_exceptions = True
      while no_exceptions and self.alive:
        try:
          self.drone.raise_if_aborted()
          time.sleep(1)
        except dss.auxiliaries.exception.AbortTask:
          # Pilot took controls, drone object is updated and aware
          no_exceptions = False

      # Wait for the controls to be handed back
      self.drone.await_controls()

    # Try to stop video recoring on proper shutdown, might be nacked du to PILOT
    # in controls
    try:
      self.drone.photo_rec(False)
    except dss.auxiliaries.exception.Nack:
      _logger.info('Stop recording got nacked')
    except:
      _logger.warning('Stop recording failed')


#--------------------------------------------------------------------#
  # Request: 'set_pattern', incoming pattern to be relayed
  def _request_set_pattern(self, msg:dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)

    # check arguments
    if not all(key in msg for key in ['id', 'pattern']):
      return dss.auxiliaries.zmq.nack(fcn, 'bad arguments: {id, pattern} are mandatory')

    # Check nack reasons
    if not self.from_owner(msg) and msg['id'] != "GUI":
      descr = 'Requester ({}) is not the APP owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    # Accept
    else:
      self._pattern = msg
      del self._pattern['fcn']
      del self._pattern['id']

      if self.drone.alive:
        self.drone.set_pattern_dict(self._pattern)
      # Ack even if there is no connected drone
      answer = dss.auxiliaries.zmq.ack(fcn)
    return answer

#--------------------------------------------------------------------#
  # Main function
  def main(self):
    cursor = ['  |o....|','  |.o...|', '  |..o..|', '  |...o.|','  |....o|',
     '  |...o.|', '  |..o..|', '  |.o...|']
    cursor_index = 7

    # Get a drone
    answer = self.crm.get_drone(force=self._drone_id_arg)
    if dss.auxiliaries.zmq.is_nack(answer):
      _logger.error('Did not receive a drone: %s', dss.auxiliaries.zmq.get_nack_reason(answer))
      return
    # We got ack, there is a drone to connect to

    # Connect to the drone, set app_id in socket
    self.drone.connect(answer['ip'], answer['port'], app_id=self.crm.app_id)
    _logger.info('Connected as owner of drone: [%s]', self.drone._dss.dss_id)

    # Main loop
    while self.alive:
      time.sleep(1)

      cursor_index += 1
      if cursor_index >= len(cursor):
        cursor_index = 0
      print(cursor[cursor_index], end = '\r', flush=True)



#--------------------------------------------------------------------#
def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='APP "app_selfie"', allow_abbrev=False, add_help=False)
  parser.add_argument('-h', '--help', action='help', help=argparse.SUPPRESS)
  parser.add_argument('--app_ip', type=str, help='ip of the app', required=True)
  parser.add_argument('--id', type=str, default=None, help='id of this app_selfie instance if started by crm')
  parser.add_argument('--crm', type=str, help='<ip>:<port> of crm', required=True)
  parser.add_argument('--camera_drone_id', type=str, help='The id of the camera drone', required=True)
  parser.add_argument('--log', type=str, default='debug', help='logging threshold')
  parser.add_argument('--owner', type=str, help='id of the instance controlling app_selfie')
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout')
  args = parser.parse_args()

  # Identify subnet to sort log files in structure
  subnet = dss.auxiliaries.zmq.get_subnet(ip=args.app_ip)
  # Initiate log file
  dss.auxiliaries.logging.configure('app_selfie', stdout=args.stdout, rotating=True, loglevel=args.log, subdir=subnet)

  # Create the Selfie class
  try:
    app = Selfie(args.app_ip, args.id, args.crm, args.camera_drone_id, args.owner)
  except dss.auxiliaries.exception.NoAnswer:
    _logger.error("Failed to instantiate the application: Probably the CRM couldn't be reached")
    sys.exit()
  except:
    _logger.error('Failed to instantiate the application\n%s', traceback.format_exc())
    sys.exit()

  # Try to setup objects and initial sockets
  try:
    # Try to run main
    app.main()
  except KeyboardInterrupt:
    print('', end='\r')
    _logger.warning('Shutdown due to keyboard interrupt')
  except dss.auxiliaries.exception.Nack as error:
    _logger.error('Nacked when sending %s, received error: %s', error.fcn, error.msg)
  except dss.auxiliaries.exception.NoAnswer as error:
    _logger.error('NoAnswer when sending: %s to %s:%s', error.fcn, error.ip, error.port)
  except:
    _logger.error('unexpected exception\n%s', traceback.format_exc())

  try:
    app.kill()
  except:
    _logger.error('unexpected exception\n%s', traceback.format_exc())


#--------------------------------------------------------------------#
if __name__ == '__main__':
  _main()
