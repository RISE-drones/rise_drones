#!/usr/bin/env python3

# TODO / Issues
# 2021-09-27
# * don't return nack on zmq socket
# * TYRApp doesn't handle if the drone/DSS disapears

# Changelog
# 2021-09-27
# * test timeout=3s for app-dss communication (app_tyra.py:103)

import argparse
import datetime
import json
import logging
import math
import sys
import threading
import time
import traceback

import zmq

import dss.auxiliaries

#--------------------------------------------------------------------#
__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.2.0'
__copyright__ = 'Copyright (c) 2021, RISE'
__status__ = 'development'


#--------------------------------------------------------------------#
_logger = logging.getLogger('dss.TYRApp')
ALT_OFFSET = 30


#--------------------------------------------------------------------#
def get_distance(lat1, lon1, lat2, lon2):
  '''
  Returns the ground distance in metres between two LocationGlobal objects.

  This method is an approximation, and will not be accurate over large distances and close to the
  earth's poles. It comes from the ArduPilot test code:
  https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
  https://github.com/ArduPilot/MAVProxy/blob/master/MAVProxy/modules/lib/mp_util.py
  '''
  # 1/180*pi*radius of earth = 111319.4906
  # 40030000 / 360 = 1.11194444444e5
  dlat = lat2 - lat1
  dlong = lon2 - lon1
  return math.sqrt(dlat**2 + (dlong*math.cos(math.pi*lat1/180))**2) * 1.11194444444e5

#====================================================================#
#====================================================================#
#====================================================================#
#==========                                                ==========#
#==========                     Drone                      ==========#
#==========                                                ==========#
#====================================================================#
#====================================================================#
#====================================================================#
#====================================================================#

class Drone:
  def __init__(self, context, app_id):
    self._context = context
    self.app_id = app_id

    self._lla = None

    self.name = None
    self.ip = None
    self.port = None
    self._dss_socket = None
    self._info_thread = None
    self._battery_low = False

    self._task_queue = dss.auxiliaries.TaskQueue()
    self._task_queue.start()

  def __del__(self) -> None:
    # stop task queue
    _logger.debug('Drone: waiting for task queue to stop')
    self._task_queue.stop()

#--------------------------------------------------------------------#

  def connected(self):
    if self.name:
      return True
    return False

#--------------------------------------------------------------------#

  def task_connect(self, name, ip, port):
    self.name = name
    self.ip = ip
    self.port = port
    self._battery_low = False

    # Connect to DSS
    dss_address = f'tcp://{ip}:{port}'
    self._dss_socket = dss.auxiliaries.zmq.Req(self._context, ip, port, label=name, timeout=3000)

    # Test connection, owner change must have gone through to get ack. Takes some time sometimes
    max_attempt = 4
    for attempt in range(max_attempt):
      answer = self._dss_socket.send_and_receive({'fcn': 'heart_beat', 'id': self.app_id})
      if dss.auxiliaries.zmq.is_ack(answer, 'heart_beat'):
        # Correctly connected
        break
      # Give up if no success after maximum number of attempts
      if attempt == max_attempt-1:
        _logger.error(f'[{self.name}] Failed to conect to DSS on %s', dss_address)
        self.name = None
        self.ip = None
        self.port = None
        self._dss_socket.close()
        return
      time.sleep(0.1)

    # Start heart beat thread
    self._dss_socket.start_heartbeat(self.app_id)

    # Enable the LLA stream
    answer = self._dss_socket.send_and_receive({'fcn': 'data_stream', 'id': self.app_id, 'stream': 'LLA', 'enable': True})
    _logger.info(f'[{self.name}] {answer}')

    _logger.info(f'[{self.name}] Connection to DSS established on %s', dss_address)

    # Set up subscription on LLA-stream and battery-stream from DSS
    info_port = self.get_info('info_pub_port')
    self._info_thread = threading.Thread(target=self._main_info_dss, args=[self.ip, info_port], daemon=True)
    self._info_thread.start()

#--------------------------------------------------------------------#

  def get_info(self, port_label):
    call = 'get_info'
    msg = {'fcn': call, 'id': self.app_id}
    answer = self._dss_socket.send_and_receive(msg)
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      return False
    return int(answer[port_label])

#--------------------------------------------------------------------#

  def who_controls(self, mode):
    call = 'who_controls'
    assert mode in ('APPLICATION', 'DSS', 'PILOT'), f'invalid argument: {mode}'
    msg = {'fcn': call, 'id': self.app_id}
    answer = self._dss_socket.send_and_receive(msg)
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      return False
    return answer['in_controls'] == mode

#--------------------------------------------------------------------#

  def await_operator(self):
    _logger.info('APPLICATION waiting for the CONTROLS')
    while not self.who_controls('APPLICATION'):
      time.sleep(1.0)
      if not self.name:
        _logger.error('drone killed')
        return

#--------------------------------------------------------------------#

  def get_height(self) -> float:
    call = 'get_posD'
    msg = {'fcn': call, 'id': self.app_id}
    answer = self._dss_socket.send_and_receive(msg)
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    return -float(answer['posD'])

#--------------------------------------------------------------------#
  # Task takeoff
  def task_takeoff(self, height):
    _logger.info('arm and takeoff (alt=%d)', height)

    # wait for controls
    self.await_operator()

    call = 'set_init_point'
    msg = {'fcn': call, 'id': self.app_id, 'heading_ref': 'drone'}
    answer = self._dss_socket.send_and_receive(msg)

    call = 'arm_take_off'
    msg = {'fcn': call, 'id': self.app_id, 'height': height}
    answer = self._dss_socket.send_and_receive(msg)
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      _logger.error('nack: arm_take_off (height=%d)', height)
      self.release()
      return

    cur_height = 0.0
    while cur_height < height * 0.95:
      time.sleep(1.0)
      if not self.name:
        _logger.error('drone killed')
        return
      try:
        cur_height = self.get_height()
      except dss.auxiliaries.exception.Nack:
        pass

    call = 'reset_dss_srtl'
    msg = {'fcn': call, 'id': self.app_id}
    answer = self._dss_socket.send_and_receive(msg)

    answer = self._dss_socket.send_and_receive({'fcn': 'photo', 'id': self.app_id, 'cmd': 'record', 'enable': True})

  # Task follow stream
  def task_follow_stream(self, enable, tyramote_ip, tyramote_info_pub_port):
    call = 'follow_stream'

    # Send the follow stream command to DSS
    msg = {'fcn': call, 'id': self.app_id, 'enable': enable, 'ip': tyramote_ip, 'port': tyramote_info_pub_port}
    answer = self._dss_socket.send_and_receive(msg)

    if not enable:
      answer = self._dss_socket.send_and_receive({'fcn': 'photo', 'id': self.app_id, 'cmd': 'record', 'enable': False})
      self.release()

  # Task set pattern
  def task_set_pattern(self, pattern):
    call = 'set_pattern'
    msg = pattern
    msg['fcn'] = call
    msg['id'] = self.app_id
    answer = self._dss_socket.send_and_receive(msg)

  # Tast set geofence
  def task_set_geofence(self, height_low, height_high, radius):
    call = 'set_geofence'
    msg = {'fcn': call, 'id': self.app_id, 'height_low': height_low, 'height_high': height_high, 'radius': radius}
    answer = self._dss_socket.send_and_receive(msg)


#--------------------------------------------------------------------#
  # Connec
  def connect(self, name, ip, port):
    self.name = name
    self._task_queue.add(self.task_connect, name, ip, port)

#--------------------------------------------------------------------#
  # Set geofence
  def set_geofence(self):
    height_low = 2
    height_high = 60
    radius = 300
    self._task_queue.add(self.task_set_geofence, height_low, height_high, radius)

#--------------------------------------------------------------------#
  # Takeoff
  def takeoff(self):
    self._task_queue.add(self.task_takeoff, 20)

#--------------------------------------------------------------------#
  # Set pattern
  def set_pattern(self, pattern):
    self._task_queue.add(self.task_set_pattern, pattern)

#--------------------------------------------------------------------#
  # Follow stream
  def follow_stream(self, enable, tyramote_ip, tyramote_pub_port):
    self._task_queue.add(self.task_follow_stream, enable, tyramote_ip, tyramote_pub_port)

#--------------------------------------------------------------------#
  # Hover
  def hover(self):
    call = 'set_vel_BODY'
    msg = {'fcn': call, 'id': self.app_id, 'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw_rate': 0.0}
    answer = self._dss_socket.send_and_receive(msg)

#--------------------------------------------------------------------#
  # Release
  def release(self):
    self.name = None
    self.ip = None
    self.port = None
    if self._dss_socket:
      self._dss_socket.close()
      self._dss_socket = None
    if self._info_thread:
      self._info_thread.join()
      self._info_thread = None

#--------------------------------------------------------------------#
  # Main info from dss, thread
  def _main_info_dss(self, ip, port):
    _info_socket = dss.auxiliaries.zmq.Sub(self._context, ip, port, 'info ' + self.name)
    _info_socket.subscribe('battery')
    _info_socket.subscribe('LLA')

    while self.name:
      try:
        topic, msg = _info_socket.recv()

        if topic == 'battery':
          if msg['remaining_time'] < 295:
            self._battery_low = True
            _logger.warning(f'[{self.name}] battery low!')
          else:
            self._battery_low = False
            _logger.warning(f'[{self.name}] battery no longer low! =D')
        elif topic == 'LLA':
          self._lla = (msg['lat'], msg['lon'])
      except zmq.error.Again:
        pass
        #_logger.error('_main_info_dss: Resource temporarily unavailable')
        # -> no message, try again
      except:
        _logger.error(f'unexpected exception\n{traceback.format_exc()}')

#====================================================================#
#====================================================================#
#====================================================================#
#==========                                                ==========#
#==========                     TYRApp                     ==========#
#==========                                                ==========#
#====================================================================#
#====================================================================#
#====================================================================#
#====================================================================#

class TYRApp:
  def __init__(self, app_id, crm, app_ip, owner):

    # Split crm connection string"
    (crm_ip, crm_port) = crm.split(':')
    crm_port = int(crm_port)

    self._app_id = app_id
    self._crm_ip = crm_ip
    self._crm_port = crm_port
    self._owner = owner
    self._tyramote_ip = None
    self._tyramote_port = None
    self._tyramote_info_pub_port = None

    self._app_ip = app_ip

    self._alive = True
    self._context = zmq.Context()

    # all sockets
    self._app_socket = None # Rep: ANY -> APP
    self._crm_socket = None # Req: APP -> CRM
    self._info_socket = None # Pub: APP -> ANY
    self._info_tyramote = None # Sub: TYRAmote -> APP (LLA)
    self._tyramote_socket = None # Req: APP -> TYRAmote

    self._mutex = threading.Lock()
    self._tyramote_id = owner # TYRAmote id

    self._lla = None

    self._drone1 = Drone(self._context, app_id) # main drone
    self._drone2 = Drone(self._context, app_id) # recovery drone

    self._pattern = {'pattern': 'above', 'rel_alt': 20, 'heading': 0}

    # commands from ANY to APP
    self._commands = {'follow_me':    self._request_follow_me,
                      'heart_beat':   self._request_heart_beat,
                      'get_info':     self._request_get_info,
                      'set_pattern':  self._request_set_pattern}

    self._task_queue = dss.auxiliaries.TaskQueue()
    self._task_queue.start()

  @property
  def alive(self):
    '''checks if CRM is alive'''
    return self._alive

  # Set pattern
  def _set_pattern(self):
    if self._drone1.connected():
      self._drone1.set_pattern(self._pattern)
    if self._drone2.connected():
      pattern2 = {'pattern': 'above', 'rel_alt': self._pattern['rel_alt']+ALT_OFFSET, 'heading': 'course'}
      self._drone2.set_pattern(pattern2)

  # Publish clients for TYRAmote
  def publish_clients(self):
    drones = list()
    if self._drone1.name:
      drones.append(self._drone1.name)
    if self._drone2.name:
      drones.append(self._drone2.name)
    self._info_socket.publish('clients', {'clients': drones})

  # Task get drone
  def task_getDrone(self, id, ip, port):
    if not self._drone1.connected():
      self._drone1.connect(id, ip, port)
      self._drone1.set_geofence()
      self._drone1.takeoff()
      self._set_pattern()
      self._drone1.follow_stream(True, self._tyramote_ip, self._tyramote_info_pub_port)
    elif not self._drone2.connected():
      self._drone2.connect(id, ip, port)
      self._drone2.set_geofence()
      self._drone2.takeoff()
      self._set_pattern()
      self._drone2.follow_stream(True, self._tyramote_ip, self._tyramote_info_pub_port)
    else:
      _logger.warning("task_getDrone: already two drones in use!")

    self.publish_clients()

  # Tast release drone
  def task_releaseDrones(self):
    drone1_name = self._drone1.name
    drone2_name = self._drone2.name

    if self._drone1.connected():
      self._drone1.follow_stream(False, self._tyramote_ip, self._tyramote_info_pub_port)
      #self._drone1.release()  # already within follow_stream
    if self._drone2.connected():
      self._drone2.follow_stream(False, self._tyramote_ip, self._tyramote_info_pub_port)
      #self._drone1.release()  # already within follow_stream

    while self._drone1.connected() or self._drone2.connected():
      time.sleep(0.1)

    if drone1_name:
      self._crm_socket.send_and_receive({'fcn': 'release_drone', 'id': self._app_id, 'id_released': drone1_name})
    if drone2_name:
      self._crm_socket.send_and_receive({'fcn': 'release_drone', 'id': self._app_id, 'id_released': drone2_name})

    self.publish_clients()

  # Task low battery
  def task_low_battery(self):
    if self._drone1.connected():
      if self._drone1._battery_low and not self._drone2.connected():
        answer = self._crm_socket.send_and_receive({'fcn': 'get_drone', 'id': self._app_id, 'capability': self.capability})
        if dss.auxiliaries.zmq.is_ack(answer):
          self._task_queue.add(self.task_getDrone, answer['id'], answer['ip'], answer['port'])
        else:
          _logger.error("Failed to get replacement drone!")
          self.task_releaseDrones()

  # Kill method
  def kill(self):
    self._alive = False
    # hover
    if self._drone1.connected():
      drone1_name = self._drone1.name
      self._drone1.hover()
      self._drone1.release()
      answer = self._crm_socket.send_and_receive({'fcn': 'release_drone', 'id': self._app_id, 'id_released': drone1_name})
      self.publish_clients()
      if not dss.auxiliaries.zmq.is_ack(answer):
        desc = answer['description'] if 'description' in answer else 'no description'
        _logger.error(f'get_drone failed: {desc}')
    if self._drone2.connected():
      drone2_name = self._drone2.name
      self._drone2.hover()
      self._drone2.release()
      answer = self._crm_socket.send_and_receive({'fcn': 'release_drone', 'id': self._app_id, 'id_released': drone2_name})
      self.publish_clients()
      if not dss.auxiliaries.zmq.is_ack(answer):
        desc = answer['description'] if 'description' in answer else 'no description'
        _logger.error(f'get_drone failed: {desc}')
    if self._info_thread:
      self._info_thread.join()
      self._info_thread = None

#----                                                            ----#
  # Update tyramote ip and ports
  def get_and_save_tyramote_info(self):
    call = 'clients'
    msg = {'fcn': call, 'id': self._app_id, 'filter': self._tyramote_id}
    answer = self._crm_socket.send_and_receive(msg)
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      _logger.error(f'clients failed')
      self.kill()
      return
    client = answer['clients'][0]
    if client['id'] != self._tyramote_id:
      _logger.error(f'owner not found')
      self.kill()
      return

    # Update ip and port info, then connect to TYRAmote
    self._tyramote_ip = client['ip']
    self._tyramote_port = client['port']
    self._tyramote_socket = dss.auxiliaries.zmq.Req(self._context, ip=self._tyramote_ip, port=self._tyramote_port, label='Tyramote req socket', self_id=self._app_id)
    self._tyramote_socket.connect()

    # Get publish port from tyramote
    call = 'get_info'
    msg = {'fcn': call, 'id': self._app_id}
    answer = self._tyramote_socket.send_and_receive(msg)
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      _logger.error(f'TYRAmote get info failed')
      self.kill()
      return
    self._tyramote_info_pub_port = int(answer['info_pub_port'])

#----                                                            ----#
  # Subscribe to LLA stream from TYRAmote, thread
  def _main_info_tyramote(self, ip, port):
    info_tyramote = dss.auxiliaries.zmq.Sub(self._context, ip, port, 'info ' + self._tyramote_id)
    info_tyramote.subscribe('LLA')

    while self._alive:
      try:
        topic, msg = info_tyramote.recv()

        if topic == 'LLA':
          self._lla = (msg['lat'], msg['lon'], msg['alt'])
          _logger.info(f'Tyramote position: {self._lla}')
      except zmq.error.Again:
        pass # -> no message, try again
      except:
        _logger.error(f'unexpected exception\n{traceback.format_exc()}')

#----                                                            ----#
  # Main
  def main(self):
    self._crm_socket = dss.auxiliaries.zmq.Req(self._context, self._crm_ip, self._crm_port, label='crm', timeout=10000)
    self._crm_socket.start_heartbeat(self._app_id)
    self._app_socket = dss.auxiliaries.zmq.Rep(self._context, label=f'app {self._app_id}', min_port=self._crm_port+1, max_port=self._crm_port+50)
    self._info_socket = dss.auxiliaries.zmq.Pub(self._context, label=f'info {self._app_id}', min_port=self._crm_port+1, max_port=self._crm_port+50)

    _logger.info('App {app_id} is listening on {ip}:{port}'.format(app_id=self._app_id, ip=self._app_socket.ip, port=self._app_socket.port))

    # Register APP
    call = 'register'
    msg = {'fcn': call, 'name': 'app_tyra.py', 'desc': 'app_tyra','type': 'da'}
    msg['id'] = self._app_id
    msg['ip'] = self._app_ip
    msg['port'] = self._app_socket.port
    answer = self._crm_socket.send_and_receive(msg)
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      _logger.error(f'register failed: {answer}')
      self.kill()

    # Get ip, port and pub port of TYRAmote, is saved to self
    self.get_and_save_tyramote_info()
    _logger.info(f'Tyramote ip: {self._tyramote_ip}, and port: {self._tyramote_port}, and pub_port: {self._tyramote_info_pub_port}')

    # Set up subscription to TYRAmote
    self._info_thread = threading.Thread(target=self._main_info_tyramote, args=[self._tyramote_ip, self._tyramote_info_pub_port], daemon=True)
    self._info_thread.start()

    timestamp = datetime.datetime.now().timestamp()
    while self._alive:
      now = datetime.datetime.now().timestamp()
      try:
        msg = self._app_socket.recv_json()
        timestamp = now
      except zmq.error.Again as error:
        if now - timestamp > 10: #seconds
          self.kill() # unregister, close, CRM will take care of the drones!
        continue # timeout: no message received; try again

      msg = json.loads(msg)

      fcn = dss.auxiliaries.zmq.get_fcn(msg)
      if fcn in self._commands:
        try:
          with self._mutex:
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

    del self._drone1
    del self._drone2

    self._app_socket.close()
    self._info_socket.close()
    self._crm_socket.close()
    _logger.debug('~ THE END ~')

#----                                                            ----#
  # Request follow me
  def _request_follow_me(self, msg:dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)

    # check arguments
    if not all(key in msg for key in ['id', 'enable', 'capability']):
      return dss.auxiliaries.zmq.nack(fcn, 'bad arguments: {id, enable, capability} are mandatory')

    if msg['enable']:
      if self._drone1.connected():
        return dss.auxiliaries.zmq.nack(fcn, f'already connected to {self._drone1.name}')

      self.capability = msg['capability']
      answer = self._crm_socket.send_and_receive({'fcn': 'get_drone', 'id': self._app_id, 'capability': self.capability})
      if not dss.auxiliaries.zmq.is_ack(answer):
        desc = answer['description'] if 'description' in answer else 'no description'
        _logger.error(f'get_drone failed: {desc}')
        return dss.auxiliaries.zmq.nack(fcn, desc)

      _logger.info(f'-> answer: {answer}')
      self._task_queue.add(self.task_getDrone, answer['id'], answer['ip'], answer['port'])
      return dss.auxiliaries.zmq.ack(fcn)
    elif self._drone1.connected():
      self._task_queue.add(self.task_releaseDrones)
      return dss.auxiliaries.zmq.ack(fcn)

#----                                                            ----#
  # Request heart beat
  def _request_heart_beat(self, msg:dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)

    # check arguments
    if not all(key in msg for key in ['id']):
      return dss.auxiliaries.zmq.nack(fcn, 'bad arguments: {id} is mandatory')

    id_ = msg['id']
    if id_ != self._tyramote_id:
      return dss.auxiliaries.zmq.nack(fcn, 'wrong id')

    self._task_queue.add(self.task_low_battery)
    if self._drone2.connected() and self._drone1._lla and self._drone2._lla:
      (lat1, lon1) = self._lla
      (lat2, lon2) = self._drone2._lla
      distance = get_distance(lat1, lon1, lat2, lon2)
      _logger.info(f'distance: {distance}')
      if distance < 15:
        drone1_name = self._drone1.name
        self._drone1.follow_stream(False, self._tyramote_ip, self._tyramote_info_pub_port)

        while self._drone1.connected():
          time.sleep(0.1)

        answer = self._crm_socket._send_and_receive({'fcn': 'release_drone', 'id': self._app_id, 'id_released': drone1_name})
        self.publish_clients()

        self._drone1, self._drone2 = self._drone2, self._drone1 # swap the drones =D
        # drone2 is now disconnected
        self._set_pattern() # to get rid of the offset

    return dss.auxiliaries.zmq.ack(fcn)

#----                                                            ----#
  # Request get info
  def _request_get_info(self, msg: dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)

    # check arguments
    if not all(key in msg for key in ['id']):
      return dss.auxiliaries.zmq.nack(fcn, 'bad arguments: {id} is mandatory')

    id_ = msg['id']
    if id_ != self._tyramote_id:
      return dss.auxiliaries.zmq.nack(fcn, 'wrong id')

    return {'fcn': 'ack', 'call': fcn, 'id': self._app_id, 'info_pub_port': self._info_socket.port}

#----                                                            ----#
  # Request set pattern
  def _request_set_pattern(self, msg:dict) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)

    # check arguments
    if not all(key in msg for key in ['id', 'pattern']):
      return dss.auxiliaries.zmq.nack(fcn, 'bad arguments: {id, pattern} are mandatory')

    id_ = msg['id']
    if id_ != self._tyramote_id:
      return dss.auxiliaries.zmq.nack(fcn, 'wrong id')

    self._pattern = msg
    del self._pattern['fcn']
    del self._pattern['id']

    self._set_pattern()
    return {'fcn': 'ack', 'call': fcn, 'id': self._app_id}

#--------------------------------------------------------------------#
# _main
def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='DSS-APP "TYRApp"', allow_abbrev=False, add_help=False)
  parser.add_argument('-h', '--help', action='help', help=argparse.SUPPRESS)
  parser.add_argument('--id', type=str, help='id of the TYRApp instance provided by CRM', required=True)
  parser.add_argument('--crm', type=str, help='<ip>:<port> of crm', required=True)
  parser.add_argument('--app_ip', type=str, help='ip of the app', required=True)
  parser.add_argument('--log', type=str, default='debug', help='logging threshold')
  parser.add_argument('--owner', type=str, help='id of the connected TYRAmote instance', required=True)
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout')
  args = parser.parse_args()

  # Identify subnet to sort log files in structure, dont use app_ip since app runs on server 160
  # Split crm connection string"
  (_, crm_port) = args.crm.split(':')
  crm_port = int(crm_port)
  subnet = dss.auxiliaries.zmq.get_subnet(port=crm_port)
  dss.auxiliaries.logging.configure(f'{args.id}_app_tyrapp', stdout=args.stdout, rotating=True, loglevel=args.log, subdir=subnet)

  # Create the TYRApp class
  try:
    client = TYRApp(args.id, args.crm, app_ip=args.app_ip, owner=args.owner)
  except dss.auxiliaries.exception.NoAnswer:
    _logger.error('Failed to instantiate application: Probably the CRM couldn\'t be reached')
    sys.exit()
  except:
    _logger.error('Failed to instantiate application\n%s', traceback.format_exc())
    sys.exit()

# Try to setup objects and initial sockets
  try:
    # Try to run main
    client.main()
  except KeyboardInterrupt:
    logging.warning('shutdown due to keyboard interrupt')
  except:
    logging.error('unexpected exception\n{}'.format(traceback.format_exc()))

  # try to kill the app
  try:
    client.kill()
  except:
    _logger.error(f'unexpected exception\n{traceback.format_exc()}')

#--------------------------------------------------------------------#
if __name__ == '__main__':
  _main()
