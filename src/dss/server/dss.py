'''Drone Safety Service Server'''

import json
import logging
import threading
import time
import traceback
import typing
import sys
import datetime

import zmq

import dss.auxiliaries
from dss.auxiliaries.config import config
import dss.client

sys.path.append('/home/pi/rise_drones_dev/modem')
from modem import Modem

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.0'
__copyright__ = 'Copyright (c) 2019-2022, RISE'
__status__ = 'development'

class Server:
  '''Drone Safety Service Server - new implementation'''

  def __init__(self, dss_ip, dss_id='', drone: str='', baud=921600, with_gcs=False, gcs_address=None, rangefinder=False, autogain=False, midstick_check=True, clearance_check=True, photo=False, crm: str='', description='crm_dss', die_gracefully: bool=False, network_log=False):
    if die_gracefully:
      # source: https://stackoverflow.com/a/31464349
      import signal
      signal.signal(signal.SIGINT, self.exit_gracefully)
      signal.signal(signal.SIGTERM, self.exit_gracefully)

    # create all objects that are used in the destructor
    self._photo = None
    self._dss_id = dss_id
    self._dss_ip = dss_ip

    self._owner = 'da000'
    self.follow_stream_enable = False      # Flag to control follow stream thread

    self._logger = logging.getLogger(__name__)
    self._zmq_context = dss.auxiliaries.zmq.Context()

    self._logger.info(f'DSS Server version: {dss.__version__}, git describe: {dss.auxiliaries.git.describe()}')

    self._midstick_check = midstick_check
    self._clearance_check = clearance_check

    # start heartbeat client
    if with_gcs:
      if gcs_address is None:
        gcs_address = config['DSS']['HeartbeatClientSocket']
      gcs_attempts = int(config['DSS']['HeartbeatAttempts'])
      self._gcs_heartbeat = dss.auxiliaries.heartbeat.Client(gcs_address, gcs_attempts, context=self._zmq_context)

      self._gcs_heartbeat.alive = True
      self._logger.info("gcs heartbeats required for this flight")
    else:
      self._gcs_heartbeat = None

    # This attribute is true if there is a connection to a dss client
    # application
    self._connected = False


    # Split crm connection string"
    if crm:
      (_, crm_port) = crm.split(':')
      crm_port = int(crm_port)

    # Split drone connection string
    (drone_ip, drone_port) = drone.split(':')
    drone_port = int(drone_port)

    # zmq sockets
    app_port = None if crm else config['DSS']['ServSocket'].split(':')[-1]
    if crm:
      # We will connect to crm, set random ports within range.
      self._serv_socket = dss.auxiliaries.zmq.Rep(self._zmq_context, port=app_port, label='dss', min_port=crm_port+1, max_port=crm_port+49)
      self._pub_socket = dss.auxiliaries.zmq.Pub(self._zmq_context, port=None, min_port=crm_port+1, max_port=crm_port+50, label='info')
    else:
      # We are running dss stand alone, set standard ports
      self._serv_socket = dss.auxiliaries.zmq.Rep(self._zmq_context, port=app_port, label='dss', min_port=6000, max_port=6100)
      self._pub_socket = dss.auxiliaries.zmq.Pub(self._zmq_context, port=5558, min_port=6000, max_port=6100, label='info')
    self._logger.info('Starting pub server on %d... done', self._pub_socket.port)

    if photo:
      self._photo = dss.server.photo.Client(self._zmq_context, config['DSS']['PhotoClient'])
      self._logger.info('Connecting to photo client on %s... done', config['DSS']['PhotoClient'])


    # all attributes are disabled by default
    self._pub_attributes = {'ATT':                   {'enabled': False, 'name': 'attitude'},
                            'LLA':                   {'enabled': False, 'name': 'location.global_frame'},
                            'NED':                   {'enabled': False, 'name': 'location.local_frame'}, # 'location.local_frame'?
                            'XYZ':                   {'enabled': False, 'name': 'TODO'},
                            'photo_LLA':             {'enabled': False, 'name': 'TODO'},
                            'photo_XYZ':             {'enabled': False, 'name': 'TODO'},
                            'currentWP':             {'enabled': False, 'name': 'TODO'},
                            'battery':               {'enabled': False, 'name': 'TODO'}}


    # create the hexacopter object
    self._hexa = dss.server.Hexacopter(f'{drone_ip}:{drone_port}', baud, rangefinder)

    # init GLANA
    self._hexa.glana = dss.server.Glana(self._zmq_context, config['DSS']['GlanaClientSocket'])
    self._hexa.glana_autogain = autogain

    # _commands is a lookup table for all the dss commands
    # 'request' points to the synchronous request call-back
    # 'task' points to an optional asynchronous task call-back

    # Functions in same order as documentation
    self._commands = {'arm_take_off':       {'request': self._request_arm_take_off,       'task': self._task_arm_take_off},
                      'data_stream':        {'request': self._request_data_stream,        'task': None},
                      'disconnect':         {'request': self._request_disconnect,         'task': self._task_disconnect},
                      'dss_srtl':           {'request': self._request_dss_srtl,           'task': self._task_dss_srtl},
                      'follow_stream':      {'request': self._request_follow_stream,      'task': self._task_follow_stream},
                      'get_armed':          {'request': self._request_get_armed,          'task': None},
                      'get_currentWP':      {'request': self._request_get_currentWP,      'task': None}, # Not implemented
                      'get_flightmode':     {'request': self._request_get_flightmode,     'task': None},
                      'get_idle':           {'request': self._request_get_idle,           'task': None},
                      'get_info':           {'request': self._request_get_info,           'task': None},
                      'get_metadata':       {'request': self._request_get_metadata,       'task': None}, # Not implemented
                      'get_owner':          {'request': self._request_get_owner,          'task': None},
                      'get_posD':           {'request': self._request_get_posD,           'task': None},
                      'get_PWM':            {'request': self._request_get_PWM,            'task': None},
                      'gogo':               {'request': self._request_gogo,               'task': self._task_gogo}, # Not fully implemented
                      'heart_beat':         {'request': self._request_heart_beat,         'task': None},
                      'land':               {'request': self._request_land,               'task': self._task_land},
                      'photo':              {'request': self._request_photo,              'task': None}, # Not implemented
                      'reset_dss_srtl':     {'request': self._request_reset_dss_srtl,     'task': None},
                      'rtl':                {'request': self._request_rtl,                'task': self._task_rtl},
                      'set_default_speed':  {'request': self._request_set_default_speed,  'task': None}, # Not implemented
                      'set_geofence':       {'request': self._request_set_geofence,       'task': None},
                      'set_gimbal':         {'request': self._request_set_gimbal,         'task': None},
                      'set_gripper':        {'request': self._request_set_gripper,        'task': self._task_set_gripper},
                      'set_heading':        {'request': self._request_set_heading,        'task': None},
                      'set_init_point':     {'request': self._request_set_init_point,     'task': None},
                      'set_owner':          {'request': self._request_set_owner,          'task': None},
                      'set_pattern':        {'request': self._request_set_pattern,        'task': None}, # Not implemented
                      'set_vel_BODY':       {'request': self._request_set_vel_BODY,       'task': None},
                      'upload_mission_LLA': {'request': self._request_upload_mission,     'task': None}, # We could clean up the API, remover REF
                      'upload_mission_NED': {'request': self._request_upload_mission,     'task': None}, # We could clean up the API, remover REF
                      'upload_mission_XYZ': {'request': self._request_upload_mission,     'task': None}, # We could clean up the API, remover REF
                      'who_controls':       {'request': self._request_who_controls,       'task': None},
                      # Not documented
                      'glana':              {'request': self._request_glana,              'task': None}}

    # create initial task
    self._task = {'fcn': ''}
    self._task_event = threading.Event()

    self._alive = True
    self._in_controls = 'PILOT'

    #start attribute_listener for clearance check
    if self._clearance_check:
      self._hexa.vehicle.add_attribute_listener("channel13", self._clearance_listener)
    #Internal state for clearance command : WAITING, HIGH, CLEARED
    self._clearance_state = 'WAITING' if self._clearance_check else 'CLEARED'
    # start main thread
    main_thread = threading.Thread(target=self._main, daemon=False)
    main_thread.start()

    # start task thread
    task_thread = threading.Thread(target=self._main_task, daemon=True)
    task_thread.start()

    # start glana thread
    glana_thread = threading.Thread(target=self._main_glana, daemon=True)
    glana_thread.start()

    # start network log
    if network_log:
      network_thread = threading.Thread(target=self._main_network, daemon=True)
      network_thread.start()

    # register dss
    if crm:
      self._crm = dss.client.CRM(self._zmq_context, crm, app_name='crm_dss.py', desc=description, app_id=self._dss_id)
      #register and start sending heartbeat to the CRM
      answer = self._crm.register(self._dss_ip, self._serv_socket.port, type='dss')
      if dss.auxiliaries.zmq.is_ack(answer):
        self._dss_id = answer['id']
      else:
        self._logger.error(f'register failed: {answer}')
        self.alive = False
    else:
      self._crm = None

  def exit_gracefully(self, *args):
    self._logger.warning('Shutdown due to interrupt')
    self.alive = False

  def lost_link_to_gcs(self):
    '''returns true if the connection to the gcs has been lost'''
    if self._gcs_heartbeat:
      return not self._gcs_heartbeat.vital
    return False

  @property
  def alive(self):
    '''Checks if the dss server is alive'''
    return self._alive

  @alive.setter
  def alive(self, value):
    self._alive = value

	# Ack nack helpers
	# Is message from owner?
  def from_owner(self, msg)->bool:
    return msg['id'] == self._owner

  # Nav not ready helper
  def nav_ready(self):
    return self._hexa.vehicle.is_armable

  # Test for Heading valid
  def heading_valid(self, heading: typing.Union[str, int]) -> bool:
    if isinstance(heading, str):
      # Test accepted string values
      return heading in ('course', 'poi')
    elif isinstance(heading, int):
      # Test accepted int values
      return 0 <= heading < 360
    else:
      return False

  #############################################################################
  # REQUESTS
  #############################################################################

  def _request_heart_beat(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
    return answer

  def _request_who_controls(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # No nack reasons, accept
    answer = dss.auxiliaries.zmq.ack(fcn, {'in_controls': self._in_controls})
    return answer

  def _request_get_owner(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # No nack reasons, accept
    answer = dss.auxiliaries.zmq.ack(fcn)
    answer['owner'] = self._owner
    return answer

  def _request_set_owner(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Test nack reasons
    if not msg['id'] == 'crm':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Requestor is not CRM')
    # Accept
    else:
      new_owner = msg['owner']
      self._owner = new_owner
      # New owner -> reset connected flag
      self._connected = False
      answer = dss.auxiliaries.zmq.ack(fcn)
    return answer

  def _request_set_geofence(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    height_low = msg['height_low']
    height_high = msg['height_high']
    radius = msg['radius']
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    # Accept
    else:
      self._hexa.geofence.set_geofence(height_low, height_high, radius)
      answer = dss.auxiliaries.zmq.ack(fcn)
    return answer

  def _request_get_idle(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # No nack reasons, accept
    answer = dss.auxiliaries.zmq.ack(fcn, {'idle': not self._task_event.is_set()})
    return answer

  def _request_get_info(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # No nack reasons, accept
    answer = dss.auxiliaries.zmq.ack(fcn, {'info_pub_port': self._pub_socket.port, 'data_pub_port': '', 'id': self._dss_id})
    return answer

  def _request_set_init_point(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif not self.nav_ready():
      answer = dss.auxiliaries.zmq.nack(fcn, 'Navigation not ready')
    elif self._hexa.init_point_wp.is_init_point:
      answer = dss.auxiliaries.zmq.nack(fcn, 'Init point already set')
    elif not self._hexa.gimbal_yaw_readable and msg['heading_ref'] == 'camera':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Gimbal yaw not readable')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
      self._hexa.set_init_point(msg['heading_ref'])
    return answer

  def _request_reset_dss_srtl(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif not self.nav_ready():
      answer = dss.auxiliaries.zmq.nack(fcn, 'Navigation not ready')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
      self._hexa.reset_dss_srtl()
    return answer


  def _request_arm_take_off(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    to_alt = msg['height']
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif self._in_controls != 'APPLICATION':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Application is not in controls')
    elif self._hexa.get_nsat() < 8:
      answer = dss.auxiliaries.zmq.nack(fcn, 'Less than 8 satellites')
    elif self._hexa.is_flying(): # Actually it is the armed state that is tested
      answer = dss.auxiliaries.zmq.nack(fcn, 'State is flying')
    elif not 2 <= to_alt <= 40:
      answer = dss.auxiliaries.zmq.nack(fcn, 'Height is out of limits')
    elif not self._hexa.is_init_point_set():
      answer = dss.auxiliaries.zmq.nack(fcn, 'Init point not set')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
    return answer

  def _request_land(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif self._in_controls != 'APPLICATION':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Application is not in controls')
    elif not self._hexa.is_flying(): # Actually it is the armed state that is tested
      answer = dss.auxiliaries.zmq.nack(fcn, 'State is not flying')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
    return answer

  def _request_rtl(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif self._in_controls != 'APPLICATION':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Application is not in controls')
    elif not self._hexa.is_flying(): # Actually it is the armed state that is tested
      answer = dss.auxiliaries.zmq.nack(fcn, 'State is not flying')
    elif False: # Think this nack reason is related to DJI-DSS.
      answer = dss.auxiliaries.zmq.nack(fcn, 'RTL failed to engage, try again')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
    return answer

  def _request_dss_srtl(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    self._logger.error(msg)
    hover_time = msg['hover_time']
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif self._in_controls != 'APPLICATION':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Application is not in controls')
    elif not self._hexa.is_flying(): # Actually it is the armed state that is tested
      answer = dss.auxiliaries.zmq.nack(fcn, 'State is not flying')
    elif not 0 <= hover_time <= 300:
      answer = dss.auxiliaries.zmq.nack(fcn, 'Hover_time is out of limits')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
    return answer

  def _request_set_vel_BODY(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    vel_x = msg['x']
    vel_y = msg['y']
    vel_z = msg['z']
    yaw_rate = msg['yaw_rate']

    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif self._in_controls != 'APPLICATION':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Application is not in controls')
    elif not self._hexa.is_flying(): # Actually it is the armed state that is tested
      answer = dss.auxiliaries.zmq.nack(fcn, 'State is not flying')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
      self._hexa.send_body_velocity(vel_x, vel_y, vel_z)
      self._hexa.send_yaw_rate(yaw_rate)
    return answer

  def _request_set_heading(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    heading = msg['heading']
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif self._in_controls != 'APPLICATION':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Application is not in controls')
    elif not self._hexa.is_flying(): # Actually it is the armed state that is tested
      answer = dss.auxiliaries.zmq.nack(fcn, 'State is not flying')
    elif not 0 <= heading < 360:
      answer = dss.auxiliaries.zmq.nack(fcn, 'Heading out of limits')
    elif False: # If mission is active, TODO
      answer = dss.auxiliaries.zmq.nack(fcn, 'Mission is active')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
      self._hexa.set_heading(heading)
    return answer

  def _request_set_default_speed(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    default_speed = msg['default_speed']
    ## TODO, high low limits, where?
    dss_low_speed = 0.1
    dss_high_speed = 10.0
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif not dss_low_speed < default_speed < dss_high_speed: # TODO, high and low speed where? - Settings.json? kind of deprecated..
      answer = dss.auxiliaries.zmq.nack(fcn, 'Default speed is out of DSS limits')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
      self._hexa.default_speed = default_speed
    return answer

  def _request_posD(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # No nack reasons, accept
    answer = dss.auxiliaries.zmq.ack(fcn)
    answer['posD'] = self._hexa.vehicle.location.global_relative_frame.alt
    return answer

  def _request_upload_mission(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    mission = msg['mission']
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
      return answer
    elif not self._hexa.is_init_point_set():
      answer = dss.auxiliaries.zmq.nack(fcn, 'Init point not set')
      return answer
    # Check the mission properties
    check_ok, descr = self._hexa.upload_mission(mission)     # check mission cannot be run prior to is_init_point_set
    if not check_ok: # Check wp numbering, geofence, action, speed, heading
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
      #Log the pending mission
      self._hexa.log_pending_mission()
    return answer

  def _request_gogo(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    next_wp = "id%d" % msg['next_wp']
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif self._in_controls != 'APPLICATION':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Application is not in controls')
    elif not self._hexa.is_flying(): # Actually it is the armed state that is tested
      answer = dss.auxiliaries.zmq.nack(fcn, 'State is not flying')
    elif self._hexa.pending_mission is None:
      answer = dss.auxiliaries.zmq.nack(fcn, 'No mission to execute')
    elif not next_wp in self._hexa.pending_mission: #wp is not available in pending mission
      answer = dss.auxiliaries.zmq.nack(fcn, 'Wp number is not available in pending mission')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
      self._hexa.active_mission = self._hexa.pending_mission
    return answer

  def _request_set_pattern(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    pattern = msg['pattern']
    rel_alt = msg['rel_alt']
    heading = msg['heading']

    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif not self.heading_valid(heading):
      answer = dss.auxiliaries.zmq.nack(fcn, 'Heading faulty')
    # Accept
    else:
      if pattern == 'circle':
        # Parse more args
        radius = msg['radius']
        yaw_rate = msg['yaw_rate']
        answer = dss.auxiliaries.zmq.ack(fcn)
        # TODO, set circle pattern
      else:
        # TODO, set above pattern
        answer = dss.auxiliaries.zmq.ack(fcn)
      # TODO, implement set pattern
      answer = dss.auxiliaries.zmq.nack(fcn, 'set_pattern not implemented')
    return answer

  def _request_follow_stream(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif self._in_controls != 'APPLICATION':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Application is not in controls')
    elif not self._hexa.is_flying(): # Actually it is the armed state that is tested
      answer = dss.auxiliaries.zmq.nack(fcn, 'State is not flying')
    elif False: #TODO, pattern not set
      answer = dss.auxiliaries.zmq.nack(fcn, 'Pattern not set')
    # Accept
    else:
      print("Follow stream in early BETA!")
      # Read enable flag directly. Handle socket in task.
      self._hexa.follow_stream_enabled = msg['enable']
      answer = dss.auxiliaries.zmq.ack(fcn)
    return answer

  def _request_set_gimbal(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    roll = msg['roll']
    pitch = msg['pitch']
    yaw = msg['yaw']
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif self._in_controls != 'APPLICATION':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Application is not in controls')
    elif False: # TODO, roll pitch yaw is out of range
      answer = dss.auxiliaries.zmq.nack(fcn, 'Roll, pitch or yaw is out of range fo the gimbal')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
      self._hexa.set_gimbal(msg['pitch'], msg['roll'], msg['yaw'])
      answer = dss.auxiliaries.zmq.nack(fcn, 'set_gimbal check roll pitch yaw not implemented')
      # TODO, implement check roll pitch yaw! ( or just pitch?)
    return answer

  def _request_set_gripper(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    enable = msg['enable']
    CAN_ID = msg['CAN_ID']
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif self._in_controls != 'APPLICATION':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Application is not in controls')
    elif False: # TODO, ohter action in execution
      answer = dss.auxiliaries.zmq.nack(fcn, 'Other action in execution')
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
    return answer

  def _request_photo(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    cmd = msg['cmd']
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    elif self._in_controls != 'APPLICATION':
      answer = dss.auxiliaries.zmq.nack(fcn, 'Application is not in controls')
    elif False: # TODO, Camera resource busy
      answer = dss.auxiliaries.zmq.nack(fcn, 'Camera resource is busy')
    elif not cmd in ('take_photo', 'continous_photo', 'download'):
      answer = dss.auxiliaries.zmq.nack(fcn, 'Cmd faulty')
    # Accept
    else:
      if cmd == 'take_photo':
        answer = dss.auxiliaries.zmq.ack(fcn)
        answer['description'] = 'take_photo'
        # TODO, take_photo
        answer = dss.auxiliaries.zmq.nack(fcn, 'Take photo not implemented')
      elif cmd == 'continous_photo':
        enable = msg['enable']
        publish = msg['publish'] #'off', 'low' or 'high'
        period = msg['period']
        answer = dss.auxiliaries.zmq.ack(fcn)
        if enable:
          descr = 'continous_photo enabled'
        else:
          descr = 'continous_photo disabled'
        answer['description'] = descr
        # TODO, enable/disable continous photo
        answer = dss.auxiliaries.zmq.nack(fcn, 'Continous photo not implemented')
      elif cmd == 'download':
        resolution = msg['resolution']
        index = msg['index']
        # Test more nack reasons
        if False:
          anser = dss.auxiliaries.zmq.nack(fcn, 'Index out of range' + index)
        elif False:
          anser = dss.auxiliaries.zmq.nack(fcn, 'Index string faulty' + index)
        # Accept
        else:
          answer = dss.auxiliaries.zmq.ack(fcn)
          answer['description'] = 'download ' + 'index'
          # TODO, download photo
          answer = dss.auxiliaries.zmq.nack(fcn, 'Download photo not implemented')
    return answer

  def _request_get_armed(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # No nack reasons, accept
    answer = dss.auxiliaries.zmq.ack(fcn)
    answer['armed'] = self._hexa.vehicle.armed
    return answer

  def _request_get_currentWP(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # No nack reasons, accept
    answer = dss.auxiliaries.zmq.ack(fcn)
    answer['currentWP'] = self._hexa.mission_next_wp
    answer['finalWP'] = len(self._hexa.active_mission)-1
    return answer

  def _request_get_flightmode(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # No nack reasons, accept
    answer = dss.auxiliaries.zmq.ack(fcn)
    answer['flightmode'] = self._hexa.get_flight_mode()
    return answer

  def _request_get_metadata(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    ref = msg['ref']
    index = msg['index']
    # Test nack reasons
    if ref not in ('XYZ', 'NED', 'LLA'):
      answer = dss.auxiliaries.zmq.nack(fcn, 'Invalid mission type')
    elif isinstance(index, int):
      if False: # Index out of range: if not 0 < index < self.latest_index
        descr = 'Index out of range, {}'.format(index)
        answer = dss.auxiliaries.zmq.nack(fcn, descr)
        return answer
    elif isinstance(index, str):
      if index not in ('all','latest'):
        descr = 'Index string fualty, {}'.format(index)
        answer = dss.auxiliaries.zmq.nack(fcn, descr)
        return answer
    # Accept (at least one elif above will be true, an accept else statement would not excecute
    answer = dss.auxiliaries.zmq.ack(fcn)
    answer['metadata'] = {'0': {'TODO': 'metadata'}}
    answer = dss.auxiliaries.zmq.nack(fcn, 'Metadata is not implemented')
    return answer

  def _request_get_posD(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # No nack reasons, accept
    answer = dss.auxiliaries.zmq.ack(fcn)
    if self._hexa.vehicle.location.local_frame.down is None:
      posD = 0.0
    else:
      posD = self._hexa.vehicle.location.local_frame.down
    answer['posD'] = posD
    return answer

  def _request_get_PWM(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    channel = msg['channel']
    # No nack reasons, accept
    answer = dss.auxiliaries.zmq.ack(fcn)
    answer['PWM'] = self._hexa.get_channel(13)
    return answer

  def _request_disconnect(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Test nack reasons
    if not self.from_owner(msg):
      descr = 'Requester ({}) is not the DSS owner'.format(msg['id'])
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
      #Enter hover mode
      self._hexa.stop()
      if self._crm is not None :
        #Send an "app_lost" msg to the CRM
        _ = self._crm.app_lost()
    return answer

  def _request_data_stream(self, msg) -> dict:
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    # Parse
    stream = msg['stream']
    enable = msg['enable']
    # Test nack reasons
    if stream not in self._pub_attributes:
      descr = 'Stream faulty, ' + stream
      answer = dss.auxiliaries.zmq.nack(fcn, descr)
    # Accept
    else:
      answer = dss.auxiliaries.zmq.ack(fcn)
      # Update publish attributes dict
      self._pub_attributes[stream]['enabled'] = enable
      # Activate publish of stream
      if enable:
        self._hexa.vehicle.add_attribute_listener(self._pub_attributes[stream]['name'], self._attribute_listener)
        self._logger.info("Global listener added: %s", stream)
      # Deactivate publish of stream
      else:
        self._hexa.vehicle.remove_attribute_listener(self._pub_attributes[stream]['name'], self._attribute_listener)
        self._logger.info("Global listener removed: %s", stream)
    return answer



  def _request_glana(self, msg):
    fcn = dss.auxiliaries.zmq.get_fcn(msg)
    if msg['arg']['cmd'] == 'connect':
      # if disconnected
      if self._hexa.glana.connect():
        return {'fcn': 'ack', 'arg': msg['fcn']}
    elif msg['arg']['cmd'] == 'disconnect':
      if self._hexa.glana.disconnect():
        return {'fcn': 'ack', 'arg': msg['fcn']}
    elif msg['arg']['cmd'] == 'start_rec':
      self._hexa.set_gimbal(-90, 0, 90)
      self._hexa.glana.start_rec()
      self._logger.info('Gimbal is set and camera is recording')
      return {'fcn': 'ack', 'arg': msg['fcn']}
    elif msg['arg']['cmd'] == 'stop_rec':
      self._hexa.glana.stop_rec()
      self._hexa.set_gimbal(-1, 0, 0)
      return {'fcn': 'ack', 'arg': msg['fcn']}

    return {'fcn': 'nack', 'arg': msg['fcn']}


# Function to handle if the link to the application is lost
  def _is_link_lost(self):
    link_lost = False
    if self._connected:
      curr_time = time.time()
      t_link_lost = 10.0
      t_diff = curr_time - self._t_last_owner_msg
      if 0.5*t_link_lost < t_diff < t_link_lost:
        self._logger.warning("Application link degraded")
      elif t_diff >= t_link_lost:
        self._logger.error("Application is disconnected")
        link_lost = True
        self._connected = False
        self._hexa.stop()
        if self._crm :
          _ = self._crm.app_lost()
        if self._in_controls == 'APPLICATION':
          self._logger.error('Lost link to the dss client; DSS took the CONTROLS')
          self._in_controls = 'DSS'
        else:
          self._logger.error('Lost link to the dss client')
    return link_lost


  #############################################################################
  # TASKS
  #############################################################################

  def _task_arm_take_off(self, msg):
    to_alt = msg['height']
    self._hexa.task_arm_take_off(to_alt)

  def _task_disconnect(self, msg):
    # Perform hexa disconnect task
    self._hexa.task_disconnect()
    self._connected = False
    # Kill the DSS if no CRM available
    if self._crm is None:
      self.alive = False

  def _task_gogo(self, msg):
    self._logger.info('Application called for gogo with arg: %s', msg['next_wp'])
    self._hexa.task_gogo(msg['next_wp'])

  # task runs in a thread
  def _task_follow_stream(self, msg):
    # Parse, enable flag parsed in request
    # self.follow_stream_enable = msg['enable']
    ip = msg['ip']
    port = msg['port']

    if self._hexa.follow_stream_enabled:
      # setup the subscription!
      self._sub_stream_socket = dss.auxiliaries.zmq.Sub(self._zmq_context, ip, port, label="follow_stream subscr")
      self._sub_stream_socket.subscribe('LLA')
      # Start follow stream thread
      self._hexa.follow_stream()

    #else: # Put else in request?
      # Stop the subscription and close socket if they exist(!), TODO
      #self._sub_stream_socket.unsubscribe('')
      #self._sub_stream_socket.close()


  def _task_set_gripper(self, msg):
    # Parse
    enable = msg['enable']
    CAN_ID = msg['CAN_ID']
    # Release
    if not enable:
      self._hexa.task_gripper_set(0, CAN_ID)
    elif enable:
      self._hexa.task_gripper_set(1, CAN_ID)
    else:
      raise dss.auxiliaries.exception.Error('gripper task error, check args')

  def _task_rtl(self, msg):
    self._hexa.task_ardupilot_rtl()
    #Application has control and has landed. Switch back to GUIDED
    self._hexa.set_guided_mode()

  def _task_dss_srtl(self, msg):
    self._hexa.task_dss_srtl(msg['hover_time'])
    #Application has control and has landed. Switch back to GUIDED
    self._hexa.set_guided_mode()

  def _task_land(self, msg):
    self._hexa.task_land()
    #Application has control and has landed. Switch back to GUIDED
    self._hexa.set_guided_mode()

  #############################################################################
  # CALLBACKS
  #############################################################################

  def _attribute_listener(self, vehicle, att_name, msg):
    if att_name == 'attitude':
      msg = {"Data": "att", "r": msg.roll, "p": msg.pitch, "y": msg.yaw}
      self._pub_socket.publish('ATT', msg)
      #print("Attitude callback sending log data:", json_msg)
    # LLA
    elif att_name == 'location.global_frame':
      msg = {'lat': msg.lat, 'lon': msg.lon, 'alt': msg.alt, 'heading': -1, 'agl': -1 } # TODO heading and AGL
      self._pub_socket.publish('LLA', msg)
    # NED
    elif att_name == 'location.local_frame':
      msg = {'north': msg.north, 'east': msg.east, 'down': msg.down, 'heading': -1, 'agl': -1} # TODO heading and AGL
      self._pub_socket.publish('NED', msg)
    else:
      self._logger.error('Unknown attribute send to listener: %s', att_name)

  def _clearance_listener(self, vehicle, att_name, value):
    if not self._midstick_check or ( 1400 < self._hexa.get_channel(3) < 1600):
      if self._clearance_state == 'WAITING':
        if value > 1500:
          self._clearance_state = 'HIGH'
      elif self._clearance_state == 'HIGH':
        if value < 1500 :
          self._clearance_state = 'CLEARED'


  #############################################################################
  # THREAD *TASKS*
  #############################################################################

  def _main_task(self):
    '''Executes the current task'''
    while self.alive:
      fcn = self._task['fcn']

      if fcn:
        try:
          task = self._commands[fcn]['task']
          task(self._task)
        except dss.auxiliaries.exception.AbortTask:
          logging.warning('abort current task')
        except dss.auxiliaries.exception.Error:
          logging.critical(traceback.format_exc())

      if self.alive:
        self._task_event.clear()
        self._task_event.wait()

  #############################################################################
  # THREAD *GLANA*
  #############################################################################

  def _main_glana(self):
    '''Monitors the glana service'''
    while True:
      time.sleep(1)
      if self._hexa.glana.connected:
        if self._hexa.glana.recording and not self._hexa.glana.rec_ok():
          self._logger.error('GLANA is not recording anymore... abort and RTL')
          self._hexa.task_ardupilot_rtl()
        if not self._hexa.glana.recording and not self._hexa.glana.up():
          self._logger.error('Lost link to GLANA... abort and RTL')
          self._hexa.task_ardupilot_rtl()
        if self._in_controls != 'APPLICATION':
          self._hexa.glana.stop_rec()
          self._hexa.set_gimbal(-1, 0, 0)
          self._logger.error("Client doesn't have the control anymore... stopped recording")

  #############################################################################
  # THREAD *NETWORK LOG*
  #############################################################################

  def _main_network(self):
    '''Monitors and logs the network status'''
    self._logger.info("Network logger thread enabled")

    # Connect to modem on specified path
    self._modem = Modem("/dev/ttyUSB3")

    # Set up Engineering mode
    # Set up reporting of cell id
    if self._modem.set_modem('network_reg_set_opt2'):
      self._logger.info("Modem set to report Cell-ID on request")
    else:
      self._logger.warning("FAILED: Modem set to report Cell-ID on request")

    # Allocate logfile
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    log_filename = '{}_{}'.format(timestamp, 'network-log.json')
    log_items_filename = '{}_{}'.format(timestamp, 'network-log-items.txt')
    log_file = 'log/' + log_filename
    log_items = 'log/' + log_items_filename

    # Allocate local json log object
    my_log = {}

    # Static information
    my_log['static_info'] = self._modem.get_static_info()

    with open(log_items, 'w', encoding="utf-8") as outfile:
          outfile.write('{ "static_info":')
          outfile.write(json.dumps(my_log['static_info']))

    # Enter loop to collect data
    k = 1
    while k < 5:
        # Get dynamic info
        log_item = self._modem.get_cell_info()

        # Add fake pos data
        log_item['pos'] = {}

        pos = self._hexa.get_position_lla_global()
        print(pos.lat)
        log_item['pos']['lat'] = pos.lat
        log_item['pos']['long'] = pos.lon
        log_item['pos']['alt'] = pos.alt

        # Print data to local log dict. this might be big and have momory issues..
        my_log[str(k)] = {}
        my_log[str(k)] = log_item

        # Make small json item to continously save to log as txt
        temp = {}
        temp[str(k)] = log_item

        log_item_str = json.dumps(log_item)
        with open(log_items, 'a', encoding="utf-8") as outfile:
          # Write comma and add newline
          outfile.write(',')
          new_key = '"' + str(k) + '"' + ':'
          outfile.write(new_key)
          outfile.write(log_item_str)
        time.sleep(1)
        k+=1

    with open(log_items, 'a', encoding="utf-8") as outfile:
      outfile.write('}')

    #print(json.dumps(my_log, indent=4))

    #log_str = json.dumps(my_log, indent=4)
    #print(log_str)

    with open(log_items, 'r', encoding="utf-8") as infile:
      big_json = json.load(infile)
      with open(log_file, 'w', encoding="utf-8") as outfile:
        log_str = json.dumps(big_json, indent=4)
        outfile.write(log_str)

    # log_str = json.dumps(test, indent=4)
    # print(log_str)
    # with open(log_file, 'w', encoding="utf-8") as outfile:
    #       outfile.write(log_str)

    # Close logfile


  #############################################################################
  # THREAD *MAIN*
  #############################################################################

  def _main(self):
    '''Listening for new requests and gcs heartbeats'''
    attempts = 0

    while self.alive:
      # check gcs heartbeats
      ######################
      if self._in_controls == 'APPLICATION' and self.lost_link_to_gcs():
        self._logger.error('Lost link to the gcs heartbeats; DSS taking the CONTROLS')
        self._in_controls = 'DSS'
        continue

      # check flight mode
      ###################
      if self._in_controls in ('APPLICATION', 'DSS'):
        if not self._hexa.expected_flight_mode:
          mode = self._hexa.get_flight_mode()
          self._logger.warning('Unexpected flight mode: %s; PILOT took the CONTROLS', mode)
          self._hexa.set_expected_flight_mode(mode)
          self._in_controls = 'PILOT'
          self._clearance_state = 'WAITING' if self._clearance_check else 'CLEARED'
          continue

      # PILOT
      ######
      if self._in_controls == 'PILOT':
        if not self._hexa.vehicle.is_armable:
          print('\033[K', end='\r') # clear to the end of line
          print('[%s has the CONTROLS] Waiting for vehicle to initialise...' % self._in_controls, end='\r')
        elif self.lost_link_to_gcs():
          print('\033[K', end='\r') # clear to the end of line
          print('[%s has the CONTROLS] Waiting for gcs heartbeats...' % self._in_controls, end='\r')
        elif not self._hexa.is_flight_mode('GUIDED'):
          print('\033[K', end='\r') # clear to the end of line
          print('[%s has the CONTROLS] Waiting for GUIDED mode...' % self._in_controls, end='\r')
        elif self._hexa.get_channel(3) is None:
          print('\033[K', end='\r') # clear to the end of line
          print('[%s has the CONTROLS] Waiting for rc channel 3 to become available...' % self._in_controls, end='\r')
        elif self._midstick_check and (not 1400 < self._hexa.get_channel(3) < 1600):
          print('\033[K', end='\r') # clear to the end of line
          print('[%s has the CONTROLS] Waiting for throttle to mid-stick...' % self._in_controls, end='\r')
        elif not self._connected:
          print('\033[K', end='\r') # clear to the end of line
          print('[%s has the CONTROLS] Waiting for APPLICATION to connect...' % self._in_controls, end='\r')
        elif not self._clearance_state == 'CLEARED':
          print('[%s has the CONTROLS] Waiting for safety pilot to give clearance...' % self._in_controls, end='\r')
        else:
          self._logger.info('APPLICATION got the the CONTROLS')
          self._hexa.set_expected_flight_mode('GUIDED')
          self._in_controls = 'APPLICATION'
          self._hexa.gimbal_stow()
          continue

      # DSS
      ########
      if self._in_controls == 'DSS':
        if self._task['fcn'] == 'rtl':
          if self._task_event.is_set():
            print('\033[K', end='\r') # clear to the end of line
            print('[%s has the CONTROLS] Smart RTL, %s' % (self._in_controls, self._hexa.status_msg), end='\r')
          else:
            self._logger.info('RTL completed. Waiting for PILOT to take CONTROLS')
            continue
        else:
          if self._task_event.is_set():
            self._hexa.abort_task = True
            print('\033[K', end='\r') # clear to the end of line
            print('[%s has the CONTROLS] Waiting for task to abort' % self._in_controls, end='\r')
          else:
            self._hexa.abort_task = False
            self._task = {'fcn': 'rtl'}
            self._task_event.set()

      # APPLICATION
      ########
      if self._in_controls == 'APPLICATION':
        if self._task_event.is_set():
          print('\033[K', end='\r') # clear to the end of line
          print('[%s has the CONTROLS] %s' % (self._in_controls, self._hexa.status_msg), end='\r')
        else:
          print('\033[K', end='\r') # clear to the end of line
          print('[%s has the CONTROLS] idle' % self._in_controls, end='\r')

      # ZMQ
      #####
      try:
        msg = self._serv_socket.recv_json()
        msg = json.loads(msg)
        if self.from_owner(msg):
          self._t_last_owner_msg = time.time()
      except zmq.error.Again:
        _ = self._is_link_lost()
        continue
      #Check if link to application is lost
      if self._is_link_lost():
        continue

      if not self._connected and self.from_owner(msg) and msg['id'] != 'crm':
        self._connected = True
        self._logger.info('Application is connected')

      fcn = msg['fcn'] if 'fcn' in msg else ''

      if fcn != 'heart_beat':
        self._logger.info('Received request: %s', str(msg))

      if fcn in self._commands:
        request = self._commands[fcn]['request']
        task = self._commands[fcn]['task']

        # TODO, we need to try the request prior to executing the task. All nack reasons are handled in the requests
        if task:
          # Nack reasons for all tasks
          if self._task_event.is_set():
            answer = {'fcn': 'nack', 'call': fcn, 'description': 'another task is still running'}
          # Accept task
          else:
            # Test request
            answer = request(msg)
            if dss.auxiliaries.zmq.is_ack(answer):
              self._task = msg
              self._task_event.set()
        else:
          # simple requests are always allowed
          answer = request(msg)
      else:
        print("request not supported")
        print(fcn)
        answer = {'fcn': 'nack', 'arg': msg['fcn'], 'arg2': 'request not supported'}

      answer = json.dumps(answer)
      self._serv_socket.send_json(answer)

      if fcn != 'heart_beat':
        self._logger.info("Replied: %s", answer)

    self._logger.info('DSS Server exited correctly. Have a nice day!')
