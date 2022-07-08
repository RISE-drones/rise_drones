'''
Drone Safety Service *Client*

This is the drone object. Commands can be send and information can be
requested from the drone. This object provides convience methods to
make communication and implementation easy.

It uses the dss.client.DSS object, which is in charge of the socket
amd the actual API as described in documentation.
'''

import json
import logging
import time

import zmq

import dss.auxiliaries
import dss.client

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.0'
__copyright__ = 'Copyright (c) 2019-2021, RISE'
__status__ = 'development'

class Client:
  '''Base class for DSS applications'''
  def __init__(self, timeout, exception_handler=None, context=None):
    '''
    The timeout defines the interval used to send heartbeats when no
    other command has been send.
    '''
    self._logger = logging.getLogger(__name__)

    self._logger.info(f'DSS client_lib {dss.auxiliaries.git.describe()}')

    self._app_id = 'da000'

    self._alive = False
    self._context = context if context else zmq.Context()
    self._dss = None
    self._exception_handler = exception_handler
    self._input_handler = None
    self._input_socket = None
    self._task_queue = dss.auxiliaries.TaskQueue(exception_handler=exception_handler)
    self._thread = None
    self._timeout = timeout
    self._in_controls = False
    self._app_abort = False


  @property
  def alive(self):
    '''Checks if the dss client is alive'''
    return self._alive

  @property
  def app_id(self):
    '''Retruns protected _app_id'''
    return self._app_id

  @property
  def app_abort(self):
    '''Returns protected _app_abort flag'''
    return self._app_abort

  @app_abort.setter
  def app_abort(self, value):
    self._app_abort = value

  @property
  def operator(self):
    '''Returns protected _in_controls for backwards compatibility'''
    self._logger.warning("Use of deprecated property 'operator', use in_controls")
    return self._in_controls

  @property
  def in_controls(self):
    '''Retruns protected _in_controls'''
    return self._in_controls

  #@alive.setter
  #def alive(self, value):
  #  self._alive = value

  # *******************
  # Client base methods
  # *******************

  def set_input_handler(self, port, input_handler):
    '''Defines an asynchronous input handler'''
    if self._input_handler:
      raise dss.auxiliaries.exception.Error('An asynchronous input handler is already defined')

    self._input_handler = input_handler
    self._input_socket = dss.auxiliaries.zmq.Rep(self._context, '*', port, label='input-rep', timeout=self._timeout)
    self._logger.info(f"Starting input server on port {port}")

  def raise_if_aborted(self):
    # Test if controls where taken
    if self.in_controls and not self.is_who_controls('APPLICATION'):
      # Controls were taken
      self._in_controls = False
      raise dss.auxiliaries.exception.AbortTask("Controls where taken")

    if not self._alive:
      raise dss.auxiliaries.exception.AbortTask()

    if self.app_abort:
      self.app_abort = False
      raise dss.auxiliaries.exception.AbortTask()

  def abort(self, msg=None, rtl=False):
    '''Aborts the mission and stops all threads'''
    if msg:
      self._logger.error(msg)

    self._task_queue.clear()
    self._alive = False
    if rtl:
      self.rtl()

  def connect(self, ip, port=None, app_id=None) -> None:
    '''Connects to dss server
    port=None is used to remain backward compatible'''
    if self._thread:
      raise dss.auxiliaries.exception.Error('DSS client is already running')

    if port:
      dss_address = f'tcp://{ip}:{port}'
    else:
      dss_address = ip
      ip, port = ip.rsplit(':', 1)
      _, ip = ip.rsplit('/', 1)

    if app_id is not None:
      self._app_id = app_id
    else:
      logging.error("Convert your code to send app_id upon connect")

    # Connect to DSS
    self._dss = dss.client.DSS(self._context, self._app_id, ip, port, None, timeout=self._timeout)
    self._alive = True

    # Test connection, owner change must have gone through to get ack. Takes some time sometimes
    max_attempt = 20
    for attempt in range(max_attempt):
      try:
        self._dss.heart_beat()
      except dss.auxiliaries.exception.Nack:
        # If the owner change has not gone through, we get nack
        pass
      else:
        # We must have received an ack, correctly connected, break for-loop
        break
      # Give up if no success after maximum number of attempts, raise exception
      if attempt == max_attempt-1:
        self._logger.error('Failed to connect to DSS on %s', dss_address)
        raise dss.auxiliaries.exception.Error('Failed to connect to DSS on %s' % dss_address)
      time.sleep(0.1)

    # DSS class will update dss_id to connected dss when get_info runs.
    try:
      _ = self._dss.get_info()
    except dss.auxiliaries.exception.Nack:
      self._logger.warning('Failed to retreive dss_id from get_info. DSS class might not have a dss_id')

    self._logger.info('Connection to DSS established on %s', dss_address)

  # Connect to a DSS as guest, without beeing the owner
  def connect_as_guest(self, ip, port, app_id) -> None:
    '''Connects to dss server
    port=None is used to remain backward compatible'''
    if self._thread:
      raise dss.auxiliaries.exception.Error('DSS client is already running')

    # Set app id in the Client object
    self._app_id = app_id

    # Connect to DSS
    self._dss = dss.client.DSS(self._context, self._app_id, ip, port, None, timeout=self._timeout)
    self._alive = True

    # DSS class will update dss_id to connected dss when get_info runs.
    try:
      _ = self._dss.get_info()
    except:
      self._logger.error(f'Error, could not connect as guest to tcp://{ip}:{port}')

    self._logger.info(f'Connection to DSS established on tcp://{ip}:{port}')

  def dss_disconnect(self) -> None:
    '''Disconnect to the DSS'''
    self._dss.disconnect()
    self.close_dss_socket()

  def close_dss_socket(self) -> None:
    '''Close the socket to the DSS'''
    self._alive = False
    self._dss._socket.close()
    self._dss = None

  def run(self):
    '''Executes the mission'''
    self._task_queue.start()

    # Handle external inputs, e.g. mission abort
    try:
      while self._alive:
        if self._input_handler:
          try:
            msg = self._input_socket.recv_json()
          except zmq.error.Again:
            pass
          else:
            msg = json.loads(msg)
            try:
              self._input_handler(msg)
            except Exception as error:
              if self._exception_handler:
                self._exception_handler(error)
              answer = json.dumps({'fcn': 'nack', 'call': msg['fcn']})
            else:
              answer = json.dumps({'fcn': 'ack', 'call': msg['fcn']})
            self._input_socket.send_json(answer)
        else:
          time.sleep(0.5)

        if self._task_queue.idling:
          self._logger.info('Mission complete')
          self.abort()
    except KeyboardInterrupt:
      self.abort('Shutdown due to keyboard interrupt', rtl=True)

    # stop task queue
    self._task_queue.stop()

    # close zmq connections
    if self._input_handler:
      self._input_socket.close()
    #self._dss._socket.close()

  def add_task(self, task, arg1=None, arg2=None, arg3=None, arg4=None):
    self._task_queue.add(task, arg1, arg2, arg3, arg4)

  # *******************
  # Convenience methods
  # *******************

  def disable_follow_stream(self):
    self._dss.follow_stream(False, '', 0)

  def enable_follow_stream(self, ip, port):
    self._dss.follow_stream(True, ip, port)

  # Enable data stream
  def enable_data_stream(self, stream):
    self._dss.data_stream(stream=stream, enable=True)

  # Disable data stream
  def disable_data_stream(self, stream):
    self._dss.data_stream(stream=stream, enable=False)

  # Get info pub port or data pub port of connected DSS
  def get_port(self, port_label) -> int:
    answer = self._dss.get_info()
    return int(answer[port_label])

  # Check flight mode
  def is_flight_mode(self, mode) -> bool:
    return self._dss.get_flightmode() == mode

  # Check who controls
  def is_who_controls(self, who) -> bool:
    return self._dss.who_controls() == who
  # Check who owns
  def is_owner(self, owner) -> bool:
    return self._dss.get_owner() == owner
  # Get height
  def get_height(self) -> float:
    return -self._dss.get_posD()

  # Check PWM channel state (used for pilot clearance)
  def is_channel_state(self, channel, state):
    value = self._dss.get_PWM(channel)
    if state == 'LOW':
      return value < 1500
    elif state == 'HIGH':
      return value > 1500
    return None

  # Await pilot clearance (wait for toggle switch)
  def get_clearance(self, channel):
    while self.is_channel(channel, 'LOW'):
      self.raise_if_aborted()
      time.sleep(0.5)
    while self.is_channel(channel, 'HIGH'):
      self.raise_if_aborted()
      time.sleep(0.1)

  # Try to set init point, make sure it is set
  def try_set_init_point(self, heading_ref = 'drone'):
    try:
      self._dss.set_init_point(heading_ref)
    except dss.auxiliaries.exception.Nack:
      # Don't handle nack, init point is probably set already
      pass

  # Arm-takeoff, wait for it to complete
  def arm_and_takeoff(self, height):
    self._logger.info('arm and takeoff (height=%d)', height)
    start_height = None
    while start_height is None:
      try:
        start_height = self.get_height()
      except dss.auxiliaries.exception.Nack:
        pass
    #compensate for init point at different altitude (takeoff to <height> m AGL)
    final_height = height+start_height
    self._dss.arm_take_off(final_height)
    current_height = start_height
    while current_height < final_height * 0.9:
      time.sleep(1.0)
      self.raise_if_aborted()
      try:
        current_height = self.get_height()
      except dss.auxiliaries.exception.Nack:
        pass
      finally:
        rel_height = current_height-start_height
        print('Current height relative takeoff position: %5.1f m' % rel_height, end='\r')

    print('\033[K', end='\r') # clear to the end of line


  # Package handling. Special routine to unload package in case of device busy.
  def load_package(self):
    self._logger.info('Load package')
    self._dss.set_gripper(True, 1)

  def unload_package(self):
    self._logger.info('Unloading package: drop')
    self._dss.set_gripper(False, 1)
    # Since gripper might have been busy, send the drop message again
    time.sleep(5.0)
    self._logger.info('Unloading package: re-drop')
    self._dss.set_gripper(False, 1)
    time.sleep(1)

  # Land and disarm, wait for it to complete
  def land_and_disarm_should_be_task(self):
    self.land()
    self._logger.info('wait and disarm')
    while self.is_armed():
      self.raise_if_aborted()
      print('Altitude: %5.1f m' % self.get_height(), end='\r')
      time.sleep(1.0)
    print('\033[K', end='\r') # clear to the end of line
    self.await_idling()

  # Wait until the controls are handed over
  def await_operator(self):
    self._logger.warning("Use of deprecated method await_operator, use await_controls")
    self.await_controls()

  # Wait until the controls are handed over
  def await_controls(self):
    while not self.is_who_controls('APPLICATION'):
      self._logger.info('APPLICATION waiting for the CONTROLS')
      time.sleep(0.5)
    self._in_controls = True

  # Wait until the copter is idle
  def await_idling(self, raise_if_aborted = True):
    self._logger.info('Waiting for dss to idle')
    while not self._dss.get_idle():
      if raise_if_aborted:
        self.raise_if_aborted()
      time.sleep(0.5)

  # Track waypoints
  def track_waypoints(self, first_wp=0, raise_if_aborted = True):
    '''
    Track wp until end of mission.
    Set raise_if_aborted = False to not throw exception on PILOT in controls
    '''

    last_answer = first_wp
    while self._dss.get_armed():
      time.sleep(1.0)
      if raise_if_aborted:
        self.raise_if_aborted()
      currentWP, _ = self._dss.get_currentWP()
      if currentWP != last_answer:
        self._logger.info('reached wp %s', last_answer)
        last_answer = currentWP
        if last_answer == -1:
          return

  # Start mission flight and track mission progress
  def fly_waypoints_lla(self, first_wp=0):
    self._logger.info('fly waypoints (lla)')
    self._logger.warning('method fly_waypoints_lla is obsolete, use fly_waypoints instead')
    self._dss.gogo(first_wp)
    self.track_waypoints(first_wp)

  def fly_waypoints(self, first_wp=0, raise_if_aborted = True):
    self._logger.info('Fly waypoints')
    self._dss.gogo(first_wp)
    self.track_waypoints(first_wp, raise_if_aborted)

  # Get current and final wp info
  def get_currentWP(self):
    (currentWP, finalWP) = self._dss.get_currentWP()
    return currentWP, finalWP

  # Activate the mission at first_wp of choice
  def gogo(self, first_wp=0):
    self._logger.info(f'Gogo, start wp={first_wp}')
    self._dss.gogo(first_wp)

  def set_default_speed(self, speed):
    self._dss.set_default_speed(speed)

  # Land
  def land(self):
    self._dss.land()
    # Wait for rtl to land
    while self._dss.get_armed():
      self.raise_if_aborted()
      print('Altitude: %5.1f m' % self.get_height(), end='\r')
      time.sleep(1.0)
    #Wait for the task to finish. Does not use raise if aborted since operator will take controls
    self.await_idling(raise_if_aborted=False)


  # Engage autopilot RTL and wait for it to complete
  def rtl(self):
    self._dss.rtl()
    # Wait for rtl to land
    while self._dss.get_armed():
      self.raise_if_aborted()
      print('Altitude: %5.1f m' % self.get_height(), end='\r')
      time.sleep(1.0)
    #Wait for the task to finish. Does not use raise if aborted since operator will take controls
    self.await_idling(raise_if_aborted=False)
    self._in_controls = False
    print('\033[K', end='\r') # clear to the end of line

  # Engage dss srtl and wait for idle
  def dss_srtl(self, hover_time):
    self._dss.dss_srtl(hover_time)
    height = 0.0
    while self._dss.get_armed():
      self.raise_if_aborted()
      try:
        height = self.get_height()
      except dss.auxiliaries.exception.Nack:
        pass
      finally:
        print('Altitude: %5.1f m' % height, end='\r')
      time.sleep(1.0)
    #Wait for the task to finish. Does not use raise if aborted since operator will take controls
    self.await_idling(raise_if_aborted=False)
    self._in_controls = False
    print('\033[K', end='\r') # clear to the end of line

  # Get drone armed state
  def is_armed(self):
    return self._dss.get_armed()

  # Above pattern
  def set_pattern_above(self, rel_alt, heading):
    self._dss.set_pattern('above', rel_alt, heading)

  # Circle pattern
  def set_pattern_circle(self, rel_alt, radius, heading, yaw_rate):
    self._dss.set_pattern('circle', rel_alt, heading, radius, yaw_rate)

  # Set pattern, dict arg
  def set_pattern_dict(self, pattern):
    call = 'set_pattern'
    msg = pattern
    msg['fcn'] = call
    msg['id'] = self.app_id
    _ = self._dss._socket.send_and_receive(msg)

  def set_geofence(self, height_low, height_high, radius):
    self._dss.set_geofence(height_low, height_high, radius)

  def set_init_point(self, heading_ref):
    self._dss.set_init_point(heading_ref)

  def reset_dss_srtl(self):
    self._dss.reset_dss_srtl()

  # All missions reference frames are handeled the same way, reduce to one function TODO
  def upload_mission_LLA(self, mission):
    self._dss.upload_mission_LLA(mission)

  # Upload NED mission
  def upload_mission_NED(self, mission):
    self._dss.upload_mission_NED(mission)

  # Upload XYZ mission
  def upload_mission_XYZ(self, mission):
    self._dss.upload_mission_XYZ(mission)

  # Set gimbal
  def set_gimbal(self, roll, pitch, yaw):
    self._dss.set_gimbal(roll, pitch, yaw)


  # *************
  # Photo library
  # *************

  def photo_connect(self, name):
    '''Not documented: keep it or remove it?'''
    raise dss.auxiliaries.exception.NotImplemented()

  def photo_disconnect(self):
    '''Not documented: keep it or remove it?'''
    raise dss.auxiliaries.exception.NotImplemented()

  def get_metadata(self, ref, index) -> dict:
    return self._dss.get_metadata(ref, index)

  # Take a photo
  def photo_take_photo(self):
    self._dss.photo('take_photo')

  # Control continous photo
  def photo_continous_photo(self, enable, period=2, publish="off"):
    self._dss.photo('continous_photo', '', '', enable, period, publish)

  # Photo download
  def photo_download(self, index, resolution):
    self._dss.photo('download', resolution, index)

  # Photo recording
  def photo_rec(self, enable):
    self._dss.photo(cmd='record', enable=enable)

  # ****************************************
  # Glana specific, should be separate file?
  #*****************************************

  # Glana library

  def glana_connect(self):
    '''Connect to GLANA service'''
    raise dss.auxiliaries.exception.NotImplemented()

  def glana_disconnect(self):
    '''Disconnect from GLANA service'''
    raise dss.auxiliaries.exception.NotImplemented()

  def glana_start_rec(self):
    '''Start recording when reaching the first wp'''
    raise dss.auxiliaries.exception.NotImplemented()

  def glana_stop_rec(self):
    '''Stop recording when reaching the last wp'''
    raise dss.auxiliaries.exception.NotImplemented()

  # NOT IMPLEMENTED FUNCTIONS FROM OLD APPLICATIONS
  def save_home_position(self):
    raise dss.auxiliaries.exception.NotImplemented()

  def return_to_home(self):
    raise dss.auxiliaries.exception.NotImplemented()

  def land_and_disarm(self):
    raise dss.auxiliaries.exception.NotImplemented()

  def is_channel(self, channel, second_argument):
    raise dss.auxiliaries.exception.NotImplemented()
