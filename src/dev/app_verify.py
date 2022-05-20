#!/usr/bin/env python3

'''
APP "app_verify"

This app is used to verify functionality on the Ardupilot DSS
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

import numpy as np

#--------------------------------------------------------------------#

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>'
__version__ = '0.1.0'
__copyright__ = 'Copyright (c) 2022, RISE'
__status__ = 'development'

#--------------------------------------------------------------------#

_logger = logging.getLogger('dss.app_verify')
_context = zmq.Context()

#--------------------------------------------------------------------#
# Verify application for one drone - README.
#
# This application helps you to connect to crm and allocate a drone
# and to verify functionality
# Quit application by calling kill() or Ctrl+C
#
# #--------------------------------------------------------------------#
#--------------------------------------------------------------------#
class Waypoint():
  def __init__(self):
    self.lat = 0.0
    self.lon = 0.0
    self.alt = 0.0

  def set_lla(self, lat, lon, alt):
    self.lat = lat
    self.lon = lon
    self.alt = alt

  def copy_lla(self, other_wp):
    self.lat = other_wp.lat
    self.lon = other_wp.lon
    self.alt = other_wp.alt

def ne_to_ll(loc1, d_northing, d_easting):
  d_lat = d_northing/(1852*60)
  d_lon = d_easting/(1852*60*np.cos(loc1.lat/180*np.pi))
  return (d_lat, d_lon)

def get_3d_distance(loc1, loc2):
  dlat = loc2.lat - loc1.lat
  dlon = loc2.lon - loc1.lon
  dalt = loc2.alt - loc1.alt

  # Convert to meters
  d_northing = dlat * 1852 * 60
  d_easting = dlon *1852 * 60 * np.cos(loc1.lat/180*np.pi)

  # Calc distances
  d_2d = np.sqrt(d_northing**2 + d_easting**2)
  d_3d = np.sqrt(d_northing**2 + d_easting**2 + dalt**2)

  # Calc bearing
  bearing = np.arctan2(d_easting, d_northing)
  return (d_northing, d_easting, dalt, d_2d, d_3d, bearing)

class AppVerify():
  def __init__(self, app_ip, app_id, crm):
    # Create Client object
    self.drone = dss.client.Client(timeout=2000, exception_handler=None, context=_context)

    # Create CRM object
    self.crm = dss.client.CRM(_context, crm, app_name='app_verify.py', desc='Verification application', app_id=app_id)

    self._alive = True
    self._dss_data_thread = None
    self._dss_data_thread_active = False
    self._dss_info_thread = None
    self._dss_info_thread_active = False

    # Find the VPN ip of host machine
    self._app_ip = app_ip
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
    _ = self.crm.register(self._app_ip, self._app_socket.port)

    # All nack reasons raises exception, registreation is successful
    _logger.info('App %s listening on %s:%s', self.crm.app_id, self._app_socket.ip, self._app_socket.port)
    _logger.info(f'App_verify registered with CRM: {self.crm.app_id}')

    # Update socket labels with received id
    self._app_socket.add_id_to_label(self.crm.app_id)
    self._info_socket.add_id_to_label(self.crm.app_id)

    # Supported commands from ANY to APP
    self._commands = {'get_info':     {'request': self._request_get_info}}
    self.init_pos_received = False
    self.drone_pos = Waypoint()
    #geofence parameters
    self.delta_r_max = 50.0
    self.height_max = 30.0
    self.height_min = 8.0
    #take-off height
    self.takeoff_height = 12.0
    self.wp_height = 20.0
    #Parameters for generate_random_wp()
    self.default_speed = 3.0
    #distance between waypoints
    self.wp_dist = 20.0

#--------------------------------------------------------------------#
  @property
  def alive(self):
    '''checks if application is alive'''
    return self._alive

#--------------------------------------------------------------------#
  # This method runs on KeyBoardInterrupt, time to release resources and clean up.
  # Disconnect connected drones and unregister from crm, close ports etc..
  def kill(self):
    _logger.info("Closing down...")
    self._alive = False
    # Kill info and data thread
    self._dss_info_thread_active = False
    self._dss_data_thread_active = False
    self._info_socket.close()

    # Unregister APP from CRM
    _logger.info("Unregister from CRM")
    answer = self.crm.unregister()
    if not dss.auxiliaries.zmq.is_ack(answer):
      _logger.error('Unregister failed: {answer}')
    _logger.info("CRM socket closed")

    # Disconnect drone if drone is alive
    if self.drone.alive:
      #wait until other DSS threads finished
      time.sleep(0.5)
      _logger.info("Closing socket to DSS")
      self.drone.close_dss_socket()

    _logger.debug('~ THE END ~')

#--------------------------------------------------------------------#
# Application reply thread
  def _main_app_reply(self):
    _logger.info(f'Reply socket is listening on: {self._app_socket.port}')
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
    _logger.info("Reply socket closed, thread exit")

#--------------------------------------------------------------------#
# Application reply: 'get_info'
  def _request_get_info(self, msg):
    answer = dss.auxiliaries.zmq.ack(msg['fcn'])
    answer['id'] = self.crm.app_id
    answer['info_pub_port'] = self._info_socket.port
    answer['data_pub_port'] = None
    return answer

#--------------------------------------------------------------------#
  # Setup the DSS info stream thread
  def setup_dss_info_stream(self):
    #Get info port from DSS
    answer = self.drone._dss.get_info()
    info_port = answer['info_pub_port']
    if info_port:
      self._dss_info_thread = threading.Thread(
        target=self._main_info_dss, args=[self.drone._dss.ip, info_port])
      self._dss_info_thread_active = True
      self._dss_info_thread.start()

#--------------------------------------------------------------------#
  # Setup the DSS data stream thread
  def setup_dss_data_stream(self):
    #Get data port from DSS
    answer = self.drone._dss.get_info()
    data_port = answer['data_pub_port']
    if data_port:
      self._dss_data_thread = threading.Thread(
        target=self._main_data_dss, args=[self.drone._dss.ip, data_port])
      self._dss_data_thread_active = True
      self._dss_data_thread.start()

#--------------------------------------------------------------------#
  # The main function for subscribing to info messages from the DSS.
  def _main_info_dss(self, ip, port):
    # Enable LLA stream
    self.drone._dss.data_stream('LLA', True)
    # Create info socket and start listening thread
    info_socket = dss.auxiliaries.zmq.Sub(_context, ip, port, "info " + self.crm.app_id)
    while self._dss_info_thread_active:
      try:
        (topic, msg) = info_socket.recv()
        if topic == "LLA":
          self.drone_pos.lat = msg['lat']
          self.drone_pos.lon = msg['lon']
          self.drone_pos.alt = msg['alt']
          if not self.init_pos_received:
            self.init_pos_received = True
        else:
          _logger.warning("Topic not recognized on info link "+topic)
      except:
        pass
    info_socket.close()
    _logger.info("Stopped thread and closed info socket")

#--------------------------------------------------------------------#
  # The main function for subscribing to data messages from the DSS.
  def _main_data_dss(self, ip, port):
    # Enable LLA stream
    # Create info socket and start listening thread
    data_socket = dss.auxiliaries.zmq.Sub(_context, ip, port, "data " + self.crm.app_id)
    while self._dss_data_thread_active:
      try:
        (topic, msg) = data_socket.recv()
        if topic in ('photo', 'photo_low'):
          data = dss.auxiliaries.zmq.string_to_bytes(msg["photo"])
          photo_filename = msg['metadata']['filename']
          dss.auxiliaries.zmq.bytes_to_image(photo_filename, data)
          json_filename = photo_filename[:-4] + ".json"
          dss.auxiliaries.zmq.save_json(json_filename, msg['metadata'])
          print("Photo saved to " + msg['metadata']['filename']  + "\r")
          print("Photo metadata saved to " + json_filename + "\r")
        else:
          print("Topic not recognized on data link: ", (topic, msg))
      except:
        pass
    data_socket.close()
    _logger.info("Stopped thread and closed data socket")

#------------------------TASKS----------------------------------------#
  def task_connect_to_drone(self):
    drone_received = False
    while self.alive and not drone_received:
      # Get a drone
      answer = self.crm.get_drone(capability='camera')
      if dss.auxiliaries.zmq.is_nack(answer):
        _logger.debug("No drone available - sleeping for 2 seconds")
        time.sleep(2.0)
      else:
        drone_received = True

    # Connect to the drone, set app_id in socket
    self.drone.connect(answer['ip'], answer['port'], app_id=self.crm.app_id)
    _logger.info("Connected as owner of drone: [%s]", self.drone._dss.dss_id)

    # Setup info stream to DSS
    self.setup_dss_info_stream()
    self.drone.try_set_init_point()
    self.drone.set_geofence(max(2,self.height_min-2), self.height_max+2, self.delta_r_max+10)

  def task_await_init_point(self):
    # Wait until info stream up and running
    while self.alive and not self.init_pos_received:
      _logger.debug("Waiting for start position from drone...")
      time.sleep(1.0)

  def task_launch_drone(self, height):
    #await for controls
    self.drone.await_controls()
    self.drone.arm_and_takeoff(height)
    self.drone.reset_dss_srtl()

  def generate_wps(self, n_wps, random=False):
    #Compute distance from start position
    mission = {}
    current_wp = Waypoint()
    current_wp.copy_lla(self.drone_pos)
    for id in range(0, n_wps):
      if random:
        delta_dir = np.random.uniform(-np.pi, np.pi)
      else:
        if id == 0:
          delta_dir = 225.0*np.pi/180
        elif id == 1:
          delta_dir = 315.0*np.pi/180
      #Compute new lat lon
      d_northing = self.wp_dist*np.cos(delta_dir)
      d_easting =  self.wp_dist*np.sin(delta_dir)
      (d_lat, d_lon) = ne_to_ll(current_wp, d_northing, d_easting)
      new_lat = current_wp.lat + d_lat
      new_lon = current_wp.lon + d_lon
      # Compute new altitude (stay on the same)
      #new_height = self.height_min + 0.5*(self.height_max-self.height_min)
      new_height = self.wp_height
      current_wp.set_lla(new_lat, new_lon, new_height)

      id_str = "id%d" % id
      mission[id_str] = {
        "lat" : new_lat, "lon": new_lon, "alt": new_height, "alt_type": "relative", "heading": "course", "speed": self.default_speed
      }

    return mission
  def task_generate_mission(self, n_wps, random=False):
    mission = self.generate_wps(n_wps, random)
    self.drone.upload_mission_LLA(mission)
    time.sleep(0.5)

  def task_goto_wps(self):
    #Compute random mission
    # Fly waypoints, allow PILOT intervention.
    start_wp = 0
    while True:
      try:
        self.drone.fly_waypoints(start_wp)
      except dss.auxiliaries.exception.Nack as nack:
        if nack.msg == 'Not flying':
          _logger.info("Pilot has landed")
        else:
          _logger.info("Fly mission was nacked %s", nack.msg)
        break
      except dss.auxiliaries.exception.AbortTask:
        # PILOT took controls
        (currentWP, _) = self.drone.get_currentWP()
        # Prepare to continue the mission
        start_wp = currentWP
        _logger.info("Pilot took controls, awaiting PILOT action")
        self.drone.await_controls()
        _logger.info("PILOT gave back controls")
        # Try to continue mission
        continue
      else:
        # Mission is completed
        break

  def task_monitor_controls(self):
    while self.alive:
      while not self.drone.is_who_controls('APPLICATION'):
        time.sleep(0.5)
      _logger.info("APPLICATION got controls")
      while not self.drone.is_who_controls('PILOT'):
        time.sleep(0.5)
      _logger.info('PILOT took controls')


#--------------------------------------------------------------------#
  # Main function
  def main(self):
    # Execute tasks
    self.task_connect_to_drone()
    self.task_await_init_point()
    #Upload a mission
    self.task_generate_mission(n_wps=2)
    # Test give and take controls
    # self.task_monitor_controls()
    self.task_launch_drone(self.takeoff_height)
    self.task_goto_wps()
    #Perform rtl
    self.drone.rtl()



#--------------------------------------------------------------------#
def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='APP "app_template"', allow_abbrev=False, add_help=False)
  parser.add_argument('-h', '--help', action='help', help=argparse.SUPPRESS)
  parser.add_argument('--app_ip', type=str, help='ip of the app', required=True)
  parser.add_argument('--id', type=str, default=None, help='id of this app_template instance if started by crm')
  parser.add_argument('--crm', type=str, help='<ip>:<port> of crm', required=True)
  parser.add_argument('--log', type=str, default='debug', help='logging threshold')
  parser.add_argument('--owner', type=str, help='id of the instance controlling app_template - not used in this use case')
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout')
  args = parser.parse_args()

  # Identify subnet to sort log files in structure
  subnet = dss.auxiliaries.zmq.get_subnet(ip=args.app_ip)
  # Initiate log file
  dss.auxiliaries.logging.configure('app_verify', stdout=args.stdout, rotating=True, loglevel=args.log, subdir=subnet)

  # Create the Template class
  try:
    app = AppVerify(args.app_ip, args.id, args.crm)
  except dss.auxiliaries.exception.NoAnswer:
    _logger.error('Failed to instantiate application: Probably the CRM couldn\'t be reached')
    sys.exit()
  except:
    _logger.error('Failed to instantiate application\n%s', traceback.format_exc())
    sys.exit()

  # Try to setup objects and initial sockets
  try:
    # Try to run main
    app.main()
  except KeyboardInterrupt:
    print('', end='\r')
    _logger.warning('Shutdown due to keyboard interrupt')
  except dss.auxiliaries.exception.Nack as error:
    _logger.error(f'Nacked when sending {error.fcn}, received error: {error.msg}')
  except dss.auxiliaries.exception.NoAnswer as error:
    _logger.error(f'NoAnswer when sending: {error.fcn} to {error.ip}:{error.port}')
  except:
    _logger.error(f'unexpected exception\n{traceback.format_exc()}')

  try:
    app.kill()
  except:
    _logger.error(f'unexpected exception\n{traceback.format_exc()}')


#--------------------------------------------------------------------#
if __name__ == '__main__':
  _main()
