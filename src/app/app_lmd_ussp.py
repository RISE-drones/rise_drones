#!/usr/bin/env python3

'''
APP "app_lmd"

Input parameters
1. Mission - the misssion to execute
2. Start WP - where to start

This application
1. Connects to the CRM & USSP
2. Asks for an available drone resource with correct capability
3. Read and parse the mission
4. Executes the mission (pickup, drop off)
5. Finish the mission by landing at the return location
'''

import argparse
import json
import logging
import sys
import threading
import time
import traceback
import datetime
import copy
import uuid
import math
import zmq

import dss.auxiliaries
import dss.client
import dss.auxiliaries.config

#--------------------------------------------------------------------#

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '0.1.0'
__copyright__ = 'Copyright (c) 2022, RISE'
__status__ = 'development'

#--------------------------------------------------------------------#

_logger = logging.getLogger('dss.app_lmd')
_context = zmq.Context()


class Waypoint():
  def __init__(self, lat=0.0, lon=0.0, alt=0.0):
    self.lat = lat
    self.lon = lon
    self.alt = alt
  def set_lla(self, lat, lon, alt):
    self.lat = lat
    self.lon = lon
    self.alt = alt

#--------------------------------------------------------------------#
class AppLmd():
  def __init__(self, app_ip, app_id, crm, mission, start_wp):
    # Create Client object
    self.drone = dss.client.Client(timeout=2000, exception_handler=None, context=_context)

    # Create CRM object
    self.crm = dss.client.CRM(_context, crm, app_name='app_lmd.py', desc='LMD mission', app_id=app_id)

    self._alive = True
    self._dss_data_thread = None
    self._dss_data_thread_active = False
    self._dss_info_thread = None
    self._dss_info_thread_active = False

    self._app_ip = app_ip

    # load mission from file
    with open(mission, encoding='utf-8') as handle:
      self.wps_to_visit = json.load(handle)
      if "source_file" in self.wps_to_visit:
        self.wps_to_visit.pop("source_file")


    for wp in self.wps_to_visit.values():
      wp['status'] = "pending"
    self.start_wp = start_wp
    # Missions
    self.missions = []
    #geofence parameters
    self.delta_r_max = dss.auxiliaries.config.config['app_lmd_ussp']['delta_r_max']
    self.height_max = dss.auxiliaries.config.config['app_lmd_ussp']['height_max']
    self.height_min = dss.auxiliaries.config.config['app_lmd_ussp']['height_min']
    #speed parameters
    self.horizontal_speed = dss.auxiliaries.config.config['app_lmd_ussp']['horizontal_speed']
    #
    self.drone_data = {"pos": Waypoint(), "time": 0.0, "heading": 0.0, "velocity": [0.0, 0.0, 0.0]}
    self.start_pos = Waypoint()
    self.start_pos_received = False
    self.drone_lla_lock = threading.Lock()
    self.uas_id = None
    self.operator_id = dss.auxiliaries.config.config['app_lmd_ussp']['operator_id']
    self.clearance_landing = False
    self.plan_withdrawn = False

    # The application sockets
    # Use ports depending on subnet used to pass RISE firewall
    # Rep: ANY -> APP
    self._app_socket = dss.auxiliaries.zmq.Rep(_context, label='app', min_port=self.crm.port, max_port=self.crm.port+50)
    # Pub: APP -> ANY
    self._info_socket = dss.auxiliaries.zmq.Pub(_context, label='info', min_port=self.crm.port, max_port=self.crm.port+50)

    # Start the app reply thread
    self._app_reply_thread = threading.Thread(target=self._main_app_reply, daemon=True)
    self._app_reply_thread.start()

    # Supported commands from ANY to APP
    self._commands = {'push_dss':         {'request': self._request_push_dss}, # Not implemented
                      'get_info':         {'request': self._request_get_info},
                      'clearance_landing':{'request': self._request_clearance_landing}}

    # Register with CRM (self.crm.app_id is first available after the register call)
    _ = self.crm.register(self._app_ip, self._app_socket.port)

    # Update socket labels with received id
    self._app_socket.add_id_to_label(self.crm.app_id)
    self._info_socket.add_id_to_label(self.crm.app_id)

    # All nack reasons raises exception, registration is successful
    _logger.info('App %s listening on %s:%s', self.crm.app_id, self._app_socket.ip, self._app_socket.port)
    _logger.info('App_lmd registered with CRM: %s', self.crm.app_id)
    #USSP parameters
    self.ussp_ip = dss.auxiliaries.config.config['app_lmd_ussp']['ussp_ip']
    self.ussp_req_port = dss.auxiliaries.config.config['app_lmd_ussp']['ussp_req_port']
    self.ussp_pub_port = dss.auxiliaries.config.config['app_lmd_ussp']['ussp_pub_port']
    self.ussp_sub_port = dss.auxiliaries.config.config['app_lmd_ussp']['ussp_sub_port']
    _logger.info(f'App LMD conneting to USSP: {self.ussp_ip}')
    self.ussp = dss.client.UsspClientLib(app_id, _context)
    self.ussp.connect(self.ussp_ip, self.ussp_req_port, self.ussp_pub_port, self.ussp_sub_port)
    self.application_state = "idle"




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
    _logger.info('Reply socket is listening on: %s', self._app_socket.port)
    while self.alive:
      try:
        msg = self._app_socket.recv_json()
        msg = json.loads(msg)
        fcn = msg['fcn'] if 'fcn' in msg else ''

        if fcn in self._commands:
          request = self._commands[fcn]['request']
          answer = request(msg)
        else:
          answer = dss.auxiliaries.zmq.nack(msg['fcn'], 'Request not supported')
        answer = json.dumps(answer)
        self._app_socket.send_json(answer)
      except:
        pass
    self._app_socket.close()
    _logger.info("Reply socket closed, thread exit")

#--------------------------------------------------------------------#
# Application reply: 'push_dss'
  def _request_push_dss(self, msg):
    answer = dss.auxiliaries.zmq.nack(msg['fcn'], 'Not implemented')
    return answer

#--------------------------------------------------------------------#
# Application reply: 'get_info'
  def _request_get_info(self, msg):
    answer = dss.auxiliaries.zmq.ack(msg['fcn'])
    answer['id'] = self.crm.app_id
    answer['info_pub_port'] = self._info_socket.port
    answer['data_pub_port'] = None
    return answer
#--------------------------------------------------------------------#
# Application reply: 'clearance_landing'
  def _request_clearance_landing(self, msg):
    self.clearance_landing = True
    answer = dss.auxiliaries.zmq.ack(msg['fcn'])
    return answer

#--------------------------------------------------------------------#
  # Setup the DSS info stream thread
  def setup_dss_info_stream(self):
    #Get info port from DSS
    info_port = self.drone.get_port('info_pub_port')
    if info_port:
      self._dss_info_thread = threading.Thread(
        target=self._main_info_dss, args=[self.drone._dss.ip, info_port])
      self._dss_info_thread_active = True
      self._dss_info_thread.start()

#--------------------------------------------------------------------#
  # The main function for subscribing to info messages from the DSS.
  def _main_info_dss(self, ip, port):
    # Enable streams
    self.drone.enable_data_stream('LLA')
    #self.drone.enable_data_stream('battery')
    # Create info socket and start listening thread
    info_socket = dss.auxiliaries.zmq.Sub(_context, ip, port, "info " + self.crm.app_id)
    while self._dss_info_thread_active:
      try:
        (topic, msg) = info_socket.recv()
        if topic == "LLA":
          self.drone_lla_lock.acquire()
          self.drone_data["time"] = datetime.datetime.utcnow()
          self.drone_data["pos"].set_lla(msg['lat'], msg['lon'], msg['alt'])
          self.drone_data["heading"] = msg['heading']
          self.drone_data["velocity"] = msg['velocity']
          self.drone_lla_lock.release()
          if not self.start_pos_received:
            self.start_pos.set_lla(msg['lat'], msg['lon'], msg['alt'])
            self.start_pos_received = True
        elif topic == 'battery':
          _logger.debug("Not implemented yet...")
        else:
          _logger.warning("Topic not recognized on info link: %s",topic)
      except:
        pass
    info_socket.close()
    _logger.info("Stopped thread and closed info socket")

  def _stream_nrid(self):
    #Initialize NRID message
    self.ussp.initialize_nrid_msg(self.operator_id, self.uas_id)
    # Assume operator at initial point
    self.ussp.update_nrid_operator_location(self.uas_id, self.start_pos.lat, self.start_pos.lon)
    # Set accuracies
    self.ussp.update_nrid_accuracies(self.uas_id, t_acc=4, alt_acc=4, h_acc=11, speed_acc=0)
    while self.alive :
      self.drone_lla_lock.acquire()
      drone_data = self.drone_data
      self.drone_lla_lock.release()
      speed = math.sqrt(drone_data['velocity'][0]**2 + drone_data['velocity'][1]**2)
      if speed > 0.1:
        bearing = (180/math.pi)*math.atan2(drone_data['velocity'][1], drone_data['velocity'][0])
      else:
        bearing = drone_data['heading']
      height = self.drone_data["pos"].alt-self.start_pos.alt
      self.ussp.update_nrid_state(self.uas_id, drone_data["time"], drone_data["pos"].lat, drone_data["pos"].lon, drone_data["pos"].alt, height=height, bearing=bearing, speed=speed, vert_speed=drone_data['velocity'][2])
      self.ussp.publish_nrid_msg(self.uas_id)
      time.sleep(1.0)

  def _ussp_subscriber_thread(self):
    self.ussp.subscribe_to_topic(self.uas_id)
    while self.alive:
      try:
        (_, msg) = self.ussp.receive_subscribe_data()
        if "message" in msg:
          if msg["message"] == "plan withdrawn":
            _logger.warning("Plan withdrawn received from the USSP. Trying to replan!")
            self.plan_withdrawn = True
            self.drone.app_abort = True
          else:
            _logger.info("Unknown message from USSP received")
      except:
        pass

  def update_wps_to_visit(self):
    self.wps_to_visit = {}
    for mission in self.missions:
      #Cancel pending missions..
      if mission["status"] == "pending":
        self.ussp.cancel_plan(mission["plan ID"])
      #Add all waypoints associated to a mission that is not ended
      if mission["status"] != "ended":
        #Add the end position to the list of wps to visit
        self.wps_to_visit[mission["type"]] = {"lat": mission["final pos"].lat, "lon": mission["final pos"].lon, "alt": mission["final pos"].alt, "status": mission["status"]}
    #Reset missions list and plan withdrawn flag
    self.plan_withdrawn = False
    self.missions = {}
  # Generate missions based on positions to visit
  def generate_ussp_lmd_missions(self):
    # Request flight authorizations from the USSP
    takeoff_time = datetime.datetime.utcnow() + datetime.timedelta(minutes=1)
    current_position = copy.deepcopy(self.drone_data["pos"])
    for wp_type, waypoint in self.wps_to_visit.items():
      position = Waypoint(waypoint["lat"], waypoint["lon"], waypoint["alt"])
      use_altitude = False
      #Check if the drone is currently in the air (plan withdrawn received)
      if waypoint["status"] == "running":
        use_altitude = True
        takeoff_time = datetime.datetime.utcnow() + datetime.timedelta(seconds=10)
        position.alt = self.ussp.query_ground_height(position.lat, position.lon)
      positions = [current_position, position]
      (plan_id, delay) = self.ussp.request_plan(self.operator_id, self.uas_id, epsg=4979, use_altitude=use_altitude, positions=positions, takeoff_time=takeoff_time, speed=self.horizontal_speed, max_speed=15.0, ascend_rate=2.0, descend_rate=1.0)
      if plan_id is None or delay is None:
        raise dss.auxiliaries.exception.Error
      time.sleep(delay)
      (status, plan) = self.ussp.get_plan(plan_id)
      _logger.info(f"status from get plan: {status}")
      if status != "authorized":
        raise dss.auxiliaries.exception.Error
      #Accept plan
      if not self.ussp.accept_plan(plan_id):
        raise dss.auxiliaries.exception.Error
      #Convert to internal representation
      wp_mission = self.ussp.transform_plan(plan, use_altitude)
      #save mission
      mission = {}
      mission["final pos"] = position
      mission["wp_mission"] = wp_mission
      mission["takeoff_time"] = datetime.datetime.fromisoformat(plan[0]["time"])
      mission["plan ID"] = plan_id
      mission["type"] = wp_type
      mission["status"] = waypoint["status"]
      if mission["status"] == "pending":
        mission["takeoff_height"] = min(plan[1]["position"][2] - plan[0]["position"][2], 30.0)
        _logger.info(f"takeoff_height: {mission['takeoff_height']}, wp_mission : {wp_mission}")
      self.missions.append(mission)
      #Update takeoff time
      current_position = position
      takeoff_time = datetime.datetime.fromisoformat(plan[-1]["time"]) + datetime.timedelta(seconds=plan[-1]["time margin"]) + datetime.timedelta(seconds=10)

#------------------------TASKS----------------------------------------#
  def connect_to_drone(self, capabilities):
    drone_received = False
    while self.alive and not drone_received:
      # Get a drone
      answer = self.crm.get_drone(capabilities=capabilities)
      if dss.auxiliaries.zmq.is_nack(answer):
        _logger.debug("No drone with LMD capability available - sleeping for 2 seconds")
        time.sleep(2.0)
      else:
        drone_received = True

    # Connect to the drone, set app_id in socket
    self.drone.connect(answer['ip'], answer['port'], app_id=self.crm.app_id)
    _logger.info("Connected as owner of drone: [%s]", self.drone._dss.dss_id)

    # Setup info stream to DSS
    self.setup_dss_info_stream()

    self.uas_id = str(uuid.uuid1())

  def initialize_waypoints(self, waypoints, reset_geofence=True):
    self.drone.try_set_init_point()
    if reset_geofence:
      self.drone.set_geofence(self.height_min, self.height_max, self.delta_r_max)
    self.drone.upload_mission_LLA(waypoints)

  def launch_drone(self, takeoff_height, reset_dss_srtl=True):
    self.drone.await_controls()
    self.drone.arm_and_takeoff(takeoff_height)
    if reset_dss_srtl:
      self.drone.reset_dss_srtl()

  def await_clearance_landing(self):
    #TODO Set to false when being implemented
    self.clearance_landing = True
    while not self.clearance_landing:
      self.drone.raise_if_aborted()
      time.sleep(1.0)

  def fly_waypoints(self):
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
        if self.plan_withdrawn:
          #USSP plan withdrawn received - need to replan!
          break
        else:
          # PILOT took controls
          (current_wp, _) = self.drone.get_currentWP()
          # Prepare to continue the mission
          start_wp = current_wp
          _logger.info("Pilot took controls, awaiting PILOT action")
          self.drone.await_controls()
          _logger.info("PILOT gave back controls")
          # Try to continue mission
          continue
      else:
        # Mission is completed
        break


#--------------------------------------------------------------------#
# Main function
  def main(self):
    # Connect to a delivery drone
    capabilities=['LMD']
    self.connect_to_drone(capabilities)
    # Start USSP subscriber thread
    ussp_sub_thread = threading.Thread(target=self._ussp_subscriber_thread, daemon=True)
    ussp_sub_thread.start()
    #Reset DSS SRTL and geofence only once
    reset_dss_srtl = True
    reset_geofence = True
    while self.alive and not self.start_pos_received:
      _logger.debug("Waiting for the drone to stream its current position")
      time.sleep(0.5)
    # Start streaming NRID
    nrid_thread = threading.Thread(target=self._stream_nrid, daemon=True)
    nrid_thread.start()
    return_reached = False
    self.application_state = "planning"
    while not return_reached:
      # Generate USSP missions
      self.generate_ussp_lmd_missions()
      self.application_state = "executing"
      for mission in self.missions:
        self.initialize_waypoints(mission["wp_mission"], reset_geofence)
        #Wait for takeoff time
        while datetime.datetime.utcnow() + datetime.timedelta(seconds=10) < mission["takeoff_time"]:
          _logger.info(f"Waiting to start mission execution, time remaining: {mission['takeoff_time']-datetime.datetime.utcnow()}")
          time.sleep(0.5)
        # activate mission
        self.ussp.activate_plan(mission["plan ID"])
        # Launch drone
        if mission["status"] == "pending":
          self.launch_drone(mission["takeoff_height"], reset_dss_srtl)
        # Fly to waypoints
        self.fly_waypoints()
        if self.plan_withdrawn:
          mission["status"] = "running"
          self.application_state = "planning"
          self.ussp.end_plan(mission["plan ID"])
          self.update_wps_to_visit()
          break
        #wait for reaching waypoint
        time.sleep(1.0)
        #Await landing clearance?
        self.await_clearance_landing()
        #Land
        self.drone.land()
        # End plan
        self.ussp.end_plan(mission["plan ID"])
        mission["status"] = "ended"
        # Check wp type
        if mission["type"] == "drop off":
          self.drone.unload_package()
        elif mission["type"] == "pickup":
          self.drone.load_package()
        elif mission["type"] == "return":
          self.application_state = "idle"
          return_reached = True
        # Do not update dss srtl and geofence
        reset_dss_srtl = False
        reset_geofence = False



#--------------------------------------------------------------------#
def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='APP "app_noise"', allow_abbrev=False, add_help=False)
  parser.add_argument('-h', '--help', action='help', help=argparse.SUPPRESS)
  parser.add_argument('--app_ip', type=str, help='ip of the app', required=True)
  parser.add_argument('--id', type=str, default=None, help='id of this app_noise instance if started by crm')
  parser.add_argument('--crm', type=str, help='<ip>:<port> of crm', required=True)
  parser.add_argument('--log', type=str, default='debug', help='logging threshold')
  parser.add_argument('--owner', type=str, help='id of the instance controlling app_noise - not used in this use case')
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout')
  parser.add_argument('--mission', type=str, required=True)
  parser.add_argument('--startwp', default=0, type=int)
  args = parser.parse_args()

  # Identify subnet to sort log files in structure
  subnet = dss.auxiliaries.zmq.get_subnet(ip=args.app_ip)
  # Initiate log file
  dss.auxiliaries.logging.configure('app_lmd_ussp', stdout=args.stdout, rotating=True, loglevel=args.log, subdir=subnet)

  # Create the AppLMD class

  try:
    app = AppLmd(args.app_ip, args.id, args.crm, args.mission, args.startwp)
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
