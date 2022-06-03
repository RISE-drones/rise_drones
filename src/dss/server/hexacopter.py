'''
Hexacopter

Wraps the actual vehicle communication using dronekit and is used by the DSS server.
'''

import copy
import json
import logging
import math
import threading
import time

import dronekit
import numpy as np
from pymavlink import mavutil

import dss.auxiliaries
import dss.auxiliaries.config

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.0'
__copyright__ = 'Copyright (c) 2019-2021, RISE'
__status__ = 'development'

def get_distance(location1, location2):
  '''
  Returns the ground distance in metres between two LocationGlobal objects.

  This method is an approximation, and will not be accurate over large distances and close to the
  earth's poles. It comes from the ArduPilot test code:
  https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
  https://github.com/ArduPilot/MAVProxy/blob/master/MAVProxy/modules/lib/mp_util.py
  '''
  # 1/180*pi*radius of earth = 111319.4906
  # 40030000 / 360 = 1.11194444444e5
  original_lat = location1.lat
  dlat = location2.lat - location1.lat
  dlong = location2.lon - location1.lon
  return math.sqrt(dlat**2 + (dlong*math.cos(math.pi*original_lat/180))**2) * 1.11194444444e5

def bearing_deg(delta_east : float, delta_north : float) -> float:
  '''
  Calculates the bearing to a relative posisiton given the delta east
  and delta north to that point. Returned value is in degrees, [0-359]
  '''
  bearing = math.atan2(delta_east, delta_north)/math.pi*180
  if bearing < 0:
    bearing += 360

  if not 0 <= bearing < 360:
    raise dss.auxiliaries.exception.Error('bearing out of range')

  return bearing

class Geofence:
  def __init__(self):
    self.height_low = 2
    self.height_high = 50
    self.radius = 50

  def set_geofence(self, height_low, height_high, radius):
    self.height_low = height_low
    self.height_high = height_high
    self.radius = radius

class Waypoint:
  def __init__(self, lat=0, lon=0, alt=0):
    # Initiate properties
    self.id_str = "id_NOT_SET"
    self.id_int = -99
    self.lat = lat
    self.lon = lon
    self.alt = alt
    self.heading = 0
    self.action = ''
    self.speed = 0
    self.is_init_point = False

  def get_3D_distance_to(self, wp)->tuple:
    dlat = wp.lat - self.lat
    dlon = wp.lon - self.lon
    dalt = wp.alt - self.alt

    # Convert to meters
    northing = dlat * 1852 * 60
    easting = dlon *1852 * 60 * math.cos(self.lat/180*math.pi)

    # Calc squares
    northing2 = northing**2
    easting2 = easting**2
    dAlt2 = dalt**2

    # Calc distances
    distance2D = math.sqrt(northing2 + easting2)
    distance3D = math.sqrt(northing2 + easting2 + dAlt2)

    # Calc bearing
    # Guard division by 0 and calculate: Bearing given northing and easting
    # Case easting == 0, i.e. bearing == 0 or -180
    bearing = 0
    if easting == 0:
      if northing > 0:
        bearing = 0
      else:
        bearing = 180
    elif easting > 0:
      bearing = (math.pi/2 - math.atan(northing/easting))/math.pi*180
    elif easting < 0:
      bearing = -(math.pi/2 + math.atan(northing/easting))/math.pi*180

    return (northing, easting, dalt, distance2D, distance3D, bearing)

  # Call this function with the init wp as the refernce
  def check_geofence(self, ref_point, radius, height_low, height_high)->tuple:
    check_ok = True
    descr = ""
    # Make sure the init point has been used
    if not ref_point.is_init_point:
      return False, "Not using init_point for reference"

    (_, _, _, distance2D, _, _) = self.get_3D_distance_to(ref_point)
    # Check radius
    if distance2D > radius:
      check_ok = False
      descr = "Geofence violation, " + self.id_str
    # Check heights (WP altitude already wrt reference position)
    elif self.alt < height_low or self.alt > height_high:
      check_ok = False
      descr = "Geofence violation, " + self.id_str

    return check_ok, descr

  def as_dict(self):
    wp_dict = {}
    wp_dict['id_str'] = self.id_str
    wp_dict['id_int'] = self.id_int
    wp_dict['lat'] = self.lat
    wp_dict['lon'] = self.lon
    wp_dict['alt'] = self.alt
    wp_dict['heading'] = self.heading
    wp_dict['action'] = self.action
    wp_dict['speed'] = self.speed
    wp_dict['is_init_point'] = self.is_init_point
    return wp_dict


  def pretty_print(self, indentation = 4):
    wp_dict = self.as_dict()
    print(json.dumps(wp_dict, indent=indentation))


  # Updates the wp position to a dronekit_loc. Note heading is not affected.
  def update(self, dronekit_loc):
    self.lat = dronekit_loc.lat
    self.lon = dronekit_loc.lon
    self.alt = dronekit_loc.alt



class Hexacopter:
  def __init__(self, connect, baud, rangefinder):
    self.logger = logging.getLogger(__name__)

    self._abort_task = False
    self._rangefinder = rangefinder
    self.glana = None
    self.glana_autogain = False
    self.init_point_wp = Waypoint()
    self.gimbal_yaw_readable = False
    self.geofence = Geofence()
    self.pending_mission = {}
    self.active_mission = {}

    self.follow_stream_enabled = False

    # Control parameters
    self.min_wp_speed = 0.1                             # From documentation
    self.lookahead_dist = 20.0

    # connect to dronekit
    connect = "tcp:" + connect
    self.logger.info('Connecting to vehicle on %s using baudrate %s...', connect, baud)
    try:
      self.vehicle = dronekit.connect(connect, baud=baud, wait_ready=True, heartbeat_timeout=10)
    except dronekit.APIException as exception:
      self.logger.error('Connection failed: %s', exception)
      raise
    else:
      self.logger.info('Connection ok')

    self.vehicle.groundspeed = 4
    self.logger.info('Ground speed set to 4m/s')
    #Internal variable for value on channel 13
    self._old_channel_13 = None
    # listener for rc updates, needed to read channels 9-16 (and channels 1-8 in simulation)
    @self.vehicle.on_message('RC_CHANNELS')
    def listener(vehicle, name, message):
      # Channel 1-8 must be updated here in the simulation. Not required on real autopilot.
      vehicle._channels._update_channel('1', message.chan1_raw)
      vehicle._channels._update_channel('2', message.chan2_raw)
      vehicle._channels._update_channel('3', message.chan3_raw)
      vehicle._channels._update_channel('4', message.chan4_raw)
      vehicle._channels._update_channel('5', message.chan5_raw)
      vehicle._channels._update_channel('6', message.chan6_raw)
      vehicle._channels._update_channel('7', message.chan7_raw)
      vehicle._channels._update_channel('8', message.chan8_raw)
      vehicle._channels._update_channel('9', message.chan9_raw)
      vehicle._channels._update_channel('10', message.chan10_raw)
      vehicle._channels._update_channel('11', message.chan11_raw)
      vehicle._channels._update_channel('12', message.chan12_raw)
      vehicle._channels._update_channel('13', message.chan13_raw)
      vehicle._channels._update_channel('14', message.chan14_raw)
      vehicle._channels._update_channel('15', message.chan15_raw)
      vehicle._channels._update_channel('16', message.chan16_raw)
      vehicle.notify_attribute_listeners('channels', vehicle.channels)
      #Notify channel13 attribute listeners if value has changed.
      if self._old_channel_13 != vehicle.channels['13']:
        if self._old_channel_13 is not None:
          vehicle.notify_attribute_listeners('channel13', vehicle.channels['13'])
        self._old_channel_13 = vehicle.channels['13']
    ############################################################################

    # Lock to object
    self.lock = threading.Lock()

    # Read limitations from ".config". This is relevant even though
    # there are limitations in the autopilot. For example it can be
    # relevant to limit accelerations when developing a velocity
    # control application. In this case DSS can limit the application
    # without limiting manual input from RC.

    # Acceleration
    self.acc_x_max = dss.auxiliaries.config.config['ACC']['acc_x_max']
    self.acc_x_min = dss.auxiliaries.config.config['ACC']['acc_x_min']
    self.acc_y_max = dss.auxiliaries.config.config['ACC']['acc_y_max']
    self.acc_y_min = dss.auxiliaries.config.config['ACC']['acc_y_min']
    self.acc_z_max = dss.auxiliaries.config.config['ACC']['acc_z_max']
    self.acc_z_min = dss.auxiliaries.config.config['ACC']['acc_z_min']
    self.yaw_turd_max = dss.auxiliaries.config.config['ACC']['yaw_turd_max']
    self.yaw_turd_min = dss.auxiliaries.config.config['ACC']['yaw_turd_min']

    # Velocity
    self.vel_x_max = dss.auxiliaries.config.config['VEL']['vel_x_max']
    self.vel_x_min = dss.auxiliaries.config.config['VEL']['vel_x_min']
    self.vel_y_max = dss.auxiliaries.config.config['VEL']['vel_y_max']
    self.vel_y_min = dss.auxiliaries.config.config['VEL']['vel_y_min']
    self.vel_z_max = dss.auxiliaries.config.config['VEL']['vel_z_max']
    self.vel_z_min = dss.auxiliaries.config.config['VEL']['vel_z_min']
    self.max_yaw_rate = dss.auxiliaries.config.config['VEL']['max_yaw_rate']
    self.min_yaw_rate = dss.auxiliaries.config.config['VEL']['min_yaw_rate']

    # Position North East Down (from launch location)
    self.max_ned_n = dss.auxiliaries.config.config['POS']['pos_ned_n_max']
    self.min_ned_n = dss.auxiliaries.config.config['POS']['pos_ned_n_min']
    self.max_ned_e = dss.auxiliaries.config.config['POS']['pos_ned_e_max']
    self.min_ned_e = dss.auxiliaries.config.config['POS']['pos_ned_e_min']
    self.max_ned_d = dss.auxiliaries.config.config['POS']['pos_ned_d_max']
    self.min_ned_d = dss.auxiliaries.config.config['POS']['pos_ned_d_min']

    # Max waypoint distance. Can be used to minimise risk of mistyped waypoints.
    self.max_wp_dist = dss.auxiliaries.config.config['WP']['max_wp_distance']

    # Dictionary for data stream subscriptions (Flag, attribute name, enable/disable)-flag
    self.data_stream = {'new_input': False, 'attribute': '', 'enable': False}

    # Missions
    self.pending_mission_ned = {}
    self.active_mission_ned = {}
    self.pending_mission_lla = {}
    self.active_mission_lla = {}
    self.mission_next_wp = 0

    # Landing
    self.land_vel_limit = 0.5
    self.land_hover_t_limit = 3
    self.start_heading_deg = 0

    # Gimbal control
    self.gimbal_stow()

    self.thread = None
    self._status_msg = ''

    self._mutex_mode = threading.Lock()
    self.mode = self.get_flight_mode()
    self._expected_flight_mode = True
    self._rtl_waypoints = list()
    self._default_speed = 5

    self._thread_flight_mode = threading.Thread(target=self._main_flight_mode, daemon=True)
    self._thread_flight_mode.start()

  @property
  def status_msg(self):
    return self._status_msg

  @property
  def expected_flight_mode(self):
    '''Non-blocking property that indicates whether the current flight mode is expected or not.'''
    return self._expected_flight_mode

  @property
  def abort_task(self):
    '''This attribute is used to abort a running task, e.g. if rtl is triggered'''
    return self._abort_task

  @property
  def default_speed(self):
    '''Set/get the default vehicle speed.'''
    return self._default_speed

  @default_speed.setter
  def default_speed(self, value):
    self._default_speed = value

  @abort_task.setter
  def abort_task(self, value):
    self._abort_task = value

  def raise_if_aborted(self):
    if self.abort_task:
      self._status_msg = 'the task was aborted'
      raise dss.auxiliaries.exception.AbortTask()

  def get_channel(self, rc):
    channel = '%d' % rc
    if self.vehicle.channels[channel] is None:
      return None
    return self.vehicle.channels[channel]

  def task_set_gimbal(self, args):
    (pitch, roll, yaw) = args
    self.set_gimbal(pitch, roll, yaw)

  # Returns number of tracked satellites
  def get_nsat(self) -> int:
    return self.vehicle.gps_0.satellites_visible

  # Method returns armed state. We concider armed state as flying.
  def is_flying(self) -> bool:
    return self.vehicle.armed

  def is_init_point_set(self) -> bool:
    return self.init_point_wp.is_init_point

  # Set init point if not set
  def set_init_point(self, ref):
    # Get the current location
    self.init_point_wp.update(self.get_position_lla_global())
    self.home_location = self.get_position_lla_global()

    # Heading
    heading_deg = self.vehicle.attitude.yaw/math.pi*180
    if heading_deg < 0:
      heading_deg += 360
    self.init_point_wp.heading = heading_deg

    if self.init_point_wp.alt is not None:
      self.init_point_wp.is_init_point = True

  def parse_heading(self, json):
    # internal code for faulty heading is -99
    heading = json.get('heading', -99)

    # heading might be string or double
    if isinstance(heading, str):
      if heading == 'course':
        # internal code for 'course' is -1
        return -1
      # internal code for faulty heading is -99
      return -99

    # since heading is not a string we treat it as a double
    if 0 <= heading < 360:
      return heading

    # internal code for faulty heading is -99
    return -99

  # Convert a jsonWP with any reference frame to our own Waypoint class with LLA frame
  # Method does not check for nack reasons, check the returned wp
  def json_to_LLA(self, jsonWP, id_str)->Waypoint:

    wp = Waypoint()
    # Set the wp id info
    wp.id_str = id_str
    wp.id = int(id_str.replace("id",""))

    # Parse heading
    wp.heading = self.parse_heading(jsonWP)
    # Get initial lat in radians
    init_lat_rad = self.init_point_wp.lat/180 * math.pi
    # Get initial heading in radians
    init_heading_rad = self.init_point_wp.heading/180*math.pi
    # Parse wp coordinates and convert to LLA
    if "lat" in jsonWP and "lon" in jsonWP and "alt" in jsonWP and "alt_type" in jsonWP:
      alt_type = jsonWP['alt_type']
      if alt_type == 'relative' or alt_type == 'amsl':
        wp.lat = jsonWP['lat']
        wp.lon = jsonWP['lon']
        if alt_type == 'relative':
          wp.alt = jsonWP['alt']
        else:
          #Transform AMSL to relative (used internally for control)
          wp.alt = jsonWP['alt'] - self.init_point_wp.alt
    elif "north" in jsonWP and "east" in jsonWP and "down" in jsonWP:
      north = jsonWP['north']
      east = jsonWP['east']
      down = jsonWP['down']
      # Calc lat, lon from north east and init_point.
      wp.lat = self.init_point_wp.lat + north/111120 # 1852 * 60 - nautical mile times 60 -> length of 1 arch in m.
      wp.lon = self.init_point_wp.lon + east/(111120 * math.cos(init_lat_rad))
      wp.alt = -down
    elif "x" in jsonWP and "y" in jsonWP and "z" in jsonWP:
      x = jsonWP['x']
      y = jsonWP['y']
      z = jsonWP['z']
      # Calculate northing, easting
      beta = -init_heading_rad
      north = x * math.cos(beta) + y * math.sin(beta)
      east = -x * math.sin(beta) + y * math.cos(beta)
      # Calc lat, lon from north east and init_point (duplicate of above)
      wp.lat = self.init_point_wp.lat + north/111120 # 1852 * 60 - nautical mile times 60 -> length of 1 arch in m.
      wp.lon = self.init_point_wp.lon + east/(111120 * math.cos(init_lat_rad))
      wp.alt = -z
      # Heading is parsed above but need correction for local reference system if positive
      if wp.heading >= 0:
        wp.heading += self.init_point_wp.heading
        # Heading might be more than 360 now, correct if so
        if wp.heading > 360:
          wp.heading -= 360

    # Set speed or default
    if "speed" in jsonWP:
      wp.speed = jsonWP['speed']
    else:
      wp.speed = self.default_speed

    # Action
    if "action" in jsonWP:
      wp.action = jsonWP['action']

    # WP tracking_precision. We talked about introducing tracking precision as a wp parameter.
    return wp

  # Function converts any mission to LLA, and checks for all nack-reasons like geofence and such
  def upload_mission(self, mission)->tuple:
    print("Upload mission", mission)
    temp_mission = {}
    i = -1
    for id_str in mission:
      # Create a Waypoint object from the jsonWP
      new_wp = self.json_to_LLA(mission[id_str], id_str)

      if new_wp.lat == 0 and new_wp.lon == 0 :
        check_ok = False
        descr = "WP position format faulty, " + id_str
        break

      # Check geofence
      (check_ok, descr) = new_wp.check_geofence(self.init_point_wp, self.geofence.radius, self.geofence.height_low, self.geofence.height_high)
      if not check_ok:
        break

      # Set and check the wp numbering
      # Note: Waypoints will not be listed in numerical order in the for loop, this works anyways
      i += 1
      ghost_id_string = "id" + str(i)
      if ghost_id_string not in mission:
        # WP id not in mission
        check_ok = False
        descr = "WP numbering faulty, missing " + ghost_id_string
        break

      # Check nack action
      if 'action' in mission[id_str]:
        # No actions supported yet. TODO
        check_ok = False
        descr = "WP action not supported, " + id_str
        break

      # Check for low speed
      if new_wp.speed < 0.1:
        check_ok = False
        descr = "Speed below 0.1, " + id_str
        break

      # Check heading
      if new_wp.heading == -99:
        check_ok = False
        descr = "Heading faulty, " + id_str
        break

      # Add to temp_mission
      temp_mission[id_str] = new_wp

    # If all waypoint passed checks, accept mission as pending mission
    if check_ok:
      self.pending_mission = temp_mission

    return check_ok, descr

  def print_mission(self, mission_dict):
    for id_str in mission_dict:
      mission_dict[id_str].pretty_print()

  def log_pending_mission(self):
    for id_str in self.pending_mission:
      self.logger.info(json.dumps(self.pending_mission[id_str].as_dict()))


  def set_gimbal(self, pitch, roll, yaw):
    self.logger.info('Gimbal rotate: pitch: %d, roll: %d, yaw: %d', pitch, roll, yaw)
    self.vehicle.gimbal.rotate(pitch, roll, yaw)

  def gimbal_stow(self):
    self.set_gimbal(0, 0, 0)

  def update_vel_input(self, des_vel_x, des_vel_y, des_vel_z, des_yaw_rate):
    # Velocity input (desired vel) could be limited by vehicle limitations, BUT, it will not affect the application -> Application responsibility.

    #Vehicle lock, use lock to prevent inconsistent data.
    self.lock.acquire()
    self.des_vel_x = des_vel_x
    self.des_vel_y = des_vel_y
    self.des_vel_z = des_vel_z
    self.des_yaw_rate = des_yaw_rate
    # Release lock
    self.lock.release()

  def get_flight_mode(self):
    return self.vehicle.mode.name

  def set_expected_flight_mode(self, mode):
    with self._mutex_mode:
      self.mode = mode
      flight_mode = self.get_flight_mode()
      self._expected_flight_mode = (flight_mode == self.mode)

  def _main_flight_mode(self):
    while True:
      with self._mutex_mode:
        mode = self.get_flight_mode()
        self._expected_flight_mode = (mode == self.mode)

      time.sleep(0.5)

  def set_flight_mode_and_wait(self, mode, timeout=0.5):
    with self._mutex_mode:
      try:
        self.vehicle.wait_for_mode(dronekit.VehicleMode(mode), timeout=timeout)
      except TimeoutError:
        self.logger.error('failed to switch flight mode to %s', mode)
        raise
      else:
        self.logger.info('new flight mode: %s', mode)
        self.mode = mode

  def is_flight_mode(self, mode):
    return self.vehicle.mode == mode

  def arm_and_wait(self, timeout=1.0):
    try:
      self.vehicle.arm(timeout=timeout)
    except TimeoutError:
      self.logger.error('failed to arm vehicle')
      raise
    else:
      self.logger.info('Vehicle armed')

  def reset_dss_srtl(self):
    self._rtl_waypoints.clear()
    wp = Waypoint()
    curr_location = self.get_position_lla()
    wp.lat = curr_location.lat
    wp.lon = curr_location.lon
    wp.alt = max(2.0, curr_location.alt)
    wp.speed = self.default_speed
    wp.heading = self.vehicle.heading # Same as current heading
    self._rtl_waypoints.append(wp)
    self.logger.info(f"New DSS SRTL Home Position: lat: {wp.lat}, lon: {wp.lon}, alt: {wp.alt}, heading: {wp.heading}")

  @staticmethod
  def project_point(p1, p2, p3):
    '''Project point p3 to the line between p1 and p2'''
    #squared distance between p1 and p2
    l2 = np.sum((p1-p2)**2)
    if l2 == 0:
      return p1
    #The line extending the segment is parameterized as p1 + t (p2 - p1).
    #The projection falls where t = [(p3-p1) . (p2-p1)] / |p2-p1|^2
    #Make sure that the projected line is on the line segment
    t = max(0, min(1, np.sum((p3 - p1) * (p2 - p1)) / l2))
    projection = p1 + t * (p2 - p1)
    return projection

  def compute_lookahead_wp(self, prev_wp, next_wp) -> Waypoint:
    curr_location = self.get_position_lla()
    # Transform to euclidean frame (origin = current location)
    wp1_n = (prev_wp.lat - curr_location.lat)*1852*60
    wp1_e = (prev_wp.lon - curr_location.lon)*1852*60*math.cos(curr_location.lat/180*math.pi)
    wp2_n = (next_wp.lat - curr_location.lat)*1852*60
    wp2_e = (next_wp.lon - curr_location.lon)*1852*60*math.cos(curr_location.lat/180*math.pi)
    # project current position (lat, lon) to the line between prev_wp and next_wp
    p1 = np.array([wp1_n, wp1_e, prev_wp.alt])
    p2 = np.array([wp2_n, wp2_e, next_wp.alt])
    p_c = np.array([0.0, 0.0, curr_location.alt])

    proj_point = self.project_point(p1, p2, p_c)
    #Compute distance to projected point
    d1 = math.sqrt(np.sum((proj_point - p_c)**2))
    d2 = 0.0
    if d1 < self.lookahead_dist :
      #Compute direction towards next waypoint
      d_wp = math.sqrt(np.sum((p2-p1)**2))
      if d_wp > 0 :
        #Compute remaining distance
        d2 = math.sqrt(self.lookahead_dist**2 - d1**2)
        # Compute new coordinates for lookahead (North, East)
        proj_point = proj_point + (d2/d_wp)*(p2-p1)
    lookahead_wp = copy.deepcopy(next_wp)
    # Compute the lookahead latitude and longitude
    lookahead_wp.lat = curr_location.lat + proj_point[0]/(1852*60)
    lookahead_wp.lon = curr_location.lon + proj_point[1]/(1852*60*math.cos(curr_location.lat/180*math.pi))
    lookahead_wp.alt = proj_point[2]
    return lookahead_wp

  def position_controller(self, wp,  curr_location):
    #Compute position error
    (d_n, d_e, d_alt, _, _, _) = wp.get_3D_distance_to(curr_location)
    # Compute horizontal velocities based on the distance to the waypoint
    # TODO Add control parameters?
    v_n = -0.5*d_n
    v_e = -0.5*d_e
    #Check if speed_limit is reached
    speed_d = math.sqrt(v_n**2 + v_e**2)
    if speed_d > wp.speed:
      v_n = v_n * (wp.speed / speed_d)
      v_e = v_e * (wp.speed / speed_d)
    # Compute vertical velocity
    v_d = 0.5*d_alt
    # Limit v_d to 2, use copysign to maintain sign after abs(v_d)
    v_d = math.copysign(min(abs(v_d),2), v_d)

    # Send velocity command
    self.send_global_velocity(v_n, v_e, v_d)

  def goto_waypoint(self, next_wp, prev_wp):
    #TODO Add as threshold as variable (or user-specified input?)
    next_wp.threshold = 2.0
    if not self.is_flight_mode('GUIDED'):
      raise dss.auxiliaries.exception.Error('Sending goto command requires flight mode GUIDED. Current flight mode is: %s' % self.get_flight_mode())
    # Set heading according to what is specified in the waypoint
    self.send_condition_yaw(next_wp)
    waypoint_reached = False
    # ONLY WHEN USING ARDUPILOT POSITION CONTROLLER - Set commanded speed
    self.send_cmd_speed(next_wp.speed)
    while not waypoint_reached :
      # While waypopint not reached- steer towards next wp based on current location
      curr_location = self.get_position_lla()
      (_, _, _, distance2D, distance3D, _) = next_wp.get_3D_distance_to(curr_location)
      # Check if waypoint reached
      #TODO Select an appropriate way to verify waypoint reached
      if distance3D < next_wp.threshold:
        self.logger.info(f'goto_waypoint - waypoint reached!!')
        waypoint_reached = True
      else:
        # Compute lookahead reference point (2D)
        if distance2D < self.lookahead_dist :
          lookahead_wp = next_wp
        else :
          lookahead_wp = self.compute_lookahead_wp(prev_wp, next_wp)
        # USE OUR OWN POSITION CONTROLLER (Send velocity command)
        #self.position_controller(lookahead_wp, curr_location)
        #time.sleep(0.25)
        # USE ARDUPILOT POSITION CONTROLLER
        self.send_goto_lla(lookahead_wp)
        time.sleep(1.0)

  def task_gogo(self, next_wp):
    self._status_msg = 'gogo'
    self.logger.info('task: gogo start')
    self.raise_if_aborted()

    self.mission_next_wp = next_wp
    self.mission_previous_wp = None

    # Check next wp id, statement only true once each time gogo_lla is switched to
    # Make sure the waypoint is still valid
    next_wp_str = "id%d" % self.mission_next_wp

    # Test if there is a wp with the requested id
    if next_wp_str not in self.active_mission:
      self.mission_next_wp = -1
      raise dss.auxiliaries.exception.Error('There is no waypoint with %s - engage rtl' % next_wp_str)
    while self.mission_next_wp != -1:
      self._status_msg = 'gogo : next wp: %s' % next_wp_str
      next_wp = self.active_mission[next_wp_str]
      if self.mission_previous_wp :
        prev_wp = self.active_mission["id%d" % self.mission_previous_wp]
      else :
        prev_wp = self.get_position_lla()
      #Goto waypoint
      self.goto_waypoint(next_wp, prev_wp)
      #Waypoint reached
      # 1. TODO Implement what to do when action is associated with waypoint
      if next_wp.action :
        self.logger.warning("Action not supported yet...")
      # 2. Add waypoint to SRTL list
      self._rtl_waypoints.insert(0, next_wp)
      # 3. Update wp to the next in the list (if any exists)
      next_wp_cand = "id%d" % (self.mission_next_wp + 1)
      if next_wp_cand not in self.active_mission:
        #Final WP, send goto command and set mission next wp to -1
        self.logger.info('task: gogo - final wp reached...')
        self.send_goto_lla(next_wp)
        self.mission_next_wp = -1
      else:
        self.mission_previous_wp = self.mission_next_wp
        self.mission_next_wp += 1
        next_wp_str = next_wp_cand

        self.logger.info(f'task: gogo - Moving towards waypoint {next_wp_str}')

  def get_position_lla(self):
    return self.vehicle.location.global_relative_frame

  def get_position_lla_global(self):
    return self.vehicle.location.global_frame

  # Print current pos NED
  def print_pos_ned(self):
    pos = self.vehicle.location.local_frame
    print("Vehicle position NED:", pos.north, "\t", pos.east, "\t", pos.down)

  # For some debugging..
  def print_vel(self, vehicle):
    current_velocity = vehicle.velocity
    # Yaw in radians
    current_heading = vehicle.attitude.yaw
    print("Current velocity and heading:", current_velocity, current_heading)
    ref_vel_x = current_velocity[0]*math.cos(-current_heading) - current_velocity[1]*math.sin(-current_heading)
    ref_vel_y = current_velocity[0]*math.sin(-current_heading) + current_velocity[1]*math.cos(-current_heading)
    ref_vel_z = -current_velocity[2]
    print("Body_fix coordinates: ", ref_vel_x, ref_vel_y, ref_vel_z)

  def send_ned_velocity(self, velocity_x, velocity_y, velocity_z):
    '''
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    '''
    msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
        0,     # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # Velocity command will be active for 3 seconds only. Resend to keep alive
    self.vehicle.send_mavlink(msg)

  def send_body_velocity(self, velocity_x, velocity_y, velocity_z):
    '''
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    '''
    msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
        0,     # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame, Deprecated according to https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/common.xml#L795, suggests using MAV_FRAME_BODY_FRD
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # Velocity command will be active for 3 seconds only. Resend to keep alive
    self.vehicle.send_mavlink(msg)

  def task_gripper_set(self, action, can_id):
    self.logger.info('task: gripper set')
    self.raise_if_aborted()

    # create the DO_GRIPPER command
    msg = self.vehicle.message_factory.command_long_encode(
        0, 0,                               # target system, target component
        mavutil.mavlink.MAV_CMD_DO_GRIPPER, # command
        0,                                  # confirmation
        can_id,                             # param 1, gripper number
        action,                             # param 2, gripper action, 0 = release, 1 = grab
        0, 0, 0, 0, 0)                      # param 3 ~ 7 not used
    self.vehicle.send_mavlink(msg)

  def send_yaw_rate(self, yaw_rate):
    '''
    By default the yaw of the vehicle will follow the direction of travel. After setting
    the yaw using this function there is no way to return to the default yaw "follow direction
    of travel" behavior (https://github.com/diydrones/ardupilot/issues/2427)

    Function actually sets a desired heading relative to body, trimming with parameter t (time)
    we implement a small change in yaw
    '''
    # cw = 1 or -1, true or false, clock wise or counter clockwise.  1 -> cw, -1 -> ccw
    if yaw_rate == 0:
      cw = 1
    else:
      cw = yaw_rate/abs(yaw_rate)

    # Limit allowed yaw rate (losing the sign of yawrate does not matter from here)
    _yaw_rate = min(abs(yaw_rate), 60)

    # Trim the send_yaw_rate function here
    # Time for rotation, _yaw_delta is always positive. cw take care of sign
    t = 1
    _yaw_delta = _yaw_rate*t

    target_system = 0
    target_component = 0
    command = mavutil.mavlink.MAV_CMD_CONDITION_YAW
    confirmation = 0
    yaw_in_degrees = _yaw_delta
    yaw_rate_limit = _yaw_rate
    direction = cw
    relative = 1
    param5 = 0  # Not used
    param6 = 0  # Not used
    param7 = 0  # Not used

    msg = self.vehicle.message_factory.command_long_encode(
      target_system,
      target_component,
      command,
      confirmation,
      yaw_in_degrees,
      yaw_rate_limit,
      direction,
      relative,
      param5,
      param6,
      param7)

    # send command to vehicle
    self.vehicle.send_mavlink(msg)


  def condition_yaw(self, heading, relative=False):
    '''
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting
    the yaw using this function there is no way to return to the default yaw "follow direction
    of travel" behavior (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see:
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    '''
    if relative:
      is_relative = 1 #yaw relative to direction of travel
    else:
      is_relative = 0 #yaw is an absolute angle
    rot_dir = 1 if self.get_angle_in_range(heading - self.vehicle.heading) > 0 else -1
    # create the CONDITION_YAW command using command_long_encode()
    msg = self.vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,  # param 1, yaw in degrees
        0,      # param 2, yaw speed deg/s
        rot_dir,      # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    # send command to vehicle
    self.vehicle.send_mavlink(msg)

  def goto_position_target_local_ned(self, north, east, down, heading_deg, speed=False):
    '''
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see:
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned
    bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.

    '''
    mask = 0b0000111111111000 # type_mask (only positions enabled)

    #heading_rad = heading_deg/180*math.pi

    msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
        0,     # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        mask,
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # TBD yaw hardcoded yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    self.vehicle.send_mavlink(msg)

    # Set heading to course or as specified
    if heading_deg == -1:
      # set heading to the straight line course towards next wp
      pos = self.vehicle.location.local_frame
      d_n = north - pos.north
      d_e = east - pos.east
      self.condition_yaw(bearing_deg(d_e, d_n))
    else:
      self.condition_yaw(heading_deg)

    if not speed:
      speed = self.vehicle.parameters.get('WPNAV_SPEED', 3)

    msg = self.vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, #command
        0, #confirmation
        0, #speed type, ignore on ArduCopter
        speed, # speed
        0, 0, 0, 0, 0 #ignore other parameters
      )
    # send command to vehicle
    self.vehicle.send_mavlink(msg)


  def send_condition_yaw(self, wp2, wp1 = None):
    '''Set heading towards the given waypoint (wp.heading = -1), or as specified by the input'''
    if wp2.heading == -1:
      if wp1 is None:
        wp1 = self.get_position_lla()
      # set heading to the straight line course towards wp2
      d_n = (wp2.lat - wp1.lat) * 1.1131949e5
      d_e = (wp2.lon - wp1.lon) * math.cos(math.pi*wp1.lat/180) * 1.1131949e5

      # set heading condition only if new wp is more than 1m away
      if d_n**2 + d_e**2 > 1:
        self.condition_yaw(bearing_deg(d_e, d_n))
      else:
        # stacked waypoints; keep heading
        pass

    else:
      self.condition_yaw(wp2.heading)



  def send_global_velocity(self, v_north, v_east, v_down):
    ''' Send global velocity command (v_north, v_east, v_down)'''
    msg = self.vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        v_north, # X velocity in NED frame in m/s
        v_east, # Y velocity in NED frame in m/s
        v_down, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    self.vehicle.send_mavlink(msg)

  def send_goto_lla(self, wp_location):
    '''
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified location.
    ref http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-position-target-global-int
    Bitmask to indicate which fields should be ignored by the vehicle (see POSITION_TARGET_TYPEMASK enum)
    MAV_FRAME_GLOBAL_RELATIVE_ALT: alt is meters above home
    MAV_FRAME_GLOBAL_INT : alt is meters above sea level

    bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw rate

    When providing Pos or Vel all 3 axis must be provided

    Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
    Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
    Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)
    Acceleration not supported
    '''
    msg = self.vehicle.message_factory.set_position_target_global_int_encode(
      0,     # time_boot_ms (not used)
      0, 0,  # target system, target component
      mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, # frame
      0b0000111111111000, # type_mask (only speeds enabled)
      int(wp_location.lat*1e7), # lat_int 1e7-     not in meters! /AG X Position in WGS84 frame in 1e7 * meters
      int(wp_location.lon*1e7), # lon_int 1e7-     not in meters! /AG Y Position in WGS84 frame in 1e7 * meters
      wp_location.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
      0, # X velocity in NED frame in m/s
      0, # Y velocity in NED frame in m/s
      0, # Z velocity in NED frame in m/s
      0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
      0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    self.vehicle.send_mavlink(msg)


  def send_cmd_speed(self, speed):
    msg = self.vehicle.message_factory.command_long_encode(
    0, 0,  # target system, target component
    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, #command
    0, #confirmation
    0, #speed type, ignore on ArduCopter
    speed, # speed
    0, 0, 0, 0, 0) #ignore other parameters

    # send command to vehicle
    self.vehicle.send_mavlink(msg)

  def goto_position_target_lla(self, wp_location):
    # send goto lla
    self.send_goto_lla(wp_location)
    # send global heading
    self.send_condition_yaw(wp_location)
    # Command desired speed
    if not wp_location.speed:
      wp_location.speed = self.vehicle.parameters.get('WPNAV_SPEED', 3)

    self.send_cmd_speed(wp_location.speed)

  def set_heading(self, heading):
    self.send_goto_lla(self.get_position_lla())
    self.condition_yaw(heading)

  def set_guided_mode(self):
    try:
      # Allow mode change to take a little while, but wait no more than 5s.
      self.set_flight_mode_and_wait('GUIDED', timeout=5.0)
      self.logger.info("Switched back to GUIDED mode")
    except TimeoutError:
      raise dss.auxiliaries.exception.Error("Failed to switch back to GUIDED")

  def task_land(self):
    self._status_msg = 'landing'
    self.logger.info("Application called for landing. Stow camera and land")
    self.gimbal_stow()

    # Horizontal velocities are low. Engage landing if mode is still GUIDED
    if self.is_flight_mode('GUIDED'):
      self.logger.info("Land: Set flight mode to LAND")
      try:
        # Allow mode change to take a little while, but wait no more than 5s.
        self.set_flight_mode_and_wait('LAND', timeout=5.0)
      except TimeoutError:
        pass
      else:
        self.logger.info("Landing is engaged")
    else:
      self.logger.info("Landing was aborted by operator")
      raise dss.auxiliaries.exception.AbortTask()

    # Wait for vehicle to land and disarm.
    while self.vehicle.armed and self.is_flight_mode('LAND'):
      self._status_msg = 'altitude: %5.1f m (NED %.2g,%.2g,%.2g)' % (self.vehicle.location.global_relative_frame.alt, self.vehicle.location.local_frame.north, self.vehicle.location.local_frame.east, self.vehicle.location.local_frame.down)
      if self.abort_task:
        try:
          # Allow mode change to take a little while, but wait no more than 5s.
          self.set_flight_mode_and_wait('GUIDED', timeout=5.0)
        except TimeoutError:
          raise dss.auxiliaries.exception.Error("Failed to switch back to GUIDED")
        else:
          raise dss.auxiliaries.exception.AbortTask()

      #pos = self.vehicle.location.local_frame
      #self.logger.info("Land: Landing engaged, north: %.2g east: %.2g down: %.2g", pos.north, pos.east, pos.down)
      time.sleep(0.5)

    # Determine if landing was completed or aborted
    if self.is_flight_mode('LAND') and not self.vehicle.armed:
      self.logger.info("Land: Vehicle landed in LAND by DSS")
    else:
      self.logger.info("Landing aborted by operator or LOL.")
      raise dss.auxiliaries.exception.AbortTask()

    self._status_msg = 'landing completed'

  def task_arm_take_off(self, alt):
    self.logger.info('task: arm and take off')
    self._status_msg = 'arm and take off'
    self.raise_if_aborted()
    # Make sure vehicle is still armable
    if not self.vehicle.is_armable:
      self.logger.info("Waiting for vehicle to initialise...")
      while not self.vehicle.is_armable:
        time.sleep(0.1)

    if not self.is_flight_mode('GUIDED'):
      raise dss.auxiliaries.exception.Error('flight mode is not GUIDED')

    if self.vehicle.armed:
      raise dss.auxiliaries.exception.Error("Arm and Take-off commanded but vehicle is already armed")

    # Arm vehicle and wait for armed state
    self.logger.info("Arming motors...")
    try:
      self.arm_and_wait(timeout=10.0)
    except TimeoutError:
      raise dss.auxiliaries.exception.Error("Vehicle did not arm")

    # Vehicle is armed, sleep to allow motors to spin up.
    # CRITICAL!!! Lowering the time.sleep period can cause crash.
    time.sleep(4)
    # Set home location before takeoff
    self.vehicle.home_location = self.home_location
    height = alt
    self.logger.info("Vehicle armed, taking off to height: %s", height)
    if not self.is_flight_mode('GUIDED'):
      self.logger.warning('Take-off aborted by operator.')
      raise dss.auxiliaries.exception.AbortTask()
    self.vehicle.simple_takeoff(height) # Take off to target altitude
    # Wait until the vehicle reaches the take-off altitude.
    # Store start heading to use for landing
    heading_deg = self.vehicle.attitude.yaw/math.pi*180
    if heading_deg < 0:
      heading_deg += 360

    while self.is_flight_mode('GUIDED'):
      self._status_msg = 'altitude: %5.1f m' % self.vehicle.location.global_relative_frame.alt
      if self.get_position_lla().alt >= height*0.9: #Trigger just below target alt.
        self.logger.info('Take-off target altitude reached')
        self._status_msg = ''
        break

      if not self.vehicle.armed:
        self.logger.info('Drone is not armed. Check pre-arm checks')
        self._status_msg = ''
        break
    #self.reset_dss_srtl()

  def task_ardupilot_rtl(self):
    self.logger.info('hexa task: ardupilot rtl')
    #self.raise_if_aborted()

    # Stow gimbal
    self.gimbal_stow()
    self._rtl = True

    # Engage RTL right away, vehicle must handle the maneuver
    if self.is_flight_mode('GUIDED'):
      self.logger.info('Changing flight mode to RTL')
      try:
        self.set_flight_mode_and_wait('RTL', timeout=3.0)
      except TimeoutError:
        pass
      else:
        self.logger.info('RTL is engaged')

      # Wait for vehicle to land and disarm.
      while self.vehicle.armed and self.is_flight_mode('RTL'):
        pos = self.vehicle.location.local_frame
        self.logger.info('RTL: NED: %s', str(pos.north) + str(pos.east) + str(pos.down))
        time.sleep(2)

    # Determine if rtl was completed or aborted
    if self.is_flight_mode('RTL') and not self.vehicle.armed:
      self.logger.info('Vehicle landed in RTL by DSS')
    else:
      self.logger.info('RTL aborted by operator')

  def task_dss_srtl(self, hover_time):
    self.logger.info("Task: DSS SRTL")
    self._status_msg = 'DSS SRTL'
    self.raise_if_aborted()
    if self.vehicle.armed and self.is_flight_mode('GUIDED'):
      # Stow gimbal
      self.gimbal_stow()
      prev_wp = self.get_position_lla()
      #Visit all SRTL waypoints
      for wp in self._rtl_waypoints :
        self.goto_waypoint(wp, prev_wp)
        self.raise_if_aborted()
        prev_wp = wp
      #Hover for a while before landing
      time.sleep(hover_time)
      self.task_land()

  def task_disconnect(self):
    self.logger.info('task: disconnect')
    self._status_msg = 'disconnect'
    self.raise_if_aborted()
    # If application disconnected during flight in mode GUIDED, invoke RTL.
    if self.is_flying() and self.is_flight_mode('GUIDED'):
      # Can we perform a SRTL?
      if len(self._rtl_waypoints) > 0 :
        self.task_dss_srtl(2.0)
      else:
        # Perform Ardupilot RTL
        self.task_ardupilot_rtl()

  def filter_reset_needed(self):
    # Dummy function for future Kalman implementation
    return False

  def stop(self):
  # Stop vehicle
    self.send_body_velocity(0,0,0)
    self.send_yaw_rate(0)

  # Returns angle in range [-180 180]
  def get_angle_in_range(self, angle):
    angle2 = angle % 360
    if angle2 > 180:
      angle2 -= 360
    if angle2 < -180:
      angle2 += 360
    return angle2

  def follow_stream(self):
    # Follow stream
    self.logger.info("Fcn follow stream")

    # Hardcode stream location for test. Use drone pos plus offset
    # TODO, subscribe to stream and filter in Kalman.
    stream_loc = self.vehicle.location.global_relative_frame
    stream_wp = Waypoint()
    stream_wp.lat = stream_loc.lat + 0.0005
    stream_wp.lon = stream_loc.lon - 0.0005
    stream_wp.alt = stream_loc.alt

    # Create a waypoint object to carry dss location
    me_wp = Waypoint()

    # Future use if filter on lla stream shall show if initialized or not
    filter_initialized = True

    i = 0
    # Hardcode pattern info
    pattern = "circle"
    heading_mode = "poi"

    radius = 20
    des_heading = 180
    des_yaw_rate = 10
    des_alt_diff = 20

    # Init ctrl signals
    ref_yaw_rate = 0
    ref_velX = 0
    ref_velY = 0
    ref_velZ = 0

    ref_velX_filt = 0
    ref_velY_filt = 0
    ref_velZ_filt = 0

    yawRateFF = 0
    ref_course = 0
    ref_yaw = 0

    yaw_errorIntegrated = 0

    # Parameter settings
    use_yaw_I_gain = False
    yaw_KP = 1
    yaw_KI = 0.15
    rad_KP = 0.25
    vPosKP = 1

    heading_range_limit = 4 # For pattern above, at a greater distance than heading_range_limit, heading = bearing

    # Use i < 700 for development only, cannot stop thread right now.. TODO
    while self.follow_stream_enabled:
      # Read the vehicle heading
      heading = round(self.vehicle.attitude.yaw/math.pi*180, 2)
      me_wp.update(self.vehicle.location.global_relative_frame)

      # Filter the stream: The receiving thread of positions (stream) updates the filter each time a measurement arrives, prior to calculating control signals, estimate the latest pos.
      # TODO Kalman implementation
      # If the filter is not yet initialized, the first measurement has not arrived, send vel = 0 and try again.
      if not filter_initialized:
        self.send_body_velocity(0, 0, 0)
        self.send_yaw_rate(0)
        time.sleep(0.1)
        continue

      # Safety check, look for non initialized stream
      if (stream_wp.lat == 0) or (stream_wp.lon == 0):
        time.sleep(0.1)
        continue

      # If a Kalman reset is needed the stream does not update, stop
      if self.filter_reset_needed():
        print('Stream does not update, stopping')
        self.send_body_velocity(0, 0, 0)
        self.send_yaw_rate(0)
        time.sleep(0.1)
        continue

      # Estimate current stream location

      # Check if max time following is reached
      # TODO, if max time reached, stop. define maxtime in seconds
      i += 1
      if i > 700:
        self.follow_stream_enabled = False

      # Follow the stream
      # Get distance and bearing to the stream
      (northing, easting, dalt, distance2D, distance3D, bearing) = me_wp.get_3D_distance_to(stream_wp)
      radiusError = distance2D - radius

      # Print distance to stream sometimes (time between prints: sampleTime/1000*40)
      if i % 40 == 0:
        print("Distance to stream: ", distance2D)

      if pattern == 'circle':
        # Desired yaw rate and radius gives the speed.
        # 2*math.pi*radius*desYawRate/360 ~ 0.01745
        speed = abs(0.01745 * radius * des_yaw_rate)

        # CounterClockWise rotation true or false?
        CCW = False
        if des_yaw_rate < 0:
          CCW = True

        # For each headingMode, calculate the ref_yaw, ref_velX and ref_velY

        # Heading mode poi
        if heading_mode == 'poi':
          # ref_yaw towards poi
          ref_yaw = bearing

          # Yawrate is non-zero in steady state, enable YawIntegreator
          use_yaw_I_gain = True

          # calc ref_course. If far away, fly straight towards stream
          if radiusError > 16:
            ref_course = bearing
          elif CCW:
            ref_course = bearing + 90
          else:
            ref_course = bearing - 90

          # Calc body velocitites to follow ref_course (parallell to course)
          alphaRad = (ref_course - heading)/180*math.pi
          ref_velX = speed*math.cos(alphaRad)
          ref_velY = speed*math.sin(alphaRad)

          # Radius tracking, add components to x and y
          betaRad = (bearing - heading)/180*math.pi
          ref_velX += rad_KP*radiusError*math.cos(betaRad)
          ref_velY += rad_KP*radiusError*math.sin(betaRad)

          # YawRate feed forward when closing in to radius
          if abs(radiusError) < 4:
            # THis interferes with the integrator of th controller TODO
            yawRateFF = des_yaw_rate

          # Gimbla control
          g_pitch = int(math.atan(dalt/distance2D)/math.pi*180)
          self.set_gimbal(g_pitch, 0, 0)

        # Heading mode absolute
        elif heading_mode == 'absolute':
          # Ref yaw defined in pattern
          ref_yaw = des_heading

          # Yawrate is zero oin steady state, disable YawIntegreator
          use_yaw_I_gain = False

          # Calc direction of travel as perpedicular to bearing towards poi.
          #var direction: Double = 0
          if CCW:
            ref_course = bearing + 90.0
          else:
            ref_course = bearing - 90.0

          # Calc body velocitites to follow ref_course (parallell to course)
          alphaRad = (ref_course - heading)/180*math.pi
          ref_velX = speed*math.cos(alphaRad)
          ref_velY = speed*math.sin(alphaRad)

          # Radius tracking, add components to x and y
          betaRad = (bearing - heading)/180*math.pi
          ref_velX += rad_KP*radiusError*math.cos(betaRad)
          ref_velY += rad_KP*radiusError*math.sin(betaRad)

        #Heading mode course
        elif heading_mode == 'course':
          # Special case of absolute where heading is same as direction of travel.
          # Calc direction of travel as perpedicular to bearing towards poi.

          # Calc ref_course
          if radiusError > 8:
            ref_course = bearing
          elif CCW:
            ref_course = bearing + 90.0
          else:
            ref_course = bearing - 90.0
          # Ref yaw is ref_course. Or should i be course..
          ref_yaw = ref_course

          # Yawrate is non-zero in steady state, enable YawIntegreator
          use_yaw_I_gain = True

          # Calc body velocitites to follow ref_course (parallell to course)
          alphaRad = (ref_course - heading)/180*math.pi
          ref_velX = speed*math.cos(alphaRad)
          ref_velY = speed*math.sin(alphaRad)

          # Radius tracking, add components to x and y
          betaRad = (bearing - heading)/180*math.pi
          ref_velX += rad_KP*radiusError*math.cos(betaRad)
          ref_velY += rad_KP*radiusError*math.sin(betaRad)

          if abs(radiusError) < 4:
            # THis interferes with the itegrator of th controller, TODO
            yawRateFF = des_yaw_rate

        else:
          print("Circle heading mode not known. Stopping follower")
          self.stop()
          return

      elif pattern == 'above':
        # For each headingMode, calculate the ref_yaw, ref_velX and ref_velY

        # Heading mode poi
        if heading_mode == 'poi':
          # If 'far' away, set heading to bearing
          if distance2D > heading_range_limit:
            ref_yaw = bearing
          # Else, maintain heading
          else:
            ref_yaw = heading

          # Yawrate is zero in steady state, disable YawIntegreator
          use_yaw_I_gain = False

          # Set speed to half the distance to target
          speed = distance2D/2

          # Direction of travel is bearing
          direction = bearing

          # Calc body velocities based on speed, direction of travel and ref_yaw
          alphaRad = (direction-ref_yaw)/180*math.pi
          ref_velX = speed*math.cos(alphaRad)
          ref_velY = speed*math.sin(alphaRad)

          # Gimbla control
          g_pitch = int(math.atan(dalt/distance2D)/math.pi*180)
          self.set_gimbal(g_pitch, 0, 0)

        # Heading mode abosolute
        elif heading_mode == "absolute":
          # Heading is defined in pattern
          ref_yaw = des_heading

          # Yawrate is zero in steady state, disable YawIntegreator
          use_yaw_I_gain = False

          # Set speed to half the distance to target
          speed = distance2D/2

          # Direction of travel is bearing
          direction = bearing

          # Calc body velocities based on speed, direction of travel and ref_yaw
          alphaRad = (direction-ref_yaw)/180*math.pi
          ref_velX = speed*math.cos(alphaRad)
          ref_velY = speed*math.sin(alphaRad)

          # Gimbal control
          g_pitch = int(math.atan(dalt/distance2D)/math.pi*180)
          self.set_gimbal(g_pitch, 0, 0)

        # Heading mode course
        elif heading_mode ==  'course':
          # The code currently overshoots when flying in fast to target and headingRangeLimit is set to 3. It turns around and tracks.

          # Set speed to half the distance to target.
          speed = distance2D/2

          # If 'far' away, set heading to bearing and only fly in body X. Makes cool manouvers
          if distance2D > heading_range_limit:
            ref_yaw = bearing
            ref_velX = speed
          # Else, maintain heading and strafe
          else:
            ref_yaw = heading
            alphaRad = (bearing-ref_yaw)/180*math.pi
            ref_velX = speed*math.cos(alphaRad)
            ref_velY = speed*math.sin(alphaRad)

          # Yawrate is zero in steady state, disable YawIntegreator
          use_yaw_I_gain = False

          # Gimbal control
          g_pitch = int(math.atan(dalt/distance2D)/math.pi*180)
          self.set_gimbal(g_pitch, 0, 0)

        else:
          print("Heading mode not supported in pattern above. Stopping follower")
          self.stop()
          return
      else:
        print("Pattern not supported. Stopping follower")
        self.stop()
        return


      # Calculate yaw-error, use shortest way (right or left?)
      yaw_error = self.get_angle_in_range(heading - ref_yaw)
      # P-controller for Yaw plus feed forward, TODO evaluate FF
      ref_yaw_rate = yawRateFF - yaw_error*yaw_KP

      # PI-controller for Yaw
      # Wind up protection, big yaw_errors probably depends on steps in reference
      if abs(yaw_error) < 30 and use_yaw_I_gain:
        yaw_errorIntegrated += yaw_error
      else:
        yaw_errorIntegrated = 0

      ref_yaw_rate = -yaw_errorIntegrated*yaw_KI - yaw_error * yaw_KP
      print("Integral part: ", -yaw_errorIntegrated*yaw_KI)
      print("refYawReate: ", ref_yaw_rate, "yaw_error: ", yaw_error, "refYaw: ", ref_yaw)

      # Punish horizontal velocity on yaw error. Otherwise drone will not fly in straight line
      turn_factor = 1
      # if abs(yaw_error) > 20:
      #   turn_factor = 1    # TODO, test higher than 0
      # else:
      #   turn_factor = 1

      # Limit speeds while turning
      ref_velX *= turn_factor
      ref_velY *= turn_factor

      # Altitude trackign, zError positive downwards, current - ref
      # dAlt is from copter to stream device (positive DOWNWARDS), desAltDiff is above stream device (positive UPWARDS)
      z_error = (dalt - (-des_alt_diff))
      # Negative feedback loop
      ref_velZ = -z_error*(vPosKP)

      # Set up a speed limit. Should use global limit TODO
      speed = 8

      # Simple low pass filer on reference velocities
      weight = 0.15
      ref_velX_filt = ref_velX * weight + ref_velX_filt * (1 - weight)
      ref_velY_filt = ref_velY * weight + ref_velY_filt * (1 - weight)
      ref_velZ_filt = ref_velZ * weight + ref_velZ_filt * (1 - weight)

      # Check speed limit, TODO!

      #self.sendControlData(velX: Float(ref_velX), velY: Float(ref_velY), velZ: Float(refZVel), yawRate: Float(_refYawRate), speed: speed)
      self.send_body_velocity(ref_velX_filt, ref_velY_filt, ref_velZ_filt)
      #self.send_yaw_rate(ref_yaw_rate)
      self.condition_yaw(ref_yaw)
      # TODO, how to control loop time?
      time.sleep(0.1)
      # Last line of while loop

    # While loop exited
    self.send_body_velocity(0, 0, 0)
    self.send_yaw_rate(0)
    return
