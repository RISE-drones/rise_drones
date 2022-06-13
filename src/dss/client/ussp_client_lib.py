'''
Drone Safety Service *API*

This class is in charge of the socket amd the actual API as described
in documentation.
'''
import datetime
import logging
import string

import dss.auxiliaries
import zmq
import dss.client
import numpy as np

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.0.0'
__copyright__ = 'Copyright (c) 2021, RISE'
__status__ = 'development'

def get_3d_distance(node1, node2):
  dlat = node2[1] - node1[1]
  dlon = node2[0] - node1[0]
  dalt = node2[2] - node1[2]

  # Convert to meters
  d_northing = dlat * 1852 * 60
  d_easting = dlon *1852 * 60 * np.cos(node1[1]/180*np.pi)

  # Calc distances
  d_2d = np.sqrt(d_northing**2 + d_easting**2)
  d_3d = np.sqrt(d_northing**2 + d_easting**2 + dalt**2)

  # Calc bearing
  bearing = np.arctan2(d_easting, d_northing)
  return (d_northing, d_easting, dalt, d_2d, d_3d, bearing)

class UsspClientLib:
  def __init__(self, app_id, context=None):
    self._logger = logging.getLogger(__name__)
    self._logger.info('U-space Client Lib')

    self._context = context if context else zmq.Context()
    self._ussp_client = None
    self._app_id = app_id
    self._nrid_msgs = {}


  def connect(self, ussp_ip, req_port, pub_port, sub_port, timeout=2000):
    self._ussp_client = dss.client.UsspClientApi(self._context, self._app_id, ussp_ip, req_port, pub_port, sub_port, timeout)

  def initialize_nrid_msg(self, operator_id, uas_id):
    '''
    Initialize a template NRID message with correct operator ID and UAS ID
    '''
    self._nrid_msgs[uas_id] = {
    "UAS ID": uas_id,
    "UAS ID type": 3,
    "UA type": 2,
    "timestamp": datetime.datetime.utcnow().isoformat(),
    "timestamp accuracy": 4,
    "operational status": 2,
    "operation description": "last mile delivery",
    "operator ID": operator_id,
    "latitude": 0.0,
    "longitude": 0.0,
    "geodetic altitude": 0.0,
    "height": 0.0,
    "height type": "above take-off",
    "geodetic vertical accuracy": 4,
    "horizontal accuracy": 11,
    "speed accuracy": 3,
    "track direction": 0.0,
    "speed": 0.0,
    "vertical speed": 0.0,
    "operator latitude": 0.0,
    "operator longitude": 0.0,
    "operator location type": "fixed",
    }

  @staticmethod
  def transform_plan(plan):
    '''
    Transforms a plan received from the USSP to a format that is compatible with the DSS
    '''
    wp_id = 0
    wp_mission = {}
    cruise_height = plan[1]["position"][2] - plan[0]["position"][2]
    if cruise_height > 30.0 :
      start_idx = 1
    else :
      start_idx = 2
    for idx in range(start_idx, len(plan)-1):
        id_str =  "id%d" % wp_id
        position = plan[idx]["position"]
        prev_position = plan[idx-1]["position"]
        (_, _, _, ds, _, _) = get_3d_distance(prev_position, position)
        dt = datetime.datetime.fromisoformat(plan[idx]["time"]) - datetime.datetime.fromisoformat(plan[idx-1]["time"])
        horizontal_speed = max(1.0, ds/dt.total_seconds())
        wp_mission[id_str] = {
          "lat" : position[1], "lon": position[0], "alt": position[2], "alt_type": "amsl", "heading": "course", "speed": horizontal_speed
        }
        wp_id += 1
    return wp_mission

  def receive_ussp_data(self):
    topic, msg = self._ussp_client.receive_subscribe_data()
    return topic, msg

  def update_nrid_operator_location(self, uas_id, operator_lat, operator_lon, loc_type="fixed"):
    self._nrid_msgs[uas_id]["operator latitude"] = operator_lat
    self._nrid_msgs[uas_id]["operator longitude"] = operator_lon
    self._nrid_msgs[uas_id]["operator location type"] = loc_type

  def update_nrid_accuracies(self, uas_id:string, t_acc:int, alt_acc:int, h_acc:int, speed_acc:int):
    self._nrid_msgs[uas_id]["timestamp accuracy"] = t_acc
    self._nrid_msgs[uas_id]["geodetic vertical accuracy"] = alt_acc
    self._nrid_msgs[uas_id]["horizontal accuracy"] = h_acc
    self._nrid_msgs[uas_id]["speed accuracy"] = speed_acc

  def update_nrid_state(self, uas_id:string, time:datetime.datetime, lat:float, lon:float, alt:float, height:float, bearing:float, speed:float, vert_speed:float):
    self._nrid_msgs[uas_id]["timestamp"] = time.isoformat()
    self._nrid_msgs[uas_id]["latitude"] = lat
    self._nrid_msgs[uas_id]["longitude"] = lon
    self._nrid_msgs[uas_id]["geodetic altitude"] = alt
    self._nrid_msgs[uas_id]["height"] = height
    self._nrid_msgs[uas_id]["track direction"] = bearing
    self._nrid_msgs[uas_id]["speed"] = speed
    self._nrid_msgs[uas_id]["vertical speed"] = vert_speed

  def publish_nrid_msg(self, uas_id:string):
    if self._nrid_msgs[uas_id] :
      self._ussp_client.publish_nrid(self._nrid_msgs[uas_id])


  def query_ground_height(self, lat, lon, epsg=4979):
    answer = self._ussp_client.query_ground_height(lat, lon, epsg)
    return answer["height"]

  def request_plan(self, operator_id:string, uas_id:string, epsg:int, positions, takeoff_time:datetime.datetime, speed:float, max_speed:float, ascend_rate:float, descend_rate:float):
    plan = []
    for position in positions:
      node = {"type": "2D path",
              "position": [position.lon, position.lat]}
      plan.append(node)
    request = {"operator ID": operator_id,
               "UAS ID": uas_id,
               "EPSG": epsg,
               "plan": plan,
               "when": takeoff_time.isoformat(),
               "preferred speed": speed,
               "maximum speed": max_speed,
               "minimum speed": 0.0,
               "preferred rate of ascend": ascend_rate,
               "maximum rate of ascend": 1.2*ascend_rate,
               "preferred rate of descend": descend_rate,
               "maximum rate of descend": 1.2*descend_rate}
    answer = self._ussp_client.request_plan(request)
    if "reply" not in answer \
      or answer["reply"] == "error":
      raise dss.auxiliaries.exception.Error
    if "plan ID" in answer and "delay" in answer:
      return (answer["plan ID"], answer["delay"])
    else:
      raise dss.auxiliaries.exception.Error

  '''
  Function: get_plan
  Input parameters:
  "plan ID": the ID of the plan (UUID)

  Return parameters:
  ("not ready", "time") - where "time is number of seconds you need to wait for the plan to be computed
  ("authorized", "plan") - The authorized flight (containing a number of nodes to visit)
  ("invalid id", "") - If the plan ID in the request is invalid
  ("not authorized", "message") - The reason for rejecting a flight authorization
  "status" - "not ready", "authorized", "not authorized", "invalid id"
  "time" - number of seconds until you should wait for the plan (only for "not ready")
  "plan" - the authorized flight, contains a number of "nodes" to visit (only for "authorized")
  "message" - the error message (only for "not authorized")
  '''
  def get_plan(self, plan_id):
    answer = self._ussp_client.get_plan(plan_id)
    if "reply" not in answer \
      or answer["reply"] == "error" \
      or "status" not in answer:
      raise dss.auxiliaries.exception.Error
    if answer["status"] == "authorized":
      res = (answer["status"], answer["plan"])
    elif answer["status"] == "not ready" :
      res = (answer["status"], answer["time"])
    else:
      if "message" in answer :
        res = (answer["status"], answer["message"])
      else :
        res = (answer["status"], None)
    return res

  '''
  Function: accept_plan
  Input parameters:
  "plan ID": the ID of the plan (UUID)

  Return parameters:
  "status" - "accepted", "invalid ID"
  '''
  def accept_plan(self, plan_id):
    answer = self._ussp_client.accept_plan(plan_id)
    if "reply" not in answer \
      or answer["reply"] == "error":
      raise dss.auxiliaries.exception.Error
    try :
      accepted = answer["status"] == "accepted"
    except KeyError:
      accepted = False
    return accepted

  '''
  Input parameters:
  Function: activate_plan
  "plan ID": the ID of the plan (UUID)
  "time until withdrawn": Debug parameter for triggering a plan withdrawn message. Represents the time until the message is sent.

  Return parameters:
  "status" - "activated", "invalid ID"
  '''
  def activate_plan(self, plan_id, time_unitl_withdrawn=None):
    answer = self._ussp_client.activate_plan(plan_id, time_unitl_withdrawn)
    if "reply" not in answer \
      or answer["reply"] == "error":
      self._logger.warning(answer)
      return False
    try :
      activated = answer["status"] == "activated"
    except KeyError:
      activated = False
    return activated

  '''
  Function: cancel_plan
  Input parameters:
  "plan ID": the ID of the plan (UUID)

  Return parameters:
  "status" - "cancelled", "invalid ID"
  '''
  def cancel_plan(self, plan_id):
    answer = self._ussp_client.cancel_plan(plan_id)
    if "reply" not in answer \
      or answer["reply"] == "error":
      self._logger.warning(answer)
      return False
    try :
      cancelled = answer["status"] == "cancelled"
    except KeyError:
      cancelled = False
    return cancelled

  '''
  Function: end_plan
  Input parameters:
  "plan ID": the ID of the plan (UUID)

  Return parameters:
  "status" - "ended", "invalid ID"
  '''
  def end_plan(self, plan_id):
    answer = self._ussp_client.end_plan(plan_id)
    if "reply" not in answer \
      or answer["reply"] == "error":
      self._logger.warning(answer)
      return False
    try :
      ended = answer["status"] == "ended"
    except KeyError:
      ended = False
    return ended
