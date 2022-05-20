'''
Drone Safety Service *API*

This class is in charge of the socket amd the actual API as described
in documentation.
'''

import logging
import typing

import dss.auxiliaries

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.0.0'
__copyright__ = 'Copyright (c) 2021, RISE'
__status__ = 'development'

class DSS:
  def __init__(self, context, app_id, ip, port, dss_id, timeout=1000):
    self._logger = logging.getLogger(__name__)
    self._logger.info(f'DSS dss_api {dss.auxiliaries.git.describe()}')

    self._context = context
    self._app_id = app_id
    self._ip = ip
    self._port = port
    self._dss_id = dss_id

    self._socket = dss.auxiliaries.zmq.Req(context, ip, port, label=dss_id, timeout=timeout, self_id=app_id)
    self._socket.start_heartbeat(app_id)

  def __del__(self):
    self._socket.close()

  @property
  def app_id(self):
    return self._app_id

  @property
  def port(self):
    return self._port

  @property
  def dss_id(self):
    return self._dss_id

  @property
  def ip(self):
    return self._ip

  def heart_beat(self) -> None:
    call = 'heart_beat'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def get_info(self) -> dict:
    call = 'get_info'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # Take the returned id and store it in the class
    if answer['id'] != self._dss_id:
      self._dss_id = answer['id']
    # return
    return answer

  def who_controls(self) -> str:
    call = 'who_controls'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return answer['in_controls']

  def get_owner(self) -> str:
    call = 'get_owner'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return answer['owner']

  def set_owner(self) -> None:
    call = 'set_owner'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def set_geofence(self, height_low, height_high, radius) -> None:
    call = 'set_geofence'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['height_low'] = height_low
    msg['height_high'] = height_high
    msg['radius'] = radius
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def get_idle(self) -> bool:
    call = 'get_idle'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return answer['idle']

  def set_init_point(self, heading_ref) -> None:
    call = 'set_init_point'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['heading_ref'] = heading_ref
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def reset_dss_srtl(self) -> None:
    call = 'reset_dss_srtl'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def arm_take_off(self, height) -> None:
    call = 'arm_take_off'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['height'] = height
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def land(self) -> None:
    call = 'land'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def rtl(self) -> None:
    call = 'rtl'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def dss_srtl(self, hover_time) -> None:
    call = 'dss_srtl'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['hover_time'] = hover_time
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def set_vel_BODY(self, x, y, z, yaw_rate) -> None:
    call = 'set_vel_BODY'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['x'] = x
    msg['y'] = y
    msg['z'] = z
    msg['yaw_rate'] = yaw_rate
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return


  def set_heading(self, heading) -> None:
    call = 'set_heading'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['heading'] = heading
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def set_default_speed(self, default_speed) -> None:
    call = 'set_default_speed'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['default_speed'] = default_speed
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def upload_mission_LLA(self, mission) -> None:
    call = 'upload_mission_LLA'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['mission'] = mission
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def upload_mission_NED(self, mission) -> None:
    call = 'upload_mission_NED'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['mission'] = mission
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def upload_mission_XYZ(self, mission) -> None:
    call = 'upload_mission_XYZ'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['mission'] = mission
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def gogo(self, next_wp) -> None:
    call = 'gogo'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['next_wp'] = next_wp
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def set_pattern(self, pattern, rel_alt, heading, radius=10, yaw_rate=10) -> None:
    call = 'set_pattern'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['pattern'] = pattern
    msg['rel_alt'] = rel_alt
    msg['heading'] = heading
    # Circle pattern has more args
    if pattern == 'circle':
      msg['radius'] = radius
      msg['yaw_rate'] = yaw_rate
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def follow_stream(self, enable, ip, port) -> None:
    call = 'follow_stream'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['enable'] = enable
    msg['ip'] = ip
    msg['port'] = port
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def set_gimbal(self, roll, pitch, yaw) -> None:
    call = 'set_gimbal'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['roll'] = roll
    msg['pitch'] = pitch
    msg['yaw'] = yaw
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def set_gripper(self, enable, can_id) -> None:
    call = 'set_gripper'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['enable'] = enable
    msg['CAN_ID'] = can_id
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def photo(self, cmd, resolution='low', index='latest', enable=False, period=10, publish="low") -> None:
    call = 'photo'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['cmd'] = cmd
    # Download and continous photo have more args
    if cmd == 'download':
      msg['resolution'] = resolution
      msg['index'] = index
    elif cmd == 'continous_photo':
      msg['enable'] = enable
      msg['publish'] = publish
      msg['period'] = period
    elif cmd == 'record':
      msg['enable'] = enable
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def get_armed(self) -> bool:
    call = 'get_armed'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return answer['armed']

  def get_currentWP(self) -> typing.Tuple[int, int]:
    call = 'get_currentWP'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return answer['currentWP'], answer['finalWP']

  def get_flightmode(self) -> str:
    call = 'get_flightmode'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return answer['flightmode']

  def get_metadata(self, ref, index) -> dict:
    call = 'get_metadata'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['ref'] = ref
    msg['index'] = index
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return answer['metadata']

  def get_posD(self) -> float:
    call = 'get_posD'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return answer['posD']

  def get_PWM(self, channel) -> int:
    call = 'get_PWM'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['channel'] = channel
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return answer['PWM']

  def disconnect(self) -> None:
    call = 'disconnect'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return

  def data_stream(self, stream: str, enable: bool) -> None:
    call = 'data_stream'
    # build message
    msg = {'fcn': call, 'id': self._app_id}
    msg['stream'] = stream
    msg['enable'] = enable
    # send and receive message
    answer = self._socket.send_and_receive(msg)
    # handle nack
    if not dss.auxiliaries.zmq.is_ack(answer, call):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer), fcn=call)
    # return
    return
