'''zmq auxiliaries'''

import base64
import ipaddress
import json
import logging
import socket
import threading
import traceback
import typing

import zmq

import dss.auxiliaries.exception
import dss.auxiliaries.config

#--------------------------------------------------------------------#

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.0'
__copyright__ = 'Copyright (c) 2020-2021, RISE'
__status__ = 'development'

#--------------------------------------------------------------------#

_logger = logging.getLogger(__name__)

#--------------------------------------------------------------------#

def Context() -> zmq.Context:
  return zmq.Context()

def get_subnet(ip: typing.Optional[str] = None, port: typing.Optional[int] = None) -> str:
  if ip:
    for subnet in dss.auxiliaries.config.config['subnets']:
      if dss.auxiliaries.config.config['subnets'][subnet]['ip'] in ip:
        return subnet

  if port:
    for subnet in dss.auxiliaries.config.config['subnets']:
      if dss.auxiliaries.config.config['subnets'][subnet]['port_min'] <= port <= dss.auxiliaries.config.config['subnets'][subnet]['port_max']:
        return subnet

  return ''

#--------------------------------------------------------------------#

def get_ip_address() -> str:
  soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  try:
    # doesn't even have to be reachable
    soc.connect(('10.255.255.255', 1))
    address = soc.getsockname()[0]
  except:
    address = '127.0.0.1'
  finally:
    soc.close()
  return address

def get_ip() -> str:
  '''Returns the ip of the vpn subnet. If that is not possible, it
  uses get_ip_address as fallback strategy.'''
  from netifaces import AF_INET, ifaddresses, interfaces

  # If dronenet VPN is used, find the VPN ip of host machine
  for ifaceName in interfaces():
    my_ip = [i['addr'] for i in ifaddresses(ifaceName).setdefault(AF_INET, [{'addr':'No IP addr'}])]
    if '10.44.1' in my_ip[0]:
      return my_ip[0]

  return get_ip_address()

def valid_ip(ip: str, localhost: bool = False, asterisk: bool = False) -> bool:
  try:
    assert isinstance(ip, str)
    ipaddress.ip_address(ip)
  except:
    if localhost and ip == 'localhost':
      return True
    if asterisk and ip == '*':
      return True
    return False
  return True

def get_fcn(msg: dict) -> str:
  return msg['fcn'] if 'fcn' in msg else ''

def ack(call: str, args: typing.Optional[dict] = None) -> dict:
  msg = {'fcn': 'ack', 'call': call}
  if args:
    args.update(msg)
    return args
  else:
    return msg

def nack(call: str, desc: str) -> dict:
  return {'fcn': 'nack', 'call': call, 'description': desc}

def send_and_receive(socket, msg: dict) -> dict:
  json_msg = json.dumps(msg)

  try:
    socket.send_json(json_msg)
  except zmq.error.ZMQError as error:
    raise dss.auxiliaries.exception.NoAnswer(msg, socket.ip, socket.port)

  try:
    json_reply = socket.recv_json()
  except zmq.error.Again as error:
    raise dss.auxiliaries.exception.NoAnswer(msg, socket.ip, socket.port)

  return json.loads(json_reply)

def is_ack(answer: dict, call: typing.Optional[str] = None) -> bool:
  if answer.get('fcn') == 'ack':
    return answer.get('call') == call if call else True
  return False

def is_nack(answer: dict, call: typing.Optional[str] = None) -> bool:
  return not is_ack(answer, call)

def get_nack_reason(answer: dict) -> str:
  '''
  Returns a string that describes the nack reason.

  :raises dss.auxiliaries.exception.InputError: if there is any error
  '''
  if is_nack(answer):
    return answer.get('description', 'unknown nack reason')
  raise dss.auxiliaries.exception.InputError(answer, 'not a nack message')

def close_socket_gracefully(socket) -> None:
  '''graceful termination'''
  if socket:
    socket.setsockopt(zmq.LINGER, 0) # to avoid hanging infinitely
    socket.close()

def mogrify(topic: str, msg: dict) -> str:
  '''Combines a topic identifier and a json representation of a dictionary'''
  return '%s %s' % (topic, json.dumps(msg))

def demogrify(msg: str) -> typing.Tuple[str, dict]:
  '''Inverse of mogrify()'''
  try:
    topic, message = msg.split(maxsplit=1)
  except ValueError:
    topic, message = (msg, '{}')

  try:
    message = json.loads(message)
  except:
    message = {}
    _logger.error(traceback.format_exc())

  return topic, message

def image_to_bytes(filename: str) -> bytes:
  '''t.ex. filename="test.jpg"'''
  with open(filename, "rb") as fh:
    data = fh.read()
    data = base64.b64encode(data)
  return data

def bytes_to_string(data: bytes) -> str:
  return data.decode('utf-8')

def string_to_bytes(data: str) -> bytes:
  return str.encode(data)

def bytes_to_image(filename: str, data: bytes) -> None:
  with open(filename, "wb") as fh:
    fh.write(base64.decodebytes(data))

def save_json(filename: str, data: dict) -> None:
  with open(filename, "w") as fh:
    fh.write(json.dumps(data, indent=4))

#--------------------------------------------------------------------#
class _Socket:
  def __init__(self, context, ip, port, label, timeout, socket_type=None, self_id=None) -> None:

    tags = list()
    if self_id:
      tags.append(f'[{self_id}]')
    if socket_type:
      tags.append(f'[{socket_type}]')
    if label:
      tags.append(f'[{label}]')

    self._context = context
    self._ip = ip
    self._label = ' '.join(tags)
    self._port = port
    self._socket = None
    self._timeout = timeout  #in milliseconds

  def __del__(self) -> None:
    self.close()

  @property
  def ip(self) -> str:
    '''Returns the ip number'''
    return self._ip

  @property
  def port(self) -> int:
    '''Returns the port number'''
    return self._port

  def close(self) -> None:
    '''graceful termination'''
    if self._socket:
      _logger.debug(f'{self._label} Disconnecting')
      self._socket.setsockopt(zmq.LINGER, 0) # to avoid hanging infinitely
      self._socket.close()
      self._socket = None

  # Set label, the label is not always known when init socket, app_id can be missing
  def add_id_to_label(self, label):
    self._label = '[' + label + '] ' + self._label
#--------------------------------------------------------------------#

class Req(_Socket):
  def __init__(self, context, ip, port, label=None, timeout=1000, self_id=None) -> None:
    _Socket.__init__(self, context, ip, port, label, timeout, socket_type='req', self_id=self_id)

    self._alive = False
    self._event = threading.Event()
    self._heartbeat_msg = None
    self._mutex = threading.Lock()
    self._thread = None

    self.connect()

  def close(self) -> None:
    if self._thread:
      # stop heartbeat thread
      self._alive = False
      self._event.set()
      self._thread.join()
      self._thread = None

    _Socket.close(self)

  def connect(self) -> None:
    #assert valid_ip(self._ip, localhost=True), f'bad ip address: {self._ip}'

    self._socket = self._context.socket(zmq.REQ)
    self._socket.connect(f'tcp://{self._ip}:{self._port}')
    self._socket.RCVTIMEO = self._timeout
    _logger.debug(f'{self._label} Connected to tcp://{self._ip}:{self._port} with timeout {self._timeout}')

  def start_heartbeat(self, client_id=None) -> None:
    # update heartbeat message
    if client_id:
      self._heartbeat_msg = {'fcn': 'heart_beat', 'id': client_id}

    # thread is already running
    if self._thread:
      return

    # start heartbeat thread
    if self._heartbeat_msg:
      self._thread = threading.Thread(target=self._main_heartbeat, daemon=True)
      self._thread.start()

  def reconnect(self):
    _logger.info(f'{self._label} reconnecting...')
    _Socket.close(self)
    self.connect()

  def _send_and_receive(self, msg: dict) -> dict:
    _logger.debug(f'{self._label} send: %s', str(msg)[:256])

    try:
      json_msg = json.dumps(msg)
      self._socket.send_json(json_msg)
    except zmq.error.ZMQError as error:
      raise dss.auxiliaries.exception.NoAnswer(msg, self.ip, self.port)
    else:
      try:
        json_reply = self._socket.recv_json()
      except zmq.error.Again as error:
        self.reconnect()
        raise dss.auxiliaries.exception.NoAnswer(msg, self.ip, self.port)
      else:
        answer = json.loads(json_reply)
        self._event.set()  # indicates successful communication

    _logger.debug(f'{self._label} recv: %s\n', str(answer)[:256])
    return answer

  def send_and_receive(self, msg: dict) -> dict:
    with self._mutex:
      return self._send_and_receive(msg)

  def _send_and_receive_string(self, msg: dict) -> dict:
    _logger.debug(f'{self._label} send: %s', str(msg)[:256])

    try:
      json_msg = json.dumps(msg)
      self._socket.send_string(json_msg)
    except zmq.error.ZMQError as error:
      raise dss.auxiliaries.exception.NoAnswer(msg, self.ip, self.port)
    else:
      try:
        json_reply = json.loads(self._socket.recv())
      except zmq.error.Again as error:
        self.reconnect()
        raise dss.auxiliaries.exception.NoAnswer(msg, self.ip, self.port)
      else:
        answer = json_reply
        self._event.set()  # indicates successful communication

    _logger.debug(f'{self._label} recv: %s\n', str(answer)[:256])
    return answer

  def send_and_receive_string(self, msg: dict) -> dict:
    with self._mutex:
      return self._send_and_receive_string(msg)



  def _main_heartbeat(self):
    '''Send a heartbeat if no other messages were sent'''
    attempts = 0
    self._alive = True
    tick = 0
    while self._alive:
      self._event.clear()
      self._event.wait(timeout=self._timeout/1000.0)
      if not self._event.is_set():
        heartbeat_msg = self._heartbeat_msg
        heartbeat_msg['tick'] = tick
        tick += 1
        answer = self.send_and_receive(heartbeat_msg)
        if not dss.auxiliaries.zmq.is_ack(answer):
          attempts += 1
          if attempts < 3:
            _logger.warning(f"{self._label} no response to heartbeat ({attempts})")
          elif attempts == 3:
            _logger.error(f"{self._label} no response to heartbeat ({attempts})")
        else:
          attempts = 0

#--------------------------------------------------------------------#

class Rep(_Socket):
  def __init__(self, context, ip='*', port=None, label=None, timeout=1000, min_port=6000, max_port=6100, self_id=None) -> None:
    _Socket.__init__(self, context, ip, port, label, timeout, socket_type='rep', self_id=self_id)
    self.min_port = min_port
    self.max_port = max_port
    self.connect()

  def connect(self) -> None:
    assert valid_ip(self._ip, asterisk=True), f'bad ip address: {self._ip}'

    self._socket = self._context.socket(zmq.REP)
    if self._port:
      self._socket.bind(f'tcp://{self._ip}:{self._port}')
    else:
      self._port = self._socket.bind_to_random_port(f'tcp://{self._ip}', min_port=self.min_port, max_port=self.max_port, max_tries=100)
    self._socket.RCVTIMEO = self._timeout  #in milliseconds
    _logger.debug(f'{self._label} Connected to tcp://{self._ip}:{self._port} with timeout {self._timeout}')

  def recv_json(self) -> str:
    request = self._socket.recv_json()
    _logger.debug(f'{self._label} recv: %s', str(request)[:256])
    return request

  def send_json(self, msg: str) -> None:
    try:
      self._socket.send_json(msg)
    except zmq.error.ZMQError as error:
      # Note to future-me:
      # This is problematic...because the recv-rep protocol would
      # become out of sync.
      _logger.warning(f'{self._label} send: {error}\n')
      raise
    else:
      _logger.debug(f'{self._label} send: %s\n', str(msg)[:256])

#--------------------------------------------------------------------#

class Pub(_Socket):
  def __init__(self, context, ip='*', port=None, label=None, timeout=1000, min_port=6000, max_port=6100, self_id=None, bind=True) -> None:
    _Socket.__init__(self, context, ip, port, label, timeout, socket_type='pub', self_id=self_id)
    self.min_port = min_port
    self.max_port = max_port
    self.connect(bind)

  def connect(self, bind) -> None:
    #assert valid_ip(self._ip, asterisk=True), f'bad ip address: {self._ip}'
    self._socket = self._context.socket(zmq.PUB)
    if bind:
      if self._port:
        self._socket.bind(f'tcp://{self._ip}:{self._port}')
      else:
        self._port = self._socket.bind_to_random_port(f'tcp://{self._ip}', min_port=self.min_port, max_port=self.max_port, max_tries=100)
      self._socket.RCVTIMEO = self._timeout  #in milliseconds
    else:
      if self._port:
        self._socket.connect(f'tcp://{self._ip}:{self._port}')
      else:
        raise dss.auxiliaries.exception.Error
    _logger.debug(f'{self._label} Connected to tcp://{self._ip}:{self._port} with timeout {self._timeout}')

  def publish(self, topic: str, msg: dict) -> None:
    json_msg = mogrify(topic, msg)
    self._socket.send_string(json_msg)
    _logger.debug(f'{self._label} %s\n', str(json_msg)[:256])

#--------------------------------------------------------------------#

class Sub(_Socket):
  def __init__(self, context, ip, port, label=None, timeout=1000, self_id=None) -> None:
    _Socket.__init__(self, context, ip, port, label, timeout, socket_type='sub', self_id=self_id)
    self.connect()

  def connect(self) -> None:
    assert valid_ip(self._ip, localhost=True), f'bad ip address: {self._ip}'
    self._socket = self._context.socket(zmq.SUB)
    self._socket.setsockopt_string(zmq.SUBSCRIBE, '')
    self._socket.RCVTIMEO = self._timeout # in milliseconds
    self._socket.connect(f'tcp://{self._ip}:{self._port}')
    _logger.debug(f'{self._label} Connected to tcp://{self._ip}:{self._port} with timeout {self._timeout}')

  def subscribe(self, topic) -> None:
    _logger.debug(f'{self._label} subscribe topic {topic}')
    self._socket.setsockopt_string(zmq.SUBSCRIBE, topic)

  def unsubscribe(self, topic) -> None:
    _logger.debug(f'{self._label} unsubscribe topic {topic}')
    self._socket.setsockopt_string(zmq.UNSUBSCRIBE, topic)

  def recv(self) -> typing.Tuple[str, dict]:
    msg = str(self._socket.recv(), 'utf-8')
    topic, msg = demogrify(msg)
    _logger.debug(f'{self._label} {topic}: %s', str(msg)[:256])
    return topic, msg
