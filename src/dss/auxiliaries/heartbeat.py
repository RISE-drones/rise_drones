'''heartbeat server and client

The heartbeat client can be started as follows:
  import heartbeat
  CLIENT = heartbeat.Client('tcp://127.0.0.1:5560', 3)
  CLIENT.alive = True

And this is how to check whether the connection to the heartbeat server still exists:
  if CLIENT.alive:
    pass
'''

import logging
import socket
import threading
import time

import zmq

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.0'
__copyright__ = 'Copyright (c) 2019-2021, RISE'
__status__ = 'development'

def _get_ip_address():
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

class _Instance:
  '''template for both the server and client implementation'''

  def __init__(self, _main, _start):
    '''Create a new heartbeat instance'''
    self._alive = False
    self._interval = 1.0
    self._logger = logging.getLogger(__name__)
    self._main_ = _main
    self._mutex = threading.Lock()
    self._start_ = _start
    self._thread = None

  @property
  def alive(self):
    '''Check if the heartbeat instance is alive'''
    with self._mutex:
      return self._alive

  @alive.setter
  def alive(self, value):
    if value is True:
      with self._mutex:
        if not self._alive:
          if self._start_():
            self._alive = True
            self._thread = threading.Thread(target=self._main_, daemon=True)
            self._thread.start()
    elif self._alive:
      self._alive = False
      self._thread.join()

  @property
  def interval(self):
    '''Returns the heartbeat interval in seconds'''
    return self._interval

class Server(_Instance):
  '''heartbeat server'''

  def __init__(self, address, interval, context=None):
    '''Create a new heartbeat server'''
    _Instance.__init__(self, self._main, self._start)

    self._interval = interval
    self._socket_str = address
    self._context = zmq.Context() if context is None else context
    self._socket = self._context.socket(zmq.PUB)

  def _start(self):
    '''Start the heartbeat server'''
    self._socket.bind(self._socket_str)
    self._logger.info('Starting heartbeat server on %s... done', self._socket_str)
    self._logger.info('Server address: %s', _get_ip_address())
    return True

  def _main(self):
    '''Internal main method of the heartbeat server'''
    topic = 'heartbeat'
    while self.alive:
      message = f'{topic} {self._interval}'
      self._logger.debug(message)
      self._socket.send_string(message)
      time.sleep(self._interval)

class Client(_Instance):
  '''heartbeat client'''

  def __init__(self, address, attempts, context=None):
    '''Create a new heartbeat client'''
    _Instance.__init__(self, self._main, self._start)

    self._attempts = attempts
    self._socket_str = address
    self._context = zmq.Context() if context is None else context
    self._socket = self._context.socket(zmq.SUB)
    self._vital = False

  @property
  def vital(self):
    '''Returns True if receiving heartbeats'''
    return self._vital

  @property
  def attempts(self):
    '''Returns the heartbeat attempts'''
    return self._attempts

  def _start(self):
    '''Start the heartbeat client'''
    self._socket.connect(self._socket_str)
    self._logger.info('Starting heartbeat client on %s... done', self._socket_str)

    self._socket.setsockopt_string(zmq.SUBSCRIBE, 'heartbeat')
    self._socket.RCVTIMEO = 1000 # in milliseconds

    return True

  def _main(self):
    '''Internal main method of the heartbeat client'''
    attempts = self._attempts
    while self.alive:
      try:
        message = str(self._socket.recv(), 'utf-8')
      except zmq.error.Again:
        if self._vital:
          if attempts > 0:
            attempts = attempts-1
            if attempts+1 < self._attempts:
              self._logger.warning('Failed to receive heartbeat (attempt #%d)', self._attempts-attempts)
          else:
            self._logger.error('Lost connection to the heartbeat server')
            self._socket.RCVTIMEO = 1000 # in milliseconds
            self._vital = False
      else:
        if not self._vital:
          self._vital = True
          self._logger.info('Connecting to heartbeat server... done')
          self._interval = float(message.split()[-1])
          self._logger.info('interval: %g', self._interval)
          self._socket.RCVTIMEO = int(self._interval*1000) # in milliseconds
        attempts = self._attempts
