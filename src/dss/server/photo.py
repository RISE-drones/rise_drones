'''Drone Safety Service Photo

This module controls a camera connected via usb using the gphoto lib. It
provides a simple interface to connect and configure a camera and to take
pictures.

Messages:
Client sends:   {'fcn': 'heartbeat'}
Server replies: {'fcn': 'ack', 'arg': 'heartbeat'}

Client sends:   {'fcn': 'connect', 'name': 'Canon EOS 1100D'}
Server replies: {'fcn': 'ack', 'arg': 'connect'}

Client sends:   {'fcn': 'take_picture'}
Server replies: {'fcn': 'ack', 'arg': 'take_picture'}

Client sends:   {'fcn': 'disconnect'}
Server replies: {'fcn': 'ack', 'arg': 'disconnect'}
'''

import datetime
import json
import logging
import os
import threading
import time

try:
  import gphoto2
except ImportError:
  pass
import zmq

import dss.auxiliaries

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.0'
__copyright__ = 'Copyright (c) 2020-2021, RISE'
__status__ = 'development'

class Server:
  def __init__(self, storage_dir: str, address: str, data_stream_addr:str, context=None):
    # create all objects that are used in the destructor
    self._alive = False
    self._att_data = None
    self._camera = None
    self._data_stream_addr = data_stream_addr
    self._gps_data = None # file handle
    self._last_timestamp = time.time()
    self._lgf_data = None
    self._mutex = threading.Lock() # used for _lgf_data
    self._recording = False
    self._serv_socket = None
    self._thread = None

    self._logger = logging.getLogger(__name__)

    # task queue
    self._task_queue = dss.auxiliaries.TaskQueue()

    # storage dir
    self._img_counter = 0
    self._storage_dir = os.path.join(storage_dir, datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
    if not os.path.isdir(self._storage_dir):
      try:
        os.makedirs(self._storage_dir)
      except OSError:
        raise dss.auxiliaries.exception.Error('Creation of the storage directory failed')

    # file handle
    self._gps_data = open(os.path.join(self._storage_dir, 'gps_data.csv'), 'w')
    self._gps_data.write('index;lgf.lat;lgf.lon;lgf.alt;att.r;att.p;att.y;elapsedtime;gain\n')

    # zmq socket
    self._zmq_context = zmq.Context() if context is None else context
    self._serv_socket = self._zmq_context.socket(zmq.REP)
    self._serv_socket.bind(address)
    self._serv_socket.RCVTIMEO = 1000 #ms
    self._logger.info('Starting photo service on %s... done', address)

    self._commands = {'autogain':     {'request': self._request_autogain,     'task': None},
                      'connect':      {'request': self._request_connect,      'task': self._task_connect},
                      'disconnect':   {'request': self._request_disconnect,   'task': None},
                      'heartbeat':    {'request': self._request_heartbeat,    'task': None},
                      'rec_ok':       {'request': self._request_rec_ok,       'task': None},
                      'start_rec':    {'request': self._request_start_rec,    'task': self._task_start_rec},
                      'stop_rec':     {'request': self._request_stop_rec,     'task': None},
                      'take_picture': {'request': self._request_take_picture, 'task': self._task_take_picture},
                      'up':           {'request': self._request_up,           'task': None}}

    # connect to camera
    self._callback_obj = gphoto2.check_result(gphoto2.use_python_logging())

    # make a list of all available cameras
    self._camera_list = list(gphoto2.Camera.autodetect())
    if not self._camera_list:
      raise dss.auxiliaries.exception.Error('No camera detected')
    self._camera_list.sort(key=lambda x: x[0])

    for (name, addr) in self._camera_list:
      self._logger.info('port "{:s}"; device "{:s}"'.format(addr, name))

  def __del__(self):
    self._recording = False
    self.alive = False

    if self._serv_socket:
      dss.auxiliaries.zmq.close_socket_gracefully(self._serv_socket)

    if self._camera:
      gphoto2.check_result(gphoto2.gp_camera_exit(self._camera))

    if self._gps_data:
      self._gps_data.close()

    if self._thread:
      self._thread.join()

  @property
  def alive(self):
    '''Checks if the service is alive'''
    return self._alive

  @alive.setter
  def alive(self, value):
    self._alive = value

  def _gps_main(self):
    socket = self._zmq_context.socket(zmq.SUB)
    socket.connect(self._data_stream_addr)
    self._logger.info('Subscribing to dss data stream on %s... done', self._data_stream_addr)
    socket.setsockopt_string(zmq.SUBSCRIBE, '')
    socket.RCVTIMEO = 1000 # in milliseconds

    while self.alive:
      try:
        message = str(socket.recv(), 'utf-8')
      except zmq.error.Again:
        pass
      else:
        (topic, data) = dss.auxiliaries.zmq.demogrify(message)
        if topic == 'LGF':
          with self._mutex:
            self._lgf_data = data
        elif topic == 'ATT':
          with self._mutex:
            self._att_data = data
        print((topic, data))

  def run(self):
    self.alive = True

    self._task_queue.clear()
    self._task_queue.start()

    self._thread = threading.Thread(target=self._gps_main)
    self._thread.start()

    try:
      while self.alive:
        try:
          msg = self._serv_socket.recv_json()
        except zmq.error.Again:
          continue

        msg = json.loads(msg)
        fcn = msg['fcn'] if 'fcn' in msg else ''

        self._logger.info('Message received: %s', msg)

        if fcn in self._commands:
          request = self._commands[fcn]['request']
          task = self._commands[fcn]['task']

          answer = request(msg)
          if task and answer['fcn'] == 'ack':
            self._task_queue.add(task, msg)
        else:
          answer = {'fcn': 'nack', 'arg': fcn, 'arg2': 'request not supported'}

        self._logger.info('Answer: %s', answer)

        answer = json.dumps(answer)
        self._serv_socket.send_json(answer)
    except KeyboardInterrupt:
      self.alive = False

    self._task_queue.stop()

    self._thread.join()
    self._thread = None

  #############################################################################
  # REQUESTS
  #############################################################################

  def _request_autogain(self, msg):
    return {'fcn': 'nack', 'arg': msg['fcn'], 'arg2': 'not yet implemented'}

  def _request_connect(self, msg):
    if self._camera:
      return {'fcn': 'nack', 'arg': msg['fcn'], 'arg2': 'already connected'}

    for (name, _) in self._camera_list:
      if name == msg['name']:
        return {'fcn': 'ack', 'arg': msg['fcn']}
    return {'fcn': 'nack', 'arg': msg['fcn'], 'arg2': 'camera not found'}

  def _request_disconnect(self, msg):
    self.alive = False
    return {'fcn': 'ack', 'arg': msg['fcn']}

  def _request_heartbeat(self, msg):
    return {'fcn': 'ack', 'arg': msg['fcn']}

  def _request_rec_ok(self, msg):
    if self._recording:
      return {'fcn': 'ack', 'arg': msg['fcn']}
    else:
      return {'fcn': 'nack', 'arg': msg['fcn'], 'arg2': ''}

  def _request_start_rec(self, msg):
    if self._recording:
      return {'fcn': 'nack', 'arg': msg['fcn'], 'arg2': 'already recording'}
    else:
      return {'fcn': 'ack', 'arg': msg['fcn']}

  def _request_stop_rec(self, msg):
    self._recording = False
    return {'fcn': 'ack', 'arg': msg['fcn']}

  def _request_take_picture(self, msg):
    if self._recording:
      return {'fcn': 'nack', 'arg': msg['fcn'], 'arg2': 'already recording'}
    else:
      return {'fcn': 'ack', 'arg': msg['fcn']}

  def _request_up(self, msg):
    return {'fcn': 'ack', 'arg': msg['fcn']}

  #############################################################################
  # TASKS
  #############################################################################

  def _task_connect(self, msg):
    # initialise chosen camera
    _addr = None
    for (name, addr) in self._camera_list:
      if name == msg['name']:
        _addr = addr

    self._camera = gphoto2.check_result(gphoto2.gp_camera_new())
    self._gp_context = gphoto2.gp_context_new()

    # search ports for camera port name
    port_info_list = gphoto2.PortInfoList()
    port_info_list.load()
    idx = port_info_list.lookup_path(_addr)
    self._camera.set_port_info(port_info_list[idx])
    gphoto2.check_result(gphoto2.gp_camera_init(self._camera, self._gp_context))

    summary = gphoto2.check_result(gphoto2.gp_camera_get_summary(self._camera))
    self._logger.info('Successfully connected:\n%s', summary.text)

  def _task_start_rec(self, msg):
    self._recording = True
    while self._recording:
      self._task_take_picture(msg)

  def _task_take_picture(self, msg):
    try:
      self._img_counter += 1
      file_path = self._camera.capture(gphoto2.GP_CAPTURE_IMAGE)

      # time
      timestamp = time.time()
      elapsedtime = timestamp - self._last_timestamp
      self._last_timestamp = timestamp

      # store metadata
      meta = os.path.join(self._storage_dir, 'img_%s.csv' % str(self._img_counter).zfill(5))
      metadata = ""
      with self._mutex:
        if self._lgf_data:
          metadata += "%g;%g;%g" % (self._lgf_data["lat"], self._lgf_data["lon"], self._lgf_data["alt"])
        else:
          metadata += ";;"

        if self._att_data:
          metadata += ";%g;%g;%g" % (self._att_data["r"], self._att_data["p"], self._att_data["y"])
        else:
          metadata += ";;;"
      metadata += ";%g;%s" % (elapsedtime, "") #elapsedtime, gain

      with open(meta, "w") as fh:
        fh.write("lgf.lat;lgf.lon;lgf.alt;att.r;att.p;att.y;elapsedtime;gain\n")
        fh.write(metadata + '\n')
      self._gps_data.write('%d;%s\n' % (self._img_counter, metadata))

      target = os.path.join(self._storage_dir, 'img_%s%s' % (str(self._img_counter).zfill(5), file_path.name[file_path.name.find('.'):]))

      self._logger.info('Copying image from %s/%s to %s', file_path.folder, file_path.name, target)
      camera_file = self._camera.file_get(file_path.folder, file_path.name, gphoto2.GP_FILE_TYPE_NORMAL)
      camera_file.save(target)
    except:
      pass

# ------------------------------------------------------------------------------
#
# CLIENT
#
# ------------------------------------------------------------------------------

class Client:
  def __init__(self, context, address):
    # create all objects that are used in the destructor
    self._photo_socket = None

    self._logger = logging.getLogger(__name__)

    self._photo_socket = context.socket(zmq.REQ)
    self._photo_socket.connect(address)
    self._photo_socket.RCVTIMEO = 500 # in milliseconds

    if not self.heartbeat():
      raise dss.auxiliaries.exception.Error("No connection to photo server")

  def __del__(self):
    if self._photo_socket:
      dss.auxiliaries.zmq.close_socket_gracefully(self._photo_socket)

  def request(self, msg: dict) -> dict:
    if not self._photo_socket:
      fcn = msg['fcn'] if 'fcn' in msg else ''
      answer = {'fcn': 'nack', 'arg': fcn, 'arg2': "Client isn't connected to the photo server"}
    else:
      answer = dss.auxiliaries.zmq.send_and_receive(self._photo_socket, msg)
    self._logger.info("Photo server replied: %s", answer)
    return answer

  def autogain(self) -> bool:
    answer = self.request({'fcn': 'autogain'})
    return dss.auxiliaries.zmq.is_ack(answer, 'autogain')

  def connect(self, name) -> bool:
    answer = self.request({'fcn': 'connect', 'name': name})
    return dss.auxiliaries.zmq.is_ack(answer, 'connect')

  def disconnect(self) -> bool:
    answer = self.request({'fcn': 'disconnect'})
    return dss.auxiliaries.zmq.is_ack(answer, 'disconnect')

  def heartbeat(self) -> bool:
    answer = self.request({'fcn': 'heartbeat'})
    return dss.auxiliaries.zmq.is_ack(answer, 'heartbeat')

  def rec_ok(self) -> bool:
    answer = self.request({'fcn': 'rec_ok'})
    return dss.auxiliaries.zmq.is_ack(answer, 'rec_ok')

  def start_rec(self) -> bool:
    answer = self.request({'fcn': 'start_rec'})
    return dss.auxiliaries.zmq.is_ack(answer, 'start_rec')

  def stop_rec(self) -> bool:
    answer = self.request({'fcn': 'stop_rec'})
    return dss.auxiliaries.zmq.is_ack(answer, 'stop_rec')

  def take_picture(self) -> bool:
    answer = self.request({'fcn': 'take_picture'})
    return dss.auxiliaries.zmq.is_ack(answer, 'take_picture')

  def up(self) -> bool:
    answer = self.request({'fcn': 'up'})
    return dss.auxiliaries.zmq.is_ack(answer, 'up')
