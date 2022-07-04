'''CRM client'''

import logging

import dss.auxiliaries

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.0.0'
__copyright__ = 'Copyright (c) 2021, RISE'
__status__ = 'development'

class CRM:
  def __init__(self, context, crm, app_name, desc='', app_id=None):
    '''Either app_id or app_name is required'''
    self._logger = logging.getLogger(__name__)
    self._logger.info(f'CRM crm_api {dss.auxiliaries.git.describe()}')

    # Split crm connection string"
    (crm_ip, crm_port) = crm.split(':')
    crm_port = int(crm_port)

    self._context = context
    self._ip = crm_ip
    self._port = crm_port
    self._app_name = app_name
    self._desc = desc
    self._app_id = app_id

    # Create request socket, don't start heartbeat thread yet.
    self._socket = dss.auxiliaries.zmq.Req(self._context, self._ip, self._port, label='crm', timeout=1000)

  def __del__(self):
    if hasattr(self, '_socket'): # this is sometimes needed if the __init__ function failed
      self._socket.close()

  @property
  def port(self) -> int:
    return self._port

  @property
  def ip(self):
    return self._ip

  @property
  def app_id(self) -> str:
    assert self._app_id, f'no valid app_id: {self._app_id}'
    return self._app_id

  @property
  def app_name(self):
    return self._app_name

  def app_lost(self):
    return self._socket.send_and_receive({'id': self._app_id, 'fcn': 'app_lost'})

  def clients(self, filter=''):
    return self._socket.send_and_receive({'id': self._app_id, 'fcn': 'clients', 'filter': filter})

  def delStaleClients(self):
    return self._socket.send_and_receive({'id': self._app_id, 'fcn': 'delStaleClients'})

  def get_drone(self, capability=None, force=None):
    msg = {'fcn': 'get_drone', 'id': self._app_id}
    if capability is not None :
      msg['capability'] = capability
    if force is not None :
      msg['force'] = force
    return self._socket.send_and_receive(msg)

  def get_info(self):
    return self._socket.send_and_receive({'id': self._app_id, 'fcn': 'get_info'})

  def launch_app(self, app_name, launch : bool=True):
    return self._socket.send_and_receive({'id': self._app_id, 'fcn': 'launch_app', 'app': app_name, 'launch': launch})

  # TODO: launch_drone_helper
  # TODO: launch_dss
  # TODO: launch_sitl

  def register(self, app_ip, app_port, type='da', capabilities=[]):
    assert self._app_id != 'root', '"root" cannot get registered as application'

    answer = self._socket.send_and_receive({'fcn': 'register', 'name': self._app_name, 'desc': self._desc, 'type': type, 'capabilities': capabilities, 'id': self._app_id, 'ip': app_ip, 'port': app_port})
    if dss.auxiliaries.zmq.is_nack(answer):
      raise dss.auxiliaries.exception.Nack(dss.auxiliaries.zmq.get_nack_reason(answer))

    self._app_id = answer['id']
    self._socket.start_heartbeat(self._app_id)
    return answer

  def release_drone(self, dss_id):
    call = 'release_drone'
    msg = {'fcn': call, 'id': self._app_id, 'id_released': dss_id}
    return self._socket.send_and_receive(msg)

  def restart(self, virgin : bool=False):
    return self._socket.send_and_receive({'id': self._app_id, 'fcn': 'restart', 'virgin': virgin})

  def unregister(self):
    if self._app_id != 'root':
      answer = self._socket.send_and_receive({'fcn': 'unregister', 'id': self._app_id})
    else:
      answer = dss.auxiliaries.zmq.ack('unregister')
    self._socket.close()
    return answer

  def upgrade(self, virgin : bool=False):
    return self._socket.send_and_receive({'id': self._app_id, 'fcn': 'upgrade', 'virgin': virgin})
