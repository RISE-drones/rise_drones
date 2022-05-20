#!/usr/bin/env python3

import argparse
import logging
import threading
import time
import traceback
import sys

import zmq


import dss.auxiliaries
import dss.client

#--------------------------------------------------------------------#

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.0.0'
__copyright__ = 'Copyright (c) 2021, RISE'
__status__ = 'development'

#--------------------------------------------------------------------#

_logger = logging.getLogger('dss.monitor')
_context = zmq.Context()

#--------------------------------------------------------------------#
class Monitor():
  def __init__(self, app_ip, app_id, crm):
    # Import the client lib
    dss.client.Client.__init__(self, timeout=2, exception_handler=None, context=_context)

    self.crm = dss.client.CRM(_context, crm, app_name='app_monitor.py', desc='Monitor application', app_id=app_id)
    #self._app_id = self.crm.app_id

    self._alive = True

    # If dronenet VPN is used, find the VPN ip of host machine
    # Find the VPN ip of host machine
    self._app_ip = app_ip
    auto_ip = dss.auxiliaries.zmq.get_ip()
    if auto_ip != app_ip:
      _logger.warning("Automatic get ip function and given ip does not agree: %s vs %s", auto_ip, app_ip)

    # all sockets
    self._app_socket = dss.auxiliaries.zmq.Rep(_context, label='app', min_port=self.crm.port, max_port=self.crm.port+50) #Rep: ANY -> APP

    # Register with CRM (self.crm.app_id is first available after the register call)
    _ = self.crm.register(self._app_ip, self._app_socket.port)

    # All nack reasons raises exception, registreation is successful
    _logger.info('App %s listening on %s:%s', self.crm.app_id, self._app_socket.ip, self._app_socket.port)
    _logger.info(f'App_monitor registered with CRM: {self.crm.app_id}')

    # Update socket labels with received id
    self._app_socket.add_id_to_label(self.crm.app_id)

    self.clients = []

#--------------------------------------------------------------------#
  @property
  def alive(self):
    '''checks if CRM is alive'''
    return self._alive

#--------------------------------------------------------------------#
  def kill(self):
    # Clean the clients list to stop subscription threads and closing sockets.
    self.clients = []

    # Unregister APP from CRM

    _logger.info("Unregister from CRM")
    answer = self.crm.unregister()
    if not dss.auxiliaries.zmq.is_ack(answer):
      _logger.error('Unregister failed: {answer}')
    _logger.info("CRM socket closed")

    if self._app_socket:
      self._app_socket.close()
      _logger.info("APP socket closed")
    _logger.debug('~ THE END ~')

#--------------------------------------------------------------------#
  def client_in_list(self, client_id, clients_list) -> bool:
    for client in clients_list:
      if client['id'] == client_id:
        return True
    return False

  def print_clients(self):
    if len(self.clients) == 0:
      print('\nThe current list of dss clients is empty')
    else:
      print('\nThe current list of dss clients, [#](ID, NAME):')
      i = 1
      for client in self.clients:
        print(f'  [{i}] {client["id"], client["name"]}')
        i += 1

#--------------------------------------------------------------------#
  def setup_client(self, client):
    self._info_thread = threading.Thread(target=self._subscriber_thread, args=(client,))
    self._info_thread.start()

#--------------------------------------------------------------------#
  # An subscribe thread. One per client will be launched. Thread is killed when client is removed from list
  def _subscriber_thread(self, client):
    ip = client['ip']
    port = client['port']
    id = client['id']
    # print("Debug: New client ip and port: ", ip, port)

    # Connect the Request socket to enable the LLA stream
    req_socket = dss.auxiliaries.zmq.Req(_context, ip, port, label=id)
    # Enable LLA stream
    self.enable_lla_stream(req_socket)
    # Get info port from DSS
    sub_port = self.get_port(req_socket, 'info_pub_port')
    # print("Info pub port: ", sub_port)

    # Create subscription socket and start listening thread
    sub_socket = dss.auxiliaries.zmq.Sub(_context, ip, sub_port, id)
    sub_socket.subscribe('LLA')

    while self.client_in_list(id, self.clients):
      try:
        (topic, msg) = sub_socket.recv()
        if topic == "LLA":
          print(
          id + ":---> lat: " + '{0:.7f}'.format(msg['lat']) +
          ", long: " + '{0:.7f}'.format(msg['lon']) +
          ", alt: " + '{0:.2f}'.format(msg['alt']) +
          ", heading: " + '{0:.1f}'.format(msg['heading']) +
          ", agl: " + '{0:.2f}'.format(msg['agl']) + "\r")
        else:
          print("Topic not recognized on info link: ", (topic, msg), '\r')
      except:
        pass

    sub_socket.unsubscribe('LLA')
    sub_socket.close()
    req_socket.close()
    _logger.info("Stopped thread and closed socket for client: %s", id)

#--------------------------------------------------------------------#
  # Call the DSS reply socket using the req_socket to enable the LLA-stream
  # Ref fcn: 'data_stream'
  def enable_lla_stream(self, socket):
    msg = {'fcn': 'data_stream', 'id': self.crm.app_id}
    msg['stream'] = 'LLA'
    msg['enable'] = True
    answer = socket.send_and_receive(msg)
    if not dss.auxiliaries.zmq.is_ack(answer):
      _logger.error('data_stream error: %s', answer)

    # TODO, return bool?

#--------------------------------------------------------------------#
  # Call the reply socket of the DSS to obtain the publish ports used
  # Ref fcn: 'ping'
  def get_port(self, socket, port_name) -> int:
    msg = {'fcn': 'get_info', 'id': self.crm.app_id}
    answer = socket.send_and_receive(msg)
    if not dss.auxiliaries.zmq.is_ack(answer):
      _logger.error('get_info error: %s', answer)
      return 0
    return int(answer[port_name])

#--------------------------------------------------------------------#
  def main(self):
    cursor = ['  |o....|','  |.o...|', '  |..o..|', '  |...o.|','  |....o|', '  |...o.|', '  |..o..|', '  |.o...|']
    cursor_index = 7


    # Prepare for main loop
    print("Monitoring dss clients registered with CRM")

    # Main loop looking for changes in crm clients list
    while self._alive:
      answer = self.crm.clients('dss')
      if dss.auxiliaries.zmq.is_ack(answer, 'clients'):
        # The latest list of clients from crm
        crm_clients = answer['clients']

        # Figure if we already have all clients in our local list. Loop through the CRM-list -> append
        for client in crm_clients:
          if not self.client_in_list(client['id'], self.clients):
            if client['ip'] != '':
              self.clients.append(client)
              print(f'Client {client["id"]} added to the list')
              self.setup_client(client)
              self.print_clients()
            else:
              print(client['id'] + " has no ip, not adding to list..")

        # Figure if there is a client on our local list that is not in the CRM-list -> pop
        index = 0
        for client in self.clients:
          if not self.client_in_list(client['id'], crm_clients):
            # Pop client, subscription will be ended and socket closed
            self.clients.pop(index)
            print('Client {the_client} popped from the list'.format(the_client=client['id']))
            self.print_clients()
          index += 1
      time.sleep(1)
      cursor_index += 1
      if cursor_index >= len(cursor):
        cursor_index = 0
      print(cursor[cursor_index], end = '\r', flush=True)


#--------------------------------------------------------------------#
def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='DSS-APP "monitor"', allow_abbrev=False, add_help=False)
  parser.add_argument('-h', '--help', action='help', help=argparse.SUPPRESS)
  parser.add_argument('--id', type=str, default=None, help='id of the app_monitor instance provided by crm')
  parser.add_argument('--crm', type=str, help='<ip>:<port> of crm', required=True)
  parser.add_argument('--app_ip', type=str, help='ip of the app', required=True)
  parser.add_argument('--log', type=str, default='debug', help='logging threshold')
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout')
  args = parser.parse_args()

  # Identify subnet to sort log files in structure
  subnet = dss.auxiliaries.zmq.get_subnet(ip=args.app_ip)
  # Initiate the log file
  dss.auxiliaries.logging.configure('app_monitor', stdout=args.stdout, rotating=True, loglevel=args.log, subdir=subnet)

  # Create the monitor class
  try:
    app = Monitor(args.app_ip, args.id, args.crm)
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
