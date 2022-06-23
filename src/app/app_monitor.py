#!/usr/bin/env python3

import argparse
import threading
import logging
import threading
import time
import traceback
import sys
import json
import zmq
import math

import dss.auxiliaries
import dss.client
from mqtt_agent.mqtt_agent import MqttAgent

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
  def __init__(self, app_ip, app_id, crm, mqtt_agent):
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
    self._commands = {'get_info':         {'request': self._request_get_info},
                      'get_drone_data':   {'request': self._request_get_drone_data}}
    # Register with CRM (self.crm.app_id is first available after the register call)
    _ = self.crm.register(self._app_ip, self._app_socket.port)

    # Update socket labels with received id
    self._app_socket.add_id_to_label(self.crm.app_id)
    self._info_socket.add_id_to_label(self.crm.app_id)

    # All nack reasons raises exception, registreation is successful
    _logger.info('App %s listening on %s:%s', self.crm.app_id, self._app_socket.ip, self._app_socket.port)
    _logger.info(f'App_monitor registered with CRM: {self.crm.app_id}')

    # Clients that are connected to the CRM
    self.clients = []
    self._info_threads = {}
    #Store the data received from the drones
    self.drone_data = {}
    self.drone_data_locks = {}
    self.mqtt_agent = mqtt_agent
    self._mqtt_threads = {}

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
# Application reply functions
  def _request_get_info(self, msg):
    answer = dss.auxiliaries.zmq.ack(msg['fcn'])
    answer['id'] = self.crm.app_id
    answer['info_pub_port'] = self._info_socket.port
    answer['data_pub_port'] = None
    return answer

  def _request_get_drone_data(self, msg):
    answer = dss.auxiliaries.zmq.ack(msg['fcn'])
    self.drone_data_lock.acquire()
    answer['data'] = self.drone_data
    self.drone_data_lock.release()
    return answer

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
    self._info_threads[client['id']] = threading.Thread(target=self._subscriber_thread, args=(client,))
    self.drone_data_locks[client['id']]= threading.Lock()
    self._info_threads[client['id']].start()

  def setup_mqtt_client(self, client):
    self._mqtt_threads[client['id']] = threading.Thread(target=self._mqtt_client, args=(client,))
    self._mqtt_threads[client['id']].start()
# MQTT-thread. Connect an agent to WARA-PS core system and report position
  def _mqtt_client(self, client):
    drone_id = client['id']
    drone_name = "RISE-" + drone_id
    drone_type = "hexacopter"
    sim_real = "simulated"
    mqtt_agent = MqttAgent(drone_name, drone_type, sim_real)
    # Wait until position has been streamed
    time.sleep(2.0)
    rate: float = 1.0 / mqtt_agent.logic.rate #1.0
    while self.client_in_list(drone_id, self.clients):
      drone_data = None
      self.drone_data_locks[drone_id].acquire()
      try:
        drone_data = self.drone_data[drone_id]
      except KeyError:
        _logger.warning("No data received from drone with with ID %s" % drone_id)
      self.drone_data_locks[drone_id].release()
      if drone_data :
        mqtt_agent.set_lla(drone_data['lat'], drone_data['lon'], drone_data['alt'])
        mqtt_agent.set_heading(drone_data['heading'])
        speed = math.sqrt(drone_data['velocity'][0]**2 + drone_data['velocity'][1]**2)
        mqtt_agent.set_speed(speed)
        if speed > 0.1 :
          course = math.atan2(drone_data['velocity'][1], drone_data['velocity'][0])
          mqtt_agent.set_course(course)
      mqtt_agent.send_heartbeat()
      mqtt_agent.send_sensor_info()
      mqtt_agent.send_position()
      mqtt_agent.send_speed()
      mqtt_agent.send_course()
      mqtt_agent.send_heading()
      mqtt_agent.send_direct_execution_info()
      time.sleep(rate)

#--------------------------------------------------------------------#
  # An subscribe thread. One per client will be launched. Thread is killed when client is removed from list
  def _subscriber_thread(self, client):
    ip = client['ip']
    port = client['port']
    drone_id = client['id']
    # print("Debug: New client ip and port: ", ip, port)

    # Connect the Request socket to enable the LLA stream
    req_socket = dss.auxiliaries.zmq.Req(_context, ip, port, label=drone_id)
    # Enable stream
    stream = 'LLA'
    self.enable_stream(stream,req_socket)
    # Get info port from DSS
    sub_port = self.get_port(req_socket, 'info_pub_port')
    # print("Info pub port: ", sub_port)

    # Create subscription socket and start listening thread
    sub_socket = dss.auxiliaries.zmq.Sub(_context, ip, sub_port, drone_id)
    sub_socket.subscribe(stream)

    if self.mqtt_agent:
      self.setup_mqtt_client(client)

    while self.client_in_list(drone_id, self.clients):
      try:
        (topic, msg) = sub_socket.recv()
        if topic == stream:
          self.drone_data_locks[drone_id].acquire()
          self.drone_data[drone_id] = msg
          self.drone_data_locks[drone_id].release()
        else:
          print("Topic not recognized on info link: ", (topic, msg), '\r')
      except:
        pass
    #Remove the drone from the map
    self.drone_data_locks[drone_id].acquire()
    try :
      self.drone_data.pop(drone_id)
    except KeyError :
      _logger.info("No data received from client with ID %s" % drone_id)
    self.drone_data_locks[drone_id].release()
    self.drone_data_locks.pop(drone_id)
    self.disable_stream(stream,req_socket)
    sub_socket.unsubscribe(stream)
    sub_socket.close()
    req_socket.close()
    _logger.info("Stopped thread and closed socket for client: %s" % drone_id)

#--------------------------------------------------------------------#
  # Call the DSS reply socket using the req_socket to enable a stream
  # Ref fcn: 'data_stream'
  def enable_stream(self, stream, socket):
    msg = {'fcn': 'data_stream', 'id': self.crm.app_id}
    msg['stream'] = stream
    msg['enable'] = True
    answer = socket.send_and_receive(msg)
    if not dss.auxiliaries.zmq.is_ack(answer):
      _logger.error('data_stream error: %s', answer)

#--------------------------------------------------------------------#
  # Call the DSS reply socket using the req_socket to disable a stream
  # Ref fcn: 'data_stream'
  def disable_stream(self, stream, socket):
    msg = {'fcn': 'data_stream', 'id': self.crm.app_id}
    msg['stream'] = stream
    msg['enable'] = False
    answer = socket.send_and_receive(msg)
    if not dss.auxiliaries.zmq.is_ack(answer):
      _logger.error('data_stream error: %s', answer)

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
  parser.add_argument('--mqtt_agent', action='store_true', help='enable MQTT sensor reporting (level 1 agent) to WARA-PS core system')
  parser.add_argument('--log', type=str, default='debug', help='logging threshold')
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout')
  args = parser.parse_args()

  # Identify subnet to sort log files in structure
  subnet = dss.auxiliaries.zmq.get_subnet(ip=args.app_ip)
  # Initiate the log file
  dss.auxiliaries.logging.configure('app_monitor', stdout=args.stdout, rotating=True, loglevel=args.log, subdir=subnet)

  # Create the monitor class
  try:
    app = Monitor(args.app_ip, args.id, args.crm, args.mqtt_agent)
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
