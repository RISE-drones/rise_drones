#!/usr/bin/env python3
'''This is a command line application that sends simple commands to a
crm instance.'''

import argparse
import json

import zmq

import dss.auxiliaries
import dss.client
from dss.auxiliaries.config import config

#--------------------------------------------------------------------#

def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='manage-crm.py', allow_abbrev=False, add_help=False)
  parser.add_argument('-h', '--help', action='help', help=argparse.SUPPRESS)
  parser.add_argument('--app', type=str, required=False)
  parser.add_argument('--delStaleClients', action='store_true', help='deletes all stale clients')
  parser.add_argument('--info', action='store_true', help='crm info')
  parser.add_argument('--ip', type=str, default=config["default_crm_ip"], required=False)
  parser.add_argument('--list', action='store_true', help='print all clients')
  parser.add_argument('--log', type=str, default='info', help='logging threshold')
  parser.add_argument('--port', type=int, required=True)
  parser.add_argument('--restart', action='store_true', help='restarts the crm service')
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout')
  parser.add_argument('--upgrade', action='store_true', help='upgrades and restarts the crm service')
  parser.add_argument('--virgin', action='store_true', help='crm will reset client counter')
  args = parser.parse_args()

  subnet = dss.auxiliaries.zmq.get_subnet(port=args.port)
  dss.auxiliaries.logging.configure('manage_crm.log', stdout=args.stdout, rotating=True, loglevel=args.log, subdir=subnet)

  # Create connection string for crm. TODO Open pandora's box and change command line string instead
  crm_connection_string = args.ip + ":" + int(args.port)
  crm = dss.client.CRM(zmq.Context(), crm_connection_string, app_name='manage_crm.py', app_id='root')

  if args.delStaleClients:
    answer = crm.delStaleClients()
    if dss.auxiliaries.zmq.is_ack(answer, 'delStaleClients'):
      print(f'deleted: {answer["deleted"]}')
    else:
      print(answer)

  answer = crm.clients()
  if dss.auxiliaries.zmq.is_ack(answer, 'clients'):
    if args.list:
      print(json.dumps(answer['clients'], indent=2))
    print(f'{len(answer["clients"])} clients are registered')
  else:
    print(answer)

  if args.info:
    answer = crm.get_info()
    print(answer)

  if args.app:
    answer = crm.launch_app(args.app)
    print(answer)

  if args.upgrade:
    answer = crm.upgrade(args.virgin)
    if dss.auxiliaries.zmq.is_ack(answer, 'upgrade'):
      print('Updating the crm service...')
    else:
      print(answer)
  elif args.restart:
    answer = crm.restart(args.virgin)
    if dss.auxiliaries.zmq.is_ack(answer, 'restart'):
      print('Restarting the crm service...')
    else:
      print(answer)

#--------------------------------------------------------------------#

if __name__ == '__main__':
  _main()
