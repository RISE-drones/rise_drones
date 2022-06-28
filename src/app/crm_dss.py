#!/usr/bin/env python3
'''This runs the dss server.'''

import argparse
import logging
import sys
import time
import traceback

import dss.auxiliaries
from dss.server import Server

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.2.0'
__copyright__ = 'Copyright (c) 2019-2021, RISE'
__status__ = 'development'

def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='DSS Server', allow_abbrev=False, add_help=False)
  parser.add_argument('-h', '--help', action='help', help=argparse.SUPPRESS)
  parser.add_argument('--crm', type=str, help='<ip>:<port> of crm', required=True)
  parser.add_argument('--drone', type=str, help='<ip>:<port> of drone/mavproxy', required=True)
  parser.add_argument('--dss_ip', type=str,help='ip of the crm_dss', required = True)
  #parser.add_argument('--owner', type=str, help='id of the connected TYRAmote instance', required=True)
  parser.add_argument('--dss_id', type=str, default='', help='id of the dss instance', required=False)
  parser.add_argument('--descr', type=str, default='crm_dss', help='description for register command', required=False)
  parser.add_argument('--log', type=str, default='debug', help='logging threshold', required=False)
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout', required=False)
  parser.add_argument('--virgin', action='store_true', help='defines if to start from a backup or not', required=False)
  parser.add_argument('--without-midstick-check', action='store_true', help='Disables the "throttle to mid-stick" check', required=False)
  parser.add_argument('--without-clearance-check', action='store_true', help='Disables the "low-high-low clearance" check from the operator', required=False)
  parser.set_defaults(feature=True)

  args = parser.parse_args()

  # Split drone connection string
  (_, drone_port) = args.drone.split(':')
  drone_port = int(drone_port)
  subnet = dss.auxiliaries.zmq.get_subnet(port=drone_port)
  dss.auxiliaries.logging.configure(f'{args.dss_id}.log', stdout=args.stdout, rotating=True, loglevel=args.log, subdir=subnet)

  # start dss
  try:
    server = Server(dss_ip=args.dss_ip, dss_id=args.dss_id, drone=args.drone, midstick_check=not args.without_midstick_check, clearance_check=not args.without_clearance_check, crm=args.crm, description=args.descr, die_gracefully=True)
  except dss.auxiliaries.exception.Error as error:
    logging.critical(str(error))
    sys.exit()
  except:
    logging.critical(traceback.format_exc())
    sys.exit()

  # run dss
  try:
    while server.alive:
      time.sleep(3.0)
  except KeyboardInterrupt:
    logging.warning('Shutdown due to keyboard interrupt')
    server.alive = False

if __name__ == '__main__':
  _main()
