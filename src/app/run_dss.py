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
  parser.add_argument('--baud', default='921600', required=False)
  parser.add_argument('--gcs-address', default=None, help='socket address', required=False)
  parser.add_argument('--log', type=str, default='debug', help='logging threshold', required=False)
  parser.add_argument('--dss_ip', type=str, default="127.0.0.1", help='the ip of the dss', required=False)
  parser.add_argument('--drone', type=str, help='<ip>:<port> of drone/mavproxy', required=True)
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout', required=False)
  parser.add_argument('--virgin', action='store_true', help='defines if to start from a backup or not', required=False)
  parser.add_argument('--with-autogain', action='store_true', help='Request GLANA to adjust the gain on every wp', required=False)
  parser.add_argument('--with-gcs', action='store_true', help='If used, flight requires connection to gcs', required=False)
  parser.add_argument('--with-photo', action='store_true', help='Specifies if dss server should connect to photo server', required=False)
  parser.add_argument('--with-rangefinder', action='store_true', help='Rangefinder is used for mission flights', required=False)
  parser.add_argument('--without-midstick-check', action='store_true', help='Disables the "throttle to mid-stick" check', required=False)
  parser.add_argument('--without-clearance-check', action='store_true', help='Disables the "low-high-low clearance" check from the operator', required=False)
  args = parser.parse_args()

  dss.auxiliaries.logging.configure('dss_stand-alone_dss.log', stdout=args.stdout, rotating=True, loglevel=args.log)

  # start dss
  try:
    server = Server(dss_ip=args.dss_ip, drone=args.drone, baud=args.baud, with_gcs=args.with_gcs, gcs_address=args.gcs_address, rangefinder=args.with_rangefinder, autogain=args.with_autogain, midstick_check=not args.without_midstick_check, clearance_check=not args.without_clearance_check, photo=args.with_photo)
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
