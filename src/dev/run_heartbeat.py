#!/usr/bin/env python3
'''heartbeat server and client

The heartbeat client can be started as follows:
  import dss.auxiliaries
  CLIENT = dss.auxiliaries.heartbeat.Client('tcp://127.0.0.1:5560', 3)
  CLIENT.alive = True

And this is how to check whether the connection to the heartbeat server still exists:
  if CLIENT.alive:
    pass
'''

import argparse
import logging
import time

import dss.auxiliaries

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.0'
__copyright__ = 'Copyright (c) 2019-2021, RISE'
__status__ = 'development'

if __name__ == '__main__':
  logging.basicConfig(level=logging.INFO, format='%(asctime)s: %(levelname)s [%(name)s] %(message)s')

  PARSER = argparse.ArgumentParser(description='This runs the heartbeat server by default', allow_abbrev=False, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  PARSER.add_argument('--address', default='tcp://*:5560', help='socket address')
  PARSER.add_argument('--attempts', type=int, default=3, help='attempts used by the heartbeat client')
  PARSER.add_argument('--client', action='store_true', help='run the heartbeat client instead')
  PARSER.add_argument('--interval', type=float, default=1.0, help='heartbeat interval used to send heartbeats by the server')
  ARGS = PARSER.parse_args()

  if ARGS.client:
    INSTANCE = dss.auxiliaries.heartbeat.Client(ARGS.address, ARGS.attempts)
  else:
    INSTANCE = dss.auxiliaries.heartbeat.Server(ARGS.address, ARGS.interval)

  INSTANCE.alive = True

  while INSTANCE.alive:
    try:
      time.sleep(INSTANCE.interval)
    except KeyboardInterrupt:
      INSTANCE.alive = False
