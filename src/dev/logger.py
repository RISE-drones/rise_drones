#!/usr/bin/env python3
'''
ZMQ logger

This connects to a client and dumps all logging messages to a log
file. The logging messages are received via a subscribe socket.

TODO: Fix the hardcoded port (ask the client, instead of using --port)
TODO: Tell the client to enable logging using their request/reply socket
'''

import argparse
import logging
import sys

import zmq

import dss.auxiliaries
from dss.auxiliaries.config import config


def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='logger.py: zmq logging', allow_abbrev=False, add_help=False)
  parser.add_argument('-h', '--help', action='help', help=argparse.SUPPRESS)
  parser.add_argument('--id', default='logger')
  parser.add_argument('--port', default=5566)
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout')
  args = parser.parse_args()

  dss.auxiliaries.logging.configure(f'{args.id}.log', stdout=args.stdout, rotating=True, loglevel='info')
  _logger = logging.getLogger(f'dss.{args.id}')

  context = zmq.Context()

  crm = dss.auxiliaries.zmq.Req(context, config["default_crm_ip"], config["default_crm_port"])
  answer = crm.send_and_receive({'id': 'root', 'fcn': 'clients', 'filter': args.id})
  del crm

  if not dss.auxiliaries.zmq.is_ack(answer, 'clients'):
    print(answer)
    sys.exit(1)

  if len(answer['clients']) != 1:
    print(answer)
    sys.exit(1)

  ip = answer['clients'][0]['ip']
  socket = dss.auxiliaries.zmq.Sub(context, ip=ip, port=args.port, timeout=10000)

  while socket:
    try:
      topic, msg = socket.recv()
      _logger.info('%s: %s', topic, str(msg)[:256])
    except zmq.error.Again:
      pass
    except KeyboardInterrupt:
      socket = None # break the loop


if __name__ == '__main__':
  _main()
