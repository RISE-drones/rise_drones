#!/usr/bin/env python3
'''Minimal running example of a ZMQ REQ socket.

> ./_zmq_req.py & ./_zmq_rep.py
'''

import argparse

import zmq

import dss.auxiliaries
from dss.auxiliaries.config import config


def _print(text):
  print(__file__ + ': ' + str(text))

def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='_zmq_req.py', allow_abbrev=False)
  parser.add_argument('--ip', default=config["default_crm_ip"], help=config["default_crm_ip"])
  parser.add_argument('--port', default=config["default_crm_port"], help=f'{config["default_crm_port"]}')
  parser.add_argument('--id', default=config["default_id"], help=config["default_id"])
  args = parser.parse_args()

  socket = dss.auxiliaries.zmq.Req(zmq.Context(), args.ip, args.port)
  _print(dss.auxiliaries.zmq.get_ip_address())

  msg = {'id':args.id, 'fcn': 'data_stream', 'enable': True, 'stream':'battery'}
  _print(str(msg))
  socket.send_and_receive(msg)

if __name__ == "__main__":
  _main()
