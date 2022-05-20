#!/usr/bin/env python3
'''Minimal running example of a ZMQ REQ socket.

> ./_zmq_req.py --ip=127.0.0.1 & ./_zmq_rep.py
'''

import argparse

import zmq

import dss.auxiliaries


def _print(text):
  print(__file__ + ': ' + str(text))


def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='_zmq_req.py', allow_abbrev=False)
  parser.add_argument('--ip', default='10.44.160.10')
  parser.add_argument('--port', default='5556')
  parser.add_argument('--id', default='da000')
  args = parser.parse_args()

  socket = dss.auxiliaries.zmq.Req(zmq.Context(), args.ip, args.port)

  _print(socket.send_and_receive({'id':args.id, 'fcn': 'data_stream', 'enable': True, 'stream':'battery'}))

if __name__ == "__main__":
  _main()
