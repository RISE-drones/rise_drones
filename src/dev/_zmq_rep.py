#!/usr/bin/env python3
'''Minimal running example of a ZMQ REP socket.

> ./_zmq_req.py --ip=127.0.0.1 & ./_zmq_rep.py
'''

import argparse
import json

import zmq

import dss.auxiliaries


def _print(text):
  print(__file__ + ': ' + str(text))

def receive_and_reply(socket):
  '''receives a json message and replies with the same message'''
  try:
    msg = socket.recv_json()
  except zmq.error.Again as error:
    _print('recv: ' + str(error))
    return

  # answer = msg
  answer = json.loads(msg)
  answer = json.dumps(answer)

  try:
    socket.send_json(answer)
  except zmq.error.ZMQError as error:
    _print('send: ' + str(error))
  else:
    _print(msg)

def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='_zmq_rep.py', allow_abbrev=False)
  parser.add_argument('--port', default='5556')
  args = parser.parse_args()

  socket = dss.auxiliaries.zmq.Rep(zmq.Context(), port=args.port)
  print(dss.auxiliaries.zmq.get_ip_address())

  while socket:
    try:
      receive_and_reply(socket)
    except KeyboardInterrupt:
      socket = None

if __name__ == "__main__":
  _main()
