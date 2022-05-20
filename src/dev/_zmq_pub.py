#!/usr/bin/env python3
'''Minimal running example of a ZMQ PUB socket.
'''

import time

import zmq

import dss.auxiliaries


def _print(text):
  print(__file__ + ': ' + str(text))

def _main():
  context = zmq.Context()

  socket = context.socket(zmq.PUB)
  socket.bind("tcp://*:5560")

  while socket:
    try:
      msg = {'key': 'value'}
      msg = dss.auxiliaries.zmq.mogrify('topic', msg)
      socket.send_string(msg)
      _print(msg)
      time.sleep(1)
    except KeyboardInterrupt:
      dss.auxiliaries.zmq.close_socket_gracefully(socket)
      socket = None

if __name__ == "__main__":
  _main()
