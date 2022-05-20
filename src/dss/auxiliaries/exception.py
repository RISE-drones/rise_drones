'''DSS exceptions'''

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.0'
__copyright__ = 'Copyright (c) 2019-2021, RISE'
__status__ = 'development'

class Error(Exception):
  '''Base class for exceptions in this module.'''

class NotImplemented(Error):
  '''Exception raised for not implemented functions.'''

class InputError(Error):
  '''Exception raised for errors in the input.

  Attributes:
  expr -- input expression in which the error occurred
  msg  -- explanation of the error
  '''
  def __init__(self, expr, msg):
    Error.__init__(self)
    self.expr = expr
    self.msg = msg

class AbortTask(Error):
  '''Exception raised if a task is aborted'''
  def __init__(self, msg=None):
    Error.__init__(self)
    if msg:
      self.msg = msg

class Nack(Error):
  '''Exception raised if a command returns nack'''
  def __init__(self, nack_reason, fcn=None):
    Error.__init__(self)
    self.msg = nack_reason
    self.fcn = fcn

class NoAnswer(Error):
  '''Exception raised if a command returns nack'''
  def __init__(self, msg, ip, port):
    Error.__init__(self)
    self.fcn = msg
    self.ip = ip
    self.port = port
