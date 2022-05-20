'''logging auxiliaries'''

import datetime
import logging
import os
import sys
import time
from logging.handlers import RotatingFileHandler

import dss.auxiliaries

#--------------------------------------------------------------------#

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.1'
__copyright__ = 'Copyright (c) 2020-2021, RISE'
__status__ = 'development'

#--------------------------------------------------------------------#

_logger = logging.getLogger(__name__)

#--------------------------------------------------------------------#

def configure(filename: str = '', stdout: bool = True, rotating = False, loglevel: str = 'INFO', subdir: str = '') -> None:
  '''configure logging'''

  dir = os.path.join('log', subdir)
  if not os.path.isdir(dir):
    try:
      os.makedirs(dir)
    except OSError:
      raise dss.auxiliaries.exception.Error(f'Creation of the log directory "{dir}" failed')

  timestamp = time.strftime('%Y%m%d_%H%M%S')

  if filename:
    filename = '{}_{}'.format(timestamp, filename)
  else:
    filename = timestamp

  if not filename.endswith('.log'):
    filename += '.log'

  log_exists = os.path.isfile(os.path.join(dir, filename))
  format = '%(asctime)s: %(levelname)s [%(name)s] %(message)s'
  formatter = logging.Formatter(format)

  if rotating:
    handler = RotatingFileHandler(filename=os.path.join(dir, filename), mode='a', maxBytes=5*1024*1024, backupCount=2)
    handler.setFormatter(formatter)
    if log_exists:
      handler.doRollover()
    logging.getLogger().addHandler(handler)
  else:
    logging.basicConfig(filename=os.path.join(dir, datetime.datetime.now().strftime('%Y%m%d-%H%M%S') + '-' + filename), level=logging.INFO, format=format)

  if stdout:
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logging.getLogger().addHandler(handler)

  # set log level
  numeric_level = getattr(logging, loglevel.upper(), None)
  if not isinstance(numeric_level, int):
    raise dss.auxiliaries.exception.InputError(loglevel, 'invalid log level')
  logging.getLogger('dss').setLevel(numeric_level)

  # always log version and command line arguments
  _logger.info(f'{sys.argv[0]} {dss.__version__} {dss.auxiliaries.git.describe()}')
  _logger.info(f'arguments: {sys.argv[1:]}')
