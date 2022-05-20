'''Drone Safety Service'''

from dss.auxiliaries import (exception, git, heartbeat, logging, spawnDaemon,
                             zmq)
from dss.auxiliaries.getch import getch
from dss.auxiliaries.task_queue import TaskQueue

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.0.0'
__copyright__ = 'Copyright (c) 2019-2021, RISE'
__status__ = 'development'
