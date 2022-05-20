'''git auxiliaries'''

import logging
import traceback

#--------------------------------------------------------------------#

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.0.0'
__copyright__ = 'Copyright (c) 2022, RISE'
__status__ = 'development'

#--------------------------------------------------------------------#

_logger = logging.getLogger(__name__)

#--------------------------------------------------------------------#

def branch() -> str:
  try:
    import git
    repo = git.Repo(search_parent_directories=True)
    return repo.active_branch.name
  except:
    _logger.error(traceback.format_exc())
    return '??'

def describe() -> str:
  try:
    import git
    repo = git.Repo(search_parent_directories=True)
    return repo.git.describe()
  except:
    _logger.error(traceback.format_exc())
    return '??'

def pull() -> None:
  try:
    import git
    repo = git.Repo(search_parent_directories=True)
    repo.remotes.origin.pull()
  except:
    _logger.error(traceback.format_exc())
