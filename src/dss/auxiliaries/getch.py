'''getch()-like unbuffered character reading from stdin on both Windows and UNIX

A small utility class to read single characters from standard input, on both
Windows and UNIX systems. It provides a getch() function-like instance.

Source: https://code.activestate.com/recipes/134892/'''

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.0'
__copyright__ = 'Copyright (c) 2021, RISE'
__status__ = 'development'

class _Getch:
  '''Gets a single character from standard input. Does not echo to the screen.'''
  def __init__(self):
    try:
      self._impl = _GetchWindows()
    except ImportError:
      self._impl = _GetchUnix()

  def __call__(self):
    return self._impl()

class _GetchUnix:
  def __call__(self):
    import sys
    import termios
    import tty
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
    finally:
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class _GetchWindows:
  def __init__(self):
    '''This should fail if msvcrt isn't available -> not on Windows'''
    import msvcrt #pylint: disable=import-outside-toplevel,unused-import

  def __call__(self):
    import msvcrt #pylint: disable=import-outside-toplevel
    return msvcrt.getch().decode()

getch = _Getch()
