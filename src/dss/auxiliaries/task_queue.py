'''Asynchronous Task Queue

The task queue executes tasks in a seperate process.

Example:
  queue = TaskQueue()
  queue.add(task1, 1)
  queue.add(task1, 2)
  queue.add(task2)

  queue.start()
  queue.add(task2)

  queue.join()
  queue.stop()
'''

import threading
import time

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.2.0'
__copyright__ = 'Copyright (c) 2019-2021, RISE'
__status__ = 'development'

class TaskQueue:
  '''Asynchronous Task Queue'''
  def __init__(self, exception_handler=None):
    self._alive = False
    self._event = threading.Event()
    self._exception_handler = exception_handler
    self._mutex = threading.Lock()
    self._tasks = list()
    self._thread = None

  def start(self):
    '''Starts the execution of the task queue.'''
    if self._thread:
      return

    self._alive = True
    self._thread = threading.Thread(target=self._main)
    self._thread.start()

  def stop(self):
    '''Stops the execution and removes all tasks from the queue'''
    if self._thread is None:
      return

    self._alive = False
    self.clear()
    self._event.set()
    self._thread.join()

  def clear(self):
    '''Remove all queued tasks.'''
    with self._mutex:
      self._tasks.clear()

  def join(self):
    '''Wait until the task queue finished all tasks.'''
    self._event.set()
    while self._event.is_set():
      time.sleep(0.1)

  def add(self, task, arg1=None, arg2=None, arg3=None, arg4=None):
    '''Insert a task into the queue.'''
    with self._mutex:
      self._tasks.append((task, arg1, arg2, arg3, arg4))
    self._event.set()

  @property
  def idling(self):
    '''Returns true if there are no tasks in the queue.'''
    return not self._event.is_set()

  @property
  def alive(self):
    '''Returns true if the task queue is actually running.'''
    return self._alive

  def _main(self):
    while self.alive:
      self._event.wait()

      with self._mutex:
        if self._tasks:
          (task, arg1, arg2, arg3, arg4) = self._tasks.pop(0)
        else:
          (task, arg1, arg2, arg3, arg4) = (None, None, None, None, None)
          self._event.clear()

      if task:
        try:
          if arg1 is None:
            task()
          elif arg2 is None:
            task(arg1)
          elif arg3 is None:
            task(arg1, arg2)
          elif arg4 is None:
            task(arg1, arg2, arg3)
          else:
            task(arg1, arg2, arg3, arg4)
        except Exception as error:
          if self._exception_handler:
            self._exception_handler(error)
          else:
            raise
    self._event.clear()
