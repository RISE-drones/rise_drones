#!/usr/bin/env python3
'''Keyboard client for controlling DSS.'''

import argparse
import json
import logging
import sys
import threading
import time
import traceback

import dss.auxiliaries
import dss.client

__author__ = 'Lennart Ochel <lennart.ochel@ri.se>, Andreas Gising <andreas.gising@ri.se>, Kristoffer Bergman <kristoffer.bergman@ri.se>, Hanna Müller <hanna.muller@ri.se>, Joel Nordahl'
__version__ = '1.1.0'
__copyright__ = 'Copyright (c) 2020-2021, RISE'
__status__ = 'development'

class KeyboardClient(dss.client.Client):
  def __init__(self, app_ip, app_port, app_id, crm, drone_name):
    dss.client.Client.__init__(self, timeout=2000, exception_handler=self.exception_handler)
    self.app_ip = app_ip
    self.app_port = app_port
    #Connect to the CRM
    self._crm = dss.client.CRM(self._context, crm, 'APP_keyboard', 'CRM compatible app keyboard', app_id)
    self._info_thread = None
    self._data_thread = None

    # store drone name (used to verify get_drone)
    self._drone_name = drone_name

  # Set up the sockets
  def setup_dss_sockets(self, dss_ip):
    answer = self._dss.get_info()
    self._info_thread = threading.Thread(target=self._main_info_dss, args=[dss_ip, answer['info_pub_port']], daemon=True)
    self._info_thread.start()
    self._data_thread = threading.Thread(target=self._main_data_dss, args=[dss_ip, answer['data_pub_port']], daemon=True)
    self._data_thread.start()

  # Helper method register
  def register_to_crm(self):
    #Register to CRM
    try:
      answer = self._crm.register(self.app_ip, self.app_port)
      assert dss.auxiliaries.zmq.is_ack(answer)
    except:
      print('Registration to CRM failed, check connections')
      return
    print("Registered to CRM, received id: ", answer['id'])
    self._app_id = answer['id']
    self._alive = True

  # Exception handler
  def exception_handler(self, error):
    '''This catches all exceptions and triggers "smart rtl"'''
    if not self.alive:
      logging.info(traceback.format_exc())
      return

    if isinstance(error, dss.auxiliaries.exception.AbortTask):
      try:
        logging.error(error.msg if error.msg else 'Abort current task')
      except AttributeError:
        logging.error('Abort current task')
    elif isinstance(error, dss.auxiliaries.exception.Nack):
      logging.error('Nack: %s', error.msg)
    else:
      logging.critical(traceback.format_exc())
    #self.abort('Stop mission', rtl=True)

  # Thread that prints subscribed data
  def _main_info_dss(self, ip, port):
    _info_socket = dss.auxiliaries.zmq.Sub(self._context, ip, port, 'info ' + self._crm.app_id)
    while _info_socket:
      try:
        (topic, msg) = _info_socket.recv()

        if topic == "ATT":
          print("Current ATT is roll: " + str(msg['r']) + ", pitch: " + str(msg['p']) + ", yaw: " + str(msg['y']) + "\r")
        elif topic == "photo_LLA":
          print("New metadata - index: " + str(msg['index']) + " filename: " + msg['filename'] + ", lat:" + str(msg['lat']) + ", lon: " + str(msg['lon']) + ", alt: " + str(msg['alt']) + ", agl: " + str(msg['agl']) + ", pitch: " + str(msg['pitch']) + ", heading: " + str(msg['heading']) + "\r")
        elif topic == "photo_XYZ":
          print("New metadata - index: " + str(msg['index']) + " filename: " + msg['filename'] + ", x:" + str(msg['x']) + ", y: " + str(msg['y']) + ", z: " + str(msg['z']) + ", agl: " + str(msg['agl']) + ", pitch: " + str(msg['pitch'])+ ", heading: " + str(msg['heading']) + "\r")
        elif topic == "LLA":
          print("Current pos is lat: " + str(msg['lat']) + ", lon: " + str(msg['lon']) + ", alt: " + str(msg['alt']) + ", agl: " + str(msg['agl']) + ", heading: " + str(msg['heading']) + "\r")
        elif topic == "NED":
          print("Current pos is north: " + str(msg['north']) + ", east: " + str(msg['east']) + ", down: " + str(msg['down']) + ", agl: " + str(msg['agl']) + ", heading: " + str(msg['heading']) + "\r")
        elif topic == "XYZ":
          print("Current pos is x: " + str(msg['x']) + ", y: " + str(msg['y']) + ", z: " + str(msg['z']) + ", heading: " + str(msg['heading']) + ", agl: " + str(msg['agl']) + "\r")
        elif topic == "currentWP":
          print("Going towards wp: " + msg['currentWP'] + ", final wp is: " + msg['finalWP'] + "\r")
        elif topic == "battery":
          print(f'Battery: remaining_time: {msg["remaining_time"]}, voltage: {msg["voltage"]} \r')
        else:
          print("Topic not recognized on info link: ", (topic, msg), '\r')
      except:
        pass

  # Thread that reads subscribed data
  def _main_data_dss(self, ip, port):
    _data_socket = dss.auxiliaries.zmq.Sub(self._context, ip, port, 'data ' + self._crm.app_id)
    while _data_socket:
      try:
        (topic, msg) = _data_socket.recv()

        if topic in ('photo', 'photo_low'):
          data = dss.auxiliaries.zmq.string_to_bytes(msg["photo"])
          photo_filename = msg['metadata']['filename']
          dss.auxiliaries.zmq.bytes_to_image(photo_filename, data)
          json_filename = photo_filename[:-4] + ".json"
          dss.auxiliaries.zmq.save_json(json_filename, msg['metadata'])
          print("Photo saved to " + msg['metadata']['filename']  + "\r")
          print("Photo metadata saved to " + json_filename + "\r")
        else:
          print("Topic not recognized on data link: ", (topic, msg))
      except:
        pass

  # Main
  def main(self):
    gimbal_pitch = 0
    att = False
    photo_lla = False
    photo_xyz = False
    battery = False
    lla = False
    ned = False
    xyz = False
    currentWP = False
    continous_photo = False
    follow_stream = False
    recording = False
    grip = False

    mission1 = {
      "id0": {
          "x": 4,
          "y": -19,
          "z": -15,
          "heading": 0,
          "speed": 4,
          "action": "take_photo"
      },
      "id1": {
          "x": 4,
          "y": 9,
          "z": -16,
          "heading": "course",
          "speed": 9,
          "action": "take_photo"
      },
      "id2": {
          "x": 4,
          "y": -9,
          "z": -16,
          "heading": "course",
          "speed": 9,
          "action": "take_photo"
      },
      "id3": {
          "x": 5,
          "y": 6,
          "z": -17,
          "heading": "course",
          "action": "take_photo"
      }
    }

    mission2 = {
      "id0": {
          "north": 4,
          "east": -9,
          "down": -15,
          "heading": 0,
          "speed": 3,
          "action": "take_photo"
      },
      "id1": {
          "north": 4,
          "east": 9,
          "down": -16,
          "heading": "course",
          "speed": 4,
          "action": "take_photo"
      },
      "id2": {
          "north": 4,
          "east": -9,
          "down": -16,
          "heading": 0,
          "speed": 2,
          "action": "take_photo"
      },
      "id3": {
          "north": 5,
          "east": 6,
          "down": -17,
          "heading": "course",
          "action": "take_photo"
      }
    }

    mission3 = {
      "id0": {"lat": 58.53310, "lon": 15.58094, "alt": 20, "alt_type": "relative", "heading": 359, "speed": 1.0},
      "id1": {"lat": 58.53319, "lon": 15.58091, "alt": 30, "alt_type": "relative", "heading": "course", "speed": 5.0},
      "id2": {"lat": 58.53316, "lon": 15.58099, "alt": 25, "alt_type": "relative", "heading": "course", "speed": 5.0}
    }

    mission4 = {
      "id0": {"x": -0.5305563525647523, "y": -0.20092107740748674, "z": -2.04372311898116, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id1": {"x": -0.6497710024298394, "y": 1.3945355949832718, "z": -2.061212366573624, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id2": {"x": -0.4113417026996652, "y": -1.7963777497982454, "z": -2.026233871388696, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id3": {"x": -0.45954293967676696, "y": -1.8131218010573675, "z": -3.2251484961645005, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id4": {"x": -0.5787575895418541, "y": -0.21766512866660895, "z": -3.2426377437569642, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id5": {"x": -0.6979722394069412, "y": 1.3777915437241497, "z": -3.260126991349428, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id6": {"x": -0.746173476384043, "y": 1.3610474924650275, "z": -4.459041616125233, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id7": {"x": -0.6269588265189558, "y": -0.23440917992573113, "z": -4.4415523685327685, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id8": {"x": -0.5077441766538686, "y": -1.8298658523164897, "z": -4.424063120940304, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id9": {"x": -0.410437639623493, "y": -2.6641446942292517, "z": -3.567027385519747, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id10": {"x": -0.31313110259311727, "y": -3.4984235361420133, "z": -2.7099916500991896, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id11": {"x": -0.2325197279020344, "y": -4.189562758384384, "z": -2.0, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id12": {"x": -0.3517343777671216, "y": -2.5941060859936256, "z": -2.017489247592464, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id13": {"x": -0.4709490276322087, "y": -0.998649413602867, "z": -2.034978495184928, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id14": {"x": -0.5901636774972958, "y": 0.5968072587878916, "z": -2.052467742777392, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id15": {"x": -0.7093783273623829, "y": 2.1922639311786503, "z": -2.069956990369856, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id16": {"x": -0.82859297722747, "y": 3.787720603569409, "z": -2.08744623796232, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id17": {"x": -0.8767942142045719, "y": 3.7709765523102865, "z": -3.2863608627381242, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id18": {"x": -0.7575795643394847, "y": 2.175519879919528, "z": -3.2688716151456605, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id19": {"x": -0.6383649144743976, "y": 0.5800632075287695, "z": -3.2513823675531963, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id20": {"x": -0.5191502646093105, "y": -1.0153934648619891, "z": -3.233893119960732, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id21": {"x": -0.3999356147442233, "y": -2.610850137252748, "z": -3.216403872368268, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id22": {"x": -0.2807209648791361, "y": -4.206306809643507, "z": -3.1989146247758042, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id23": {"x": -0.3289222018562379, "y": -4.223050860902629, "z": -4.3978292495516085, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id24": {"x": -0.44813685172132506, "y": -2.62759418851187, "z": -4.415318497144073, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id25": {"x": -0.5673515015864121, "y": -1.0321375161211113, "z": -4.432807744736536, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id26": {"x": -0.6865661514514994, "y": 0.5633191562696472, "z": -4.450296992329001, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id27": {"x": -0.8057808013165865, "y": 2.158775828660406, "z": -4.467786239921464, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id28": {"x": -0.9249954511816736, "y": 3.7542325010511646, "z": -4.4852754875139285, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id29": {"x": -0.9731966881587754, "y": 3.737488449792042, "z": -5.684190112289733, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id30": {"x": -0.8539820382936882, "y": 2.1420317774012836, "z": -5.666700864697269, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id31": {"x": -0.734767388428601, "y": 0.5465751050105251, "z": -5.649211617104806, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id32": {"x": -0.6155527385635139, "y": -1.0488815673802334, "z": -5.631722369512341, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id33": {"x": -0.49633808869842677, "y": -2.6443382397709922, "z": -5.614233121919877, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id34": {"x": -0.37712343883333965, "y": -4.23979491216175, "z": -5.596743874327413, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id35": {"x": -0.4253246758104414, "y": -4.256538963420873, "z": -6.795658499103217, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id36": {"x": -0.5445393256755285, "y": -2.661082291030114, "z": -6.813147746695681, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id37": {"x": -0.6637539755406157, "y": -1.0656256186393558, "z": -6.830636994288145, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id38": {"x": -0.7829686254057028, "y": 0.5298310537514028, "z": -6.848126241880609, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id39": {"x": -0.9021832752707899, "y": 2.1252877261421617, "z": -6.865615489473073, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id40": {"x": -1.0213979251358771, "y": 3.7207443985329203, "z": -6.883104737065537, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id41": {"x": -1.0695991621129788, "y": 3.704000347273798, "z": -8.082019361841342, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id42": {"x": -0.9503845122478917, "y": 2.1085436748830393, "z": -8.064530114248878, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id43": {"x": -0.8311698623828045, "y": 0.5130870024922807, "z": -8.047040866656413, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id44": {"x": -0.7119552125177174, "y": -1.082369669898478, "z": -8.02955161906395, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id45": {"x": -0.5927405626526303, "y": -2.6778263422892365, "z": -8.012062371471487, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id46": {"x": -0.47352591278754314, "y": -4.273283014679995, "z": -7.994573123879022, "heading": 4.2475163706801204, "speed": 1.0, "action": "take_photo"},
      "id47": {"x": 0.0, "y": 0.0, "z": -4.0, "heading": 0, "speed": 1.0, "action": "take_photo", "gimbal_pitch": 0}
    }

    while self.alive:
      print('> ', end = '', flush=True)
      key = dss.auxiliaries.getch()

      try:
        if key == 'h':
          print("help")
          print("  [1] upload XYZ mission")
          print("  [2] upload NED mission")
          print("  [3] upload LLA mission")
          print("  [4] upload Spotscale mission")
          print("  [5] get drone with specific name")
          print("  [6] get drone with camera capability")
          print("  [7] verify who_controls the drone")
          print("  [8] Release the current drone (CRM)")
          print("  [9] Disconnect to the current drone (DSS)")
          print("  [0] Await controls")
          print("  [a] neg yawrate")
          print("  [A] set heading to 270")
          print("  [d] pos yawrate")
          print("  [D] set heading to 90")
          print("  [f] toggle follow stream, dummy ip")
          print("  [g] toggle gimbal pitch")
          print("  [G] gogo")
          print("  [i] fwd")
          print("  [I] set init point - drone ref")
          print("  [j] left")
          print("  [k] back")
          print("  [L] land and disarm")
          print("  [l] right")
          print("  [m] get latest metadata XYZ")
          print("  [M] get all metadata XYZ")
          print("  [n] get latest metadata LLA")
          print("  [N] get all metadata LLA")
          print("  [o] download latest photo")
          print("  [O] toggle continous photo")
          print("  [P] download all photos")
          print("  [y] toggle recording on/off")
          print("  [p] take photo")
          print("  [Q] exit")
          print("  [q] toggle_gripper")
          print("  [r] engage autopilot rtl")
          print("  [R] engage DSS smart rtl")
          print("  [s] down")
          print("  [T] arm and takeoff")
          print("  [w] up")
          print("  [W] set_alt 20")
          print("  [z] toggle data stream: currentWP")
          print("  [x] toggle data stream: ATT")
          print("  [c] toggle data stream: XYZ")
          print("  [C] toggle data stream: photo_XYZ")
          print("  [v] toggle data stream: NED")
          print("  [b] toggle data stream: LLA")
          print("  [B] toggle data stream: photo_LLA")
          print("  [V] toggle data stream: battery")

        elif key == 'a':
          print("set_vel_BODY: neg yawrate")
          self._dss.set_vel_BODY(0.0, 0.0, 0.0, -10.0)
        elif key == 'A':
          print("set_heading: 270")
          self._dss.set_heading(270)
        elif key == 'd':
          print("set_vel_BODY: pos yawrate")
          self._dss.set_vel_BODY(0.0, 0.0, 0.0, 10.0)
        elif key == 'D':
          print("set_heading: 90")
          self._dss.set_heading(90)
        elif key == 'f':
          follow_stream = not follow_stream
          if follow_stream:
            print("Enable follow stream")
          else:
            print("Disable follow stream")
          self._dss.follow_stream(follow_stream, "127.0.0.1", "1234")

        elif key == 'g':
          print("toggle gimbal pitch", end='')
          if gimbal_pitch == 0:
            gimbal_pitch = -20
          elif gimbal_pitch < 0:
            gimbal_pitch = 20
          else:
            gimbal_pitch = 0
          print(' to %d' % gimbal_pitch)
          self._dss.set_gimbal(gimbal_pitch, 0, 0)
        elif key == 'G':
          print("gogo, start wp 0")
          self._dss.gogo(0)
        elif key == 'i':
          print("set_vel_BODY: fwd")
          self._dss.set_vel_BODY(1.0, 0.0, 0.0, 0.0)
        elif key == 'I':
          print("set_init_point - drone")
          self._dss.set_init_point("drone")
        elif key == 'j':
          print("set_vel_BODY: left")
          self._dss.set_vel_BODY(0.0, -1.0, 0.0, 0.0)
        elif key == 'k':
          print("set_vel_BODY: back")
          self._dss.set_vel_BODY(-1.0, 0.0, 0.0, 0.0)
        elif key == 'L':
          print("land")
          self._dss.land()
        elif key == 'l':
          print("set_vel_BODY: right")
          self._dss.set_vel_BODY(0.0, 1.0, 0.0, 0.0)
        elif key == 'm':
          json_metadata = self._dss.get_metadata("XYZ","latest")
          with open('metadata.json', "w") as fh:
            fh.write(json.dumps(json_metadata, indent=4))
          print('Metadata saved to metadata.json')
        elif key == 'M':
          json_metadata = self._dss.get_metadata("XYZ", "all")
          with open('metadata.json', "w") as fh:
            fh.write(json.dumps(json_metadata, indent=4))
          print('Metadata saved to metadata.json')
        elif key == 'n':
          json_metadata = self._dss.get_metadata("LLA","latest")
          with open('metadata.json', "w") as fh:
            fh.write(json.dumps(json_metadata, indent=4))
          print('Metadata saved to metadata.json')
        elif key == 'N':
          json_metadata = self._dss.get_metadata("LLA", "all")
          with open('metadata.json', "w") as fh:
            fh.write(json.dumps(json_metadata, indent=4))
          print('Metadata saved to metadata.json')
        elif key == 'P':
          print("download photo")
          self.photo_download("all", "high")
        elif key == 'p':
          print("take photo")
          self.photo_take_photo()
        elif key == 'o':
          print("download last photo")
          self.photo_download("latest", "low")
        elif key == 'O':
          print("toggle continous photo")
          if continous_photo:
            continous_photo = False
          else:
            continous_photo = True
          self.photo_continous_photo(continous_photo, 0, "low")
        elif key == 'y':
          print("Toggle recording")
          recording = not recording
          self.photo_rec(recording)
        elif key == 'q':
          print("Toggle gripper")
          grip = not grip
          if grip:
            self.load_package()
          else :
            self.unload_package()
        elif key == 'Q':
          print("Abort, unregister (CRM) and disconnect (DSS)")
          if self._dss is not None:
            self._crm.release_drone(self._drone_name)
            self.close_dss_socket()
          self._crm.unregister()
          self._alive = False
          return
        elif key == 'r':
          print("rtl (the autopilots built in rtl)")
          self.rtl()
        elif key == 'R':
          print("DSS smart rtl")
          self.dss_srtl(5)
        elif key == 's':
          print("set_vel_BODY: down")
          self._dss.set_vel_BODY(0.0, 0.0, 1.0, 0.0)
        elif key == 'T':
          print("arm and takeoff")
          self.arm_and_takeoff(4.0)
          self._dss.reset_dss_srtl()
        elif key == 'w':
          print("set_vel_BODY: up")
          self._dss.set_vel_BODY(0.0, 0.0, -1.0, 0.0)
        elif key == 'z':
          if currentWP:
            print("disable data stream: currentWP")
            self._dss.data_stream('currentWP', False)
            currentWP = False
          else:
            print("enable data stream: currentWP")
            self._dss.data_stream('currentWP', True)
            currentWP = True
        elif key == 'x':
          if att:
            print("disable data stream: ATT")
            self._dss.data_stream('ATT', False)
            att = False
          else:
            print("enable data stream: ATT")
            self._dss.data_stream('ATT', True)
            att = True
        elif key == 'c':
          if xyz:
            print("disable data stream: XYZ")
            self._dss.data_stream('XYZ', False)
            xyz = False
          else:
            print("enable data stream: XYZ")
            self._dss.data_stream('XYZ', True)
            xyz = True
        elif key == 'C':
          if photo_xyz:
            print("disable data stream: photo_XYZ")
            self._dss.data_stream('photo_XYZ', False)
            photo_xyz = False
          else:
            print("enable data stream: photo_XYZ")
            self._dss.data_stream('photo_XYZ', True)
            photo_xyz = True
        elif key == 'v':
          if ned:
            print("disable data stream: NED")
            self._dss.data_stream('NED', False)
            ned = False
          else:
            print("enable data stream: NED")
            self._dss.data_stream('NED', True)
            ned = True
        elif key == 'b':
          if lla:
            print("disable data stream: LLA")
            self._dss.data_stream('LLA', False)
            lla = False
          else:
            print("enable data stream: LLA")
            self._dss.data_stream('LLA', True)
            lla = True
        elif key == 'B':
          if photo_lla:
            print("disable data stream: photo_LLA")
            self._dss.data_stream('photo_LLA', False)
            photo_lla = False
          else:
            print("enable data stream: photo_LLA")
            self._dss.data_stream('photo_LLA', True)
            photo_lla = True
        elif key == 'V':
          if battery:
            print("disable data stream: battery")
            self._dss.data_stream('battery', False)
            battery = False
          else:
            print("enable data stream: battery")
            self._dss.data_stream('battery', True)
            battery = True

        elif key == '1':
          print("Upload mission XYZ")
          print(json.dumps(mission1, indent=4))
          self._dss.upload_mission_XYZ(mission1)
        elif key == '2':
          print("Upload mission NED")
          print(json.dumps(mission2, indent=4))
          self._dss.upload_mission_NED(mission2)
        elif key == '3':
          print("Upload mission LLA")
          print(json.dumps(mission3, indent=4))
          self._dss.upload_mission_LLA(mission3)
        elif key == '4':
          print("Upload Spotscale mission")
          print(json.dumps(mission4, indent=4))
          self._dss.upload_mission_LLA(mission4)
        elif key == '5':
          if self._dss is not None:
            print("Already connected to a drone")
          else:
            print(f"Trying to connect to drone with name: [{self._drone_name}]")
            answer = self._crm.get_drone(force=self._drone_name)
            print(json.dumps(answer, indent=4))
            if not dss.auxiliaries.zmq.is_ack(answer, 'get_drone'):
              print(f"No available drone with name: [{self._drone_name}]")
            else:
              self._drone_name = answer['id']
              time.sleep(1.0)
              self.connect(answer['ip'], answer['port'])
              print(f"Successfully connected to drone: [{self._drone_name}]")
        elif key == '6':
          if self._dss is not None:
            print("Already connected to a drone")
          else:
            print("Trying to connect to drone with capability: [camera]")
            answer = self._crm.get_drone(capability='camera')
            if not dss.auxiliaries.zmq.is_ack(answer, 'get_drone'):
              print("No available drone with capability: [camera]")
            else:
              self._drone_name = answer['id']
              time.sleep(1.0)
              self.connect(answer['ip'], answer['port'])
              print(f"Successfully connected to drone: [{self._drone_name}]")
        elif key == '7':
          if self._dss is not None:
            print(f"result from who_controls command to drone: {self._dss.who_controls()}")
            print(f"result from get_owner command to drone: {self._dss.get_owner()}")
          else:
            print("Not connected to any drone, unable to run command: [who_controls]")
        elif key == '8':
          if self._dss is None:
            print("No connected drone")
          else:
            print("Running command release drone")
            answer = self._crm.release_drone(self._drone_name)
            if not dss.auxiliaries.zmq.is_ack(answer, 'release_drone'):
              print(f"Not possible to release drone, reason: {answer['description']}")
            else:
              # Close the DSS socket such that no heartbeat messages are sent.
              print(f"The drone with id {self._drone_name} has successfully been released")
              self.close_dss_socket()
        elif key == '9':
          if self._dss is None:
            print("No connected drone")
          else:
            print("Running command DSS disconnect")
            self.dss_disconnect()
        elif key == '0':
          print('Add task await_controls')
          print('APPLICATION waiting for the controls')
          self.await_controls()
          print('APPLICATION has the controls')
        else:
          print("invalid command; press h for help")
      except dss.auxiliaries.exception.Nack as nack:
        print('NACK: {}'.format(nack.msg))

def _main():
  # parse command-line arguments
  parser = argparse.ArgumentParser(description='DSS-APP "APP-Keyboard"', allow_abbrev=False, add_help=False)
  parser.add_argument('-h', '--help', action='help', help=argparse.SUPPRESS)
  parser.add_argument('--drone', type=str, default='', help='A specific drone ID to connect to')
  parser.add_argument('--crm', type=str, help='<ip>:<port> of crm', required=True)
  parser.add_argument('--app_ip', type=str, help='ip of the application', required=True)
  parser.add_argument('--app_port', type=int, default=17180, help='port of the application, use subnet*100+x')
  parser.add_argument('--id', type=str, default='', help='id of the app provided by CRM (set to '' if not launched by CRM)')
  parser.add_argument('--log', type=str, default='debug', help='logging threshold')
  parser.add_argument('--stdout', action='store_true', help='enables logging to stdout')
  args = parser.parse_args()

  subnet = dss.auxiliaries.zmq.get_subnet(ip=args.app_ip)
  dss.auxiliaries.logging.configure(f'{args.id}_app_keyboard_crm', stdout=args.stdout, rotating=True, loglevel=args.log, subdir=subnet)

  try:
    client = KeyboardClient(args.app_ip, args.app_port, args.id, args.crm, args.drone)
    client.register_to_crm()
  except:
    print('Registration to CRM failed, check connections')
    logging.exception('Registration to CRM failed, check connections')
    sys.exit()

  try:
    client.main()
  except dss.auxiliaries.exception.AbortTask as error:
    print("PILOT took controls! Game is over, press Crtl-C..")
    logging.error(f'Task was aborted, {error.msg}')
    client.abort()
    # We get stuck in getch?
  except:
    logging.exception(traceback.format_exc())


if __name__ == '__main__':
  _main()
