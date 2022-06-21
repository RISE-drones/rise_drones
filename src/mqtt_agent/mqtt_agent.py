import json
import ssl
import time
import traceback
from mqtt_agent.classes import Logic, MqttClient, NavData

'''
This code is used to create an MQTT agent to the WARA-PS core system. It is based on the
core system API specification v0.7 https://wasp-sweden.org/research/research-arenas/wara-ps-public-safety/
'''

class MqttAgent:
  def __init__(self, name, drone_type, sim_real) -> None:

    ###Nav data
    self.nav_data = NavData()

    ###Agent Logic
    self.logic = Logic(name, drone_type, sim_real)


    ###MQTT SETUP###
    self.mqtt_client = MqttClient(name, sim_real)
    self.mqtt_client.client.on_connect = self.on_connect
    self.mqtt_client.client.on_message = self.on_message
    self.mqtt_client.client.on_disconnect = self.on_disconnect
    self.connect()


  ########################################
  ########################################
  #################MQTT###################
  ########################################
  ########################################

  def connect(self):
    if self.mqtt_client.tls_connection:
      self.mqtt_client.client.username_pw_set(self.mqtt_client.user, self.mqtt_client.password)
      self.mqtt_client.client.tls_set(cert_reqs=ssl.CERT_NONE)
      self.mqtt_client.client.tls_insecure_set(True)

    self.mqtt_client.client.connect(self.mqtt_client.broker, self.mqtt_client.port, 60)
    self.mqtt_client.client.loop_start()

  def publish(self, topic, msg):
    _ = self.mqtt_client.client.publish(topic, msg)

  def disconnect(self):
    self.mqtt_client.client.disconnect()
    self.mqtt_client.client.loop_stop()

  # Callback function for PAHO
  def on_connect(self, clinet, userdata, flags, r_c):
    try:
      if r_c == 0:
        print(f"Connected to MQTT Broker: {self.mqtt_client.broker}:{self.mqtt_client.port}")
        self.mqtt_client.client.subscribe(f"{self.mqtt_client.base_topic}/exec/command")
        print(f"Subscribing to {self.mqtt_client.base_topic}/exec/command")
      else:
        print(f"Error to connect : {r_c}")
    except Exception as exc:
      print(traceback.format_exc())

  # Callback function for PAHO
  def on_message(self, client, userdata, msg):
    try:
      msg_str = msg.payload.decode("utf-8")
      msg_json = json.loads(msg_str)
      print(msg_json)

      if msg_json["command"] == "ping":
        print("RECEIVED COMMAND 'PING'")
        msg_res_json = {
          "com-uuid": msg_json["com-uuid"],
          "response": "pong",
          "response-to": msg_json["com-uuid"]
        }
        msg_res_str = json.dumps(msg_res_json)
        self.mqtt_client.client.publish(f'{self.mqtt_client.base_topic}/exec/response', msg_res_str)
        print(f"SENT RESPONSE! : {msg_res_str}")

      elif msg_json["command"] == "start-task":
        print("RECEIVED COMMAND 'start-task'")

        task_uuid = msg_json["task-uuid"]
        task = msg_json["task"]
        com_uuid = msg_json["com-uuid"]

        msg_res_json = {
          "agent-uuid": self.logic.uuid,
          "com-uuid": com_uuid,
          "fail-reason": "",
          "response": "",
          "response-to": com_uuid,
          "task-uuid": task_uuid
        }

      elif msg_json["command"] == "signal-task":
        print("RECEIVED COMMAND 'signal-task'")
        signal = msg_json["signal"]
        signal_task_uuid = msg_json["task-uuid"]
        com_uuid = msg_json["com-uuid"]

        msg_res_json = {
          "com-uuid": com_uuid,
          "response": "",
          "response-to": com_uuid
        }

        if self.logic.task_running_uuid == signal_task_uuid:
          if signal == "$abort":
            self.logic.task_running = False
          elif signal == "$enough":
            self.logic.task_running = False
          elif signal == "$pause":
            self.logic.task_pause_flag = True
          elif signal == "$continue":
            self.logic.task_pause_flag = False
          msg_res_json["response"] = "ok"
        else:
          msg_res_json["response"] = "failed"

        msg_res_str = json.dumps(msg_res_json)
        self.mqtt_client.client.publish(f'{self.mqtt_client.base_topic}/exec/response', msg_res_str)
        print(f"SENT RESPONSE! : {msg_res_str}")

    except Exception as e:
      print(traceback.format_exc())


  # Callback function for PAHO
  def on_disconnect(self, client, userdata, r_c):
    print(f"Client Got Disconnected from the broker with code {r_c}")
    if r_c == 5:
      print("No (or Wrong) Credentials, Edit username and password")

  def send_heartbeat(self):
    json_msg = {
      "name": self.logic.name,
      "agent-type": self.logic.type,
      "agent-description": self.logic.description,
      "agent-uuid": self.logic.uuid,
      "levels": self.logic.level,
      "rate": self.logic.rate,
      "stamp": time.time(),
      "type": "HeartBeat"
    }
    str_msg = json.dumps(json_msg)
    self.publish(
      f"{self.mqtt_client.base_topic}/heartbeat", str_msg)

  def send_sensor_info(self):
    json_msg = {
      "name": self.logic.name,
      "rate": self.logic.rate,
      "sensor-data-provided": [
        "position",
        "speed",
        "course",
        "heading",
      ],
      "stamp": time.time(),
      "type": "SensorInfo"
    }
    str_msg = json.dumps(json_msg)
    self.publish(
      f"{self.mqtt_client.base_topic}/sensor_info", str_msg)

  def send_position(self):
    json_msg = {
      "latitude": self.nav_data.lat,
      "longitude": self.nav_data.lon,
      "altitude": self.nav_data.alt,
      "type": "GeoPoint"
    }
    str_msg = json.dumps(json_msg)
    self.publish(
      f"{self.mqtt_client.base_topic}/sensor/position", str_msg)

  def send_speed(self):
    speed = self.nav_data.speed
    self.publish(f"{self.mqtt_client.base_topic}/sensor/speed", speed)

  def send_course(self):
    course = self.nav_data.course
    self.publish(f"{self.mqtt_client.base_topic}/sensor/course", course)

  def send_heading(self):
    heading = self.nav_data.heading
    self.publish(
      f"{self.mqtt_client.base_topic}/sensor/heading", heading)

  def send_direct_execution_info(self):
    json_msg = {
      "type": "DirectExecutionInfo",
      "name": self.logic.name,
      "rate": self.logic.rate,
      "stamp": time.time(),
      "tasks-available": self.logic.tasks_available
    }
    str_msg = json.dumps(json_msg)
    self.publish(f"{self.mqtt_client.base_topic}/direct_execution_info", str_msg)

  def set_speed(self, speed: float) -> None:
    self.nav_data.speed = speed

  def set_heading(self, heading: float) -> None:
    self.nav_data.heading = heading

  def set_course(self, course: float) -> None:
    self.nav_data.course = course

  def set_lla(self, lat: float, lon: float, alt: float) -> None:
    self.nav_data.lat = lat
    self.nav_data.lon = lon
    self.nav_data.alt = alt

  def is_task_supported(self, task: json) -> bool:
    """Checks if the task is supported by the agent"""
    name: str = task["name"]
    task_supported: bool = False
    for ava_task in self.logic.tasks_available:
      if name == ava_task["name"]:
        task_supported = True
        break
    return task_supported
