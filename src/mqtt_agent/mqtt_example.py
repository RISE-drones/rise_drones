import time
from mqtt_agent.mqtt_agent import MqttAgent
import traceback

def main():
    try:
        name = "dss001"
        drone_type = "hexacopter"
        sim_real = "simulated"
        my_agent = MqttAgent(name, drone_type, sim_real)
        my_agent.set_lla(58.411003, 15.616561, 59.2)
        # Main loop
        rate: float = 1.0 / my_agent.logic.rate #1.0
        while True:
            my_agent.send_heartbeat()
            my_agent.send_sensor_info()
            my_agent.send_position()
            my_agent.send_speed()
            my_agent.send_course()
            my_agent.send_heading()
            my_agent.send_direct_execution_info()
            time.sleep(rate)

    except Exception as e:
        print(traceback.format_exc())

if __name__ == "__main__":
    main()
