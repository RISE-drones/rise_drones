# RISE drone system
Hi there, we are happy that you are here!✨ <br />
In this document you'll find brief information about RISE drone system, after which we help you get started. In order to grow and get better we greatly appreciate your feedback, feel free to contribute by following the contribution guidelines. In the last section you can find information about licensing and how you can utilize our system.

## What is RISE drone system?
The platform is built to simplify the process of developing applications for autonomous systems and to make it easy to get a sensor in the air or lay out search patterns.

There are three basic building blocks of the software platform:

**Application:** <br />
Utilizes one or several drones to execute missions defined in the application. This software is typically built from a python template. The application code decides what control commands that should be sent to the drone and when, such as take-off, goto waypoint, take photo etc. The application code can utilize the handy DSS-library or just implement the commands as they are described in the API. The application can run anywhere on the network; on the drone, on the server or as a mobile app.

**DSS:** <br />
Drone Safety Service (DSS) acts as a bridge between applications and the autopilot. The DSS receives commands from applications or other modules if necessary, it interprets them and tries to execute them through the autopilot. Currently there are two DSS versions, one for Ardupilot and one for DJI. Both DSS offer the same API resulting in identical code on application level.

**CRM:**<br />
Central Resource Manager (CRM) is a resource manager that runs in the network. The main responsibility for the CRM is to manage ownership of available resources and supply connection information. An application can request a specific drone or a drone per capability, if there are any suitable drone available in the pool of drones, the CRM will assign it and supply connection information. A simple scenario would be that the application connects directly to the DSS, this requires knowledge about IP and ports. Now imagine managing several applications and drones (i.e several DSS's) manually, the task quickly becomes cumbersome managing different ip numbers and ports. Using the CRM makes the task of dealing with several applications and drones more manageable. Every application and DSS shall register to the CRM and supply their connection information. The CRM then automatically becomes the owner of the drone resources and assigns resources and connection information when requested. In this setup the only knowledge required is the IP and port for the CRM.

## Getting started

1. Install necessary dependencies

> pip install -r requirements.txt

2. Install SITL - ardupilot

> git clone git@github.com:ArduPilot/ardupilot.git
> git submodule update --init --recursive
> python3 -m venv .ardupilot
> pip3 install -r requirements.txt
> Modify ardupilot/Tools/autotest/locations.txt

## Contributing
If you would want to contribute to RISE drone system please take a look at [the guide for contributing]() to find out more about the guidelines on how to proceed.

## License
RISE drone system is realeased under the [BSD 3-Clause License](https://opensource.org/licenses/BSD-3-Clause)
