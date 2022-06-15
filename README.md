# RISE drone system
Hi there, we are happy that you are here!✨ <br />
In this document you'll find brief information about RISE drone system, after which we help you get started. In order to grow and get better we greatly appreciate your feedback, feel free to contribute by following the contribution guidelines. In the last section you can find information about licensing and how you can utilize our system. 

## What is RISE drone system?
The platform is built to make it simplify the process of developing applications for autonomous systems and to make it easy to get a sensor in the air or lay out search patterns.

There are four basic building blocks of the software platform: 

**Application:** <br />
Utilizes one or several drones to execute missions defined in the application. This software is typically built from a python template. The application code decides what control commands that should be sent to the drone and when, such as take-off, goto waypoint, take photo etc. The application code can utilize the handy DSS-library or just implement the commands as they are described in the API. THe application can run anywhere on the network; on the drone, on the server or as a mobile app. 

**DSS:** <br />
Drone Safety Service (DSS) acts as a bridege between applications and the autopilot. The DSS receives commands from applications or other modules if necessary, it interprets them and tries to execute them thorugh the autopilot. Currently there are two DSS versions, one for Ardupilot and one for DJI. Both DSS offer the same API resulting in identical code on application level. 

**CRM:**<br />
Central Resource Manager (CRM) is a resource manager that runs in the network. THe main responibility for the CRM is to help applications ot connect to a drone resource (i.e a DSS) from the pool of available drone. A simple scenario would be that the application connect directly to the DSS. Now imagine managing several applications and drones (i.e several DSS's) manually, the task quickly becomes combersome. Using the CRM makes the task of dealing with several applications and drones more managable. Every applicaiton and DSS shall register to the CRM. The CRM then automatically vecomes the owner of the drones resources and assigns resources when they are needded. CRM is also responsible for sharing connection information between connected clients. 

**Network:** <br />
We use VPN to gather all involved drone resources and applications on the same network and to handle the security in the system. We consider all clients in the network friendly. The VPN-service that is normally used is an OpenVPN-server hosted on a RISE server. OpenVPN is compatible with 'all' platforms, including mobile devices.

To read more about the system go to the official documentation at [RISE.drone_platform](https://github.com/lochel/RISE.drone_platform)

## Getting started

1. Install necessary dependencies

> pip3 install GitPython
> sudo apt install libxml2-dev libxslt-dev
> pip3 install dronekit
> sudo apt install libgphoto2-dev
> pip3 install gphoto2

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
