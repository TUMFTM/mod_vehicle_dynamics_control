# Autonomous Driving Control Software of TUM Roborace Team
### Overview
This software stack has been developed and used for the Roborace Event at the Berlin Formula-E Track 2018. It achieved approximately 150kph and 80% of the combined lateral and longitudinal acceleration potential of the DevBot. The overall research project is a joint effort of the Chair of Automotive Technology and the Chair of Automatic Control.

This software component covers the trajectory tracking, state estimation and vehicle dynamics control aspects of the stack. It takes trajectories from the planner as the main input and delivers appropriate steering, powertrain and brake commands. Furthermore, it handles vehicle startup and emergency brake situations.

To get started, take a look at the `veh_passenger` repository and follow the tutorial for the example vehicle.

# Modules - Vehicle Dynamics Control
This repository holds the core functionality of the controller. This covers state estimation, control and low level system handling, e.g. state machine.

# List of components
* `control`: everything that actuates the low level vehicle behavior and therefore acts as a control system, especially curvature, velocity and lateral tracking controller. Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)  
* `estimation`: Vehicle dynamics sensor fusion and fusion of different localization pipelines. Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)
* `interfaces`: Contains interpackage interface definitions. Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)
* `main`: Contains the main module model which collects all subcomponents. Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)
* `network`: Interfaces to other system components, e.g. via CAN or ethernet. Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)
* `scripts`: A collection of useful scripts for operating this module. Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)
* `sensordrivers`: Low level processing modules for vehicle dynamics sensors. Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)
* `softwareEmulation`: Replacments used for simulation purposes of the more complex planning parts in the full software stack. Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)
* `system`: Vehicle diagnosis, system startup and state machine logic. Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)
