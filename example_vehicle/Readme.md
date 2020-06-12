# Package overview
The *example_vehicle* package provides a reference implementation for the vehicle dynamics control module.

Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)

# Models
* `VehicleECU.slx` shows how an ECU model utilizing the software could look like.
* `PassengerVehicle.slx` is a simulation model consisting of a vehicle physics model and sensor models
* `controller_dev.slx` provides a simulation framework for controller development (requires sim_vehicle_dynamcis repository).
* `controller_dev_sg.slx` provides a loopback simulation framework for controller development with a Speedgoat Real-Time Computer (requires sim_vehicle_dynamcis repository).
* `controller_dev_RCP.slx` provides a control ECU simulation framework for controller development with a Speedgoat Real-Time Computer
* `controller_dev_HIL.slx` provides a vehicle physics HIL framework for controller development with a Speedgoat Real-Time Computer (requires sim_vehicle_dynamcis repository).
* `host_visualization.slx` provides a an interface to the Unreal Engine via the Vehicle Dynamics blockset for the controller development with a Speedgoat Real-Time Computer. 
