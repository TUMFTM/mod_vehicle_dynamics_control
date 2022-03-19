====================================
Adapt the software to your vehicle
====================================

In the following, you find a guide how to modify the example vehicle such that it corresponds to your own vehicle. The software is currently deployed via code generation and a custom ROS2 integration, however it should be straight-forward to use it on other targets which support a Simulink Toolchain. The guide covers the functional changes necessary and gives a starting point which signals have to be used. The software stack is run by the Roborace Team of the TUM in an autonomous racing car. It aims at being developed independent from the specific use -case, however, development focus is high-performance driving. Therefore, some changes might be required depending on your vehicle configuration.

In general, it is recommended to modify the example vehicle. The only case where it is necessary to add a new vehicle is if you have to maintain multiple vehicle configurations or want to contribute to the upstream development. Please have in mind, that every vehicle has a two character identifier, which is used extensively throughout the project. In the case of this example vehicle it is `pa`. You will find this in front of many file names in this tutorials. All of these files are vehicle specific and may be subject to reconfiguration in a multiple vehicle setup.

The example vehicle is provided with two main models. The first, the simulation model (`example_vehicle/tests/controller_dev/controller_dev.slx`), is intended to test the functionality of the Vehicle ECU code with a vehicle which is parametrized very similiar to your vehicle on your computer using Simulink. The second (`example_vehicle/tests/controller_dev_py/controller_dev_py.slx`), is a basis for creating the model with the hardware interfaces of your ECU to generate code. It contains all necessary interfaces which have to be connected according to your vehicle configuration. Details on this process can be found in the section below.

The workflow to adapt the software to your vehicle has three steps:

1. Adjust vehicle parameters for simulation and control software
2. Tune controllers to fit your needs in the simulation
3. Add vehicle specific interfaces to the target software

General advice
=============================
The main software interfaces are the trajectory planner and the vehicle actuators. In the Roborace project we based the former on an Ethernet link, the latter is using a Controller Area Network (CAN) interface. The hardware implementation of those interfaces may change depending on your prototype. However, all signals are expected to come at a fast update rate (~100Hz) for all low-level sensors and a moderate update rate (~10-20Hz) for localization sources. Slower sample rates could cause performance degradation. Trajectory updates are required also at an moderate update rate (~10-20Hz). Another point to be stressed is that the controller design requires high-quality derivative information for the target trajectory. It performs safety checks on those signals and they are used directly in the controller. Noisy derivatives can significantly degrade performance and even affect vehicle stability. Note that the controller requires two distinct trajectories, the performance trajectory (standard) as well as the emergency trajectory. The latter is required to always come to a full-stop and provide a backup in case of network failure or a fault in the planning software.

Depending on the vehicle configuration, the actuator interface might differ significantly. As the controller requests a longitudinal force, it is required to map that to the corresponding actuator requests (e.g. combustion engine, electric motor(s) or hydraulic brakes) to achieve the requested value. Please note that the software expects the request to be conducted without delay. Especially for combustion engines, this might not be true and require additional modifications. Experience has shown that the lateral control performance depends heavily on the steering actuator response. Again, the controller does not compensate for response delay or low-pass characteristics. We have seen good performance when the requested steering angle is tracked with a low-pass time-constant of roughly 50ms. In case your hardware does not match these requirements, modifications in the lateral controller design might be necessary.

From a sensors perspective, the minimum requirement is one IMU sensor (providing at least longitudinal and lateral acceleration and yaw rate) and one localization measurement. Furthermore, using wheel speed sensors is strongly recommended, even though they are mainly used for anti-wheel-lock control. Adding a velocity sensor is optional, as the required information can be estimated based upon localization measurements and IMU signals. The software provides inputs for two localization measurements, two velocity measurements and two IMU measurements. Automatic failure detection is performed for the velocity measurements, however, online reconfiguration for all six sensors is possible (maintaining the minimum configuration mentioned above). This allows you to feed additional diagnostics and quality information into the sensor fusion.

General notes on the coordinate systems used and the sign conventions of the signals can be found under :doc:`../software/system`. 

Adjust vehicle parameters
=============================
The vehicle parameters are located in two different sources. The reason for this is that it should be possible to simulate parameter mismatch between the model and the software by applying different vehicle parameter for the simulation model than in the software.

The control software vehicle parameters must be adjusted in `modules/mod_vehicle_dynamics_control/parameters/vehicle/pa_vehicleparameter.sldd`. A description of the parameters can be found under :doc:`../software/vehicleparam`.

The simulation vehicle parameters must be adjusted in `simulation/sim_vehicle_dynamics/parameters/pa_sim_vehicleparameter.sldd`. A description of the parameters can be found in the related documentation.

Tune controllers
=============================
There are three components which definitely require tuning to your vehicle. You can find the detailed tuning guides in the documentation of the subcomponents:

* :doc:`../software/estimation`
* :doc:`../software/control/control`
* :doc:`../software/system`

In general, we recommend to use our graphical debug tool to adjust the parameters of the algorithms. Please see the *Working with the Software Stack* section for an introduction.

Setup vehicle interfaces (Receive)
====================================
Open the vehicle ECU model `modules/mod_vehicle_dynamics_control/example_vehicle/models/VehicleECU.slx`. This is your main model which can be used to flash your ECU. It is already partitioned in a *Receive*, *Software* and *Send* part. You have already done the necessary changes to *Software* if you have adjusted all controllers as described above. We will work through the other sections step by step.

The most important thing comes first: The **TrajectoryPlanning** Interface. This is the core of your communication with the controller. It must be filled with trajectories which are then realized by the controller. They can be send asynchronously, however if the vehicle reaches the end of the current trajectory it goes to a hard emergency brake. The frequency to provide new trajectories is therefore determined by the length and speed profile of the trajectories. Each trajectory *must* have 50 discretization points, due to limitations in code generation for variable sized signals. However, the spatial spacing between the points is arbitrary and can also be variable within a single trajectory.

In general, there are two possibilities: Either you supply a trajectory with all derivative information (acceleration and curvature) or you have to configure the controller to do that (see controller tuning section). In this case, you can provide zeros to these signals.

The software distinguishes between a *Performance Trajectory* and a *Emergency Trajectory*. Both have to be provided by the planner. Under normal circumstances, the *Performance Trajectory* is driven. The *Emergency Trajectory* is used for soft emergency cases, where the vehicle brakes at fast as possible but path tracking is still enabled. This can be used for system failures not directly related to steering or powertrain, e.g. Battery or Temperature Problems, or just to stop the vehicle before it reaches the mission finished command. A detailed descprition of the signals provided for the trajectories, can be found in :doc:`../software/system`.

It is necessary to signalize the controller that the trajectory planner and mission planner are active. This is done via setting `Trajectories_CommsStatus` and `Strategy_CommsStatus` to `TUMHealthStatus.OK`. In case they loose communication, they should be set to `TUMHealthStatus.ERROR`. This triggers a soft emergency brake. Furthermore, the mission planner has to signal to the controller that it wants to start the mission via setting `Strategy_Status` to `TUMStrategyState.S_DRIVING_NORMAL`. Again, its possible to trigger an emergency brake via setting the status to `TUMStrategyState.S_DRIVING_EMERGENCY`. All these signals *must* be included in your architecture for the controller to function properly. A reference implementation can be found in the simulation model.

The **VehicleSensorData** bus is the second crucial interface to the controller. A description of what signals are expected can be found in :doc:`../software/system`. An example for how to run the software can be found in the implementation of the simulation in `modules/mod_vehicle_dynamics_control/example_vehicle/tests/controller_dev/controller_dev.slx`. It is possible to use the controller with less signals than this, however it is recommended to calculate 'replacement' signals, e.g. the longitudinal velocity based on the wheelspeeds and the lateral velocity from a steady-state assumption.

The **VehicleSystemStatus** controls the startup process of the vehicle. Set the `Hardware` signal to `TUMHealthStatus.OK` if all devices are powered up and the system is ready to be used. Set `AIDriver` to the same value if all startup conditions from the Autonomous System are met, e.g. localization available or data logging activated. `ParkBrakeApplied_b` gives feedback to the controller whether a low-level park brake system is still applied. If this is not available, set it to false. Finally, `VehicleReady2Drive` indicates that the vehicle has given control to the control unit. This means that the controller expects steering and powertrain to respond immediatly to commands. This signal is used as a condition to go to internal driving state.

Finally, **ActuatorLimiations** tells the controller what are the current maximum and minimum control values for steering angle at the wheels and the overall powertrain force. These can also be set dynamically and prevent the controller from requesting values which cannot be used. This helps to prevent wind-up of the control outputs.

Setup vehicle interfaces (Send)
====================================
The send subsystem shows a possible implementation of the output processing. `EnableControlOutputs` and `EnableHardEmergencyBrake` provide additional safety mechanisms, to ensure the vehicle never starts driving before the state machine allows and applies the brakes in case that a hard emergency brake is requested. The negative force value for the hard emergency brake should be adjusted to your vehicle. It is recommended to have a value corresponding to -0.6 to -1g. The signal `RequestControl` can be used to signalize a lower level control unit that the motion controller wants to take control over the actuators. Usually, the lower level control unit responds to this request with the `VehicleReady2Drive` signal mentioned earlier in the VehicleSystemStatus.

The controller requests a steering angle at the wheel via `RequestSteeringAngle_rad`. It is assumed that this is the mean of both front wheels. This signal should be send to the steering actuator. The powertrain force is requested via `RequestLongForce_N`. It is assumed to be the sum of the forces acting upon the vehicle in longitudinal direction. These requests must be split according to the motor and brake configuration and can vary between several vehicles. Furthermore, it is possible to utilize a park brake via `ParkBrakeActive`. The controller state machine handles release and locking properly, to be able to start the vehicle on inclined roads.

Finally, it is recommended to use the `Debug` bus provided by the software to inspect whats going on. The simulation model is already configured to log these signals and the description how to convert them to a usable format is given in the section *Working with the Software Stack*. If you want to use the same data analysis workflow on your vehicle, you can implement a similar conversion script for your ECU logs.

Setup vehicle interfaces (Config)
====================================
Simulink requires to specify the Target Hardware in the modelconfig. The project uses a quite complex structure to maintain these modelconfigs. This emerged from the need to support multiple targets with the same code and very different configs. The configs are located in `modules/mod_vehicle_dynamics_control/interfaces/modelconfig.sldd`. The standard config used for the passenger vehicle repository is called `GRT`. Adjust the configuration according to your needs, e.g. Target Hardware and save everything. It is required to reload the project to apply the changes to all models.
