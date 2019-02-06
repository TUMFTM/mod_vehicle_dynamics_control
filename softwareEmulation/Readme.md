# Package overview
The *softwareEmulation* package implements several additional models which are necessary to simulate the vehicle software. This includes mainly replacements of the trajectory planning.

Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)

# Models
* `TrajectoryPlanningEmulation.slx` implements a basic emulation of the trajectory generation and the interface to the control software.
* `TrajectoryScalingEmulation.slx` implements a basic emulation velocity and speed scaling applied by the trajectoryplanner and the interface to the control software.
* `LidarLocalizationEmulation.slx` implements a basic emulation of the LIDAR localization and the interface to the control software.
* `ControllerLearning.slx` implements a basic emulation of the controller learning component and the interface to the control software.

# Most important parameters
In the following, there is a list of the most important parameters for tuning of the module. They are sorted by the corresponding data dictionary. Take care that some data dictionaries have another, vehicle specific version. This is always named e.g. `db_xxxxxxxx.sldd`. If you change a parameter, you have to do it in the vehicle specific version. 
