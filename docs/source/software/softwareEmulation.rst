========================
Software Emulation
========================
The *softwareEmulation* package implements several additional models which are necessary to simulate the vehicle software. This includes mainly replacements of the trajectory planning.

Contact person: `alexander.wischnewski@tum.de <alexander.wischnewski@tum.de>`_

Models
===========================
* `TrajectoryPlanningEmulation.slx` implements a basic emulation of the trajectory generation and the interface to the control software.
* `TrajectoryScalingEmulation.slx` implements a basic emulation velocity and speed scaling applied by the trajectoryplanner and the interface to the control software.
* `LidarLocalizationEmulation.slx` implements a basic emulation of the LIDAR localization and the interface to the control software.
* `trajectoryReplay.slx` allows to simulate an opponent following a prescribed trajectory

Most important parameters
===========================
In the following, there is a list of the most important parameters for tuning of the module. They are sorted by the corresponding data dictionary. Take care that some data dictionaries have another, vehicle specific version. This is always named e.g. `db_mvdc_state_estimation.sldd`. If you change a parameter, you have to do it in the vehicle specific version. In standard configuration, the `db_xxxxxxxx` data dictionaries are used which hold the DevBot parameters.
