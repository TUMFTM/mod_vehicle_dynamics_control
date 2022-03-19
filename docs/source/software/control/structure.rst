========================
Module structure
========================

Models
====================================
* `mvdc_trajectory_driver.slx` is the main model of this package and is therefore a good starting point to dive deeper into the package. It combines the different subcomponents to a single function used by the software.
* `mvdc_curvvel_tracking.slx` collects the curvature and velocity control algorithms of the basic controller. Both utilize proportional feedback and a disturbance observer to add integral effect (optional) to control the low level vehicle dynamics.
* `mvdc_path_feedback.slx` is the lateral feedback controller of the basic controller. It is based on a point-mass assumption and does not alter the speed profile. It does not respect the physical limits of the vehicle directly.
* `mvdc_mpc.slx` is the main model for the Tube-MPC controller. It prepares the linearization for the OSQP solver, handles the interface to the OSQP solver and generates the control outputs.
* `mvdc_nmpc_acados.slx` is the main model for the nonlinear MPC controller based on the embedded optimization library acados. It provides a controller based on a kinematic bicycle model as well as a nonlinear Pacejka tire model.
* `mvdc_path_matching.slx` handles the path interface to the trajectory planner and provides the vehicle position in path coordinates (lateral distance and heading angle). This is used for all controllers.

Source
====================================
* `calcPathAx.m` calculates the acceleration values corresponding to the given velocity profile for the feedforward control as an alternative to the provided one.
* `calcPathHeading.m` calculates the path heading values corresponding to the given points for the feedforward control as an alternative to the provided one.
* `calcPathCurvature.m` calculates the acceleration values corresponding to the given path for the feedforward control as an alternative to the provided one.
* `interp1_angle.m` a custom interpolation function which applies correct arithmetics for angular interpolation between -pi and +pi
* `local_path_matching.m` calculates the vehicle position in path coordinates and some other related useful values
* `learnSteeringCharacteristic.m` adapts the under-/oversteering characteristic based on previous data.
