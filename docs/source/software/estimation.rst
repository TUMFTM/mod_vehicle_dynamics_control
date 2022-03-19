========================
Estimation module
========================
The *estimation* package collects all functionality related to sensor fusion and state estimation. The core components is an implementation of an Extended Kalman Filter (EKF) which is able to fuse pose, velocity and acceleration measurements to achieve less noisy and more precise localization and state estimation. Furthermore, different pre- and postprocessing components are added to provide interfaces to other software parts and add diagnosis functionality and configuration possibilities.

Contact person: `alexander.wischnewski@tum.de <alexander.wischnewski@tum.de>`_

Models
====================================
* `mvdc_state_estimation.slx` collects all functionalty related to the sensor fusion. This includes the chosen Kalman Filter implementation and adds diagnosis and other related functionality not specifically related to the fusion algorithm. This model is the main model of this module and a good starting point to start working with the code.
* `mvdc_KF_PointMassJoint.slx` is an implementation of an Extended Kalman Filter (EKF) based on a point-mass model using acceleration measurements as system inputs and position and velocity measurements as correction signals.
* `mloc_estimation_preprocessing.slx` provides several smaller functionalities to include the state estimation into the whole software architecture. This includes mainly calculation of a vehicle pose purely based on odometry and preparing odometry information for other components. Furthermore, it applies fault diagnosis to the velocity sensors. This allows to disable them for fusion in case they are delivering faulty values.

Source
====================================
* `ekf_prediction_analytic.m` implements the KF prediction step using an arbitrary function handle as system update equation. The linearization is given explicitly to the function.
* `ekf_update.m` implements the update step of the Kalman Filter using given measurement matrices (only linear measurements possible)
* `kf_joint.m` implements the KF algorithm for the point mass model (used by `mvdc_KF_PointMassJoint.slx`)
* `integrateOdometry.m` implements the calculation of the vehicle pose based on odometry signals using a forward euler integration scheme

General concept
====================================
The main focus during the development of this component was to achieve a robust and reliable localization and vehicle dynamics state estimation. Therefore, the Kalman Filter uses a physics model purely derived from first principles which prevents the introduction of model bias. An outlier rejection algorithm limits the residual signals used for measurement feedback. In addition, it applies fault diagnosis and dynamic reconfiguation of the fusion process for the two velocity measurements.

The sensor fusion has access to all sensor data to enable dynamic reconfiguration of the filter. It can use up to two localization methods, two velocity measurements (longitudinal and lateral) and two IMUs. They can be found in the `VehicleSensorData` bus, named `Loc`, `Vel` and `IMU` and the corresponding sensor ID.

Scientific publication
========================
Details on the presented sensor fusion algorithm can be found in the following publication:

.. code-block:: none

  @article{Wischnewski2019,
    author = {Alexander Wischnewski and Tim Stahl and Johannes Betz and Boris Lohmann},
    doi = {10.1016/j.ifacol.2019.08.064},
    url = {https://doi.org/10.1016/j.ifacol.2019.08.064},
    year = {2019},
    month = sep,
    title = {Vehicle Dynamics State Estimation and Localization for High Performance Race Cars},
    journal = {IFAC-PapersOnLine}
  }

Basic setup
====================================
In the following, the basic tuning process is described for all algorithms in this component. Take care that some data dictionaries have a vehicle specific version. This is always named e.g. `db_xxxxxxxx.sldd`. If you change a parameter, you have to do it in the vehicle specific version. This is configured before simulation or building the model automatically by the vehicle project.

State Estimation General Parameters `xx_mvdc_state_estimation.sldd`
-----------------------------------------------------------------------
The sensor fusion is activated via `P_VDC_ActivateSensorFusion`. In case of deactivation, the sensor signals from source one (localization 1, velocities 1 and IMU 1) are directly fed to the controller and other algorithms.

Here it is possible to choose between different modes for curvature estimation with `P_VDC_CurvatureEstimationMode`, however it is recommend to stay with mode 1 as this is the most robust. `P_VDC_CurvatureEstimateMax_radpm` has to be adjusted to your vehicle.

Kalman Filter Parameters `xx_mvdc_KF_PointMassJoint.sldd`
-----------------------------------------------------------------------
This is the main tuning area for the Kalman Filter. The vector `P_VDC_InputCov` specifies the covariance of the input signals. The vector `P_VDC_MeasCov` specifies the measurement covariances. See `mvdc_KF_PointMassJoint.slx/CreateKFVectors` for sensor matching of the variances. Those vectors are used to create diagonal covariance matrices. An exception are the position variances for the localization sensors. They can either be used in normal mode or utilize an adaptive covariance based on the vehicle heading. This enables to weight sensors differently in longitudinal and lateral direction. The source of this heading can be set via `P_VDC_EnableTrackDepCov`.

In general, it is recommended to set the yaw rate, acceleration and velocity measurement covariances to their true values. The fine tuning can then be done via the measurement covariances. To do this, take a look at the residuals for the position measurements. If they are significantly biased for the whole dataset, you should check your sensor setup first. If the values persist to be biased, you should decrease the variance corresponding to this value until the residual starts to get noisy. In general, small covariances lead to the state estimate being closer to the sensor value, while large covariances lead to more trust in the model and its forward integration capabilities.

Preprocessing Parameters `xx_mloc_estimation_preprocessing.sldd`
-----------------------------------------------------------------------
The sensors used for state estimation can be activated via `P_VDC_EnableSensorsForFusion`. They are specified in the order Loc1, Loc2, Vel1, Vel2, IMU1 and IMU2.

The online diagnosis of sensors faults is enabled via `P_VDC_ActivateOnlineDiagnosis`.
