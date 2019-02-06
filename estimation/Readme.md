# Package overview
The *estimation* package collects all functionality related to sensor fusion and state estimation. The core components are different implementations of a Kalman Filter (KF) which is able to fuse pose, velocity and acceleration measurements to achieve a less noisy and more precise localization and state estimation. Furthermore, different pre- and postprocessing components are added to provide interfaces to other software parts and add diagnosis functionality and configuration possibilities.

Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)

# Models
* `mvdc_state_estimation.slx` collects all functionalty related to the sensor fusion. This includes the chosen Kalman Filter implementation and adds diagnosis and other related functionality not specifically related to the fusion algorithm. This model is the main model of this module and a good starting point to start working with the code.
* `mvdc_KF_PointMassJoint.slx` is an implementation of an Extended Kalman Filter (EKF) based on a point-mass model using acceleration measurements as system inputs and position and velocity measurements as correction signals.
* `mvdc_KF_PointMassSep.slx` is an implementation of an Extended Kalman Filter (EKF) based on a point-mass model. In contrast to `mvdc_KF_PointMassJoint.slx`, the vehicle dynamic and the pose states are estimated by a cascaded filter.
* `mvdc_KF_NonlinSingleTrack.slx` is an implementation of an Unscented Kalman Filter (UKF) based on a nonlinear single track model. Position and velocity measurements are used for correction.
* `mloc_estimation_postprocessing.slx` provides several smaller functionalities to include the state estimation into the whole software architecture. This includes mainly calculation of a vehicle pose purely based on odometry and preparing all information of the state estimation used by LIDAR localization.

Take care: `mvdc_KF_PointMassJoint.slx`, `mvdc_KF_PointMassSep.slx` and `mvdc_KF_NonlinSingleTrack.slx` are variants of a state estimator with the same interfaces and can be used interchangeably. Further, they also share the same data dictionary `mvdc_KF.sldd`.

# Source
* `ekf_prediction_numeric.m` implements the KF prediction step using an arbitrary function handle as system update equation. The linearization is calculated numerically based on the complex step differentiation trick.
* `ekf_prediction_analytic.m` implements the KF prediction step using an arbitrary function handle as system update equation. The linearization is given explicitly to the function.
* `ekf_update.m` implements the update step of the Kalman Filter using given measurement matrices (only linear measurements possible)
* `jaccsd_input.m` calculates the jacobian of a nonlinear system update function using complex step differentiation with respect to the system inputs
* `jaccsd_state.m` calculates the jacobian of a nonlinear system update function using complex step differentiation with respect to the system states
* `kf_joint.m` implements the full KF algorithm for the point mass model within a single filter (used by `mvdc_KF_PointMassJoint.slx`)
* `kf_pose.m` implements the pose states part of the cascaded KF (used by `mvdc_KF_PointMassSep.slx`)
* `kf_speed.m` implements the speed states part of the cascaded KF (used by `mvdc_KF_PointMassSep.slx`)
* `kf_stm.m` implements the UKF based on a nonlinear single track model (used by `mvdc_KF_NonlinSingleTrack.slx`)
* `integrateOdometry.m` implements the calculation of the vehicle pose based on odometry signals using a forward euler integration scheme
* `transCov2TrackOrientation.m` rotates the measurement covariance matrix according to the track orientation to force the KF to weight LIDAR and GPS differently based on the actual track orientation

# Basic setup
In the following, the basic tuning process is described for all algorithms in this component. Take care that some data dictionaries have vehicle specific version. This is always named e.g. `db_xxxxxxxx.sldd`. If you change a parameter, you have to do it in the vehicle specific version. This is configured before simulation or building the model automatically by the vehicle project.

#### State Estimation General Parameters `xx_mvdc_state_estimation.sldd`
The sensor fusion can be deactivated via `P_VDC_deactivateEKF`. This will directly feedthrough the sensor values to the controller and other algorithms.

`P_VDC_LocalizationMode` specifies the sensors used for localization. The setup depends on your vehicle and sensors.

| Mode | Sensors |
| ---- | ------- |
| 0    | GPS     |
| 1    | GPS + LIDAR |
| 2    | LIDAR   |

`KalmanFilterMode` specifies which Kalman Filter is used to fuse the sensors. The recommended choice is to use the joint filter, mode 0. This mode is most robust to sensor failure, easy to tune and delivers reasonable good results.

| Mode | KF |
| ---- | ------- |
| 0    | mvdc_KF_PointMassJoint.slx     |
| 1    | mvdc_KF_PointMassSep.slx |
| 2    | mvdc_KF_NonlinSingleTrack.slx   |

#### Kalman Filter Parameters `xx_mvdc_KF.sldd`
The parameters in this data dictionary are used by all Kalman Filter implementations and can therefore be used universally. In the following, the process for the setup of the point mass joint filter is described. Usage of other options is considered to be done by advanced users only. Please refer to the code for more details on the parameters in this case.

`P_VDC_Var_yawRate_radps`, `P_VDC_Var_ax_mps2`, `P_VDC_Var_ax_mps2` should be set to the variances of the corresponding sensors. The variance should be taken from a reasonable driving situation and not in standstill! The same holds for `P_VDC_measCov_velocity` which is a vector with the variance of the longitudinal and lateral velocity measurements.

The fusion of the position measurements can be controlled via the covariance vectors `P_VDC_measCov_GPS` and `P_VDC_measCov_VLOC` for the LIDAR. Each holds the diagonal entries of the covariance matrix. The meaning of the value depends on the value of `P_VDC_EnableTrackDepCov`. If this is set to one, the covariance matrices for each of the measurements are adjusted according to the vehicle orientation on the track. The vectors therefore hold the longitdinal value, the lateral value and the orientation value (in this order). If `P_VDC_EnableTrackDepCov` is set to zero, the vectors hold the x value, the y value and the orientation value (in this order).

In general, it is recommended to set the yaw rate, acceleration and velocity measurement covariances to their true values. The fine tuning can then be done via the position measurement covariances. To do this, take a look at the residuals for the position measurements. If they are significantly biased for the whole dataset, you should check your sensor setup first. If the values persist to be biased, you should decrease the variance corresponding to this value until the residual starts to get noisy. In general, small covariances lead to the state estimate being closer to the sensor value, while large covariances lead to more trust in the model and its forward integration capabilities. 

#### Postprocessing Parameters `xx_mloc_estimation_postprocessing.sldd`
Here it is possible to choose between different modes for curvature estimation with `P_VDC_CurvatureEstimationMode`, however it is recommend to stay with mode 1 as this is the most robust. `P_VDC_CurvatureEstimateMax_radpm` has to be adjusted to your vehicle.

Furthermore, there are different possiblities to calculate the odometry (only used by LIDAR localization) chosen via `P_MLOC_OdometryMode`. If there are velocity measurements for longitudinal and lateral direction are available, it is recommended to use mode 3 (raw velocity sensors). Even if these are not available directly, one can often calculate replacements from e.g. wheelspeed sensors. If you choose mode 1, the current state estimate is taken to calculate the odometry. However, this makes tuning of the LIDAR localization more difficult as it is coupled now with state estimation performance.
