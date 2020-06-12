# Package overview
The *control* package collects all functionality related to controllers and directly related pre- or postprocessing methods. The core components are the lateral tracking controller (high level control) and the curvature controller and the velocity controller (low level controller). The latter are mainly responsible of abstracting the vehicle dynamics influence to the high level controller. The lateral tracking controller itself can therefore be designed on very basic model assumptions.

Contact person: [Alexander Wischnewski](mailto:alexander.wischnewski@tum.de)

# Models
* `mvdc_trajectory_driver.slx` is the main model of this package and is therefore a good starting point to dive deeper into the package. It combines the different subcomponents to a single function used by the software.
* `mvdc_curvvel_tracking.slx` collects the curvature and velocity control algorithms. Both utilize proportional feedback and a disturbance observer to add integral effect (optional) to control the low level vehicle dynamics. Furthermore, handling of stillstand situations (controller settings).
* `mvdc_path_feedback.slx` is the standard lateral feedback controller. It is based on a point-mass assumption and does not alter the speed profile. It does not respect the physical limits of the vehicle directly.
* `mvdc_path_matching.slx` handles the path interface to the trajectory planner and provides the vehicle position in path coordinates (lateral distance and heading angle).

# Source
* `calcPathAx.m` calculates the acceleration values corresponding to the given velocity profile for the feedforward control as an alternative to the provided one.
* `calcPathHeading.m` calculates the path heading values corresponding to the given points for the feedforward control as an alternative to the provided one.
* `calcPathCurvature.m` calculates the acceleration values corresponding to the given path for the feedforward control as an alternative to the provided one.
* `exactlin_feedback.m` implements the nonlinear exact linearization based trajectory tracking controller.
* `interp1_angle.m` a custom interpolation function which applies correct arithmetics for angular interpolation between -pi and +pi
* `local_path_matching.m` calculates the vehicle position in path coordinates and some other related useful values
* `learnSteeringCharacteristic.m` adapts the under-/oversteering characteristic based on previous data.

# Basic setup
In the following, the basic tuning process is described for all algorithms in this component. Take care that some data dictionaries have vehicle specific version. This is always named e.g. `db_xxxxxxxx.sldd`. If you change a parameter, you have to do it in the vehicle specific version. This is configured before simulation or building the model automatically by the vehicle project.

#### Path Matching: `xx_mvdc_path_matching.sldd`
The most important parameter in this algorithm is the handling of safety checks related to relative path position. `P_VDC_ActivateHighPathDeviationDetection_b` activates these checks. `P_VDC_LateralPathDeviationMax_m` describes the maximum allowed lateral path deviation and `P_VDC_HeadingPathDeviationMax_rad` the maximum allowed heading deviation. These apply during startup as well as during driving. If these criteria are not met, the vehicle either does not start driving or switches to emergency mode. They should be adjusted according to the safety distances in the planning algorithm and the track properties.

If only cartesian coordinates and a velocity profile is provided to the controller, `P_VDC_LocalFFCalculationActive_b` must be set to true. This triggers local calculation of the required feedforward information about the path based on numeric derivatives.

#### Path Feedback: `xx_mvdc_path_feedback`
This algorithm has several parameters which have to be set in a vehicle specific way and fine tuned in real world driving.

`P_VDC_MinSpeedGainCorrection_mps` specifies minimum speed for which the velocity gain correction is applied. For velocities above this value, the lateral error dynamics are kept constant independently from the vehicle speed. This leads to a gain reduction for higher speeds. This value should be set to approx. 20-30% of the vehicles maximum driving speed. If you experience problems with steering oscillations at low speeds, this might be increased.

`P_VDC_UseBetaFeedback` activates the usage of the estimated side slip angle for calculation of the lateral error derivatives. If deactivated, a basic estimate based on steering angle and wheel base is used. In general, this should be activated as long as the side slip estimate is of descent quality. This significantly increases tracking quality. It is also possible to turn of the usage of the side slip angle for feedback control purposes completly via `P_VDC_UseBetaFeedback`.

`P_VDC_LatConvergence` and `P_VDC_LatDamping` are the main controller tuning parameters. They specify the closed loop eigenfrequency and the damping factor. For an initial setup, it is recommend to choose the damping between 0.7 and 1 and the convergence factor between 1 and 1.5. These values are valid for most vehicles. While tuning them on the real vehicle, the damping is usually left untouched. Increase the convergence value until the vehicle shows noticable oscillations. You have found the upper limit. We recommend to choose approximately 80-90% of this value as final setup. Lower values might decrease tracking quality as disturbances are not sufficiently rejected.

`P_VDC_LongFF_PT1_s` specifies the time constant for the low pass filter applied to the longitudinal feed forward value. If you experience stuttering behavior, increase this value.

#### Curvature and Velocity Tracking: `xx_mvdc_curvvel_tracking`
In general, the low level controllers switch between a *slow* and a *fast* mode. The vehicle enters fast mode, if the speed is above `P_VDC_FullControlSpeed_mps` and falls back to slow mode if the speed is below `P_VDC_SlowControlSpeed_mps`. In general, this mechanism is intended to prevent misbehavior in standstill and very slow driving. All normal operation points of the vehicle should be in fast mode.

During startup, after the parkbarke is released, the vehicle applies the brakes to prevent moving before the controller requests so. The force used for this can be specified via `P_VDC_NegativeFxStandstill_N`.

The velocity controller consists of three main parts. First, the feedforward based on the acceleration values specified for the trajectory. Second, the proportional feedback calculated from the velocity tracking error. Third, the mismatch between the expected acceleration based on the applied force and the actual acceleration is used to calculate an estimate of the disturbances acting on the vehicle. This estimate is fed back to the controller and used for disturbance compensation. Furthermore, the information about vehicle drag and driving resistances are used to calculate a feedforward value for the longitudinal disturbances. This can be deactivated via `P_VDC_VelDistFFActive`. The online estimation of the longitudinal disturbances can be deactivated via `P_VDC_VelDistFBActive`. It is recommoned to activate both and set the vehicle parameters to proper values.
The proportional feedback gain can be adjusted via `P_VDC_LongKp`. A value similar to half of the vehicle mass is a good starting point for tuning. `P_VDC_VelDistEst_PT1_s` setups the response behavior of the disturbance estimation. Increase this value if you have noisy sensors and experience problems with oscillations, otherwise a value of one second is a reasonable starting point. `P_VDC_FxMaxDistComp_N`, `P_VDC_FxMaxDistFF_N`, `P_VDC_FxMaxFB_N`, `P_VDC_FxMaxFF_N` and the corresponding minimum values have to be adjusted to the mass of the vehicle. In general, all should be around 2-3 times the vehicle mass despite the pure feedforward value `P_VDC_FxMaxFF_N`. Have a look in the model to get more details on how to choose these parameters.

The curvature controller has a similar structure to the velocity controller: Propotional feedback, disturbance compensation and a feedforward control based on the neutral steering assumption. However, you might deactivate the disturbance compensation via `P_VDC_CurvDistActive` to get a stiffer steering control in case you have a well calibrated steering and the vehicle drives sufficiently straight for zero steering angle request. The disturbance estimation response time can be adjusted via `P_VDC_CurvDistEst_PT1_s`, one second is a good starting point. The proportional feedback can be tuned via `P_VDC_CurvKp`. It is recommended to have rather small contribution from the proportional part in the curvature controller, as its benefit depends heavily on the quality of the curvature estimate. The control limits can be adjusted via `P_VDC_LatDeltaMaxDistComp_rad`, `P_VDC_LatDeltaMaxFB_rad` and `P_VDC_LatDeltaMaxFF_rad`.

The curvature controller can learn the self-steering characteristic of the vehicle via time. This information is then used to improve the feedforward control law. It can be activated via `P_VDC_NonlinCurvDeltaActive_b`. Be aware, this algorithm is computational intense and should only be enabled if enough resources are available on the ECU. It is only recommended for advanced users. Please refer to the code for all setup parameters which have to be adjusted.
