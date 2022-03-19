========================
Basic Control
========================

Concept
========================
The control concept is visualized in the Figure below. It is based upon a split between a high-level lateral path tracking controller (depicted in green) and a low-level controller (depicted in blue) accounting for the details of the vehicle dynamics. Details on the path matching process can be found in :doc:`PathMatching`.

.. image:: BasicControl_architecture.png
  :width: 800
  :alt: Basic Control architecture diagram

One of the key aspects of this control concept is its independence of the tire parameters. The lateral tracking controller outputs a target curvature and is therefore independent of the vehicle dynamic details. The velocity controller only requires the vehicle mass to function properly. The situation is a little different for the curvature controller: It uses the wheelbase to calculate an appropriate steering wheel angle based on the requested curvature but this is accompanied with a learning component to gather information about the under- and oversteering behavior of the vehicle.

Tuning
========================

In the following, the basic tuning process is described for all algorithms in this component. Take care that some data dictionaries have vehicle specific version. This is always named e.g. `db_xxxxxxxx.sldd`. If you change a parameter, you have to do it in the vehicle specific version. This is configured before simulation or building the model automatically by the vehicle project.

Path Feedback: `xx_mvdc_path_feedback`:
---------------------------------------------
This algorithm has several parameters which have to be set in a vehicle specific way and fine tuned in real world driving.

`P_VDC_MinSpeedGainCorrection_mps` specifies minimum speed for which the velocity gain correction is applied. For velocities above this value, the lateral error dynamics are kept constant independently from the vehicle speed. This leads to a gain reduction for higher speeds. This value should be set to approx. 20-30% of the vehicles maximum driving speed. If you experience problems with steering oscillations at low speeds, this might be increased.

`P_VDC_UseBetaFeedback` activates the usage of the estimated side slip angle for calculation of the lateral error derivatives. If deactivated, a basic estimate based on steering angle and wheel base is used. In general, this should be activated as long as the side slip estimate is of descent quality. This significantly increases tracking quality. It is also possible to turn of the usage of the side slip angle for feedback control purposes completly via `P_VDC_UseBetaFeedback`.

`P_VDC_LatConvergence` and `P_VDC_LatDamping` are the main controller tuning parameters. They specify the closed loop eigenfrequency and the damping factor. For an initial setup, it is recommend to choose the damping between 0.7 and 1 and the convergence factor between 1 and 1.5. These values are valid for most vehicles. While tuning them on the real vehicle, the damping is usually left untouched. Increase the convergence value until the vehicle shows noticable oscillations. You have found the upper limit. We recommend to choose approximately 80-90% of this value as final setup. Lower values might decrease tracking quality as disturbances are not sufficiently rejected.

`P_VDC_LongFF_PT1_s` specifies the time constant for the low pass filter applied to the longitudinal feed forward value. If you experience stuttering behavior, increase this value.

Curvature and Velocity Tracking: `xx_mvdc_curvvel_tracking`:
---------------------------------------------------------------
In general, the low level controllers switch between a *slow* and a *fast* mode. The vehicle enters fast mode, if the speed is above `P_VDC_FullControlSpeed_mps` and falls back to slow mode if the speed is below `P_VDC_SlowControlSpeed_mps`. In general, this mechanism is intended to prevent misbehavior in standstill and very slow driving. All normal operation points of the vehicle should be in fast mode.

During startup, after the parkbarke is released, the vehicle applies the brakes to prevent moving before the controller requests so. The force used for this can be specified via `P_VDC_NegativeFxStandstill_N`.

The velocity controller consists of three main parts. First, the feedforward based on the acceleration values specified for the trajectory. Second, the proportional feedback calculated from the velocity tracking error. Third, the mismatch between the expected acceleration based on the applied force and the actual acceleration is used to calculate an estimate of the disturbances acting on the vehicle. This estimate is fed back to the controller and used for disturbance compensation. Furthermore, the information about vehicle drag and driving resistances are used to calculate a feedforward value for the longitudinal disturbances. This can be deactivated via `P_VDC_VelDistFFActive`. The online estimation of the longitudinal disturbances can be deactivated via `P_VDC_VelDistFBActive`. It is recommoned to activate both and set the vehicle parameters to proper values.
The proportional feedback gain can be adjusted via `P_VDC_LongKp`. A value similar to half of the vehicle mass is a good starting point for tuning. `P_VDC_VelDistEst_PT1_s` setups the response behavior of the disturbance estimation. Increase this value if you have noisy sensors and experience problems with oscillations, otherwise a value of one second is a reasonable starting point. `P_VDC_FxMaxDistComp_N`, `P_VDC_FxMaxDistFF_N`, `P_VDC_FxMaxFB_N`, `P_VDC_FxMaxFF_N` and the corresponding minimum values have to be adjusted to the mass of the vehicle. In general, all should be around 2-3 times the vehicle mass despite the pure feedforward value `P_VDC_FxMaxFF_N`. Have a look in the model to get more details on how to choose these parameters.

The curvature controller has a similar structure to the velocity controller: Propotional feedback, disturbance compensation and a feedforward control based on the neutral steering assumption. However, you might deactivate the disturbance compensation via `P_VDC_CurvDistActive` to get a stiffer steering control in case you have a well calibrated steering and the vehicle drives sufficiently straight for zero steering angle request. The disturbance estimation response time can be adjusted via `P_VDC_CurvDistEst_PT1_s`, one second is a good starting point. The proportional feedback can be tuned via `P_VDC_CurvKp`. It is recommended to have rather small contribution from the proportional part in the curvature controller, as its benefit depends heavily on the quality of the curvature estimate. The control limits can be adjusted via `P_VDC_LatDeltaMaxDistComp_rad`, `P_VDC_LatDeltaMaxFB_rad` and `P_VDC_LatDeltaMaxFF_rad`.

The curvature controller can learn the self-steering characteristic of the vehicle via time. This information is then used to improve the feedforward control law. It can be activated via `P_VDC_NonlinCurvDeltaActive_b`. Be aware, this algorithm is computational intense and should only be enabled if enough resources are available on the ECU. It is only recommended for advanced users. Please refer to the code for all setup parameters which have to be adjusted.


Scientific publications
========================
Details on the presented control algorithms can be found in the following publications:

.. code-block:: none

  @article{Heilmeier2019,
    author = {Alexander Heilmeier and Alexander Wischnewski and Leonhard Hermansdorfer and Johannes Betz and Markus Lienkamp and Boris Lohmann},
    doi = {10.1080/00423114.2019.1631455},
    url = {https://doi.org/10.1080/00423114.2019.1631455},
    year = {2019},
    month = jun,
    publisher = {Informa {UK} Limited},
    pages = {1--31},
    title = {Minimum curvature trajectory planning and control for an autonomous race car},
    journal = {Vehicle System Dynamics}
  }

.. code-block:: none

  @inproceedings{Betz2019,
  author = {Johannes Betz and Alexander Wischnewski and Alexander Heilmeier and Felix Nobis and Leonhard Hermansdorfer and Tim Stahl and Thomas Herrmann and Markus Lienkamp},
  doi = {10.1109/iccve45908.2019.8965238},
  url = {https://doi.org/10.1109/iccve45908.2019.8965238},
  year = {2019},
  month = nov,
  publisher = {{IEEE}},
  title = {A Software Architecture for the Dynamic Path Planning of an Autonomous Racecar at the Limits of Handling},
  booktitle = {2019 {IEEE} International Conference on Connected Vehicles and Expo ({ICCVE})}
  }

Parameter list `xx_mvdc_path_feedback`
============================

This list only serves as a brief description, see the code and the documentation therein for details.

+-------------------------------------+------------------------------------------------------------------------------+
| Parameter                           | Description                                                                  |
+=====================================+==============================================================================+
| P_VDC_NumDerivativeTFilter_s        | Time constant of low pass filter for numerical error derivative estimation   |
+-------------------------------------+------------------------------------------------------------------------------+
| P_VDC_UseNumDerivativePathFeedback_b| Use numerative control error derivatives instead of analytic verions         |
+-------------------------------------+------------------------------------------------------------------------------+
| P_VDC_LongFF_PT1_s                  | Time constant of low pass filter for acceleration feedforward request        |
+-------------------------------------+------------------------------------------------------------------------------+
| P_VDC_UseLongFF_PT1_b               | Activate low pass filter for acceleration feedforward request                |
+-------------------------------------+------------------------------------------------------------------------------+
| P_VDC_UseLongFF_PT1_b               | Activate low pass filter for acceleration feedforward request                |
+-------------------------------------+------------------------------------------------------------------------------+
| P_VDC_LatKappaMaxFB_radpm           | Maximum curvature requested by feedback controller (positive/negative)       |
+-------------------------------------+------------------------------------------------------------------------------+
| P_VDC_LatDamping                    | Target damping of lateral path tracking error closed loop dynamics           |
+-------------------------------------+------------------------------------------------------------------------------+
| P_VDC_Convergence                   | Target eigenfrequency of lateral path tracking error closed loop dynamics    |
+-------------------------------------+------------------------------------------------------------------------------+
| P_VDC_MinSpeedGainCorrection_mps    | Minimum speed used for gain-scheduling of lateral controller                 |
+-------------------------------------+------------------------------------------------------------------------------+
| P_VDC_UseBetaFeedback               | Use side slip angle for improved estimation of analytic error derivatives    |
+-------------------------------------+------------------------------------------------------------------------------+
| P_VDC_UseSSCDynamicsComp_b          | Use compensation of linearization errors during cornering                    |
+-------------------------------------+------------------------------------------------------------------------------+

Parameter list `xx_mvdc_curvvel_tracking`
============================

This list only serves as a brief description, see the code and the documentation therein for details.

Velocity controller:

+---------------------------------------+-------------------------------------------------------------------------------------------------+
| Parameter                             | Description                                                                                     |
+=======================================+=================================================================================================+
| P_VDC_LongKp                          | Velocity controller proportional feedback gain                                                  |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_VelDistEst_PT1_s                | Velocity controller disturbance compensation response time constant                             |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_VelDistFFActive                 | Velocity controller feedforward disturbance compensation active                                 |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_VelDistFBActive                 | Velocity controller feedback disturbance compensation active                                    |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_FxMaxFF_N                       | Max. force for feedforward of velocity controller                                               |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_FxMaxFB_N                       | Max. force for proportional velocity controller                                                 |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_FxMaxDistComp_N                 | Max. force for feedback disturbance compensation velocity controller                            |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_FxMaxDistFF_N                   | Max. force for feedforward disturbance compensation velocity controller                         |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_VelCon_vMaxStatic_mps           | Max. setpoint for velocity controller                                                           |
+---------------------------------------+-------------------------------------------------------------------------------------------------+

Curvature controller:

+---------------------------------------+-------------------------------------------------------------------------------------------------+
| Parameter                             | Description                                                                                     |
+=======================================+=================================================================================================+
| P_VDC_CurvKp                          | Curvature controller proportional feedback gain                                                 |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_CurvDistActive                  | Curvature controller disturbance compensation active                                            |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_CurvDistEst_PT1_s               | Curvature controller disturbance compensation response time constant                            |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_LatDeltaMaxFF_rad               | Max. steering request for feedforward curvature controller                                      |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_LatDeltaMaxFB_rad               | Max. steering request for feedback curvature controller                                         |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_LatDeltaMaxDistComp_rad         | Max. steering request for disturbance compensation curvature controller                         |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_NonlinCurvDeltaActive_b         | Nonlinear under-/oversteering compensation (NUOC) active                                        |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_CurvNonlinFilter_s              | Low pass filter constant for curvature in sample aggregation of NUOC                            |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_SCPreload_Delta                 | Preloaded characteristic for NUOC (steering angle)                                              |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_SCPreload_Kappa                 | Preloaded characteristic for NUOC (curvature)                                                   |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_SCPreload_Vel                   | Preloaded characteristic for NUOC (velocity)                                                    |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_UsePreloadedSC                  | Use preloaded characteristic for NUOC                                                           |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_SCLearn_KappaMax_radpm          | Maximum curvature for online learned NUOC                                                       |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_SCLearn_FilterCoeff             | Low pass filter coefficient for sample aggregation for online learned NUOC                      |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_SCLearn_Sigma                   | Measurement noise assumed for learning samples for online learned NUOC                          |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_SCLearn_LengthScales            | Length scales assumed for gaussian process in online learned NUOC                               |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_UndersteerCompensationActive_b  | Understeer compensation active for linear feedforward                                           |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_USCLowPassActive_b              | Low pass for linear understeer compensation active                                              |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_UndersteerGradient              | Linear understeer gradient                                                                      |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_LowPassUSCT1                    | Low pass time constant for linear understeer compensation                                       |
+---------------------------------------+-------------------------------------------------------------------------------------------------+

Controller state machine:

+---------------------------------------+-------------------------------------------------------------------------------------------------+
| Parameter                             | Description                                                                                     |
+=======================================+=================================================================================================+
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_EmergencyBrakeAcceleration_mps2 | Deceleration request when controller is disabled and vehicle is still driving                   |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_FullControlSpeed_mps            | Above this speed the controller switches to full control mode                                   |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_SlowControlSpeed_mps            | Below this speed the controller switches to slow control mode                                   |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
