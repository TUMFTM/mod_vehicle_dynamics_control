========================
System module
========================
The *system* package collects all basic system functionality necessary to operate the car. This includes mainly basic diagnosis routines and the vehicle state machine.

Contact person: `alexander.wischnewski@tum.de <alexander.wischnewski@tum.de>`_

Models
========================
* `mloc_diag_vehicle.slx` collects several minor diagnosis functionality.
* `mloc_statemachine.slx` is the main state machine of the vehicle managing the startup, driving and emergency states.
* `mvdc_performance_assessment.slx` monitors performance related criterions of the controllers and the general driving behavior

Basic Setup
========================
In the following, the basic tuning process is described for all algorithms in this component. Take care that some data dictionaries have vehicle specific version. This is always named e.g. `db_xxxxxxxx.sldd`. If you change a parameter, you have to do it in the vehicle specific version. This is configured before simulation or building the model automatically by the vehicle project.

Diagnosis 'xx_mloc_diag_vehicle'
---------------------------------------------
The diagnosis component provides a check, whether the vehicle is in a safe takeover condition. `P_MLOC_SafeForTakeOver_FMax_N` specifies the maximum allowed force request, this should be negative and approximately 10% of the maximum braking force. Furthermore, `P_MLOC_SafeForTakeOver_vabsMax_mps` gives the maximum admissable speed for takeover. These are checked during the startup sequence, therefore it is essential to calibrate it properly.

The vehicle unstable diagnostic can be activated via `P_MLOC_ActivateVehicleUnstableDetection_b`. It will check the side slip angle and compare it to the maximum safe value specified via `P_MLOC_BetaMax_rad`. If this is reached, an emergency brake is triggered.

State Machine
---------------------------------------------
`P_VDC_SoftEmergencyTimeout_s` specifies the time until a hard emergency brake is performed after a non-critical failure was detected. This time should only be changed if you are aware of the consequences!

TUM Vehicle State:

+--------+------------------------------+----------------------------------------------+
| Number | State                        | Description                                  |
+========+==============================+==============================================+
|  0     | OFF                          | System is off (no state)                     |
+--------+------------------------------+----------------------------------------------+
| 10     | STARTUP_Init                 | System startup started                       |
+--------+------------------------------+----------------------------------------------+
| 11     | STARTUP_WaitForSystem        | Wait for CAN and hardware components         |
+--------+------------------------------+----------------------------------------------+
| 12     | STARTUP_WaitForAIDriverComms | Wait for other AI driver components          |
+--------+------------------------------+----------------------------------------------+
| 20     | IDLE_ActivateSensorFusion    | Enable sensor fusion                         |
+--------+------------------------------+----------------------------------------------+
| 21     | IDLE_WaitForSensorFusion     | Wait until sensor fusion is ok               |
+--------+------------------------------+----------------------------------------------+
| 22     | IDLE_ActivateDriver          | Enable trajectory driver                     |
+--------+------------------------------+----------------------------------------------+
| 23     | IDLE_WaitForDriver           | Wait until trajectory driver is ok           |
+--------+------------------------------+----------------------------------------------+
| 24     | IDLE_WaitForVehicle          | Wait until vehicle is ready to drive         |
+--------+------------------------------+----------------------------------------------+
| 30     | DRIVING                      | Normal driving                               |
+--------+------------------------------+----------------------------------------------+
| 50     | EMERGENCY                    | Soft emergency (uses emergency trajectory)   |
+--------+------------------------------+----------------------------------------------+
| 60     | HARDEMERGENCY                | Hit brakes and put steering straight         |
+--------+------------------------------+----------------------------------------------+

TUM Strategy Strategy:

+--------+------------------------------+----------------------------------------------+
| Number | State                        | Description                                  |
+========+==============================+==============================================+
|  0     | S_OFF                        | System is off (no state)                     |
+--------+------------------------------+----------------------------------------------+
| 10     | S_STARTUP                    | Process started and initializing             |
+--------+------------------------------+----------------------------------------------+
| 20     | S_IDLE                       | Standstill and waiting for vehicle controller|
+--------+------------------------------+----------------------------------------------+
| 30     | S_DRIVING                    | Normal driving                               |
+--------+------------------------------+----------------------------------------------+
| 40     | S_BRAKE2STOP                 | Mission finished and stopping the vehicle    |
+--------+------------------------------+----------------------------------------------+
| 50     | S_EMERGENCY                  | Emergency situation detected                 |
+--------+------------------------------+----------------------------------------------+

Error codes
========================
The debug signal *debug_ErrorCode* (in the debug struct) holds a combined error code which encodes the most likely errors in a bitwise fashion. It is logged as a uint8 and therefore it is required to break the code into the bits manually (e.g. a value of 7 means that code 1, code 2 and code 4 are present). The error codes are:

+--------+----------------------------------------------+
| Code   | Description                                  |
+========+==============================================+
| 001    |  Performance trajectory not ok               |
+--------+----------------------------------------------+
| 002    |  Emergency trajectory not ok                 |
+--------+----------------------------------------------+
| 004    |  Path Deviation too high (below critical)    |
+--------+----------------------------------------------+
| 008    |  Path Deviation too high (above critical)    |
+--------+----------------------------------------------+
| 016    |  Trajectory Comms not ok                     |
+--------+----------------------------------------------+
| 032    |  Strategy Comms not ok                       |
+--------+----------------------------------------------+
| 064    |  Controller not ok                           |
+--------+----------------------------------------------+
| 128    |  Trajectory too short for TMPC               |
+--------+----------------------------------------------+
| 256    |  Vehicle unstable                            |
+--------+----------------------------------------------+
| 512    |  External module not ok                      |
+--------+----------------------------------------------+
