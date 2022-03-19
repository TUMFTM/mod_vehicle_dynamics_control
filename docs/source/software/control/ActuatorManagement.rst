========================
Actuator Management and Traction Control
========================

Concept
=============
The actuator management is responsible for safety checks on the calculated control requests such as rate limitations and enforcing actuator limitations. The traction control is a very basic controller which kicks in if one of the wheels has more slip than a certain target value (braking and accelerating). It uses a PI-like behavior if it intervenes.

Parameter list `xx_mvdc_trajectory_driver`
============================

+---------------------------------------+-------------------------------------------------------------------------------------------------+
| Parameter                             | Description                                                                                     |
+=======================================+=================================================================================================+
| P_VDC_EnableTC                        | Enable traction control                                                                         |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_TC_TargetSlip_perc              | Target slip for traction control                                                                |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_TC_Kp                           | Traction control proportional control gain                                                      |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_TC_Ki                           | Traction control integral control gain                                                          |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_TCActiveSpeed_mps               | Traction control minimum speed                                                                  |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_TCMaxF_N                        | Maximum force modification via traction control                                                 |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_SlipFilterTC_s                  | Low pass time constant for actual slip value in traction control                                |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_TCReductionFactor               | Forgetting factor of traction control after usage                                               |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_NegativeFxStandstill_N          | Control request when controller is disabled                                                     |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_MaxSlopeDelta_radps             | Maximum change rate of steering wheel request                                                   |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
| P_VDC_FxMaxSlope_Nps                  | Maximum change rate of longitudinal force request                                               |
+---------------------------------------+-------------------------------------------------------------------------------------------------+
