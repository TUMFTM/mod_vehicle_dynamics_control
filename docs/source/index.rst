Welcome to TUM Autonomous Motorsport Vehicle Motion Control documentation!
==========================================================================
This software stack has been developed and used for the Roborace and the Indy Autonomous Challenge Competition. It achieved 220kph and 95% of the combined lateral and longitudinal acceleration potential of the DevBot (Roborace) in 2019, which allowed to drive within 2% of the laptime of an amateuer human race driver. This was achieved with a feedforward/feedback control design based on simple kinematic relations and an LQR controller design. Based on these results, the motion control algorithm was re-designed and is now based on a Tube-MPC approach handling additional information about the tire-road friction limits. This controller was used for the software stack of the team for the Indy Autonomous Challenge, where it achieved 270kph and about 28mps2 of lateral acceleration in a two vehicle passing competition. This software component covers the trajectory tracking, state estimation and vehicle dynamics control aspects of the stack. It takes trajectories from the planner as the main input and delivers appropriate steering, powertrain and brake commands. Furthermore, it handles vehicle startup and emergency brake situations. 

A video of the performance at the Monteblanco track can be found `here <https://www.youtube.com/watch?v=-vqQBuTQhQw>`_. Videos from the Indy Autonomous Challenge can be found `here <https://www.youtube.com/watch?v=uVF_JVhOrU4>`_ and  `here <https://www.youtube.com/watch?v=df9f4Qfa0uU>`_. Current updates on the project status and a list of related scientific publications are available `here <https://www.mos.ed.tum.de/ftm/forschungsfelder/intelligente-fahrzeugsysteme/indy-autonomous-challenge/>`_. If you find this repository useful and base your work upon it, please cite `Minimum curvature trajectory planning and control for an autonomous race car <https://www.tandfonline.com/doi/abs/10.1080/00423114.2019.1631455>`_ (LQR controller) or `Tube model predictive control for an autonomous race car <https://www.tandfonline.com/doi/abs/10.1080/00423114.2021.1943461?journalCode=nvsd20>`_ (Tube-MPC).

.. warning::
  This software is provided *as-is* and has not been subject to a certified safety validation. Autonomous Driving is a highly complex and dangerous task. In case you plan to use this software on a vehicle, it is by all means required that you assess the overall safety of your project as a whole. By no means is this software a replacement for a valid safety-concept. See the license for more details.

Acknowledgements & Contributions
=================================
Core developers of the package

* Alexander Wischnewski (Control, State Estimation, System Components)
* Frederik Werner (Control)

Several students contributed to the success of the project during their Bachelor's, Master's or Project Thesis.

* Fabian Christ (Trajectory Planning)
* Marco Kappertz (State Estimation)
* Johannes Schwarz (Flatness based Control)
* Madeline Wolz (State Estimation)
* Markus Schreiber (Drifting Control)
* Sandra Kühbacher (Continuous Integration & Testing)
* Salih Gümüs (Nonlinear Model Predictive Control)
* Martin Euler (Robust Nonlinear Model Predictive Control)
* Christoph Ibel (Robust Nonlinear Model Predictive Control)
* Max Schager (Vehicle Dynamics Control)

.. toctree::
   :maxdepth: 1
   :caption: Contents:

   start/overview.rst
   start/installation.rst
   start/gettingstarted.rst
   start/debugtool.rst
   start/adapt.rst
   software/Software.rst

Contact Information
===================

:Email: `alexander.wischnewski@tum.de <alexander.wischnewski@tum.de>`_
:Email: `frederik.werner@tum.de <frederik.werner@tum.de>`_

