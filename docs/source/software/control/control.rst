========================
Control module
========================

The *control* package collects all functionality related to controllers and directly related pre- or postprocessing methods. It provides three different implementations for trajectory tracking control of a racing vehicle:

* The :doc:`BasicControl` is the controller used within the Roborace competitions in 2018 and 2019. It is based upon simple point mass model assumptions and has shown reliable performance under varying circumstances.
* The :doc:`TMPC` is the robust model predictive controller which will be used within the Indy Autonomous Challenge in 2020 and 2021. It uses the QP-solver OSQP and a problem tailored approach to linearization and preparation of the optimization problem. The vehicle model used is a kinematic bicycle model with combined acceleration constraints on lateral and longitudinal accelerations.

Each controller is described in more detail on its respective page. There you find information about the tuning as well as general information about the algorithm. All of them share the :doc:`PathMatching` functionality which calculates the vehicle position in a path-centered coordinate frame. Furthermore, they share the :doc:`ActuatorManagement`.

The control algorithm is selected via the variable `P_VDC_ControllerChoice`:

* Set to 1 for the TMPC
* Set to 3 for the Basic Controller

Contact person: `alexander.wischnewski@tum.de <alexander.wischnewski@tum.de>`_

.. toctree::
   :maxdepth: 1
   :caption: Contents:

   structure.rst
   PathMatching.rst
   ActuatorManagement.rst
   BasicControl.rst
   TMPC.rst
