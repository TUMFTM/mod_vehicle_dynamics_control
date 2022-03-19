===================================
Getting started with the software
===================================
The repository provides multiple examples to get you started. The first runs on a Desktop computer based and simulates the control software, a vehicle model and a low-fidelity emulation of the trajectory planning software. It can be found under `/example_vehicle/tests/controller_dev/`. 

Desktop simulation
=============================
* Open the main project `modules/mod_vehicle_dynamics_control/Mod_vehicle_dynamics_control.prj`
* Open the model `modules/mod_vehicle_dynamics_control/example_vehicle/tests/controller_dev/controller_dev.slx` in Simulink and run it via the *Run* button.

Vehicle dynamics model
=============================
All the above variants can be used with one of three different vehicle dynamic models. They are chosen based on the parameter `P_PassengerVehicleSimModel` located in `example_vehicle/datadict/PassengerVehicle.sldd`. Details on the variants are available in the documentation of the `sim_vehicle_dynamics` repository. The parameter can be set as follows:

* Nonlinear single track model (recommended for basic development tasks, low computation times) - `P_PassengerVehicleSimModel = 1`
* Customized nonlinear dual track model for racing applications based on the Mathworks `Vehicle Dynamics Blockset <https://de.mathworks.com/products/vehicle-dynamics.html>`_ - `P_PassengerVehicleSimModel = 2`
* Standard nonlinear dual track model from the examples of the Mathworks `Vehicle Dynamics Blockset <https://de.mathworks.com/products/vehicle-dynamics.html>`_ - `P_PassengerVehicleSimModel = 3`

Use different racelines
=============================
The racelines available for simulation can be found under `racelines`. Each csv-file represents a single raceline.
The format specification can be found in the function documentation of the file. An example is provided in `racelines/TrackCreationExample.csv`. Run the script to load the chosen raceline into the `raceline.sldd` data dictionary which is then used for trajectory emulation. This data dictionary also holds corresponding scenario information (e.g. vehicle start position).
To load a new raceline from a csv-file, do the following:

* Call `loadRaceline('<YourRacelineName>')`, e.g. `loadRaceline('Modena')`

If you want to create a custom scenario for later use, you can store it in a separate data dictionary:

* Call `loadRaceline('<YourRacelineName>', '<YourDDname>')`, e.g. `loadRaceline('Modena', 'Modena_test')`

To load an existing scenario from a data dictionary into the `raceline.sldd`, run:

* Call `loadScenarioDD('<YourScenarioName>')`, e.g. `loadScenarioDD('Modena_test')`

Data inspection
=============================

The logs are stored using a common mat file format. It can be obtained from the Simulink Simulation as well as from the compiled python bindings using one of the following workflows:

**Simulink Simulation**

* Call `convertSimLogs(logsout, '<YOURFILENAME>.mat')` with a file name of your choice.

The software stack provides a MATLAB GUI for basic visualization of all important data for the algorithms. It can be started by running `trajectorycontrolvisualizer` from the MATLAB command line. A general introduction to the usage of the tool can be found in :doc:`debugtool`. Details on the signal meanings are documented in the models.

Common problems
=============================
There are a few common issues when the vehicle is not starting to drive. To facilitate easy debugging, they are indicated by specific error codes available in the debug logs. A list of all codes and a short tutorial on how to read them can be found in the documentation of the :doc:`../software/system`.

In general, the following workflow for debugging potential problems during driving is recommended using the data inspection tool:

* Check the sensor signals and sensor fusion using the *RawIMU*, *SensorFusion* and 'StateEstimation' pages. All values should be in reasonable ranges for your vehicle application. Make sure the units match the units of the labels.
* Check the localization on the target path using the *RawPathLocalization* page. The vehicle is expected to be close to the target path and oriented correctly.
* Check the behavior of the controllers using *CurvatureTracking*, *VelocityTrackingControl* and *PathTrackingControl*. Check if the control errors are reasonable and if the control actions might be too aggressive for your application.
