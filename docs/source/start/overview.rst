=============================
Repository overview
=============================

Folder structure
=============================
* `CI`: functionality related to continuous integration jobs.
* `control`: everything that actuates the low level vehicle behavior and therefore acts as a control system, especially curvature, velocity and lateral tracking controller.
* `estimation`: Vehicle dynamics sensor fusion and fusion of different localization pipelines.
* `example_vehicle`: Example implementation of the software for a passenger vehicle.
* `friction`: Friction estimation and tire analysis.
* `interfaces`: Contains interpackage interface definitions.
* `main`: Contains the main module model which collects all subcomponents.
* `misc`: Several small functions, e.g. transformations.
* `scripts`: A collection of useful scripts for operating this module.
* `softwareEmulation`: Replacements used for simulation purposes of the more complex planning parts in the full software stack.
* `system`: Vehicle diagnosis, system startup and state machine logic.
* `racelines`: Example racelines for controller development.


Software development tools
=============================
Due to the requirement, to manage multiple vehicle with the same code, we use Data Dictionaries and Simulink Project extensively. You can find information on `Simulink Project <https://de.mathworks.com/products/simulink/projects.html>`_ and `Data Dictionaries <https://de.mathworks.com/help/simulink/ug/what-is-a-data-dictionary.html>`_ in the Mathworks Simulink Documentation. As long as you do not plan to restructure the repositories or add multiple vehicles, it is not necessary to dive very deep into these topics. There are high level functions available to help you with software configuration (see Working with the Software Stack section).
