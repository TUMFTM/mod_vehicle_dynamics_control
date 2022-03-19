=============================
Installation
=============================
This is a brief tutorial how to setup your computer to work on the controller software for Windows using the MinGW Compiler. 
Different build options can be found here: :doc:`../software/control/TMPC`. See the section on build instructions.

#. Install MATLAB 2018b
#. Create a new folder which will contain the software stack
#. Create subfolders `modules` and `simulation`
#. Checkout the following repositories in the corresponding subfolders on your local machine
    * `iac_software/mod_control` from `here <https://github.com/TUMFTM/mod_vehicle_dynamics_control>`_ 
    * `iac_software/sim_vehicle_dynamics` from `here <https://github.com/TUMFTM/sim_vehicle_dynamics>`_ 
#. Install and setup a C/C++ compiler for MATLAB
    * Install the `MATLAB Support for MinGW-w64 C/C++ Compiler` from the MATLAB Add-Ons
    * Run `mex -setup` in MATLAB to setup the compiler
#. Install the Speedgoat IO Blockset version 9.2.0.1 for MATLAB R2018b (optional)
    * This driver is only required if you want to run the HiL setup
    * It requires a license which you can obtain directly from Speedgoat
#. Install OSQP (https://github.com/osqp/osqp)
    * Clone the repository and initialize submodules if prompted.
    * Check if cmake is installed via typing `cmake` into a shell (e.g. classical commandline or powershell)
    * Check if gcc is installed in the correct version (tested against tdm64-gcc-5.1.0-2.exe for Windows) via typing `gcc -v` into a shell. Can be downloaded `here <https://sourceforge.net/projects/tdm-gcc/files/TDM-GCC%20Installer/>`_.
    * In oder to for the following commands to work, navigate to the OSQP repository first and use PowerShell or cmd for the following commands. 

    .. code-block::

        mkdir build
        cd build
        cmake -G "MinGW Makefiles" ..
        cmake --build .

    * The file libosqp.a has to be renamed to libosqp_mingw.lib and copied from '/osqp/build/out' to 'mod_control/misc/osqp/lib'.
    * Copy the content of the include folder `/osqp/include` to `/mod_control/misc/osqp/header`
#. Check if you setup everything correctly
    * Go to the :doc:`gettingstarted` section below and run a simulation
    * If it runs without errors, your setup is ready to go
