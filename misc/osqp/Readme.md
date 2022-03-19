CMake generate commands required for OSQP:

### Build for MATLAB (MinGW):
`cmake -G "MinGW Makefiles" ..` \
`cmake --build .`

### Build for MATLAB (Visual Studio)
`cmake -G "Visual Studio 15 2017" -A x64 ..` \
`cmake --build .`

### Build for Cython (Visual Studio)
`cmake -G "Visual Studio 15 2017" -A x64 .. -DCMAKE_BUILD_TYPE=Release` \
`cmake --build . --config Release`

### Build for Speedgoat (use TUM fork of osqp due to additional cmake flags)
`cmake -G "Visual Studio 15 2017" .. -DCMAKE_BUILD_TYPE=Release` \
`cmake --build . --config Release`

### Additional remarks:
Speedgoat builds require that the flag DLONG is not set in osqp_configure.h. This can be deactivated with the flag SPEEDGOAT_BUILD.
