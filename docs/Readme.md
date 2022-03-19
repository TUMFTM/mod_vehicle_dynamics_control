Generate the docu using
```shell script
sphinx-apidoc -f -o ./source/tum_mcs ../misc/py_binds/tum_mcs/src_dist
```
to generate the docu chapters from the bindings source and subsequently
```
make clean html
```
 to create the build.