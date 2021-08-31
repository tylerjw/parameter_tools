# parameter_tools
Patterns and sugar for ROS2 parameters

## structure

* parameter_set - C++ library for standardizing declaring, validating, getting, and handling of dynamic parameters
* parameter_builder - Python library for loading parameters in launch files

## absl Dependency on Ubuntu 20.04

The `absl` library is not released into Ubuntu 20.04 so it must be installed via a PPA before building parameter_set.

```bash
sudo add-apt-repository --yes ppa:bleedingedge/focal-bleed
sudo apt-get update
sudo apt-get install --yes libabsl-dev
```
