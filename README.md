# CoppeliaDiffDrive

## Building the CoppeliaSim ROS 2 Interface

```bash 
export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
ulimit -s unlimited # Otherwise, compilation might freeze/crash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
