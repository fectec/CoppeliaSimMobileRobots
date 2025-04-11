# CoppeliaDiffDrive

## Building the CoppeliaSim ROS2 Interface

```bash 
# Navigate to your repository root (adjust the path as needed)
cd ~/CoppeliaDiffDrive

# Initialize and update submodules (this fetches the submodule in ros2_ws/src/sim_ros2_interface)
git submodule update --init --recursive

# Checkout the desired branch/tag in the simROS2 submodule using the -C flag
git -C ros2_ws/src/sim_ros2_interface checkout coppeliasim-v4.9.0-rev6

# (Optional) Install xsltproc, a required dependency for the build process
sudo apt install xsltproc

# Set the CoppeliaSim root directory (replace with your actual installation path)
export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder

# Increase stack size to prevent potential compilation freeze/crash
ulimit -s unlimited

# Navigate to the ROS 2 workspace directory and build the workspace
cd ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------------------
# The plugin is now ready to be used.
#
# Next steps:
# 1. Open a terminal.
# 2. Move to the CoppeliaSim installation folder.
# 3. Start CoppeliaSim and load the ROS2 plugin using Lua or Python:
#       simROS2 = require('simROS2')
#
# Upon successful ROS2 Interface load, you should see the node /sim_ros2_interface.
# To confirm, you can run:
# ---------------------------------------------------------------------------

ros2 node list
```
