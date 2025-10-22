# my_robot_gazebo (ROS Noetic + Gazebo Classic)

This package lets you spawn your **URDF** robot in Gazebo quickly, without scaling changes.

## 1) Drop your files
- Put your URDF at: `urdf/robot.urdf` (rename your file to `robot.urdf` or update the launch arg `urdf_file`).
- Put all mesh files under: `meshes/`. Keep the same relative paths used in your URDF.
  - Mesh references inside URDF should look like: `filename="package://my_robot_gazebo/meshes/<your_mesh.stl or dae>"`

> **Note:** No scaling is applied. If your meshes were exported with real dimensions, they will be used as-is.

## 2) Build in a Catkin workspace
```bash
# If you don't have a workspace yet:
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# Copy or unzip this package into src/
# e.g., cp -r /path/to/my_robot_gazebo .
cd ..
catkin_make
source devel/setup.bash
# For convenience, add to your ~/.bashrc:
# echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## 3) Spawn in Gazebo
```bash
roslaunch my_robot_gazebo spawn_gazebo.launch
```
- To use a different URDF file name:
```bash
roslaunch my_robot_gazebo spawn_gazebo.launch urdf_file:=$(rospack find my_robot_gazebo)/urdf/YOUR_FILE.urdf
```
- Adjust initial pose:
```bash
roslaunch my_robot_gazebo spawn_gazebo.launch x:=1.0 y:=2.0 z:=0.2 Y:=1.57
```

## 4) Quick RViz check (no Gazebo)
```bash
roslaunch my_robot_gazebo view_model.launch
```

## Troubleshooting
- **Meshes not showing**: Ensure URDF uses `package://my_robot_gazebo/meshes/...` and the files exist.
- **xacro error**: Even with plain URDF, `xacro` can parse it. If it fails, change:
  - In launch files, replace `$(find xacro)/xacro $(arg urdf_file)` with `cat $(arg urdf_file)`.
- **Collisions/inertias**: Gazebo needs valid `<inertial>` and `<collision>` for each link for stable physics.
- **Gazebo not installed**: Use `sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control`.
