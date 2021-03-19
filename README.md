# 3D Reconstruction
### Author: Arun Kumar
* reconstruct: package that provides launch files to create a point cloud map and services to perform 3D reconstruction

#### Usage Instructions
1. Add package to the src folder in your ROS workspace
1. Install necessary dependencies
1. Compile: `catkin_make`
1. Connect RealSense Camera (only tested with Intel RealSense D435i)

#### Example Usage
```
roslaunch reconstruct visual_slam_scan.launch
rosservice call /create_gp_mesh "filename: '/home/arun/gp_mesh.stl'"
rosservice call /create_mc_mesh "filename: '/home/arun/mc_mesh.stl'"
rosservice call /create_ply "filename: '/home/arun/normals.ply'"
```

#### Configuration Instructions
* configure Greedy Projection Triangulation meshing parameters in config/gp_params.yaml
* configure Marching Cubes meshing parameters in config/mc_params.yaml
* configure Moving Least Squares parameters in config/mls_params.yaml

#### Launchfiles
* visual_slam_scan.launch
    * primary launch file
    * launches all nodes on one machine
    * allows you to create point cloud map and run meshing algorithms
    * contains arguements to launch rviz, set voxel size, set camera clipping distance
    * other arguements allow user to collect point clouds from a remote machine
* icp_scan.launch
    * creates point cloud map using ICP
    * functional but not recommended for meshing
* rtabmap.launch
    * rtabmap's defaul visual slam launch file
    * added an arguement, cell_size, to set point cloud map voxel size

#### Nodes
* reconstruct_scanner: uses ICP to create point cloud map
* reconstruct_triangulate: provies services to create .stl and .ply files

#### Video
[![Scanner](https://img.youtube.com/vi/oK1snEow3yI/0.jpg)](https://www.youtube.com/watch?v=oK1snEow3yI "Handheld 3D Scanner")