#lmono
##LMONO-Fusion: An Online SLAM System based on LiDAR-Monocular Camera Sensor Fusion
This is a framework of LiDAR-monocular camera fusion system. Visual information from monocular camera assist in scene recognization in LiDAR odometry and dense mapping.
automatic calibration between LiDAR and a single camera is provided as well.

- [x] laser odometry
- [x] scene recognition/ loop detection
- [x] automatic and real-time calibration
- [x] 3D dense map with color information

##1. Prerequisites
###1.1 Ubuntu and ROS
   * Ubuntu 64-bit 18.04
   * ROS Melodic. [ROS INSTALLATION](http://wiki.ros.org/ROS/Installation)
   
###1.2 Ceres Solver
   * Ceres. [Ceres INSTALLATION](http://ceres-solver.org/installation.html)

###1.3 PCL
* PCL [PCL INSTALLATION](http://www.pointclouds.org/downloads/linux.html)

##2. Build


###4.1 Clone repository:
    cd ~/catkin_ws/src
    git clone https://github.com/bobocode/lmono.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
    

###4.2 Download dataset and test rosbag
* Download [KITTI sequence](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)
* To generate rosbag file of kitti dataset, you may use the tools
      roslaunch aloam_velodyne kitti_helper.launch

###4.3 Launch ROS in different terminals

* Rosbag Play
      rosbag play xxx.bag
* Launch [ALOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) to gain LiDAR measurements
      roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch
or
       roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch

* Launch Estimator Node
       roslaunch monolio kitti_estimator_xx.launch

* If you want to open loop detection
       roslaunch monolio kitti_loop_xx.launch

* If you want to open fusion mapping
       roslaunch monolio kitti_map_00.launch

##5.Acknowledgements
 #####Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and [VINS-MONO]()
