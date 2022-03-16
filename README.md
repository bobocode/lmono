<h1>lmono</h1>
<h2>LMONO-Fusion: An Online SLAM System based on LiDAR-Monocular Camera Sensor Fusion</h2>
This is a framework of LiDAR-monocular camera fusion system. Visual information from monocular camera assist in scene recognization in LiDAR odometry and dense mapping. Automatic calibration between LiDAR and a single camera is provided as well. The pre-print version of our paper is available [here](). 
  
  ***More code is coming...***

- [x] laser odometry
- [x] scene recognition/ loop detection
- [x] automatic and real-time calibration
- [x] 3D dense map with color information
- [ ] ground segmentation
- [ ] communication interface between LiDAR and Camera
- [ ] speeding up 

**Feb, 2022** This work is already submitted to TVT.

<h2>1. Prerequisites</h2>
<h3>1.1 Ubuntu and ROS</h3>

   * Ubuntu 64-bit 18.04  
   
   * ROS Melodic. [ROS INSTALLATION](http://wiki.ros.org/ROS/Installation)  
   
<h3>1.2 Ceres Solver</h3>  

   * Ceres. [Ceres INSTALLATION](http://ceres-solver.org/installation.html)  

<h3>1.3 PCL</h3>  

* PCL [PCL INSTALLATION](http://www.pointclouds.org/downloads/linux.html)  

<h2>2. Build</h2>

<h3>2.1 Clone repository</h3>

     cd ~/catkin_ws/src
     git clone https://github.com/bobocode/lmono.git
     cd ..
     catkin_make
     source ~/catkin_ws/devel/setup.bash
    

<h3>2.2 Download dataset and test rosbag</h3>

* Download [KITTI sequence](http://www.cvlibs.net/datasets/kitti/eval_odometry.php)

* To generate rosbag file of kitti dataset, you may use the tools

      roslaunch aloam_velodyne kitti_helper.launch

<h3>2.3 Launch ROS in different terminals</h3>
      
* Launch [ALOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) to gain LiDAR measurements

      roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch
      
or

       roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch
       

* Launch Estimator Node

       roslaunch monolio kitti_estimator_xx.launch
       
* Rosbag Play

      rosbag play xxx.bag

* If you want to open loop detection

       roslaunch monolio kitti_loop_xx.launch

* If you want to open fusion mapping

       roslaunch monolio kitti_map_00.launch

<h2>3.Acknowledgements</h2>

 **Thanks to** [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) and [VINS-MONO](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) code authors. The major codes in this repository are borrowed from their efforts.

  
 
