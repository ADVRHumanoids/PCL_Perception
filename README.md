# PCL_Perception
The repository can be used to perform a planar segmentation of a Point Cloud and to extract the relative normal-to-plane using the Centauro robot.

# Prerequisites
In order to make the project compile and run, those packages are required:
  - centauro-simulator: https://github.com/ADVRHumanoids/centauro-simulator
  - octomap: `sudo apt-get install ros-kinetic-octomap*`
  - CartesianInterface: https://github.com/ADVRHumanoids/CartesianInterface
  - PCL 1.7: `sudo apt-get install ros-kinetic-pcl*`
  - velodyne_gazebo_plugin: https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/
  
# Install
The package is built as a catkin package and can be easily installed in your catkin workspace. As an alternative, the usual cmake procedure can be used.

# Launch the simulation
Explaining the nodes 
-
The package contains a simulation in which the Centauro robot, using the velodyne sensor mounted on its head, acquires the full surrounding environment as a Point Cloud, and extract only those inliers belonging to the widest plane. In most cases, the ground results being the biggest plane and it must filtelered out. This is done using a pass through along the z-axis (eventually, also the ground_filtering option of octomap can be enabled).

Run the simulation
-
perception.launch file contains several flags that can be enabled. By default they are all set to true. 
The launch file includes the gazebo_sim.launche (if flag 'gazebo' is set to true) that loads XBotCore, Centauro's urdf and srdf, and all the pugins required by the velodyne. It also opens an rviz environment in which the velodyne point cloud should be visible, after pushing the play button on gazebo (be sure to select /velodyne_points as topic).

**TBN**: if PointCloud returns an error, typically when using rosmon, simply restart the launch file.

To rotate the velodyne joint:
  - `rosservice call /xbotcore/XBotCommunicationPlugin_switch 1`

```
rostopic pub /xbotcore/command xbot_msgs/JointCommand "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['neck_velodyne']
position: [0]
velocity: [3.14]
effort: [0]
stiffness: [0]
damping: [10]
ctrl_mode: [26]
aux_name: ''
aux: [0]" -r 20 
     
```
     
The velodyne should be now rotating and selecting /octomap_point_cloud_centers in rviz topic the whole environment should be visible as PointCloud

Use `rosservice call /set_filter_parameters` to set the desired pass through filter behaviour in order to filter the ground and keep only the object previously added on gazebo.

Once the object is visible on rviz, in /filtered_cloud topic, you can run the plane segmentation node: `rosrun PCL_Perception planar_segmentation_node`. This will display only the inliers in /plane_segmentation topic and the normal should be displayed as a frame and a marker. If the result is not satisfying, try to play with the segmentation parameters using `rosservice call /set_segmentation_parameters`.
