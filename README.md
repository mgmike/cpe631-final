A few packages are needed to run this project.

As seen in the package.xml, the following are needed:
    gmapping
    pedsim
    roscpp
    map_server
    move_base
    move_base_msgs
    geometry_msgs
    std_msgs
    gazebo_msgs
    people_msgs
    actionlib
    tf


# Installation


The following will need to be installed manually. 

gmapping:

sudo apt install ros-melodic-gmapping

pedsim:

cd ~/catkin_ws/src
git clone --recurse-submodules https://github.com/ral-stevens/CPE631Final.git
cd ~/catkin_ws

people_msgs:

sudo apt install ros-melodic-people-msgs

catkin_make


## Pioneer3AT
To run the Pioneer3AT robot that runs into the wall, four terminals are needed:

With the first one, run 

$roslaunch cpe631-final gmappingLnch.launch

This launch file launches the following nodes:
    joint _state_publisher
    Robot_state_publisher
    Rviz
    Pedestrian_simulator
    Gazebo
	
With the second one, run

$roslaunch cpe631-final move_base.launch

This launch file launches the following nodes:
    Map_server
    All move_base nodes

With the third one, run
$rosrun amcl amcl scan:=laser/scan _initial_pose_x:=1.0 _initial_pose_y:=-3.0
This command launches AMCL

With the fourth one, run 
$rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 14.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}'

This command publishes a goal topic.


## Turtlebot3 no pedestrians
To run the Turtlebot3 Waffle with no pedestrians and using the usual navigation stack, three terminals are needed:

With the first one, run 
$roslaunch cpe631-final turtlebot_init.launch ped:=false

This launch file will either start Gazebo with no pedestrians or start Gazebo with pedestrians and also start the pedsim_simulator node to move the pedestrians.

With the second one, run
$roslaunch cpe631-final turtlebot_nav.launch han:=false

This launch file will launch either:
    Robot_state_publisher
    Map_server
    AMCL
    Move_base
    Rviz
Or:
    Map_server
    Move_base with human aware navigation algorithms
    rviz

With the third one, run
$rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 14.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}'


## Turtlebot3 with pedestrians
To run the Turtlebot3 Waffle with pedestrians, simply run the previous 3 commands but omit the “ped:=false” argument from the first command. 

To run the Turtlebot3 Waffle with the Human Aware Navigation stack instead of the typical Navigation stack remove the “han:=false” argument from the second command. In addition add the command:
$rosrun cpe631-final pedestrian_streamer output:=screen
Which will run the custom created people converter.
