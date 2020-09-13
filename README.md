To run the Pioneer3AT robot that runs into the wall, four terminals are needed:

With the first one, run 
$roslaunch cpe631_final_with_nav gmappingLnch.launch

This launch file launches the following nodes:
    joint _state_publisher
    Robot_state_publisher
    Rviz
    Pedestrian_simulator
    Gazebo
	
With the second one, run
$roslaunch cpe631_final_with_nav move_base.launch

This launch file launches the following nodes:
    Map_server
    All move_base nodes

With the third one, run
$rosrun amcl amcl scan:=laser/scan _initial_pose_x:=1.0 _initial_pose_y:=-3.0
This command launches AMCL

With the fourth one, run 
$rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 14.0, y: -1.0, z: 0.0}, orientation: {w: 1.0}}}'

This command publishes a goal topic.



To run the Turtlebot3 Waffle with no pedestrians and using the usual navigation stack, three terminals are needed:

With the first one, run 
$roslaunch cpe631_final_with_nav turtlebot_init.launch ped:=false

This launch file will either start Gazebo with no pedestrians or start Gazebo with pedestrians and also start the pedsim_simulator node to move the pedestrians.

With the second one, run
$roslaunch cpe631_final_with_nav turtlebot_nav.launch han:=false

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



To run the Turtlebot3 Waffle with pedestrians, simply run the previous 3 commands but omit the “ped:=false” argument from the first command. 



To run the Turtlebot3 Waffle with the Human Aware Navigation stack instead of the typical Navigation stack remove the “han:=false” argument from the second command. In addition add the command:
$rosrun cpe631_final_with_nav pedestrian_streamer output:=screen
Which will run the custom created people converter.
