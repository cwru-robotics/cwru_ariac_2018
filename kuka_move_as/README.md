# kuka_move_as

Kuka robot action server to abstract behaviors.

Helper library: TransitionTrajectories.  This library precomputes safe and efficient moves from specified start location code
to specified goal location code.  E.g., Q1 station hover to Bin3 hover;



## Example usage

## Running tests/demos
test pgm for transition trajectories:
`rosrun kuka_move_as transition_trajectories_test_main`
start up ariac simu, e.g.:
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2b.yaml ~/ariac_ws/src/cwru_ariac_2018/copy_of_team_config/depthcam_config.yaml`

OR:
rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2b.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml


Start up behavior server, kuka_move_as:

`rosrun kuka_move_as kuka_behavior_as`

can test pick_part_from_bin operations with this node:
`rosrun shipment_filler test_pick_from_inventory`

this will attempt to remove ALL parts from ALL bins, as observed by inventory manager.  
Use this test node to debug bin picking exhaustively.

To find better joint-space poses, edit the values below, then copy/paste into a terminal.  This will send a trajectory
command directly to the robot.  Approach taken here is to freeze iiwa_joint_3 at 0.

rostopic pub /ariac/arm/command trajectory_msgs/JointTrajectory \
"{joint_names: ['iiwa_joint_1', 'iiwa_joint_2', 'iiwa_joint_3', 'iiwa_joint_4', 'iiwa_joint_5', 'iiwa_joint_6',
  'iiwa_joint_7', 'linear_arm_actuator_joint'], \
points: [ \
{time_from_start: {secs: 3}, \
positions: [0.386, -1.35, 0, 0.8, -0.113, -0.807, 2.8085, -0.400]}, \
]}" -1

update 5/2/2018:
Working on new version of robot behavior action server: kuka_move_as2
Test bin-picking as follows.
Start up a simu, e.g.:
rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2b.yaml ~/ariac_ws/src/cwru_ariac_2018/copy_of_team_config/depthcam_config.yaml

Start the  new action server:
`rosrun kuka_move_as kuka_behavior_as2`

Start the bin-picking test node:
`rosrun shipment_filler test_pick_from_inventory`
    
