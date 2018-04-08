# shipment_filler

top-level code goes here.  Coordinates receiving, processing and shipping orders.

## Example usage
start up gear simulation, e.g., on Ava:
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2a.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

vs
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2a.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`


or on Atlas6:
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2a_no_logging.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

Start up the robot-motion action server:
`rosrun kuka_move_as kuka_behavior_as`

Start up the shipment filler (top level program):
`rosrun shipment_filler simple_shipment_filler`

## Running tests/demos
    
