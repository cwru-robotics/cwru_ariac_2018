# cwru_ariac_2018
for NIST ARIAC competition

example:
Start up demo simulation:
`roslaunch osrf_gear sample_environment.launch`
or,  to test with faulty part:
`roslaunch osrf_gear sample_environment.launch fill_demo_shipment:=true`

better:
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual1a.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual1_config.yaml`


rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual1a.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual1_config.yaml 

or:
`rosrun osrf_gear gear.py --visualize-sensor-views -f `catkin_find --share --first-only osrf_gear`/config/quals/qual1a.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual1_config.yaml'

Start up robot move action server:
`rosrun robot_move_as robot_move_as`

start the shipment filler:
`rosrun shipment_filler simple_shipment_filler`

Send a test order:
`rosrun order_sender order_sender2`

OR, send orders via competition interface:
`rosservice call /ariac/start_competition`

#sub-tests:
Run an example move-part command (hard coded):
`rosrun robot_move_as robot_move_as_tester`


