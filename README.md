# cwru_ariac_2018
for NIST ARIAC competition

Start up one of the simulation scenarios (choose 1 from below):

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2a.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2b.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_dropped_products.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_interruption1.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_interruption2.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_not_enough_products.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_order_update.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_sensor_blackout.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

Start up 4 of our nodes:

Start up robot move action server:
`rosrun kuka_move_as kuka_behavior_as2`

Start up the conveyor action server:
`rosrun conveyor_as conveyor_as`

Start up the shipment selection service (this will get changed to a new name/version):
`rosrun optimizer_func dumb_optimizer`

Start up the  new shipment filler (top-level node):
`rosrun shipment_filler simple_shipment_filler_w_optimizer`


##tests:
see README in shipment_filler for example demo_order_filler


