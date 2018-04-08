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

or, for qual2 setup:

rosrun osrf_gear gear.py --visualize-sensor-views -f `catkin_find --share --first-only osrf_gear`/config/sample.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml

ON ATLAS6 USE:
rosrun osrf_gear gear.py --visualize-sensor-views -f `catkin_find --share --first-only osrf_gear`/config/sample.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml

Qualifier 2:
4 piston rods in bin1, 2 flanges in bin2, 2 disks in bin3, bin4 is empty, 3 gears in bin5
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2a.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

The following ariac config files present different scenarios:
sample_dropped_products.yaml, sample_flipped.yaml, sample_interruption1.yaml, sample_interruption2.yaml, 
sample_not_enough_products.yaml, sample_order_update.yaml, sample_sensor_blackout.yaml, sample.yaml

Run these variations, e.g., on Ava (wsn laptop) with:

The following setup is trouble--gears close together, and piston rods in bin5 (not ready for bin5)
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_dropped_products.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

The following has pulley parts--not ready to do pulley flips.  Also has piston-rod parts in bin5
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_interruption1.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

The following has no pulleys and no parts in bin5, but gear parts are very close together:
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_interruption2.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

The following has piston-rod parts in bin5; the rest are disk parts in bins 1 and 2
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_not_enough_products.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

order-update: order change during filling!   has tightly-spaced gears, pulleys, and bin5 piston rods
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_order_update.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

sample blackout: has piston rods in bin2, gears in bin3 and flanges in bin4
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_sensor_blackout.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

sample: 
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`



ON ATLAS6:
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2a.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_dropped_products.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_interruption1.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_interruption2.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_not_enough_products.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_order_update.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample_sensor_blackout.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`

`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/sample.yaml ~/ros_ws/ariac-docker/team_config/team_case_config/qual2_config.yaml`


Start up robot move action server:
`rosrun kuka_move_as kuka_behavior_as`

start the shipment filler:
`rosrun shipment_filler simple_shipment_filler`

For isolated tests, e.g.
For a simple test that grabs items from inventory, puts them in a box, then discards them, run:
`rosrun shipment_filler test_part_placement_from_inventory`

Send a test order:
`rosrun order_sender order_sender2`

OR, send orders via competition interface:
`rosservice call /ariac/start_competition`



