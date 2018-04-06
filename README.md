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

  <node name="ariac_sim" pkg="osrf_gear" type="gear.py"
        args="--development-mode
          $(arg verbose_args)
          $(arg state_logging_args)
          $(arg gui_args)
          $(arg fill_demo_shipment_args)
          --visualize-sensor-views
          -f $(find osrf_gear)/config/sample.yaml
          $(find osrf_gear)/config/sample_user_config.yaml
          " required="true" output="screen" />
Start up robot move action server:
`rosrun kuka_move_as kuka_behavior_as`

For a simple test that grabs items from inventory, puts them in a box, then discards them, run:
`rosrun shipment_filler test_part_placement_from_inventory`



THE FOLLOWING NEEDS UPDATING...
start the shipment filler:
`rosrun shipment_filler simple_shipment_filler`

Send a test order:
`rosrun order_sender order_sender2`

OR, send orders via competition interface:
`rosservice call /ariac/start_competition`

#sub-tests:



