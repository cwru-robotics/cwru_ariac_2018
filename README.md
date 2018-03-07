# cwru_ariac_2018
for NIST ARIAC competition

example:
Start up demo simulation:
`roslaunch osrf_gear sample_environment.launch`
better:
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual1a.yaml ~/ariac_ws/ariac-docker/team_config/team_case/qual1_config.yaml`

Start up robot move action server:
`rosrun robot_move_as robot_move_as`

Run an example move-part command (hard coded):
`rosrun robot_move_as robot_move_as_tester`

