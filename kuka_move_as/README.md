# kuka_move_as

Kuka robot action server to abstract behaviors.

Helper library: TransitionTrajectories.  This library precomputes safe and efficient moves from specified start location code
to specified goal location code.  E.g., Q1 station hover to Bin3 hover;



## Example usage

## Running tests/demos
test pgm for transition trajectories:
`rosrun kuka_move_as transition_trajectories_test_main`

Start of kuka_move_as:
start up ariac simu, then:
`rosrun kuka_move_as kuka_behavior_as`

does robot init move; need to build out all behaviors in this action server, as well as complementary RobotBehaviorInterface 

Example main using RobotBehaviorInterface:
`rosrun kuka_move_as robot_behavior_test_main`


TODO:  add more trajectories to transitionTrajectories.  
extend test fnc to run/eval trajectories in simu
    
