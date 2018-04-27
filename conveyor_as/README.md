# conveyor_as
This package defines a conveyor action server and a complementary interface library (class).  One can invoke conveyor
behaviors by instantiating an object of type ConveyorInterface (as illustrated in the simple example "example_conveyor_interface_main"),
then invoking member functions of this class (also, as illustrated in the example main).  Example functions include:
"move_new_box_to_Q1()", "move_box_Q1_to_Q2()" and "move_box_Q2_to_drone_depot()".  These are NOT blocking functions.

The action server "conveyor_as" should be started first, then it can be sent conveyor goals (as simplified through the
conveyorInterface class).  This allows the conveyor to be controlled as a concurrent process.

The status of the conveyor must be polled.  E.g., if the conveyor interface object is called "conveyorInterface", then
the action server has reached its goal when the function conveyorInterface.action_server_returned() returns true.  The result status
of the action server can be examined with: conveyorInterface.get_box_status(), which describes the result status through
result codes defined in the conveyor_as package action message.


The action server also
provides feedback, including the estimated remaining time to reach the goal, and the positions (distance from box
dispenser) of up to three boxes.  (box1 is the lead box, and boxes 2 and 3 trail, if they exist.  Distance is negative
if these  do not exist).

The estimated time to task completion can be found by examining  conveyorInterface.estimated_seconds_to_goal;

The action server keeps track of the box positions.

If the sensors black out, the action server will continue by using dead reckoning to reach the specified goal state.
If the sensors are blacked out, a feedback callback will be informed, and this can be tested by examining:
conveyorInterface.sensors_are_active;



## Example usage
start up ariac simu, e.g.:
`rosrun osrf_gear gear.py -f `catkin_find --share --first-only osrf_gear`/config/quals/qual2b.yaml ~/ariac_ws/src/cwru_ariac_2018/copy_of_team_config/depthcam_config.yaml`

enable the conveyor by starting the competition:
`rosservice call /ariac/start_competition`
(note: can test conveyor OK with: rosservice call /ariac/conveyor/control "power: 100")

`rosrun conveyor_as conveyor_as`
Then run a node that contains an object of type ConveyorInterface.

## Running tests/demos
`rosrun conveyor_as example_conveyor_interface_main`
    
