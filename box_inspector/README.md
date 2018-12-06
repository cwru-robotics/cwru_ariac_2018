# box_inspector
Code extracted from ARIAC competition to develop box-inspection library.
Given a desired shipment, consisting of a list of parts and their desired poses with respect to a shipping box,
use an overhead (logical) camera to compare desired to inspection.  Parse the sensed objects in terms of:
parts that are in the requested shipment, are in the box, and are precisely located;
parts that are in the requested  shipment, are in the box, but are misplaced (separately list the actual coords and desired coords);
parts that are missing from the box;
parts that are in the box that don't belong there (not in the requested shipment)

Box inspection depends on receiving a shipment specification and receiving sensor data.
The box_inspection library is instantiated  and used in an example main program.
The shipment is published by a separate (hard-coded) node, and a fake camera image is published by a separate node.

## Example usage
Start the box inspector node:
`rosrun box_inspector box_inspector_example_main`
Publish an order to be filled, by running:
`rosrun box_inspector order_sender`
Publish fake camera images with:
`rosrun box_inspector fake_logical_camera`

Or, start with ariac test version with partially loaded box:
`roslaunch cwru_ariac_launch sample_environment.launch fill_demo_shipment:=true`
manually start the competition:
`rosservice call /ariac/start_competition`
start up servers (in separate  windows):
`rosrun kuka_move_as kuka_behavior_as2`
`rosrun conveyor_as conveyor_as`
start a simple demo node to advance box to Q1 station:
`rosrun shipment_filler unload_box`

Run a box-inspector demo:
`rosrun box_inspector box_inspector_example_main`
this will show one defective part (piston rod) and two desired  parts (gears), but slightly dislocated


new 12/16/2018: created boxInspector2: extends inspection to station Q2

## Running tests/demos
    
