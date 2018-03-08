# order_manager

subscribes to order topic.  queues up orders. 
creates separate array of priority orders
instantiates an inventory object
tests orders in queue against inventory. sends unfillable orders to separate array.
picks a fillable order
steps through components defining pick locations
transforms "place" locations to world coords, based on location of box
steps through robot action-server commands for pick/place


## Example usage
Start ariac:
`rosrun osrf_gear gear.py --visualize-sensor-views -f `catkin_find --share --first-only osrf_gear`/config/quals/qual1a.yaml ~/ariac_ws/ariac-docker/team_config/team_case_config/qual1_config.yaml`
Start order manager:
`rosrun order_manager order_manager_test_main`
Send example order(s):
`rosrun order_sender order_sender`
    
