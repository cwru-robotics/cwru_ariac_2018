# order_filler

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
`roslaunch osrf_gear sample_environment.launch`
Start order manager:
`rosrun order_filler order_manager_test_main`
Send example order(s):
`rosrun order_sender order_sender`

## Running tests/demos
    
