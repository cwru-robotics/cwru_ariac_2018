# optimizer_func
This is the optimizer service that will select which shipments to work on next.

## Dumb Example
A "dumb" implementation that should tell respond to keep working on the current shipment until it is full then work on the next shipment.

rosrun optimizer_func dumb_optimizer

## Real Optimizer
The real optimizer will be run using the following command.  It functions, but is not the greatest at this point.

rosrun optimizer_func decision_optimizer



