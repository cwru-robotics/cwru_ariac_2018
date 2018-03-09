#include<order_manager/order_manager.h>

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "example_inventory_test_main"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    ROS_INFO("main: instantiating an object of type OrderManager");
    OrderManager orderManager(&nh);
    inventory_msgs::Inventory inventory_msg;
    osrf_gear::Order order;
    bool got_order = false;
    bool successfully_filled_order = false;
    while (ros::ok()) {
        orderManager.update_inventory();
        //process all the queued order messages
        for (int i = 0; i < 10; i++) {
            ros::spinOnce();
            //ros::Duration(0.05).sleep();
        }


        //ROS_INFO("trying orderManager.get_inventory()");
        orderManager.get_inventory(inventory_msg);
        ROS_INFO("test_main printing inventory");
        orderManager.print_inventory_succinct(inventory_msg);
        got_order = orderManager.choose_order(order);
        successfully_filled_order = false;
        if (got_order) {
            ROS_INFO_STREAM("selected order: " << order << endl);
            //try to fill order; 
            //successfully_filled_order = orderManager.fill_order(order);
        }


        //if dropped  or faulty parts, retry;
        //if out of parts for this  order, go ahead and ship partial order
        //move conveyor and inform drone

        //if shipped order, do:
        if (successfully_filled_order) {
            orderManager.current_order_has_been_filled();
            //to delete this order from the list;
            //note: if order is not deleted from list, will likely show up again as chosen order
            //  unless, e.g., there are no longer enough parts to fill this order
            // might choose to call orderManager.current_order_has_been_filled(); to eliminate this order, even if not filled
        }

    }

    return 0;

}
