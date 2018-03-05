//OrderManager class
#include<order_filler/order_manager.h>


OrderManager::OrderManager(ros::NodeHandle* nodehandle): nh_(*nodehandle) {
    //set queue size to hold messages 
    order_subscriber_ = nh_.subscribe("/ariac/orders", 10,
            &OrderManager::order_callback, this);
    p_binInventory_ = new BinInventory(&nh_);
    order_is_in_process_=false;
}

//check inventory to see if order is fillable
bool OrderManager::order_is_fillable(osrf_gear::Order order) {
  //start by making up a parts list required for the "order"
    std::vector<int> parts_list;
    int part_id;
    parts_list.resize(NUM_PART_TYPES+1,0); //init part-count=0 to all part types
    int num_shipments = order.shipments.size();
    for (int i_shipment=0;i_shipment<num_shipments;i_shipment++) {
        //walk through all of the parts:
        osrf_gear::Shipment shipment = order.shipments[i_shipment];
        int num_parts_in_order = shipment.products.size();
        for (int i_prod=0;i_prod<num_parts_in_order;i_prod++) {            
            part_id = mappings[shipment.products[i_prod].type];
            parts_list[part_id]++;            
        }
        print_parts_list(parts_list);
    }

 return parts_available(parts_list);
}

//returns an order to be filled
bool OrderManager::choose_order(osrf_gear::Order &order) {
  //put intelligent selection here.  Among orders in queue, pick the "best one to process,
  // including considerations of priority, fillability, 
  //with selection of order, put this order in local var current_order_in_process_
  //also set values for:
  //int current_order_vector_code_;
  //int current_order_index_;
  //order remains in its vector until the user invokes "order_filled()"
    osrf_gear::Order temp_order;
    //examine priority order vector first:
    int num_priority_orders = priority_orders_.size();
    if (num_priority_orders>0) {
        ROS_INFO("returning a priority order");
        //default: choose first priority order:
        current_order_vector_code_ = ORDER_VECTOR_PRIORITY;
        current_order_in_process_ = priority_orders_[0];
        order = current_order_in_process_;
        current_order_index_ = 0;
        order_is_in_process_ = true;
        if (order_is_fillable(current_order_in_process_)) {
            return true; // this order will do--high priority, and fillable
        }
        //not fillable--see if there are any other priority orders that are fillable:
        for (int i = 1;i<num_priority_orders;i++) {
            temp_order = priority_orders_[i];
            if (order_is_fillable(temp_order)) {
                 current_order_vector_code_ = ORDER_VECTOR_PRIORITY;
                 current_order_in_process_ = temp_order;
                 order = current_order_in_process_;
                 current_order_index_ = i;
                 return true;
            }
        }
    }
    //here if no priority orders; 
    int num_pending_orders = pending_orders_.size();
    if (num_pending_orders>0) {
        ROS_INFO("returning a pending order");
        //default: choose first order:
        current_order_vector_code_ = ORDER_VECTOR_PENDING;
        current_order_in_process_ = pending_orders_[0];
        order = current_order_in_process_;
        current_order_index_ = 0;
        order_is_in_process_ = true;
        if (order_is_fillable(current_order_in_process_)) {
            return true; // this order will do--pending and fillable
        }        
        //not fillable--see if there are any other priority orders that are fillable:
        for (int i = 1;i<num_pending_orders;i++) {
            temp_order = pending_orders_[i];
            if (order_is_fillable(temp_order)) {
                 current_order_in_process_ = temp_order;
                 order = current_order_in_process_;
                 current_order_index_ = i;
                 return true;
            }
        }        
    }
    //last option: choose an unfillable order:
    int num_unfillable_orders = unfillable_orders_.size();
    if (num_unfillable_orders>0) {
        ROS_INFO("returning an unfillable order");
        //default: choose first order:
        current_order_vector_code_ = ORDER_VECTOR_UNFILLABLE;
        current_order_in_process_ = unfillable_orders_[0];
        order = current_order_in_process_;
        current_order_index_ = 0;
        //ROS_INFO_STREAM("returning order: "<<unfillable_orders_[0]<<endl);
        //ROS_INFO_STREAM("copy of order: "<<current_order_in_process_<<endl);
        return true; // this order will do--pending and fillable
    }        
    //if here, there are no orders in any vector; return false
     order_is_in_process_ = false;
     ROS_INFO("no orders to be  filled");
     return false;   
}

bool OrderManager::current_order_has_been_filled() {
    //WRITE THIS: remove current order from its queue
    ROS_WARN("current_order_has_been_filled: has not yet been written!");
    switch (current_order_vector_code_) {

       case ORDER_VECTOR_PENDING:            // Note the colon, not a semicolon
           //delete entry current_order_index_ from vector pending_orders_
           return true;
           break;
        case ORDER_VECTOR_PRIORITY:
            //delete entry current_order_index_ from vector priority_orders_
            return true;
            break;
        case ORDER_VECTOR_UNFILLABLE:
            //delete entry current_order_index_ from vector unfillable_orders_
            return true;
            break;
        default:
            ROS_WARN("cannot delete order--something is wrong");
            return false;
    }
}

bool OrderManager::parts_available(std::vector<int> parts_list) {
    ROS_INFO("evaluating availability: ");
    int num_rqd,num_avail;
    for (int part_id=1;part_id<NUM_PART_TYPES;part_id++) {
        num_rqd = parts_list[part_id];
        num_avail = p_binInventory_->num_parts(part_id);
        if (num_rqd>0) {
            ROS_INFO("part_id %d, num_rqd = %d, num_avail = %d",part_id,num_rqd,num_avail);
        }
        if (num_rqd>num_avail) {
            ROS_INFO("can't fill order");
            return false;
        }
    }
    ROS_INFO("able to  fill order");
    return true;
}


void OrderManager::print_parts_list(std::vector<int> parts_list) {
    ROS_INFO("Parts list for order: ");
    for (int i=1;i<NUM_PART_TYPES;i++) {
        ROS_INFO("part_id: %d,   num parts rqd = %d",i,parts_list[i]);
    }
}

void OrderManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    osrf_gear::Order order;
    order = *order_msg;
    if (!order_is_fillable(order)) {
     unfillable_orders_.push_back(order);
    }
    //else, order if fillable
    else if (is_priority(order)) {
         priority_orders_.push_back(order);
     }
    //else, is fillable and is not priority:
    else {
       pending_orders_.push_back(order);
    }
    int n_unfillable = unfillable_orders_.size();
    int n_priority = priority_orders_.size();
    int n_regular = pending_orders_.size();
    ROS_INFO("n_unfillable = %d, n_priority= %d, n_pending = %d",n_unfillable,n_priority,n_regular);
}

bool OrderManager::is_priority(osrf_gear::Order order) {
  //need to fill this out, testing if an order is priority
  return false; //does not recognize priority orders
}

void OrderManager::move_order_to_unfillable(int order_num,std::vector<osrf_gear::Order> order_vec) {
  //write  me!
}

geometry_msgs::PoseStamped OrderManager::target_pose_to_world_coords(geometry_msgs::Pose part_pose_wrt_box, 
     geometry_msgs::PoseStamped box_pose_wrt_world) {

   geometry_msgs::PoseStamped target_part_pose_wrt_world;
   //write xform code here:

   return target_part_pose_wrt_world;
}

 void OrderManager::update_inventory() {
   p_binInventory_->update();
}
 
 //broken...
 void OrderManager::print_inventory(std::vector<PartInventory> inventory) {
    ROS_INFO("printing inventory");
    for (int part_id=1;part_id<NUM_PART_TYPES+1;part_id++) {
        int num_parts = inventory_[part_id].bins.size(); //same as part_stamped_poses.size()
        ROS_INFO("part_id = %d; num parts = %d",part_id,num_parts);
        for (int i=0;i<num_parts;i++) {
            ROS_INFO_STREAM("ipart: "<<i<<" pose: "<<inventory_[part_id].part_stamped_poses[i]<<endl);
        } 
    }
    ROS_INFO("  ");
    
}
 
void OrderManager::get_inventory(std::vector<PartInventory> &inventory) {
    p_binInventory_->get_inventory(inventory); //does not work
    print_inventory(inventory); //this crashes!!
}
  

