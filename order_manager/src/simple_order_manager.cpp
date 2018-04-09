//OrderManager class
#include<order_manager/order_manager.h>


OrderManager::OrderManager(ros::NodeHandle* nodehandle): nh_(*nodehandle) {
    //set queue size to hold messages 
    order_subscriber_ = nh_.subscribe("/ariac/orders", 10,
            &OrderManager::order_callback, this);
    p_binInventory_ = new BinInventory(&nh_);
    order_is_in_process_=false;
    shipment_is_in_process_=false;
}

void OrderManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    //ROS_INFO("received order %s",);
    osrf_gear::Order order;
    order = *order_msg;
    mark_shipments_unfilled(order); //make sure all shipments are initially labelled as unfilled
    if (is_priority(order)) {
         priority_orders_.push_back(order);
     }   
    //else if (!order_is_fillable(order)) {
    // unfillable_orders_.push_back(order);
    //}
    //else, is fillable and is not priority:
    else {
       pending_orders_.push_back(order);
    }
    int n_unfillable = unfillable_orders_.size();
    int n_priority = priority_orders_.size();
    int n_regular = pending_orders_.size();
    ROS_INFO("order queue status: n_unfillable = %d, n_priority= %d, n_pending = %d",n_unfillable,n_priority,n_regular);
}

//I am using shipment.shipment_type to mark shipments as pending or fulfilled.  Need to  make sure incoming orders
// have placed a string here (any string will do).  If  not, PUT a string there.
void OrderManager::mark_shipments_unfilled(osrf_gear::Order &order) {
    int num_shipments = order.shipments.size();
    for (int i_shipment=0;i_shipment<num_shipments;i_shipment++) {
        string shipment_label(order.shipments[i_shipment].shipment_type);
        if (shipment_label.size()<1) {
            ROS_WARN("incoming shipment unlabelled; assigning label");
            order.shipments[i_shipment].shipment_type = "dummy";
        }
    }
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
    }
    print_parts_list(parts_list);
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
    ROS_INFO("trying to select an order to be filled...");
    int n_unfillable = unfillable_orders_.size();
    int n_priority = priority_orders_.size();
    int n_regular = pending_orders_.size();
    ROS_INFO("order queue status: n_unfillable = %d, n_priority= %d, n_pending = %d",n_unfillable,n_priority,n_regular);    
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

//# Shipment message
//# This structure contains the information of shipment.

//# The type of shipment
//string shipment_type

//# Collection of products
//Product[] products



bool OrderManager::choose_shipment(osrf_gear::Shipment &shipment) {
    osrf_gear::Order temp_order;

    ROS_INFO("choose_shipment: start by choosing an order");
    if (!choose_order(temp_order))  return false;
    //choose a shipment within this order:
    int num_shipments = temp_order.shipments.size();
    ROS_INFO("chosen order has %d shipments",num_shipments);
    for (int i_shipment=0;i_shipment< num_shipments;i_shipment++) {
        string filled_code(temp_order.shipments[i_shipment].shipment_type);
        int string_size = filled_code.size();
        ROS_INFO("shipment %d has string length %d: %s",i_shipment,string_size,filled_code.c_str());
        if (string_size>0) {
            ROS_INFO("choosing this  shipment");
            shipment = temp_order.shipments[i_shipment];
            ROS_INFO_STREAM("shipment: "<<shipment<<endl);
            current_shipment_index_= i_shipment;
            return true;
        }
    }
    ROS_WARN("choose_shipment()  error--found no unfilled  shipments in order");
    return false;
}

bool OrderManager::current_order_has_been_filled() {
    //WRITE THIS: remove current order from its queue
    ROS_WARN("deleting filled order");
    switch (current_order_vector_code_) {

       case ORDER_VECTOR_PENDING:            // Note the colon, not a semicolon
           //delete entry current_order_index_ from vector pending_orders_
           delete_from_order_queue(current_order_index_, pending_orders_);
           order_is_in_process_ = false;
           break;
        case ORDER_VECTOR_PRIORITY:
            //delete entry current_order_index_ from vector priority_orders_
           delete_from_order_queue(current_order_index_, priority_orders_);
           order_is_in_process_ = false;            
            break;
        case ORDER_VECTOR_UNFILLABLE:
            //delete entry current_order_index_ from vector unfillable_orders_
           delete_from_order_queue(current_order_index_, unfillable_orders_);
           order_is_in_process_ = false;                 
            break;
        default:
            ROS_WARN("cannot delete order--something is wrong");
            return false;
    }
    int n_unfillable = unfillable_orders_.size();
    int n_priority = priority_orders_.size();
    int n_regular = pending_orders_.size();
    ROS_INFO("order queue status: n_unfillable = %d, n_priority= %d, n_pending = %d",n_unfillable,n_priority,n_regular);    
}

bool OrderManager::current_shipment_has_been_filled() {
    ROS_WARN("marking current shipment filled");
    switch (current_order_vector_code_) {

       case ORDER_VECTOR_PENDING:            
           mark_shipment_filled(current_order_index_, current_shipment_index_, pending_orders_);
           test_order_complete(current_order_index_, current_shipment_index_, pending_orders_);
           shipment_is_in_process_ = false;
           break;
        case ORDER_VECTOR_PRIORITY:
           mark_shipment_filled(current_order_index_, current_shipment_index_, priority_orders_);    
           test_order_complete(current_order_index_, current_shipment_index_, priority_orders_);
           shipment_is_in_process_ = false;            
            break;
        case ORDER_VECTOR_UNFILLABLE:
           mark_shipment_filled(current_order_index_, current_shipment_index_, unfillable_orders_);           
           test_order_complete(current_order_index_, current_shipment_index_, unfillable_orders_);
           shipment_is_in_process_ = false;                 
            break;
        default:
            ROS_WARN("cannot delete shipment--something is wrong w/ current_order_vector_code");
            return false;
    }    
}

//examine order, described by current_order_vector_code_, current_order_index_, current_shipment_index_
//if all  shipments are complete, delete the order
//examine current_order_in_process_
void OrderManager::test_order_complete(int current_order_index, int current_shipment_index, std::vector<osrf_gear::Order> &order_vec) {
    ROS_INFO("testing if order is complete");
    osrf_gear::Order current_order;
    current_order = order_vec[current_order_index];
    int num_shipments = current_order.shipments.size();
    for (int i_shipment=0;i_shipment<num_shipments;i_shipment++) {
        string filled_code(current_order.shipments[i_shipment].shipment_type);
        if (filled_code.size()>0) {
            ROS_INFO("shipment %d code is %s; still needs to be fulfilled",i_shipment,filled_code.c_str());
            return;//found an unfilled  shipment
        }
    }
    //if survive  to here, all shipments in order are marked filled (unnamed), so delete  this order
    ROS_INFO("current order is complete");
    current_order_has_been_filled();
}

bool OrderManager::delete_from_order_queue(int order_index, std::vector<osrf_gear::Order> &order_vec)  {
    //e.g., myvector.erase (myvector.begin()+5);
    order_vec.erase(order_vec.begin()+order_index); 
}

bool OrderManager::mark_shipment_filled(int order_index, int shipment_index, std::vector<osrf_gear::Order> &order_vec)  {
    ROS_INFO("marking shipment %d of order %d filled",shipment_index,order_index);
    order_vec[order_index].shipments[current_shipment_index_].shipment_type = "";
}

bool OrderManager::parts_available(std::vector<int> parts_list) {
    ROS_INFO("evaluating availability: ");
    std::vector<int> parts_counts;
    p_binInventory_->counts_all_part_types(parts_counts);

    int num_rqd,num_avail;
    for (int part_id=1;part_id<NUM_PART_TYPES;part_id++) {
        num_rqd = parts_list[part_id];
        num_avail = parts_counts[part_id]; //p_binInventory_->num_parts(part_id);
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
        if (parts_list[i]>0) {
           ROS_INFO("part_id: %d,   num parts rqd = %d",i,parts_list[i]);
        }
    }
}



bool OrderManager::is_priority(osrf_gear::Order order) {
  //if order is currently in process, and new order is received, then new order is priority
  return  order_is_in_process_;
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
 

 //get_inventory(inventory_msgs::Inventory &inventory_msgs);
void OrderManager::get_inventory(inventory_msgs::Inventory &inventory_msg) {
    p_binInventory_->get_inventory(inventory_msg);
}

void OrderManager::print_inventory(inventory_msgs::Inventory inventory_msg) {
    ROS_INFO_STREAM("inventory: "<<inventory_msg<<endl);
}

void OrderManager::print_inventory_succinct(inventory_msgs::Inventory inventory_msg) {
    int num_parts;
    //ROS_INFO("inventory summary: ");
    for (int part_id=1;part_id<NUM_PART_TYPES+1;part_id++) {
        num_parts = inventory_msg.inventory[part_id].bins.size();
        if (num_parts>0) {
            ROS_INFO("there are %d instances of part_id %d = %s",num_parts,part_id,part_id_to_name_mappings[part_id].c_str());  
        }
    }
}

bool OrderManager::find_part(std::string part_name,int &bin_num,geometry_msgs::PoseStamped &part_pose) {
    //Use the BinInventory object to find part in inventory
    return p_binInventory_->find_part(part_name,bin_num,part_pose);
}

  

