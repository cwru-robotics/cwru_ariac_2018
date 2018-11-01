//
//  wsn 3/2018
//

#include <robot_behavior_interface/RobotBehaviorInterface.h>
RobotBehaviorInterface::RobotBehaviorInterface(ros::NodeHandle* nodehandle) : nh_(*nodehandle),ac("robot_behavior_server", true) {
    ROS_INFO("constructor of RobotBehaviorInterface");

    errorCodeFinder.insert(pair<int8_t, string>(robot_behavior_interface::RobotBehaviorResult::NO_ERROR, "NO_ERROR"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_behavior_interface::RobotBehaviorResult::CANCELLED, "CANCELLED"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_behavior_interface::RobotBehaviorResult::WRONG_PARAMETER, "WRONG_PARAMETER"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_behavior_interface::RobotBehaviorResult::TIMEOUT, "TIMEOUT"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_behavior_interface::RobotBehaviorResult::UNREACHABLE, "UNREACHABLE"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_behavior_interface::RobotBehaviorResult::GRIPPER_FAULT, "GRIPPER_FAULT"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_behavior_interface::RobotBehaviorResult::COLLISION, "COLLISION"));
    errorCodeFinder.insert(pair<int8_t, string>(robot_behavior_interface::RobotBehaviorResult::PART_DROPPED, "PART_DROPPED"));  
    errorCodeFinder.insert(pair<int8_t, string>(robot_behavior_interface::RobotBehaviorResult::PRECOMPUTED_TRAJ_ERR, "PRECOMPUTED_TRAJ_ERR"));  


    ROS_INFO("waiting for robot_behavior_server: ");
    bool server_exists = ac.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting robot_behavior_server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = ac.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to robot_behavior_server"); // if here, then we connected to the server;   

    //gripper = nh.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    //gripperStateSubscriber = nh.subscribe("/ariac/gripper/state", 10, &RobotInterface::gripperStateCallback, this);
    /*
    called = false;
    attached = false;
    while(!called && ros::ok()) {
        //ROS_INFO("Waiting for joint feedback...");
        ros::spinOnce();
        ros::Duration(0.02).sleep();
    }
    if (!gripper.exists()) {
        gripper.waitForExistence();
    }
    attach.request.enable = 1;
    detach.request.enable = 0;
    arrivalTime = 0.5;
     * */
}
//    return( sendGoal(goal_type,part,timeout));
bool RobotBehaviorInterface::sendGoal(unsigned short int goal_type, double timeout) {
    behaviorServerGoal_.type = goal_type;
    behaviorServerGoal_.timeout = timeout;
    goal_start_time_ = ros::Time::now();    
    action_server_returned_ = false;
    ac.sendGoal(behaviorServerGoal_, boost::bind(&RobotBehaviorInterface::doneCb, this, _1, _2), boost::bind(&RobotBehaviorInterface::activeCb, this), boost::bind(&RobotBehaviorInterface::feedbackCb, this, _1));
    //BLOCKING!!
    return(wrap_up());    
}

//    bool sendGoal(unsigned short int goal_type, Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME);

bool RobotBehaviorInterface::sendGoal(unsigned short int goal_type, Part part, double timeout) {
    behaviorServerGoal_.type = goal_type;
    behaviorServerGoal_.sourcePart = part;
    behaviorServerGoal_.destinationPart = part; //redundant...but some fncs look at "destination" part
    behaviorServerGoal_.timeout = timeout;
    goal_start_time_ = ros::Time::now();    
    action_server_returned_ = false;
    ac.sendGoal(behaviorServerGoal_, boost::bind(&RobotBehaviorInterface::doneCb, this, _1, _2), boost::bind(&RobotBehaviorInterface::activeCb, this), boost::bind(&RobotBehaviorInterface::feedbackCb, this, _1));
    //BLOCKING!!
    return(wrap_up());    
}

bool RobotBehaviorInterface::sendGoal(unsigned short int goal_type, Part sourcePart, Part destinationPart,double timeout) {
    behaviorServerGoal_.type = goal_type;
    behaviorServerGoal_.sourcePart = sourcePart;
    behaviorServerGoal_.destinationPart = destinationPart; //redundant...but some fncs look at "destination" part
    behaviorServerGoal_.timeout = timeout;
    goal_start_time_ = ros::Time::now();    
    action_server_returned_ = false;
    ac.sendGoal(behaviorServerGoal_, boost::bind(&RobotBehaviorInterface::doneCb, this, _1, _2), boost::bind(&RobotBehaviorInterface::activeCb, this), boost::bind(&RobotBehaviorInterface::feedbackCb, this, _1));
    //BLOCKING!!
    return(wrap_up());    
}

void RobotBehaviorInterface::doneCb(const actionlib::SimpleClientGoalState &state, const robot_behavior_interface::RobotBehaviorResultConstPtr &result) {
    action_server_returned_ = true;
    goal_success_ = result->success;
    errorCode_ = result->errorCode;
    //currentRobotState = result->robotState;
    //error_code_name_map
    //    ROS_INFO("attempting move from %s  to %s ", map_pose_code_to_name[posecode_start].c_str(),
    //         map_pose_code_to_name[posecode_goal].c_str());
    ROS_INFO("Action finished  with result error code: %s",  errorCodeFinder[errorCode_].c_str());
//    ROS_INFO("Gripper position is: %f, %f, %f\n",
//             currentRobotState.gripperPose.pose.position.x, currentRobotState.gripperPose.pose.position.y,
//             currentRobotState.gripperPose.pose.position.z);
//    showJointState(currentRobotState.jointNames, currentRobotState.jointStates);
}

void RobotBehaviorInterface::feedbackCb(const robot_behavior_interface::RobotBehaviorFeedbackConstPtr &feedback) {
    //do nothing
    //currentRobotState = feedback->robotState;
}

void RobotBehaviorInterface::cancel() {
    ac.cancelGoal();
}

void RobotBehaviorInterface::activeCb() {
    // ROS_INFO("Goal sent");
}

bool RobotBehaviorInterface::test_pick_part_from_bin(Part part, double timeout) {
    ROS_INFO("test pick fnc called");
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::TEST_PICK_PART_FROM_BIN;
    return( sendGoal(goal_type,part,timeout));    
}

    
//blocking function!!
bool RobotBehaviorInterface::pick_part_from_bin(Part part, double timeout) { 
    ROS_INFO("pick fnc called");
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::PICK_PART_FROM_BIN;
    return( sendGoal(goal_type,part,timeout));
}
//    bool discard_grasped_part(double timeout=0);

//    bool discard_grasped_part(Part part,double timeout = MAX_ACTION_SERVER_WAIT_TIME);
bool RobotBehaviorInterface::discard_grasped_part(inventory_msgs::Part part, double timeout) {
    ROS_INFO("discard fnc called");
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::DISCARD_GRASPED_PART_Q1;   
    return( sendGoal(goal_type,part,timeout));
}

bool RobotBehaviorInterface::place_part_in_box_no_release(Part part,double timeout) {
    ROS_INFO("place_part_in_box_no_release fnc called");
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::PLACE_PART_IN_BOX_NO_RELEASE;   
    return( sendGoal(goal_type,part,timeout));    
}

bool RobotBehaviorInterface::evaluate_key_pick_and_place_poses(Part sourcePart, Part destinationPart, double timeout) {
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::EVALUATE_KEY_PICK_AND_PLACE_POSES; 
    return(sendGoal(goal_type, sourcePart, destinationPart,  timeout));
}


bool RobotBehaviorInterface::re_evaluate_approach_and_place_poses(Part sourcePart, Part destinationPart, double timeout) {
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::RE_EVALUATE_APPROACH_AND_PLACE_POSES;   
    return(sendGoal(goal_type, sourcePart, destinationPart,  timeout));
}
bool RobotBehaviorInterface::place_part_in_box_from_approach_no_release(Part part, double timeout) {
    ROS_INFO("place_part_in_box_from_approach_no_release  fnc called");
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::PLACE_PART_IN_BOX_FROM_APPROACH_NO_RELEASE;   
    return( sendGoal(goal_type,part,timeout));      
}

bool RobotBehaviorInterface::move_part_to_approach_pose(inventory_msgs::Part part,double timeout) {
    ROS_INFO("move grasped part to approach pose");
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::MOVE_GRASPED_PART_TO_APPROACH_POSE;   
    return( sendGoal(goal_type,part,timeout));       
} 


bool RobotBehaviorInterface::place_part_in_box_with_release(Part part,double timeout) {
    ROS_INFO("place_part_in_box_with_release fnc called");
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::PLACE_PART_IN_BOX_WITH_RELEASE;   
    return( sendGoal(goal_type,part,timeout));    
}
    

bool RobotBehaviorInterface::adjust_part_location_no_release(Part sourcePart, Part destinationPart, double timeout) {
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::ADJUST_PART_LOCATION;   
    return(sendGoal(goal_type, sourcePart, destinationPart,  timeout)); 
}

bool RobotBehaviorInterface::adjust_part_location_with_release(Part sourcePart, Part destinationPart, double timeout) {
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::ADJUST_PART_LOCATION_WITH_RELEASE;   
    return(sendGoal(goal_type, sourcePart, destinationPart,  timeout)); 
}

bool RobotBehaviorInterface::release(double timeout)  {
     ROS_INFO("release fnc called");
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::RELEASE;
    return( sendGoal(goal_type,timeout));    
}

bool RobotBehaviorInterface::release_and_retract(double timeout) {
     ROS_INFO("release_and_retract called");
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::RELEASE_AND_RETRACT;
    return( sendGoal(goal_type,timeout));      
}


bool RobotBehaviorInterface::pick_part_from_box(Part part, double timeout) {
    ROS_INFO("pick_part_from_box fnc called");
    short unsigned  int goal_type = robot_behavior_interface::RobotBehaviorGoal::PICK_PART_FROM_BOX;   
    return( sendGoal(goal_type,part,timeout));    
}


/*

osrf_gear::VacuumGripperState RobotInterface::getGripperState() {
    ros::spinOnce();
    return currentGripperState;
}

bool RobotInterface::isGripperAttached() {
    ros::spinOnce();
    return attached;
}

bool RobotInterface::waitForGripperAttach(double timeout) {
    timeout = timeout <= 0? FLT_MAX:timeout;
    ros::spinOnce();
    while((!attached) && timeout > 0 && ros::ok()) {
        ROS_INFO("Retry grasp");
        release();
        ros::Duration(0.2).sleep();
        ros::spinOnce();
        grab();
        ros::Duration(0.8).sleep();
        ros::spinOnce();
        timeout -= 1.0;
    }
    return attached;
}




void RobotInterface::grab() {
    //ROS_INFO("enable gripper");
    gripper.call(attach);
}

void RobotInterface::release() {
    //ROS_INFO("release gripper");
    gripper.call(detach);
}

 * */
bool RobotBehaviorInterface::wrap_up() {
    double dt = 0.1;
    int pose_count=0;
    while (!action_server_returned_) {
        pose_count++;
        if (pose_count%10==0) {
          ROS_INFO("waiting on action  server");
        }
        ros::Duration(dt).sleep();
        ros::spinOnce();
    }
    if (errorCode_==robot_behavior_interface::RobotBehaviorResult::NO_ERROR) {
      return true;
    }
    return false;
}
