//  wsn 3/30/2018
//

//this node presents an action server called "robot_behavior_server"
//it receives high-level behavior goals from an action client (see RobotBehaviorInterface library for fncs that
// bundle behavior codes into goals and send them to this behavior server)

//this node has an action client that sends goals to the Kuka trajectory action server to invoke motions
// it also communicates with the vacuum gripper controls
//This node is responsible for invoking safe motions safe trajectories to perform Part manipulations
// "Part" objects are defined in inventory_msgs, and are specified as part of the goal message to this action server

// a client of this server (via a RobotBehaviorInterface object) refers ONLY to parts,  desired actions on parts, and
// spatial coords w/rt the world

//this action server is specialized for the kuka robot
// the RobotBehaviorInterface is robot agnostic.



#include <kuka_move_as/KukaBehaviorActionServer.h>
int g_ans; //for cout breakpoint debugging
  
//define these funcs in separate files
#include "pick_part_fnc.cpp"
#include "compute_key_poses.cpp"
#include "place_part_fnc.cpp"
#include "test_pick_part_fnc.cpp"
#include "test_pick_part_fnc_bin5.cpp"

//#include "grasp_and_release_fncs.cpp"
/*
#include "flip_part_fnc.cpp"

#include "place_part_fnc_no_release.cpp"
#include "move_part.cpp"
#include "set_key_poses.cpp"


 */

//CONSTRUCTOR:
//this node presents an action server, and it also owns an action client;
// incoming goals are behaviors to be invoked, and outgoing goals are trajectories to the Kuka robot

KukaBehaviorActionServer::KukaBehaviorActionServer(ros::NodeHandle nodeHandle, string topic) :
nh(nodeHandle), robot_behavior_as(nh, topic, boost::bind(&KukaBehaviorActionServer::executeCB, this, _1), false),
traj_ctl_ac_("/ariac/arm/follow_joint_trajectory", true),gripperInterface_(nh) {
    isPreempt_ = false;
    traj_goal_complete_ = false;
    //populate code mappings
    placeFinder_.insert(pair<int8_t, string>(Part::BIN1, "BIN1"));
    placeFinder_.insert(pair<int8_t, string>(Part::BIN2, "BIN2"));
    placeFinder_.insert(pair<int8_t, string>(Part::BIN3, "BIN3"));
    placeFinder_.insert(pair<int8_t, string>(Part::BIN4, "BIN4"));
    placeFinder_.insert(pair<int8_t, string>(Part::BIN5, "BIN5"));
    placeFinder_.insert(pair<int8_t, string>(Part::BIN6, "BIN6"));
    placeFinder_.insert(pair<int8_t, string>(Part::BIN7, "BIN7"));
    placeFinder_.insert(pair<int8_t, string>(Part::BIN8, "BIN8"));
    placeFinder_.insert(pair<int8_t, string>(Part::QUALITY_SENSOR_1, "QUALITY_SENSOR_1"));
    //placeFinder_.insert(pair<int8_t, string>(Part::CAMERA, "CAMERA"));
    placeFinder_.insert(pair<int8_t, string>(Part::QUALITY_SENSOR_2, "QUALITY_SENSOR_2"));   
    
    approach_dist_ = APPROACH_OFFSET_DIST;  //arbitrary, tunable
    depart_dist_ = DEPART_OFFSET_DIST;
    deep_grasp_dist_ = DEEP_GRASP_MOVE_DIST;  //ditto
    
    robot_behavior_as.registerPreemptCallback(boost::bind(&KukaBehaviorActionServer::preemptCB, this));

    // attempt to connect to the Kuka robot-motion action server:
    ROS_INFO("waiting for arm server: ");
    bool server_exists = traj_ctl_ac_.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = traj_ctl_ac_.waitForServer(ros::Duration(1.0));
    }
    ROS_INFO("connected to arm action server"); // if here, then we connected to the server;   

    trajectory_msgs::JointTrajectory transition_traj;
    // new code:
    ROS_INFO("moving to init pose");
    if (!move_posecode1_to_posecode2(HOME_POSE_CODE, INIT_POSE_CODE)) {
        
        ROS_WARN("error with initial move! quitting");
        exit(1);
    }
    //here are key poses, re-expressed as Eigen-type vecs
    nom_bin_cruise_pose_.resize(NDOF);
    q1_cruise_pose_.resize(NDOF);
    q1_hover_pose_.resize(NDOF);
    q2_hover_pose_.resize(NDOF);
    for (int i=0;i<NDOF;i++) {
        nom_bin_cruise_pose_[i] = NOM_BIN_CRUISE_array[i];
        q1_cruise_pose_[i] = Q1_CRUISE_array[i];
        q1_hover_pose_[i] = Q1_HOVER_array[i];
        q2_hover_pose_[i] = Q2_HOVER_array[i];        
    }
    joint_state_vec_.resize(8);
    jointstate_subscriber_  =  nh.subscribe("/ariac/joint_states", 1, &KukaBehaviorActionServer::jointstateCB,this); 
    
    //set_key_poses();
    bad_state_ = string("ABORTED");

    robot_behavior_as.start();
    ROS_INFO("Start Robot Behavior Action Server");
    ROS_INFO_STREAM("robot behavior action server is ready!" << endl << endl);
}

void KukaBehaviorActionServer::trajDoneCb_(const actionlib::SimpleClientGoalState& state,
        const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    //ROS_INFO(" trajDoneCb: server responded with state [%s]", state.toString().c_str());
    
    string succeeded("SUCCEEDED");
    rtn_state_ = state.toString();
    if (bad_state_==rtn_state_) {
        ROS_WARN("TRAJ RETURNED ABORTED!!");
        int ans;
        //cout<<"enter  1 to continue: ";
        //cin>>ans;
    }
    if (succeeded==rtn_state_) {
        ROS_INFO("traj returned succeeded");
    }

    traj_goal_complete_ = true;
}

bool KukaBehaviorActionServer::send_traj_goal(trajectory_msgs::JointTrajectory des_trajectory, int destination_code) {
    robot_goal_.trajectory = des_trajectory;
    //ROS_INFO("sending goal to arm: ");
    traj_goal_complete_ = false;
    ROS_INFO_STREAM("sending traj goal: "<<endl<<robot_goal_<<endl);
    traj_ctl_ac_.sendGoal(robot_goal_, boost::bind(&KukaBehaviorActionServer::trajDoneCb_, this, _1, _2));
    int print_count=0;
    while (!traj_goal_complete_) { //write a fnc for this: wait_for_goal_w_timeout
        //put timeout here...   possibly return falses
        print_count++;
        if (print_count%10==0) {
           ROS_INFO("waiting for trajectory to finish...");
        }
        ros::Duration(0.1).sleep();
    }
    current_pose_code_ = destination_code;
    return true;
}

/*
bool KukaBehaviorActionServer::send_traj_goal(trajectory_msgs::JointTrajectory des_trajectory) {
        robot_goal_.trajectory = des_trajectory;
      //ROS_INFO("sending goal to arm: ");
        traj_goal_complete_=false;
        traj_ctl_ac_.sendGoal(robot_goal_, boost::bind(&KukaBehaviorActionServer::trajDoneCb_, this, _1, _2));

      while (!traj_goal_complete_) { //write a fnc for this: wait_for_goal_w_timeout
        //put timeout here...   possibly return falses
        ROS_INFO("waiting for trajectory to finish...");
        ros::Duration(0.1).sleep();
       }       
       //current_pose_code_ = destination_code;
       return true;
}*/
bool KukaBehaviorActionServer::send_traj_goal(trajectory_msgs::JointTrajectory des_trajectory) {
    robot_goal_.trajectory = des_trajectory;
    //ROS_INFO("sending goal to arm: ");
    traj_goal_complete_ = false;
    traj_ctl_ac_.sendGoal(robot_goal_, boost::bind(&KukaBehaviorActionServer::trajDoneCb_, this, _1, _2));
    return true; //should test this
}

bool KukaBehaviorActionServer::try_recover_from_abort(Eigen::VectorXd q_vec, double tolerance) {
   ROS_WARN("trying to recover from ABORT");
   int max_retries = 3;
   int n_retries = 0;
   get_fresh_joint_states(); // update joint_state_vec_
   double jspace_err_norm = (joint_state_vec_-q_vec).norm();
   double move_time;
   
   while ((n_retries<max_retries)&&(jspace_err_norm>tolerance)) {
       n_retries++;
       move_time = estimate_move_time(joint_state_vec_,q_vec)+2.0*n_retries; //go slower each iteration
       move_to_jspace_pose(q_vec, move_time);
       ros::Duration(1.0).sleep();
       get_fresh_joint_states(); // update joint_state_vec_
       jspace_err_norm = (joint_state_vec_-q_vec).norm();
   }
   if (jspace_err_norm>tolerance) return false;
   return true; 
}

//helper fnc for  joint-space moves; puts a single jspace  pose into a trajectory msg
/*
trajectory_msgs::JointTrajectory RobotMoveActionServer::jspace_pose_to_traj(Eigen::VectorXd joints, double dtime) {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;
    msg.header.stamp = ros::Time::now();
    // Copy the joint names from the msg off the '/ariac/joint_states' topic.
    msg.joint_names = robotInterface.getJointsNames();
    int njnts = msg.joint_names.size();
    // Create one point in the trajectory.
    msg.points.resize(1);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(njnts);
    for (int i = 0; i < njnts; ++i) {
        msg.points[0].positions[i] = joints[i];
    }
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(dtime);
    // ROS_INFO_STREAM("populated traj msg:\n" << msg);
    return msg;
}


void RobotMoveActionServer::move_to_jspace_pose(Eigen::VectorXd q_vec, double dtime) {
    traj_ = jspace_pose_to_traj(q_vec, dtime);
    joint_trajectory_publisher_.publish(traj_);
    //ros::Duration(dtime).sleep(); //must do timing externally
}
 */

//callback fnc to subscribe to joint-state publications;
//make joint_states available as member var
void KukaBehaviorActionServer::jointstateCB(const sensor_msgs::JointState& message_holder) {
    
    joint_state_ = message_holder;
    for (int i=0;i<8;i++) {
      joint_state_vec_[i] = joint_state_.position[i];
      got_new_joint_states_=true;
    }
}



//EXECUTE_CB: does function  switching

void KukaBehaviorActionServer::executeCB(const robot_behavior_interface::RobotBehaviorGoalConstPtr &goal) {
    ROS_INFO("Received goal type: %d", goal->type);
    double start_time = ros::Time::now().toSec();
    double dt;
    double timeout = goal->timeout <= 0 ? FLT_MAX : goal->timeout;
    //unsigned short int errorCode;
    bool source_is_up, target_is_up, flip_part;
    double t_wait = 0.0;
    double dt_wait = 0.2;
    double t_wait_timeout = MAX_BEHAVIOR_SERVER_WAIT_TIME;
    double timeout_arg = MAX_BEHAVIOR_SERVER_WAIT_TIME;
    is_attached_ = false;
    inventory_msgs::Part part, source_part,place_part;

    switch (goal->type) {
        case robot_behavior_interface::RobotBehaviorGoal::NONE:
            ROS_INFO("NONE");
            errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;
            //report_success_or_abort(); //populate result message and inform client of status
            break;
        case robot_behavior_interface::RobotBehaviorGoal::TEST_PICK_PART_FROM_BIN:
            ROS_WARN("kinematic test fnc");
            errorCode_ = test_pick_part_from_bin(goal);
        break;

        case robot_behavior_interface::RobotBehaviorGoal::PICK_PART_FROM_BIN:  //re-wrote 5/2/2018; needs more testing
            ROS_INFO("PICK_PART_FROM_BIN");
            // use "goal", but only need to populate the "sourcePart" component
            part = goal->sourcePart;
            if (part.location==Part::BIN5||part.location==Part::BIN4) {
                errorCode_ = test_pick_part_from_bin5(goal); //temp testing using modified moves
            }
            else {
               errorCode_ = test_pick_part_from_bin(goal); //temp testing using modified moves
               //errorCode_ = pick_part_from_bin(goal);//pick_part_from_bin
            }
            break;
            
        case robot_behavior_interface::RobotBehaviorGoal::PLACE_PART_IN_BOX_NO_RELEASE:
            ROS_INFO("PLACE_PART_NO_RELEASE (place part in box)");
            part = goal->destinationPart;
            errorCode_ = place_part_in_box_no_release(part);
            break;
        case robot_behavior_interface::RobotBehaviorGoal::MOVE_GRASPED_PART_TO_APPROACH_POSE:
            ROS_INFO("MOVE_GRASPED_PART_TO_APPROACH_POSE (and view before place part in box)");
            part = goal->destinationPart;
            errorCode_ = move_grasped_part_to_approach_pose(part);
            break;            
            //
        case robot_behavior_interface::RobotBehaviorGoal::PLACE_PART_IN_BOX_WITH_RELEASE:
            ROS_INFO("PLACE_PART_NO_RELEASE (place part in box)");
            part = goal->destinationPart;
            timeout_arg = goal->timeout;
            errorCode_ = place_part_in_box_with_release(part,timeout_arg);
            break;
            
        case robot_behavior_interface::RobotBehaviorGoal::DISCARD_GRASPED_PART_Q1:
            part = goal->destinationPart;
            ROS_INFO("DISCARD_GRASPED_PART_Q1");
            errorCode_ = discard_grasped_part(part);
            break;


        case robot_behavior_interface::RobotBehaviorGoal::RELEASE:
            ROS_INFO("RELEASE");
            timeout_arg = goal->timeout;
            errorCode_ = gripperInterface_.release_fnc(timeout_arg); //this version includes testing for release and timeout monitoring, same as below
            break;            
        case robot_behavior_interface::RobotBehaviorGoal::RELEASE_AND_RETRACT:
            ROS_INFO("RELEASE_AND_RETRACT");
            timeout_arg = goal->timeout;
            errorCode_ = release_and_retract(timeout_arg);
            //errorCode_ = gripperInterface_.release_fnc(timeout_arg); //this version includes testing for release and timeout monitoring, same as below
            break;              

        case robot_behavior_interface::RobotBehaviorGoal::ADJUST_PART_LOCATION:
            ROS_INFO("ADJUST_PART_LOCATION");
            place_part= goal->destinationPart;
            source_part = goal->sourcePart;
            errorCode_ = adjust_part_location_no_release(source_part, place_part);
            break;
        case robot_behavior_interface::RobotBehaviorGoal::ADJUST_PART_LOCATION_WITH_RELEASE:
            ROS_INFO("ADJUST_PART_LOCATION_WITH_RELEASE");
            place_part= goal->destinationPart;
            source_part = goal->sourcePart;
            errorCode_ = adjust_part_location_with_release(source_part, place_part);
            break;
            
        case robot_behavior_interface::RobotBehaviorGoal::PICK_PART_FROM_BOX:
            ROS_INFO("PICK_PART_FROM_BOX ");
            part = goal->destinationPart;
            timeout_arg = goal->timeout;
            errorCode_ = pick_part_from_box(part,timeout);
            break;            
            
        case robot_behavior_interface::RobotBehaviorGoal::EVALUATE_KEY_PICK_AND_PLACE_POSES:
            ROS_INFO("EVALUATE_KEY_PICK_AND_PLACE_POSES ");
            place_part= goal->destinationPart;
            source_part = goal->sourcePart;
            timeout_arg = goal->timeout;
            errorCode_ = evaluate_key_pick_and_place_poses(source_part, place_part);
            break;         
            
        case robot_behavior_interface::RobotBehaviorGoal::RE_EVALUATE_APPROACH_AND_PLACE_POSES:
            ROS_INFO("RE_EVALUATE_APPROACH_AND_PLACE_POSES ");
            place_part= goal->destinationPart;
            source_part = goal->sourcePart;
            timeout_arg = goal->timeout;
            errorCode_ = re_evaluate_approach_and_place_poses(source_part, place_part);
            break;      
        case robot_behavior_interface::RobotBehaviorGoal::PLACE_PART_IN_BOX_FROM_APPROACH_NO_RELEASE:
            ROS_INFO("PLACE_PART_IN_BOX_FROM_APPROACH_NO_RELEASE ");
            part = goal->destinationPart;
            timeout_arg = goal->timeout;
            errorCode_ = place_part_in_box_from_approach_no_release(part,timeout);
            break;             

            
            
            
            /*
        case kuka_move_as::RobotBehaviorGoal::GRASP:
            ROS_INFO("GRASP");
            errorCode_ = grasp_fnc();
            if (errorCode != kuka_move_as::RobotBehaviorResult::NO_ERROR) {
                   ROS_WARN("grasp unsuccessful");
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    robot_behavior_as.setAborted(result_);
                    return;
                }
             //if here, all is well:
                ROS_INFO("part is grasped");
                result_.success = true;
                result_.errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR;
                //result_.robotState = robotState;
                robot_behavior_as.setSucceeded(result_);
            break;
                         
            move_to_jspace_pose(q_box_Q1_hover_pose_,2.0); //need to check all moves for valid timing
            ros::Duration(2.0).sleep();
            move_to_jspace_pose(q_Q1_discard_pose_,2.0);
            ros::Duration(2.0).sleep();
            ROS_INFO("releasing part");
            errorCode = release_fnc(2.0); //timeout after 2 secs waiting
            result_.success = true;
            result_.errorCode = errorCode;
            robot_behavior_as.setSucceeded(result_);            
            break;
           
         case kuka_move_as::RobotBehaviorGoal::FLIP_PART: // special case to flip a part
           ROS_INFO("attempting to flip a part");
            errorCode = flip_part_fnc(goal);
            result_.errorCode = errorCode;
            if (errorCode == kuka_move_as::RobotBehaviorResult::NO_ERROR) {
                result_.success = true;
                //result_.robotState = robotState;
                robot_behavior_as.setSucceeded(result_);
                ROS_INFO("done with part-flip attempt");
            } else {
                ROS_INFO("failed to flip part");
                ROS_INFO("error code: %d", (int) errorCode);
                //result_.robotState = robotState;
                robot_behavior_as.setAborted(result_);
            }
            break;

        case kuka_move_as::RobotBehaviorGoal::MOVE:  //Here is the primary function of this server: pick and place
            ROS_INFO("MOVE");
                errorCode = move_part(goal);
                if (errorCode != kuka_move_as::RobotBehaviorResult::NO_ERROR) {
                   ROS_WARN("move_part returned error code: %d", (int) errorCode);
                    result_.success = false;
                    result_.errorCode = errorCode;
                    robot_behavior_as.setAborted(result_);
                    return;
                }
             //if here, all is well:
                ROS_INFO("Completed MOVE action");
                result_.success = true;
                result_.errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR;
                //result_.robotState = robotState;
                robot_behavior_as.setSucceeded(result_);
              break;
        case kuka_move_as::RobotBehaviorGoal::TEST_IS_PICKABLE:
            ROS_INFO("TEST_IS_PICKABLE");
            // use "goal", but only need to populate the "sourcePart" component
               errorCode = is_pickable(goal);
                if (errorCode != kuka_move_as::RobotBehaviorResult::NO_ERROR) {
                   ROS_WARN("is_pickable returned error code: %d", (int) errorCode);
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    robot_behavior_as.setAborted(result_);
                    return;
                }
             //if here, all is well:
                ROS_INFO("Completed TEST_IS_PICKABLE action");
                result_.success = true;
                result_.errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR;
                //result_.robotState = robotState;
                robot_behavior_as.setSucceeded(result_);
            break;

            
        case kuka_move_as::RobotBehaviorGoal::TEST_IS_PLACEABLE:
            ROS_INFO("PLACE_PART_NO_RELEASE; targetPart is:");
            part_of_interest_ = goal->targetPart;
            ROS_INFO_STREAM(part_of_interest_);      
               errorCode = is_placeable(part_of_interest_);
                if (errorCode != kuka_move_as::RobotBehaviorResult::NO_ERROR) {
                   ROS_WARN("place_part_fnc_no_release returned error code: %d", (int) errorCode);
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    robot_behavior_as.setAborted(result_);
                    return;
                }
             //if here, all is well:
                ROS_INFO("Completed TEST_IS_PLACEABLE action");
                result_.success = true;
                result_.errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR;
                //result_.robotState = robotState;
                robot_behavior_as.setSucceeded(result_);            
            break;
            
        case kuka_move_as::RobotBehaviorGoal::PLACE_PART_NO_RELEASE:
            ROS_INFO("PLACE_PART_NO_RELEASE; targetPart is:");
            part_of_interest_ = goal->targetPart;
            ROS_INFO_STREAM(part_of_interest_);      
               errorCode = place_part_fnc_no_release(part_of_interest_);
                if (errorCode != kuka_move_as::RobotBehaviorResult::NO_ERROR) {
                   ROS_WARN("place_part_fnc_no_release returned error code: %d", (int) errorCode);
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    robot_behavior_as.setAborted(result_);
                    return;
                }
             //if here, all is well:
                ROS_INFO("Completed PLACE_PART_NO_RELEASE action");
                result_.success = true;
                result_.errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR;
                //result_.robotState = robotState;
                robot_behavior_as.setSucceeded(result_);            
            break;
             * */

            //NOW  OBSOLETE; have lefty and righty discard  poses
            /*
        case kuka_move_as::RobotBehaviorGoal::DISCARD_GRASPED_PART_Q1:
            ROS_INFO("DISCARD_GRASPED_PART_Q1");
            ROS_INFO("moving to discard pose");
            move_to_jspace_pose(q_box_Q1_hover_pose_,2.0); //need to check all moves for valid timing
            ros::Duration(2.0).sleep();
            move_to_jspace_pose(q_Q1_discard_pose_,2.0);
            ros::Duration(2.0).sleep();
            ROS_INFO("releasing part");
            errorCode = release_fnc(2.0); //timeout after 2 secs waiting
            result_.success = true;
            result_.errorCode = errorCode;
            robot_behavior_as.setSucceeded(result_);            
            break;
            
        case kuka_move_as::RobotBehaviorGoal::DISCARD_GRASPED_PART_Q2:
            ROS_INFO("DISCARD_GRASPED_PART_Q2");
            move_to_jspace_pose(q_box_Q2_hover_pose_,2.0);
            ros::Duration(2.0).sleep();            
            move_to_jspace_pose(q_Q2_discard_pose_,2.0);
            ros::Duration(2.0).sleep();
            errorCode = release_fnc(2.0);
            result_.success = true;
            result_.errorCode = errorCode;
            robot_behavior_as.setSucceeded(result_);            
            break;            
             */
            /*
        case kuka_move_as::RobotBehaviorGoal::RELEASE_PLACED_PART:
            //will release grasp, depart, and move  to hover pose (not cruise pose)
            ROS_INFO("RELEASE_PLACED_PART");
            //release_fnc is a blocking fnc; will wait for confirmation of release up to max time set in arg
            errorCode = release_fnc(5.0); //fix error handling here
            ROS_INFO("grasp should be released now; moving to depart pose");
            //cout<<"enter 1 to move  to depart pose: ";
            //cin>>ans;
            //move to  approach/depart pose
            move_to_jspace_pose(approach_dropoff_jspace_pose_,2.0);
            ros::Duration(2.0).sleep();    

            //cout<<"enter 1 to move to hover pose: ";
            //cin>>ans;
            //move to  hover pose:
            ROS_INFO("moving to hover pose");
            move_to_jspace_pose(bin_hover_jspace_pose_,2.0);
            ros::Duration(2.0).sleep();

            result_.success = true;
            result_.errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR;
            ROS_INFO("completed RELEASE_PLACED_PART");
            robot_behavior_as.setSucceeded(result_);            
            break; 

        case kuka_move_as::RobotBehaviorGoal::PLACE:
            ROS_INFO("PLACE");
            ROS_INFO("The part is %s, should be place to %s, with pose:", goal->targetPart.name.c_str(),
                     placeFinder[goal->targetPart.location].c_str());
            ROS_INFO_STREAM(goal->targetPart.pose);
            ROS_INFO("Time limit is %f", timeout);
            ros::Duration(0.5).sleep();
            //feedback_.robotState = robotState;
            robot_behavior_as.publishFeedback(feedback_);
            ros::Duration(0.5).sleep();
            ROS_INFO("I got the part");
            dt = ros::Time::now().toSec() - start_time;
            if (dt < timeout) {
                ROS_INFO("I completed the action");
                result_.success = true;
                result_.errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR;
                //result_.robotState = robotState;
                robot_behavior_as.setSucceeded(result_);
            } else {
                ROS_INFO("I am running out of time");
                result_.success = false;
                result_.errorCode = kuka_move_as::RobotBehaviorResult::TIMEOUT;
                //result_.robotState = robotState;
                robot_behavior_as.setAborted(result_);
            }
            break;
            
        case kuka_move_as::RobotBehaviorGoal::TO_CRUISE_POSE:
            ROS_INFO("TO_CRUISE_POSE");
           if(!cruise_jspace_pose(goal->sourcePart.location, source_cruise_pose_) ) {
                    ROS_WARN("destination code not recognized for get_cruise_pose");
                    result_.success = false;
                    result_.errorCode = kuka_move_as::RobotBehaviorResult::WRONG_PARAMETER;
                    robot_behavior_as.setAborted(result_);
           }
           else{
               ROS_INFO("moving to cruise pose for specified location code");
               ROS_INFO_STREAM("cruise pose; "<<source_cruise_pose_<<endl);
               move_to_jspace_pose(source_cruise_pose_);
               result_.success = true;
               result_.errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR;
               robot_behavior_as.setSucceeded(result_);
           }
            break;

            
        case kuka_move_as::RobotBehaviorGoal::TO_PREDEFINED_POSE:
            //bool RobotMoveActionServer::get_pose_from_code(unsigned short int POSE_CODE, Eigen::VectorXd &q_vec) ;q_des_7dof_
            if (get_pose_from_code(goal->predfinedPoseCode,q_des_7dof_)) {
                current_pose_code_=goal->predfinedPoseCode;
                move_to_jspace_pose(q_des_7dof_);               
                joint_trajectory_publisher_.publish(traj_);
                result_.success = true;
                result_.errorCode = kuka_move_as::RobotBehaviorResult::NO_ERROR;
                //result_.robotState = robotState;
                ros::Duration(2.0).sleep();
                robot_behavior_as.setSucceeded(result_);                
            }
            else {
                    ROS_WARN("predefined move code not implemented!");
                    result_.success = false;
                    result_.errorCode = kuka_move_as::RobotBehaviorResult::WRONG_PARAMETER;
                    robot_behavior_as.setAborted(result_);
            }
            break;
            


             */
        default:
            ROS_INFO("Wrong parameter received for goal");
            //result_.success = false;
            errorCode_ = robot_behavior_interface::RobotBehaviorResult::WRONG_PARAMETER;
            //result_.errorCode = errorCode_;
            //robot_behavior_as.setAborted(result_);
    }
    isPreempt_ = false;
    report_success_or_abort(); //this is part of the fnc call

}

//utility to convert a single jspace desired  pose to a trajectory message
trajectory_msgs::JointTrajectory KukaBehaviorActionServer::jspace_pose_to_traj(Eigen::VectorXd joints, double dtime) {
    trajectory_msgs::JointTrajectory jspace_traj;
    trajectory_msgs::JointTrajectoryPoint trajectory_point;
    jspace_traj.joint_names.push_back("iiwa_joint_1");
    jspace_traj.joint_names.push_back("iiwa_joint_2");
    jspace_traj.joint_names.push_back("iiwa_joint_3");
    jspace_traj.joint_names.push_back("iiwa_joint_4");
    jspace_traj.joint_names.push_back("iiwa_joint_5");
    jspace_traj.joint_names.push_back("iiwa_joint_6");
    jspace_traj.joint_names.push_back("iiwa_joint_7");
    jspace_traj.joint_names.push_back("linear_arm_actuator_joint");    
    trajectory_point.positions.clear();
    for (int i = 0; i < 8; i++) {
                trajectory_point.positions.push_back(joints[i]);
            }
    trajectory_point.time_from_start = ros::Duration(dtime);
    jspace_traj.points.push_back(trajectory_point);  
    return jspace_traj;
}

//helper fnc to invoke motion to a single point, using action server interface to robot:
// this function refers to one of 3 key poses--approach/depart, pickup/dropoff, pickup_deeper
//these are computed on the fly and stored  in:
//    Eigen::VectorXd pickup_deeper_jspace_pose_;
//    Eigen::VectorXd desired_approach_depart_pose_,desired_grasp_dropoff_pose_;
// oops--looks like this fnc is broken now!!
bool KukaBehaviorActionServer::move_to_jspace_pose(const int pose_code, double arrival_time) {
    trajectory_msgs::JointTrajectory transition_traj;

    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;
    switch (pose_code) {
        case APPROACH_DEPART_CODE:
            ROS_INFO("move_to_jspace_pose(APPROACH_DEPART_CODE)");
            ROS_INFO_STREAM("desired_approach_jspace_pose_"<<endl<<desired_approach_jspace_pose_.transpose()<<endl);
            transition_traj = jspace_pose_to_traj(desired_approach_jspace_pose_);
            break;
        case GRASP_PLACE_CODE:
            ROS_INFO("move_to_jspace_pose(GRASP_PLACE_CODE)");
            ROS_INFO_STREAM("desired_grasp_dropoff_pose_"<<endl<<desired_grasp_dropoff_pose_.transpose()<<endl);  
            transition_traj = jspace_pose_to_traj(desired_grasp_dropoff_pose_);
            break;
        case CURRENT_HOVER_CODE:
            ROS_INFO("move_to_jspace_pose(CURRENT_HOVER_CODE)");
            ROS_INFO_STREAM("current_hover_pose_"<<endl<<current_hover_pose_.transpose()<<endl);    
            transition_traj = jspace_pose_to_traj(current_hover_pose_);
            break;
        default:
            ROS_WARN("move_to_jspace_pose(): pose code not recognized");
            errorCode_ = robot_behavior_interface::RobotBehaviorResult::WRONG_PARAMETER;
            return false;
            break;
    }
    //if here, have a single-point trajectory to execute:
    robot_goal_.trajectory = transition_traj;
    //ROS_INFO("sending goal to arm: ");
    //ROS_INFO_STREAM("sending traj: "<<endl<<transition_traj<<endl);

    send_traj_goal(transition_traj, pose_code); 
    int print_count=0;
    while (!traj_goal_complete_) { //write a fnc for this: wait_for_goal_w_timeout
        //put timeout here...   possibly return falses
         print_count++;
        if (print_count%10==0) {
           ROS_INFO("waiting for trajectory to finish...");
        }
        ros::Duration(0.1).sleep();
    }
    current_pose_code_ = pose_code;    
    if (errorCode_ != robot_behavior_interface::RobotBehaviorResult::NO_ERROR) return false;
    return true;
}

//helper fnc...to be used only internally as part of a behavior;
// does NOT return an errorcode to an action  client
bool KukaBehaviorActionServer::move_to_jspace_pose(Eigen::VectorXd desired_jspace_pose, double arrival_time) {
    trajectory_msgs::JointTrajectory transition_traj;
    transition_traj = jspace_pose_to_traj(desired_jspace_pose,arrival_time);


    robot_goal_.trajectory = transition_traj;
    //ROS_INFO("sending goal to arm: ");
    ROS_INFO_STREAM("sending traj: "<<endl<<transition_traj<<endl);

    send_traj_goal(transition_traj,CUSTOM_JSPACE_POSE); //, pose_code);
    double timer=0;
    double dt = 0.1;
    ROS_INFO("waiting for trajectory to finish...");    
    while ((!traj_goal_complete_) && (timer<MAX_BEHAVIOR_SERVER_WAIT_TIME)) { //write a fnc for this: wait_for_goal_w_timeout
        ros::Duration(dt).sleep();
        ros::spinOnce();
        timer+=dt;
    }
    if (timer>=MAX_BEHAVIOR_SERVER_WAIT_TIME)  {
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::TIMEOUT;
        return false;
    }
    current_pose_code_ = CUSTOM_JSPACE_POSE;  //  this code will not be in the transition matrix
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;
    return true;
}




 

//this function is a helper to send a trajectory goal to the Kuka motion action server,
//where the trajectory is implied by a start posecode and an end posecode
//useful for large motions; still  need fine motions for grasp and placement with exact coords

bool KukaBehaviorActionServer::move_posecode1_to_posecode2(int posecode_start, int posecode_goal) {

    trajectory_msgs::JointTrajectory transition_traj;
    //default: assume success, unless detect something went wrong
    errorCode_ = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;
    //placeFinder_[part.location].c_str()
    ROS_INFO("attempting move from %s  to %s ", map_pose_code_to_name[posecode_start].c_str(),
             map_pose_code_to_name[posecode_goal].c_str());
    //ROS_INFO_STREAM("attempting move from "<<map_pose_code_to_name[posecode_start].c_str()  <<"to "<< map_pose_code_to_name[posecode_goal].c_str() <<endl);
    int npts = transitionTrajectories_.get_trajectory(posecode_start, posecode_goal, transition_traj);
    if (npts < 1) {
        ROS_WARN("precomputed traj does not exist!");
        errorCode_ = robot_behavior_interface::RobotBehaviorResult::PRECOMPUTED_TRAJ_ERR;
        return false;
    }
    //if here, then traj is valid:
    ROS_INFO_STREAM("transition array traj = " << endl << transition_traj << endl);
    robot_goal_.trajectory = transition_traj;
    //ROS_INFO("sending goal to arm: ");
    traj_goal_complete_ = false;
    traj_ctl_ac_.sendGoal(robot_goal_, boost::bind(&KukaBehaviorActionServer::trajDoneCb_, this, _1, _2));
    int print_count=0;
    while (!traj_goal_complete_) { //write a fnc for this: wait_for_goal_w_timeout
        //put timeout here...   possibly return falses
        print_count++;
        if (print_count%10==0) {
           ROS_INFO("waiting for trajectory to finish...");
        }
        ros::Duration(0.1).sleep();
    }
    current_pose_code_ = posecode_goal;    
    if (errorCode_ != robot_behavior_interface::RobotBehaviorResult::NO_ERROR) return false;
    return true;
}

//estimate move time for a point-to-point move:
//naive implementation: IGNORES accel/decel; only based on max vel
//FIX THIS
double KukaBehaviorActionServer::estimate_move_time(Eigen::VectorXd q_vec_start,Eigen::VectorXd q_vec_end) {
    double max_time = 0.0;
    double jnt_time;
    //ros::spinOnce();
    for (int ijnt=0;ijnt<8;ijnt++) {
        jnt_time = fabs(q_vec_end[ijnt]-q_vec_start[ijnt])/MAX_JNT_SPEEDS[ijnt];
        if (jnt_time>max_time) max_time=jnt_time;
    }
    return max_time; 
}

bool KukaBehaviorActionServer::report_success_or_abort() {
    if (errorCode_ != robot_behavior_interface::RobotBehaviorResult::NO_ERROR) {
        ROS_WARN("behavior returned error code: %d", (int) errorCode_);
        result_.success = false;
        result_.errorCode = errorCode_;
        //result_.robotState = robotState;
        robot_behavior_as.setAborted(result_);
        return false;
    }
    //if here, all is well:
    ROS_INFO("Completed behavior successfully");
    result_.success = true;
    result_.errorCode = robot_behavior_interface::RobotBehaviorResult::NO_ERROR;
    //result_.robotState = robotState;
    robot_behavior_as.setSucceeded(result_);
    return true;
}

void KukaBehaviorActionServer::preemptCB() {
    isPreempt_ = true;
    ROS_WARN("robot-motion interface woke up preemption callback");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_behavior_server"); //name this node
    ros::NodeHandle nh;
    KukaBehaviorActionServer kukaBehaviorActionServer(nh, "robot_behavior_server");
    ros::spin();
    return 0;
}
