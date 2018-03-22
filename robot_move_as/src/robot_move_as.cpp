//
// Created by shipei on 2/17/17.
// modified wsn 3/5/2018
//

#include <robot_move_as/RobotMoveActionServer.h>


//define this func in separate file--just to focus on its devel
//#include "fetch_part_from_conveyor_fnc.cpp"
#include "flip_part_fnc.cpp"
#include "pick_part_fnc.cpp"
//place_part_fnc_no_release():  
//sequence: goes to hover pose, approach pose, dropoff pose, halts with gripper still attached
//will want to use this for Q1 and Q2 stations
#include "place_part_fnc_no_release.cpp"
#include "move_part.cpp"
#include "set_key_poses.cpp"
#include "grasp_and_release_fncs.cpp"
#include "compute_key_poses.cpp"


//CONSTRUCTOR:
RobotMoveActionServer::RobotMoveActionServer(ros::NodeHandle nodeHandle, string topic) :
        nh(nodeHandle), as(nh, topic, boost::bind(&RobotMoveActionServer::executeCB, this, _1), false),
        robotInterface(nh) {
    isPreempt = false;
    as.registerPreemptCallback(boost::bind(&RobotMoveActionServer::preemptCB, this));
    as.start();
    ROS_INFO("Start Robot Move Action Server");
    //placeFinder.insert(pair<int8_t, string>(Part::AGV, "AGV"));
    //placeFinder.insert(pair<int8_t, string>(Part::AGV1, "AGV1"));
    //placeFinder.insert(pair<int8_t, string>(Part::AGV2, "AGV2"));
    //placeFinder.insert(pair<int8_t, string>(Part::BIN, "BIN"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN1, "BIN1"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN2, "BIN2"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN3, "BIN3"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN4, "BIN4"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN5, "BIN5"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN6, "BIN6"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN7, "BIN7"));
    placeFinder.insert(pair<int8_t, string>(Part::BIN8, "BIN8"));
    placeFinder.insert(pair<int8_t, string>(Part::QUALITY_SENSOR_1, "QUALITY_SENSOR_1"));    
    //placeFinder.insert(pair<int8_t, string>(Part::CAMERA, "CAMERA"));
    placeFinder.insert(pair<int8_t, string>(Part::QUALITY_SENSOR_2, "QUALITY_SENSOR_2"));

    joint_trajectory_publisher_ = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/ariac/arm/command", 10);

    set_key_poses();

    tfListener_ = new tf::TransformListener;
    bool tferr = true;

    ROS_INFO("waiting for tf between world and base_link...");
    tf::StampedTransform tfBaseLinkWrtWorld;
    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform, link2-frame w/rt base_link frame; this will test if
            // a valid transform chain has been published from base_frame to link2
            tfListener_->lookupTransform("world", "base_link", ros::Time(0), tfBaseLinkWrtWorld);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    tferr = true;
    ROS_INFO("waiting for tf between base_link and vacuum_gripper_link...");
    tf::StampedTransform tfGripperWrtWorld;

    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform, link2-frame w/rt base_link frame; this will test if
            // a valid transform chain has been published from base_frame to link2
            tfListener_->lookupTransform("base_link", "vacuum_gripper_link", ros::Time(0), tfGripperWrtWorld);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    tferr = true;
    ROS_INFO("waiting for tf between world and logical_camera_frame...");

    while (tferr) {
        tferr = false;
        try {
            //try to lookup transform, link2-frame w/rt base_link frame; this will test if
            // a valid transform chain has been published from base_frame to link2
            tfListener_->lookupTransform("world", "logical_camera_1_frame", ros::Time(0), tfCameraWrtWorld_);
        } catch (tf::TransformException &exception) {
            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }


    gripper_client = nodeHandle.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
    ROS_INFO("waiting to connect to gripper service");
    if (!gripper_client.exists()) {
        gripper_client.waitForExistence();
    }
    ROS_INFO("gripper service exists");
    attach_.request.enable = 1;
    detach_.request.enable = 0;
    ROS_INFO_STREAM("robot move action server is ready!"<<endl<<endl);
}

//helper fnc for  joint-space moves; puts a single jspace  pose into a trajectory msg
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

/*
#a bunch of key pose codes for box at quality-sensor 1:
int8 Q1_RIGHTY_HOVER              =30
int8 Q1_RIGHTY_DISCARD            =31
int8 Q1_RIGHTY_CRUISE             =32
int8 Q1_RIGHTY_HOVER_FLIP         =33

int8 Q1_LEFTY_CRUISE              =34
int8 Q1_LEFTY_HOVER               =35
int8 Q1_LEFTY_HOVER_FLIP          =36
int8 Q1_LEFTY_DISCARD             =37

#     //safe move sequence, lefty/righty: Q1_RIGHTY_CRUISE<-->Q1_ARM_VERTICAL<-->v<-->Q1_LEFTY_CRUISE
int8 Q1_ARM_VERTICAL              =38

//some key poses near candidate box dropoff locations at Qsensor 1
#    //------RIGHTY POSES------
#    //covers cases for  left-side of box dropoff (from robot's perspective); 
#    //the following are safe transitions:
#    //Q1_DROPOFF_FAR_LEFT<->Q1_RIGHTY_HOVER<->Q1_RIGHTY_HOVER_FLIP<->Q1_DROPOFF_NEAR_LEFT
int8 Q1_DROPOFF_NEAR_LEFT         =40
int8 Q1_DROPOFF_FAR_LEFT          =41

#         //----LEFTY  POSES----
#     //covers cases for right-side dropoff; the following are safe transitions:
#     //Q1_DROPOFF_FAR_RIGHT<->Q1_LEFTY_HOVER_FLIP<->Q1_LEFTY_HOVER<->Q1_DROPOFF_NEAR_RIGHT   
int8 Q1_DROPOFF_NEAR_RIGHT         =42
int8 Q1_DROPOFF_FAR_RIGHT          =43
 * */


//EXECUTE_CB: does function  switching
void RobotMoveActionServer::executeCB(const robot_move_as::RobotMoveGoalConstPtr &goal) {
    ROS_INFO("Received goal type: %d", goal->type);
    double start_time = ros::Time::now().toSec();
    double dt;
    double timeout = goal->timeout <= 0 ? FLT_MAX : goal->timeout;
    unsigned short int errorCode;
    bool source_is_up, target_is_up, flip_part;
    double t_wait=0.0;
    double dt_wait = 0.2;
    double t_wait_timeout = 5.0;
    bool is_attached=false;

    switch (goal->type) {
        case robot_move_as::RobotMoveGoal::NONE:
            ROS_INFO("NONE");
            result_.success = true;
            result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
            as.setSucceeded(result_);
            break;
            
         case robot_move_as::RobotMoveGoal::FLIP_PART: // special case to flip a part
           ROS_INFO("attempting to flip a part");
            errorCode = flip_part_fnc(goal);
            result_.errorCode = errorCode;
            if (errorCode == robot_move_as::RobotMoveResult::NO_ERROR) {
                result_.success = true;
                //result_.robotState = robotState;
                as.setSucceeded(result_);
                ROS_INFO("done with part-flip attempt");
            } else {
                ROS_INFO("failed to flip part");
                ROS_INFO("error code: %d", (int) errorCode);
                //result_.robotState = robotState;
                as.setAborted(result_);
            }
            break;

        case robot_move_as::RobotMoveGoal::MOVE:  //Here is the primary function of this server: pick and place
            ROS_INFO("MOVE");
                errorCode = move_part(goal);
                if (errorCode != robot_move_as::RobotMoveResult::NO_ERROR) {
                   ROS_WARN("move_part returned error code: %d", (int) errorCode);
                    result_.success = false;
                    result_.errorCode = errorCode;
                    as.setAborted(result_);
                    return;
                }
             //if here, all is well:
                ROS_INFO("Completed MOVE action");
                result_.success = true;
                result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
                //result_.robotState = robotState;
                as.setSucceeded(result_);
              break;
        case robot_move_as::RobotMoveGoal::TEST_IS_PICKABLE:
            ROS_INFO("TEST_IS_PICKABLE");
            // use "goal", but only need to populate the "sourcePart" component
               errorCode = is_pickable(goal);
                if (errorCode != robot_move_as::RobotMoveResult::NO_ERROR) {
                   ROS_WARN("is_pickable returned error code: %d", (int) errorCode);
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    as.setAborted(result_);
                    return;
                }
             //if here, all is well:
                ROS_INFO("Completed TEST_IS_PICKABLE action");
                result_.success = true;
                result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
                //result_.robotState = robotState;
                as.setSucceeded(result_);
            break;
        case robot_move_as::RobotMoveGoal::PICK:
            ROS_INFO("PICK");
            // use "goal", but only need to populate the "sourcePart" component
               errorCode = pick_part_fnc(goal);
                if (errorCode != robot_move_as::RobotMoveResult::NO_ERROR) {
                   ROS_WARN("pick_part_fnc returned error code: %d", (int) errorCode);
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    as.setAborted(result_);
                    return;
                }
             //if here, all is well:
                ROS_INFO("Completed PICK action");
                result_.success = true;
                result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
                //result_.robotState = robotState;
                as.setSucceeded(result_);
            break;
            
        case robot_move_as::RobotMoveGoal::TEST_IS_PLACEABLE:
            ROS_INFO("PLACE_PART_NO_RELEASE; targetPart is:");
            part_of_interest_ = goal->targetPart;
            ROS_INFO_STREAM(part_of_interest_);      
               errorCode = is_placeable(part_of_interest_);
                if (errorCode != robot_move_as::RobotMoveResult::NO_ERROR) {
                   ROS_WARN("place_part_fnc_no_release returned error code: %d", (int) errorCode);
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    as.setAborted(result_);
                    return;
                }
             //if here, all is well:
                ROS_INFO("Completed TEST_IS_PLACEABLE action");
                result_.success = true;
                result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
                //result_.robotState = robotState;
                as.setSucceeded(result_);            
            break;
            
        case robot_move_as::RobotMoveGoal::PLACE_PART_NO_RELEASE:
            ROS_INFO("PLACE_PART_NO_RELEASE; targetPart is:");
            part_of_interest_ = goal->targetPart;
            ROS_INFO_STREAM(part_of_interest_);      
               errorCode = place_part_fnc_no_release(part_of_interest_);
                if (errorCode != robot_move_as::RobotMoveResult::NO_ERROR) {
                   ROS_WARN("place_part_fnc_no_release returned error code: %d", (int) errorCode);
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    as.setAborted(result_);
                    return;
                }
             //if here, all is well:
                ROS_INFO("Completed PLACE_PART_NO_RELEASE action");
                result_.success = true;
                result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
                //result_.robotState = robotState;
                as.setSucceeded(result_);            
            break;
            
        //NOW  OBSOLETE; have lefty and righty discard  poses
            /*
        case robot_move_as::RobotMoveGoal::DISCARD_GRASPED_PART_Q1:
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
            as.setSucceeded(result_);            
            break;
            
        case robot_move_as::RobotMoveGoal::DISCARD_GRASPED_PART_Q2:
            ROS_INFO("DISCARD_GRASPED_PART_Q2");
            move_to_jspace_pose(q_box_Q2_hover_pose_,2.0);
            ros::Duration(2.0).sleep();            
            move_to_jspace_pose(q_Q2_discard_pose_,2.0);
            ros::Duration(2.0).sleep();
            errorCode = release_fnc(2.0);
            result_.success = true;
            result_.errorCode = errorCode;
            as.setSucceeded(result_);            
            break;            
*/
        case robot_move_as::RobotMoveGoal::RELEASE_PLACED_PART:
            ROS_INFO("RELEASE_PLACED_PART");
            errorCode = release_fnc(2.0); //fix error handling here
            //move to  approach/depart pose
            move_to_jspace_pose(approach_dropoff_jspace_pose_,2.0);
            ros::Duration(2.0).sleep();    
            //move to  hover pose:
            move_to_jspace_pose(bin_hover_jspace_pose_,2.0);
            ros::Duration(2.0).sleep();

            result_.success = true;
            result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
            as.setSucceeded(result_);            
            break; 

        case robot_move_as::RobotMoveGoal::PLACE:
            ROS_INFO("PLACE");
            ROS_INFO("The part is %s, should be place to %s, with pose:", goal->targetPart.name.c_str(),
                     placeFinder[goal->targetPart.location].c_str());
            ROS_INFO_STREAM(goal->targetPart.pose);
            ROS_INFO("Time limit is %f", timeout);
            ros::Duration(0.5).sleep();
            //feedback_.robotState = robotState;
            as.publishFeedback(feedback_);
            ros::Duration(0.5).sleep();
            ROS_INFO("I got the part");
            dt = ros::Time::now().toSec() - start_time;
            if (dt < timeout) {
                ROS_INFO("I completed the action");
                result_.success = true;
                result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
                //result_.robotState = robotState;
                as.setSucceeded(result_);
            } else {
                ROS_INFO("I am running out of time");
                result_.success = false;
                result_.errorCode = robot_move_as::RobotMoveResult::TIMEOUT;
                //result_.robotState = robotState;
                as.setAborted(result_);
            }
            break;
            
        case robot_move_as::RobotMoveGoal::TO_CRUISE_POSE:
            ROS_INFO("TO_CRUISE_POSE");
           if(!cruise_jspace_pose(goal->sourcePart.location, source_cruise_pose_) ) {
                    ROS_WARN("destination code not recognized for get_cruise_pose");
                    result_.success = false;
                    result_.errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
                    as.setAborted(result_);
           }
           else{
               ROS_INFO("moving to cruise pose for specified location code");
               ROS_INFO_STREAM("cruise pose; "<<source_cruise_pose_<<endl);
               move_to_jspace_pose(source_cruise_pose_);
               result_.success = true;
               result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
               as.setSucceeded(result_);
           }
            break;

            
        case robot_move_as::RobotMoveGoal::TO_PREDEFINED_POSE:
            //bool RobotMoveActionServer::get_pose_from_code(unsigned short int POSE_CODE, Eigen::VectorXd &q_vec) ;q_des_7dof_
            if (get_pose_from_code(goal->predfinedPoseCode,q_des_7dof_)) {
                current_pose_code_=goal->predfinedPoseCode;
                move_to_jspace_pose(q_des_7dof_);               
                joint_trajectory_publisher_.publish(traj_);
                result_.success = true;
                result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
                //result_.robotState = robotState;
                ros::Duration(2.0).sleep();
                as.setSucceeded(result_);                
            }
            else {
                    ROS_WARN("predefined move code not implemented!");
                    result_.success = false;
                    result_.errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
                    as.setAborted(result_);
            }
            break;
            

        case robot_move_as::RobotMoveGoal::GRASP:
            ROS_INFO("GRASP");
            errorCode = grasp_fnc();
            if (errorCode != robot_move_as::RobotMoveResult::NO_ERROR) {
                   ROS_WARN("grasp unsuccessful");
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    as.setAborted(result_);
                    return;
                }
             //if here, all is well:
                ROS_INFO("part is grasped");
                result_.success = true;
                result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
                //result_.robotState = robotState;
                as.setSucceeded(result_);
            break;

        case robot_move_as::RobotMoveGoal::RELEASE:
            errorCode = release_fnc();
            if (errorCode != robot_move_as::RobotMoveResult::NO_ERROR) {
                   ROS_WARN("grasp unsuccessful; timed out");
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    as.setAborted(result_);
                    return;
                }
                result_.success = true;
                result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
                //result_.robotState = robotState;
                as.setSucceeded(result_);
            break;

        default:
            ROS_INFO("Wrong parameter received for goal");
            result_.success = false;
            result_.errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
            as.setAborted(result_);
    }
    isPreempt = false;
}

void RobotMoveActionServer::preemptCB() {
    isPreempt = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_move_as"); //name this node
    ros::NodeHandle nh;
    RobotMoveActionServer actionServer(nh, "robot_move");
    ros::spin();
    return 0;
}
