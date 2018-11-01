//
// created by wsn, 3/30/18
//

#ifndef ROBOT_BEHAVIOR_INTERFACE_H
#define ROBOT_BEHAVIOR_INTERFACE_H


//#include <AriacBase.h>
#include <map>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <xform_utils/xform_utils.h>
#include <inventory_msgs/Part.h>
#include <actionlib/client/simple_action_client.h>
using namespace std;
using namespace inventory_msgs;
//using namespace robot_move_as;
const double MAX_ACTION_SERVER_WAIT_TIME = 30.0; //to prevent deadlocks

#include <kuka_move_as/RobotBehaviorAction.h>  //this is specific to Kuka; should generalize

/*
std::map<short unsigned int, string> error_code_name_map = {
    {kuka_move_as::RobotBehaviorResult::NO_ERROR, "NO_ERROR"},
    {kuka_move_as::RobotBehaviorResult::WRONG_PARAMETER, "WRONG_PARAMETER"},
    {kuka_move_as::RobotBehaviorResult::TIMEOUT, "TIMEOUT"},
    {kuka_move_as::RobotBehaviorResult::GRIPPER_FAULT, "GRIPPER_FAULT"},
    {kuka_move_as::RobotBehaviorResult::PART_DROPPED, "PART_DROPPED"},
    {kuka_move_as::RobotBehaviorResult::PRECOMPUTED_TRAJ_ERR, "PRECOMPUTED_TRAJ_ERR"},
    {kuka_move_as::RobotBehaviorResult::CANCELLED, "CANCELLED"}    
};
 */
class RobotBehaviorInterface {
public:
    RobotBehaviorInterface(ros::NodeHandle* nodehandle);
    bool sendGoal(unsigned short int goal_type, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool sendGoal(unsigned short int goal_type, Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool sendGoal(unsigned short int goal_type, Part sourcePart, Part destinationPart,double timeout= MAX_ACTION_SERVER_WAIT_TIME);


    void cancel();

    bool action_server_returned() {
        return action_server_returned_;
    };

    short unsigned int get_error_code() {
        return errorCode_;
    };
    //bool pick(Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool pick_part_from_bin(Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool test_pick_part_from_bin(Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME);

    bool place_part_in_box_no_release(Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool place_part_in_box_with_release(Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME); 
    bool release_and_retract(double timeout= MAX_ACTION_SERVER_WAIT_TIME);
    bool discard_grasped_part(Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool release(double timeout= MAX_ACTION_SERVER_WAIT_TIME);
    bool adjust_part_location_no_release(Part sourcePart, Part destinationPart, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool adjust_part_location_with_release(Part sourcePart, Part destinationPart, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool pick_part_from_box(Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool move_part_to_approach_pose(inventory_msgs::Part part,double timeout = MAX_ACTION_SERVER_WAIT_TIME); 
    bool re_evaluate_approach_and_place_poses(Part sourcePart, Part destinationPart, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool evaluate_key_pick_and_place_poses(Part sourcePart, Part destinationPart, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool place_part_in_box_from_approach_no_release(Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME);

    /*
     the following may be useful additions:
    bool test_is_pickable(Part part);
    bool test_is_placeable(Part part);
    bool flipPart(Part part, double timeout = 0);
    bool grasp(double timeout = 0);
     get_error_code()??
     */
protected:

    ros::NodeHandle nh_;
    ros::Time goal_start_time_;
    actionlib::SimpleActionClient<kuka_move_as::RobotBehaviorAction> ac;
    kuka_move_as::RobotBehaviorGoal behaviorServerGoal_;
    int8_t errorCode_;
    bool goal_success_;
    //RobotState currentRobotState;
    bool action_server_returned_;
    double time_tolerance;
    bool async_mode;
    std::map<int8_t, string> errorCodeFinder;
    //bool ac_timed_out(double wait_time);
    //bool wait_for_goal(double expiration_time=MAX_ACTION_SERVER_WAIT_TIME); //default wait time

    void doneCb(const actionlib::SimpleClientGoalState& state, const kuka_move_as::RobotBehaviorResultConstPtr& result);
    void activeCb();
    void feedbackCb(const kuka_move_as::RobotBehaviorFeedbackConstPtr& feedback);
    bool wrap_up();
};


#endif //ROBOT_BEHAVIOR_INTERFACE_H
