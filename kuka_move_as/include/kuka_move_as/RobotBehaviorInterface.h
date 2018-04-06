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
const double MAX_ACTION_SERVER_WAIT_TIME=30.0;  //to prevent deadlocks

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


    void cancel();
    bool action_server_returned() { return action_server_returned_;};
    short unsigned  int get_error_code() {return errorCode_;};
    //bool pick(Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    bool pick_part_from_bin(Part part, double timeout = MAX_ACTION_SERVER_WAIT_TIME); 
    bool place_part_in_box_no_release(Part part,double timeout = MAX_ACTION_SERVER_WAIT_TIME); 
    //bool discard_grasped_part(double timeout=0);
    //unsigned short int discard_grasped_part(const kuka_move_as::RobotBehaviorGoalConstPtr &goal);
    bool discard_grasped_part(Part part,double timeout = MAX_ACTION_SERVER_WAIT_TIME);
    /*
    bool toHome(double timeout = 0);

    bool move_cruise_pose(Part part, double timeout);
    //bool move_hover_pose(Part part, double timeout);    
    bool toPredefinedPose(int8_t goal_code, double timeout = 0);

    bool test_is_pickable(Part part);
    bool test_is_placeable(Part part);
    bool place(Part destination, double timeout = 0);
    bool place_part_no_release(Part destination, double timeout = 0);
    bool move(Part part, Part destination, double timeout = 0);
    //bool fetchPartFromConveyor(Part part,Part destination, double timeout=0);
    bool flipPart(Part part, double timeout = 0);
    //bool setJointValues(vector<double> joints, double timeout = 0);
    bool grasp(double timeout = 0);
    bool release(double timeout = 0);
    bool isGripperAttached();
    bool discard_grasped_part_Q1(double timeout=0);//bool RobotBehavior::discard_grasped_part(double timeout)
    bool discard_grasped_part_Q2(double timeout=0);//bool RobotBehavior::discard_grasped_part(double timeout)    
    bool release_placed_part(double timeout=2.0);
    //bool getRobotState(RobotState& robotState);
    //vector<double> getJointsState();
    //void showJointState(vector<string> joint_names, vector<double> joint_values);

    int8_t getErrorCode() { return errorCode; }
    string getErrorCodeString() { return errorCodeFinder[getErrorCode()]; }
    bool getResult() { return goal_success_; }
    bool actionFinished() { return action_server_returned_; }
    void setTimeTolerance(double newTimeTolerance) { time_tolerance = newTimeTolerance; }
    void enableAsync() { async_mode = true; }
    void disableAsync() { async_mode = false; }
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
