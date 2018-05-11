#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include<conveyor_as/conveyorAction.h>
#include <osrf_gear/ConveyorBeltState.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <xform_utils/xform_utils.h>
#include <string.h>
using namespace std;

const double BOX_SURFACE_HT_WRT_WORLD = 0.585; // from Gazebo
const int BOX_INSPECTION1_LOCATION_CODE = 2;
const int BOX_INSPECTION2_LOCATION_CODE = 3;
const int DRONE_DOCK_LOCATION_CODE = 4;

const double CONVEYOR_SPEED = 0.2; // m/sec
const double SENSOR_BLACKOUT_TIME_THREHSHOLD = 1.0; // if no sensor update within 1 sec, declare blackout

//note: travel time 2nd box was only 12.5 sec to Q1 after advancing 1st box to Q2
const double EXPECTED_CONVEYOR_ADVANCE_TIME_BOX_DISPENSER_TO_Q1 = 16.5;
const double EXPECTED_CONVEYOR_ADVANCE_TIME_BOX_Q1_TO_Q2 = 4.5;
const double EXPECTED_CONVEYOR_ADVANCE_TIME_BOX2_TO_Q1 = 16.5;
const double EXPECTED_CONVEYOR_ADVANCE_TIME_BOX_Q2_TO_DEPOT = 16.0;

const double EXPECTED_CONVEYOR_ADVANCE_TIME_BOX_Q1_TO_DEPOT = 45.0;



class ConveyorActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<conveyor_as::conveyorAction> as_;
    
    // here are some message types to communicate with our client(s)
    conveyor_as::conveyorGoal goal_; // goal message, received from client
    conveyor_as::conveyorResult result_; // put results here, to be sent back to the client when done w/ goal
    conveyor_as::conveyorFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int conveyor_task_code_;
    double travel_time_;
    ros::Time start_time_;
    ros::Time time_at_last_sensor_update_;
    double seconds_since_last_sensor_update_;
    double expected_travel_time_;


    ros::ServiceClient conveyor_client_;
    osrf_gear::ConveyorBeltControl conveyor_svc_msg_GO_;
    osrf_gear::ConveyorBeltControl conveyor_svc_msg_STOP_;
    void box_camera_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    ros::Subscriber box_camera_1_subscriber_;
    void box_camera_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
    ros::Subscriber box_camera_2_subscriber_; 
    ros::Subscriber drone_depot_laser_scan_subscriber_;
    void drone_depot_laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr & range_msg);
    ros::Subscriber congestion_sensor_;
    void congestion_sensor_callback(const osrf_gear::ConveyorBeltState::ConstPtr & conveyor_state_msg);

  double box_cam_1_dist_to_go_;
  bool box_cam1_sees_box_; 
  double box_cam_2_dist_to_go_;
  bool box_cam2_sees_box_;   
  bool drone_depot_sensor_sees_box_;
  geometry_msgs::PoseStamped box_1_stamped_pose_;
  geometry_msgs::PoseStamped box_2_stamped_pose_; 
  osrf_gear::LogicalCameraImage box_cam_1_image_, box_cam_2_image_;
  bool find_box(osrf_gear::LogicalCameraImage cam_image, double &y_val,
        geometry_msgs::Pose &cam_pose, geometry_msgs::Pose &box_pose);
  geometry_msgs::PoseStamped compute_stPose(geometry_msgs::Pose cam_pose,geometry_msgs::Pose part_pose);
   unsigned short int advance_shipment_on_conveyor(unsigned short int command_code);
   void update_state(double dt);
   void update_state_stalled(double dt);

  double conveyor_power_;

public:
    double box1_position_est;  //distance of lead box from box dispenser
    double box2_position_est;  //distance of 2nd box from box dispenser  (if exists)
    double box3_position_est;  //distance of 3rd box from box dispenser (if exists)
    bool sensors_are_active;
    bool conveyor_enabled;
    //bool drone_depot_sensor_sees_box() { return drone_depot_sensor_sees_box_;};
    ConveyorActionServer(); //define the body of the constructor outside of class definition

    ~ConveyorActionServer(void) {
    }
    XformUtils xformUtils;
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<conveyor_as::conveyorAction>::GoalConstPtr& goal);
};


ConveyorActionServer::ConveyorActionServer() :
   as_(nh_, "conveyor_as", boost::bind(&ConveyorActionServer::executeCB, this, _1),false) 
{
    ROS_INFO("in constructor of ConveyorActionServer...");
    box1_position_est=4.0; //start w/ box at dispenser
    box2_position_est = 6.4; //anticipate 2nd box located 2.4m behind lead box
    box3_position_est = 8.8; //anticipate 3rd box 4.8m behind lead box
    conveyor_client_ = nh_.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    conveyor_svc_msg_GO_.request.power = 100.0;
    conveyor_svc_msg_STOP_.request.power = 0.0;
    ROS_INFO("warming up conveyor service: ");
    for (int i = 0; i < 3; i++) {//client.call(srv)
        conveyor_client_.call(conveyor_svc_msg_STOP_);
        //ROS_INFO_STREAM("response: "<<conveyor_svc_msg_STOP_);
        ros::Duration(0.1).sleep();
    }

    //ariac/box_camera_1
    box_camera_1_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1,
            &ConveyorActionServer::box_camera_1_callback, this);
    box_cam_1_dist_to_go_ = 100.0; //init s.t. box is not yet under camera
    box_cam1_sees_box_ = false;
    //ariac/box_camera_2
    box_camera_2_subscriber_ = nh_.subscribe("/ariac/box_camera_2", 1,
            &ConveyorActionServer::box_camera_2_callback, this);
    box_cam_2_dist_to_go_ = 100.0; //init s.t. box is not yet under camera
    box_cam2_sees_box_ = false;

    drone_depot_laser_scan_subscriber_ = nh_.subscribe("/ariac/laser_profiler_drone_depot", 1,
            &ConveyorActionServer::drone_depot_laser_scan_callback, this);

    congestion_sensor_ = nh_.subscribe("/ariac/conveyor/state",1,&ConveyorActionServer::congestion_sensor_callback, this);

    drone_depot_sensor_sees_box_ = false;
    time_at_last_sensor_update_ = ros::Time::now();
    seconds_since_last_sensor_update_ = 0;
    sensors_are_active = true;

    as_.start(); //start the server running
}


void ConveyorActionServer::congestion_sensor_callback(const osrf_gear::ConveyorBeltState::ConstPtr & conveyor_state_msg) {
  conveyor_enabled = conveyor_state_msg->enabled;
  conveyor_power_ = conveyor_state_msg->power;  
}

void ConveyorActionServer::drone_depot_laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr & scan_msg) {
    double max_range = scan_msg->range_max;
    //double min_sensed_range = max_range;
    //ROS_INFO("max_range = %f", max_range);
    time_at_last_sensor_update_ = ros::Time::now();
    drone_depot_sensor_sees_box_ = false;
    int num_rays = scan_msg->ranges.size();
    for (int iray = 0; iray < num_rays; iray++) {
        if (scan_msg->ranges[iray] < max_range - 0.01) {
            drone_depot_sensor_sees_box_ = true;
            //ROS_INFO("shipment seen at drone loading dock");
        }
    }
    if (drone_depot_sensor_sees_box_) ROS_WARN("box seen at drone depot");
}

void ConveyorActionServer::box_camera_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    box_cam_1_image_ = *image_msg;
    double box_y_val; //cam_y_val;
    //bool ShipmentFiller::find_box(osrf_gear::LogicalCameraImage cam_image,double &y_val, 
    //    geometry_msgs::Pose &cam_pose, geometry_msgs::Pose &box_pose)
    geometry_msgs::Pose cam_pose, box_pose;
    box_cam1_sees_box_ = find_box(box_cam_1_image_, box_y_val, cam_pose, box_pose);
    if (box_cam1_sees_box_) {
        //ROS_INFO("cam_y_val = %f",cam_y_val);
        box_cam_1_dist_to_go_ = -box_y_val;
        //compute the box pose w/rt world
        box_1_stamped_pose_ = compute_stPose(cam_pose, box_pose);
        //correct this for known height:
        box_1_stamped_pose_.pose.position.z = BOX_SURFACE_HT_WRT_WORLD;
        //ROS_INFO("box_cam_1_dist_to_go_ = %f",box_cam_1_dist_to_go_);
    }
}

void ConveyorActionServer::box_camera_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
    box_cam_2_image_ = *image_msg;
    double box_y_val; //cam_y_val;
    //ROS_INFO("box cam2 calling find_box");
    geometry_msgs::Pose cam_pose, box_pose;
    box_cam2_sees_box_ = find_box(box_cam_2_image_, box_y_val, cam_pose, box_pose);

    if (box_cam2_sees_box_) {
        //ROS_INFO("box_cam2_sees box at cam_y_val = %f", box_y_val);
        box_cam_2_dist_to_go_ = -box_y_val;
        //ROS_INFO("box_cam_2_dist_to_go_ = %f",box_cam_2_dist_to_go_);
        box_2_stamped_pose_ = compute_stPose(cam_pose, box_pose);
    }
}

bool ConveyorActionServer::find_box(osrf_gear::LogicalCameraImage cam_image, double &y_val,
        geometry_msgs::Pose &cam_pose, geometry_msgs::Pose &box_pose) {
    int num_models = cam_image.models.size();
    if (num_models == 0) return false;
    string box_name("shipping_box");
    //cam_y_val = cam_image.pose.position.y;
    osrf_gear::Model model;
    cam_pose = cam_image.pose;
    //ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
        model = cam_image.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            y_val = model.pose.position.y;
            box_pose = model.pose;
            //ROS_INFO("found box at y_val = %f", y_val);
            return true;
        }
    }
    //if reach here, did not find shipping_box
    return false;
}


//BUG: box at Q1, can't do a 2nd Q1 command until after a Q2 cmd

unsigned short int ConveyorActionServer::advance_shipment_on_conveyor(unsigned short int command_code) {
        //$ rosservice call /ariac/conveyor/control "power: 100"
        ROS_INFO("advancing conveyor with command code %d", command_code);
        bool conveyor_ok = false;
        travel_time_=0;
        double dt = 0.1;

        switch (command_code) {
            case conveyor_as::conveyorGoal::MOVE_NEW_BOX_TO_Q1_STATION:
                conveyor_client_.call(conveyor_svc_msg_GO_);
                travel_time_=0;
                expected_travel_time_= EXPECTED_CONVEYOR_ADVANCE_TIME_BOX_DISPENSER_TO_Q1;
                ROS_INFO("expected travel time = %f",expected_travel_time_);
                ROS_INFO("trying to command conveyor to move box to Q1: ");
                box_cam1_sees_box_=false;
                while ((!box_cam1_sees_box_) && (travel_time_<expected_travel_time_) )
                { 
                   if (conveyor_enabled) { update_state(dt); } //only update travel time, states, and feedback if conveyor moving
                   else { 
                       ROS_WARN("conveyor not enabled; retriggering"); 
                       update_state_stalled(dt);
                   }
                    //be a nag--keep insisting on conveyor motion, regardless
                    conveyor_client_.call(conveyor_svc_msg_GO_); //keep trying, if congested
                }
                //stopped either due to timeout or saw first glimpse of box, so stop conveyor:
                //if here, finally dropped out of "while"; either saw box or time expired
                if (travel_time_>expected_travel_time_) {
                  ROS_WARN("should have seen box at box-cam 1, but did not; stopping");
                  ROS_WARN("most recent sensor update was %f sec ago",seconds_since_last_sensor_update_);
                  conveyor_client_.call(conveyor_svc_msg_STOP_);
                  return conveyor_as::conveyorResult::BOX_ESTIMATED_AT_Q1;
                }
                
                //if here, time not yet expired,  but DID see box; start computing distances
                ROS_INFO("now see box at box_cam_1_dist_to_go_ = %f", box_cam_1_dist_to_go_);
                while ((box_cam_1_dist_to_go_ > 0)&&(travel_time_<expected_travel_time_)) {
                   if (conveyor_enabled) { update_state(dt); } //only update travel time, states, and feedback if conveyor moving
                   else { 
                       ROS_WARN("conveyor not enabled; retriggering"); 
                       update_state_stalled(dt);
                   }
                    //be a nag--keep insisting on conveyor motion, regardless
                    conveyor_client_.call(conveyor_svc_msg_GO_); //keep trying, if congested
                }
                conveyor_client_.call(conveyor_svc_msg_STOP_);

                if (travel_time_>expected_travel_time_) {
                  ROS_WARN("should have seen box at box-cam 1, but did not; stopping");
                  ROS_WARN("most recent sensor update was %f sec ago",seconds_since_last_sensor_update_);
                  return conveyor_as::conveyorResult::BOX_ESTIMATED_AT_Q1;
                 }
                
                //if here, successfully saw box stopped at desired location
                ROS_INFO("now see box at box_cam_1_dist_to_go_ = %f", box_cam_1_dist_to_go_);
                ROS_INFO_STREAM("travel time was "<<travel_time_<<endl);
                return conveyor_as::conveyorResult::BOX_SEEN_AT_Q1;
                break;

            case conveyor_as::conveyorGoal::MOVE_BOX_Q1_TO_Q2_STATION:
                ROS_INFO("advancing box to 2nd inspection station at box cam2");

                conveyor_client_.call(conveyor_svc_msg_GO_);
                travel_time_=0;
                expected_travel_time_= EXPECTED_CONVEYOR_ADVANCE_TIME_BOX_Q1_TO_Q2;
                box_cam2_sees_box_=false;
                ROS_INFO("conveyor commanded to move");
                while ((!box_cam2_sees_box_) && (travel_time_<expected_travel_time_) )
                { 
                   ros::spinOnce();
                   if (conveyor_enabled) { update_state(dt); } //only update travel time, states, and feedback if conveyor moving
                   else { 
                       ROS_WARN("conveyor not enabled; retriggering"); 
                       update_state_stalled(dt);
                   }
                    //be a nag--keep insisting on conveyor motion, regardless
                    conveyor_client_.call(conveyor_svc_msg_GO_); //keep trying, if congested
                }

                //if here, finally dropped out of "while"; either saw box or time expired
                conveyor_client_.call(conveyor_svc_msg_STOP_);

                if (travel_time_>expected_travel_time_) {
                  ROS_WARN("should have seen box at box-cam 2, but did not; stopping");
                  ROS_WARN("most recent sensor update was %f sec ago",seconds_since_last_sensor_update_);
                  conveyor_client_.call(conveyor_svc_msg_STOP_);
                  return conveyor_as::conveyorResult::BOX_ESTIMATED_AT_Q2;
                }
                //if here, time not yet expired,  but DID see box; start computing distances
                ROS_INFO("now see box at box_cam_2_dist_to_go_ = %f", box_cam_2_dist_to_go_);
                while ((box_cam_2_dist_to_go_ > 0)&&(travel_time_<expected_travel_time_)) {
                   if (conveyor_enabled) { update_state(dt); } //only update travel time, states, and feedback if conveyor moving
                   else { 
                       ROS_WARN("conveyor not enabled; retriggering"); 
                       update_state_stalled(dt);
                   }
                    //be a nag--keep insisting on conveyor motion, regardless
                    conveyor_client_.call(conveyor_svc_msg_GO_); //keep trying, if congested
                }
                //dropped out of loop--either timeout or see box where it belongs
                conveyor_client_.call(conveyor_svc_msg_STOP_);

                if (travel_time_>expected_travel_time_) {
                  ROS_WARN("should have seen box at box-cam 2, but did not; stopping");
                  ROS_WARN("most recent sensor update was %f sec ago",seconds_since_last_sensor_update_);
                  return conveyor_as::conveyorResult::BOX_ESTIMATED_AT_Q2;
                 }
          
                //if here, successfully saw box stopped at desired location
                ROS_INFO("now see box at box_cam_2_dist_to_go_ = %f", box_cam_2_dist_to_go_);
                ROS_INFO("travel time was %f",travel_time_);
                return conveyor_as::conveyorResult::BOX_SEEN_AT_Q2;
                break;

            case conveyor_as::conveyorGoal::MOVE_BOX_Q2_TO_DRONE_DEPOT:
                ROS_INFO("advancing box from Q2 to drone depot");
                conveyor_client_.call(conveyor_svc_msg_GO_);
                travel_time_=0;
                expected_travel_time_= EXPECTED_CONVEYOR_ADVANCE_TIME_BOX_Q2_TO_DEPOT;
                drone_depot_sensor_sees_box_ = false;
                while ((!drone_depot_sensor_sees_box_)&&(travel_time_<expected_travel_time_))
                {
                   if (conveyor_enabled) { update_state(dt); } //only update travel time, states, and feedback if conveyor moving
                   else { 
                       ROS_WARN("conveyor not enabled; retriggering"); 
                       update_state_stalled(dt);
                   }
                    //be a nag--keep insisting on conveyor motion, regardless
                    conveyor_client_.call(conveyor_svc_msg_GO_); //keep trying, if congested
                }
                //dropped  out of loop, so stop the  conveyor
                conveyor_client_.call(conveyor_svc_msg_STOP_);
                
                if (travel_time_>expected_travel_time_) {
                  ROS_WARN("should have seen box at depot, but did not; stopped conveyor");
                  ROS_WARN("most recent sensor update was %f sec ago",seconds_since_last_sensor_update_);
                  return conveyor_as::conveyorResult::BOX_ESTIMATED_AT_DRONE_DEPOT;
                 }

                //if here, then sensor sees box
                ROS_INFO("prox sensor sees box!");
                ROS_INFO("travel time was %f",travel_time_);
                return conveyor_as::conveyorResult::BOX_SENSED_AT_DRONE_DEPOT;
                break;

            default: ROS_WARN("advance_shipment_on_conveyor: action code not recognized");
                return conveyor_as::conveyorResult::BAD_ACTION_CODE;
                break;
        }

        return true;
    }

geometry_msgs::PoseStamped ConveyorActionServer::compute_stPose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {
    Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;
    cam_wrt_world = xformUtils.transformPoseToEigenAffine3d(cam_pose);
    part_wrt_cam = xformUtils.transformPoseToEigenAffine3d(part_pose);
    part_wrt_world = cam_wrt_world*part_wrt_cam;
    geometry_msgs::Pose pose_part_wrt_world = xformUtils.transformEigenAffine3dToPose(part_wrt_world);
    geometry_msgs::PoseStamped part_pose_stamped;
    part_pose_stamped.header.stamp = ros::Time::now();
    part_pose_stamped.header.frame_id = "world";
    part_pose_stamped.pose = pose_part_wrt_world;
    return part_pose_stamped;
}


void ConveyorActionServer::executeCB(const actionlib::SimpleActionServer<conveyor_as::conveyorAction>::GoalConstPtr& goal) {
    conveyor_task_code_ = goal->conveyor_task_code;
    start_time_=ros::Time::now();

    result_.box_status = advance_shipment_on_conveyor(conveyor_task_code_);
    as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "conveyor_action_server_node"); // name this node 

    ROS_INFO("instantiating the conveyor action server: ");

    ConveyorActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    ros::spin();

    return 0;
}

void ConveyorActionServer::update_state(double dt) {
     conveyor_client_.call(conveyor_svc_msg_GO_);
     ros::spinOnce();
     ros::Duration(dt).sleep();
     travel_time_+=dt;
     ROS_INFO_STREAM("travel time: "<<travel_time_<<endl);
     seconds_since_last_sensor_update_ = (ros::Time::now() -time_at_last_sensor_update_).toSec();
     if (box1_position_est>0) box1_position_est+= CONVEYOR_SPEED*dt;
     if (box2_position_est>0) box2_position_est+= CONVEYOR_SPEED*dt;
     if (box3_position_est>0) box3_position_est+= CONVEYOR_SPEED*dt;
    if (seconds_since_last_sensor_update_ < SENSOR_BLACKOUT_TIME_THREHSHOLD) { sensors_are_active=true; }
    else { sensors_are_active = false; }
    feedback_.box1_distance = box1_position_est;
	feedback_.box2_distance = box2_position_est;
	feedback_.box3_distance = box3_position_est;
	feedback_.sensors_are_active = sensors_are_active;
	feedback_.estimated_seconds_to_goal = expected_travel_time_ - travel_time_; //FIX ME
        feedback_.drone_depot_sensor_sees_box = drone_depot_sensor_sees_box_;
        feedback_.conveyor_enabled = conveyor_enabled;
     as_.publishFeedback(feedback_);
}

void ConveyorActionServer::update_state_stalled(double dt) {
     conveyor_client_.call(conveyor_svc_msg_GO_);
     ros::spinOnce();
     ros::Duration(dt).sleep();
     //travel_time_+=dt;
     //ROS_INFO_STREAM("travel time: "<<travel_time_<<endl);
     seconds_since_last_sensor_update_ = (ros::Time::now() -time_at_last_sensor_update_).toSec();
    if (seconds_since_last_sensor_update_ < SENSOR_BLACKOUT_TIME_THREHSHOLD) { sensors_are_active=true; }
    else { sensors_are_active = false; }
    feedback_.box1_distance = box1_position_est;
	feedback_.box2_distance = box2_position_est;
	feedback_.box3_distance = box3_position_est;
	feedback_.sensors_are_active = sensors_are_active;
	feedback_.estimated_seconds_to_goal = expected_travel_time_ - travel_time_; //FIX ME
        feedback_.drone_depot_sensor_sees_box = drone_depot_sensor_sees_box_;
        feedback_.conveyor_enabled = conveyor_enabled;
     as_.publishFeedback(feedback_);
}
