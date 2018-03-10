//
// Created by shipei on 2/17/17.
// modified wsn 3/5/2018
//

#include <robot_move_as/RobotMoveActionServer.h>


//define this func in separate file--just to focus on its devel
//#include "fetch_part_from_conveyor_fnc.cpp"
#include "flip_part_fnc.cpp"
#include "pick_part_fnc.cpp"
#include "place_part_fnc_no_release.cpp"
#include "move_part.cpp"
#include "set_key_poses.cpp"


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

void RobotMoveActionServer::grab() {
    //ROS_INFO("enable gripper");
    gripper_client.call(attach_);
}

void RobotMoveActionServer::release() {
    //ROS_INFO("release gripper");
    gripper_client.call(detach_);
}


//given a part pose w/rt world, decide if the part is right-side up or up-side down
bool RobotMoveActionServer::eval_up_down(geometry_msgs::PoseStamped part_pose_wrt_world) {
//geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;
 double qx,qy;
 qx =  part_pose_wrt_world.pose.orientation.x;
 qy = part_pose_wrt_world.pose.orientation.y;
 double sum_sqd = qx*qx+qy*qy;
 if (sum_sqd>0.5) return DOWN;
 else  return UP;
}


//for each of the 10 key poses, extract the rail position
bool RobotMoveActionServer::rail_prepose(int8_t location, double &q_rail) {
    switch (location) {
        case Part::BIN1:
            q_rail = q_bin1_hover_pose_[1]; //extract rail position for bin1 key pose
            break;
        case Part::BIN2:
            q_rail = q_bin2_hover_pose_[1]; //extract rail position for bin1 key pose
            break;
        case Part::BIN3:
            q_rail = q_bin3_hover_pose_[1]; //extract rail position for bin1 key pose
            break;
        case Part::BIN4:
            q_rail = q_bin4_hover_pose_[1]; //extract rail position for bin1 key pose
            break;
        case Part::BIN5:
            q_rail = q_bin5_hover_pose_[1]; //extract rail position for bin1 key pose
            break;
        case Part::BIN6:
            q_rail = q_bin6_hover_pose_[1]; //extract rail position for bin1 key pose
            break;
        case Part::BIN7:
            q_rail = q_bin7_hover_pose_[1]; //extract rail position for bin1 key pose
            break;
        case Part::BIN8:
            q_rail = q_bin8_hover_pose_[1]; //extract rail position for bin1 key pose
            break;
        case Part::QUALITY_SENSOR_1:
            q_rail = q_box_Q1_hover_pose_[1]; //extract rail position for key pose
            break;     
        case Part::QUALITY_SENSOR_2:
            q_rail = q_box_Q2_hover_pose_[1]; //extract rail position for key pose
            break;               
        default:
            ROS_WARN("unrecognized location code");
            return false;
    }
    return true; // if here, got valid bin code and filled in q_rail
}

bool RobotMoveActionServer::hover_jspace_pose(int8_t bin, Eigen::VectorXd &qvec) {
    switch (bin) {
        case Part::BIN1:
            qvec = q_bin1_hover_pose_;
            return true; //valid code
            break;
        case Part::BIN2:
            qvec = q_bin2_hover_pose_;
            return true; //valid code
            break;
        case Part::BIN3:
            qvec = q_bin3_hover_pose_;
            return true; //valid code
            break;
        case Part::BIN4:
            qvec = q_bin4_hover_pose_;
            return true; //valid code
            break;
        case Part::BIN5:
            qvec = q_bin5_hover_pose_;
            return true; //valid code
            break;
        case Part::BIN6:
            qvec = q_bin6_hover_pose_;
            return true; //valid code
            break;
        case Part::BIN7:
            qvec = q_bin7_hover_pose_;
            return true; //valid code
            break;
        case Part::BIN8:
            qvec = q_bin8_hover_pose_;
            return true; //valid code
            break;
        case Part::QUALITY_SENSOR_1:
            qvec = q_box_Q1_hover_pose_;
            return true; //valid code
            break;
        case Part::QUALITY_SENSOR_2:
            qvec = q_box_Q2_hover_pose_;
            return true; //valid code
            break;            
        default:
            ROS_WARN("location code not recognized");
            return false;
    }
}

bool RobotMoveActionServer::cruise_jspace_pose(int8_t agv, Eigen::VectorXd &q_vec) {
    switch (agv) {
        case Part::BIN1:
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            break;
        case Part::BIN2:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin2_hover_pose_[1]; //set rail position same as hover pose
            break;            
        case Part::BIN3:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin3_hover_pose_[1]; //set rail position same as hover pose
            break;             
        case Part::BIN4:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin4_hover_pose_[1]; //set rail position same as hover pose
            break;  
        case Part::BIN5:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin5_hover_pose_[1]; //set rail position same as hover pose
            break;  
        case Part::BIN6:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin6_hover_pose_[1]; //set rail position same as hover pose
            break;  
        case Part::BIN7:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin7_hover_pose_[1]; //set rail position same as hover pose
            break;  
        case Part::BIN8:    
            q_vec = q_bin1_cruise_pose_; //shape is good, but change sled position
            q_vec[1] = q_bin8_hover_pose_[1]; //set rail position same as hover pose
            break;              
        case Part::QUALITY_SENSOR_1:
            q_vec = q_box_Q1_cruise_pose_;
            break;        
        case Part::QUALITY_SENSOR_2:
            q_vec = q_box_Q2_cruise_pose_;
            break; 

        default:
            ROS_WARN("unknown destination code");
            return false;
    }
    return true;
}


//3/29/17 new function, generalizes on pickup offset;
//w/ gasket, cannot pick up part at part origin, due to hole in center
//define a desired transform between gripper frame and part frame
// this should include part thickness as part of any necessary displacement from part origin
// extend this to return grasp transforms for inverted parts
//return grasp_transform, which is part pose w/rt gripper for vacuum grasp
bool RobotMoveActionServer::get_grasp_transform(Part part, Eigen::Affine3d &grasp_transform) {
//Eigen::Affine3d grasp_transform;
    bool part_is_up = eval_up_down(part.pose);
    if (!part_is_up) ROS_WARN("part is inverted");
    Eigen::Matrix3d R, R_inverted;
    R = Eigen::MatrixXd::Identity(3, 3);
    R_inverted = R; //rot pi about x--> x is 1,0,0; y = 0, -1, 0; z = 0,0,-1
    Eigen::Vector3d y_inv,z_inv;
    y_inv<<0,-1,0;
    z_inv<<0,0,-1;
    R_inverted.col(1) = y_inv;
    R_inverted.col(2) = z_inv;
    Eigen::Vector3d O_part_wrt_gripper;
    O_part_wrt_gripper << 0, 0, 0; //= Eigen::MatrixXd::Zero(3, 1);
    grasp_transform.linear() = R;
    //default: transform is identity, zero offset--> part frame = gripper frame
    grasp_transform.translation() = O_part_wrt_gripper;

    string part_name(part.name); //a C++ string
    if (part_name.compare("gear_part") == 0) {
      if(part_is_up) {
        O_part_wrt_gripper[2] = -(GEAR_PART_THICKNESS);
        O_part_wrt_gripper[1] = GEAR_PART_GRASP_Y_OFFSET; // offset to avoid touching dowell
        grasp_transform.translation() = O_part_wrt_gripper;
        return true;
       }
      else {
        ROS_WARN("inverted grasp not defined for this part");
        return false;
      }
    }
    //piston_rod_part
    if (part_name.compare("piston_rod_part") == 0) {
       if(part_is_up) {
        O_part_wrt_gripper[2] = -(PISTON_ROD_PART_THICKNESS + 0.003);
        grasp_transform.translation() = O_part_wrt_gripper;
        return true;
       }
      else {
        ROS_WARN("inverted grasp not defined for this part");
        return false;
      }
    }
    //disk_part
    if (part_name.compare("disk_part") == 0) {
       if(part_is_up) {
        O_part_wrt_gripper[2] = -(DISK_PART_THICKNESS + DISK_PART_GRASP_Z_OFFSET);
        grasp_transform.translation() = O_part_wrt_gripper;
        return true;
       }
      else {
        ROS_WARN("inverted grasp not defined for this part");
        return false;
      }
    }
    //gasket_part
    if (part_name.compare("gasket_part") == 0) {
       if(part_is_up) {
        O_part_wrt_gripper[2] = -(GASKET_PART_THICKNESS)+GASKET_PART_GRASP_Z_OFFSET; // manual tweak for grasp from conveyor
        //for gasket, CANNOT grab at center!! there is a hole there
        O_part_wrt_gripper[0] = GASKET_PART_GRASP_X_OFFSET; // TUNE ME; if negative, then hit
        grasp_transform.translation() = O_part_wrt_gripper;
        return true;
       }
      else {
        ROS_WARN("inverted grasp not defined for this part");
        return false;
      }
    }
    if (part_name.compare("pulley_part") == 0) {
       if(part_is_up) {
        O_part_wrt_gripper[2] = -(PULLEY_PART_THICKNESS + PULLEY_PART_GRASP_Z_OFFSET);
        grasp_transform.translation() = O_part_wrt_gripper;
        return true;
       }
      else {
        ROS_WARN("using inverted pulley-part grasp transform");
        O_part_wrt_gripper[2] = -PULLEY_PART_GRASP_Z_OFFSET-PULLEY_PART_GRASP_Z_OFFSET; //-(PULLEY_PART_THICKNESS + PULLEY_PART_GRASP_Z_OFFSET);
        grasp_transform.translation() = O_part_wrt_gripper;
        grasp_transform.linear() =  R_inverted;
        return true;
      }
    }
    ROS_WARN("get_grasp_transform: part name not recognized: %s", part.name.c_str());
    return false; // don't recognize part, so just return zero

}


//for each part, there is a vertical offset from the part frame to the gripper frame on top surface
//return this value; only needs part.name
//******** generalize this to get T_grasp = T_part_frame/gripper_frame



double RobotMoveActionServer::get_pickup_offset(Part part) {
    double offset;
    string part_name(part.name); //a C++ string
    if (part_name.compare("gear_part") == 0) {
        offset = GEAR_PART_THICKNESS +
                 0.007; //0.007 seems perfect, within 1mm, for bin; but had to add addl 4mm for pickup from tray
        return offset;
    }
    //piston_rod_part
    if (part_name.compare("piston_rod_part") == 0) {
        offset = PISTON_ROD_PART_THICKNESS + 0.005; //0.005 correction looks very good for pickup from bin
        return offset;
    }
    //disk_part
    if (part_name.compare("disk_part") == 0) {
        offset = DISK_PART_THICKNESS + 0.005; //0.005 correction looks very good for pickup from bin
        return offset;
    }
    //gasket_part
    if (part_name.compare("gasket_part") == 0) {
        offset = GASKET_PART_THICKNESS + 0.005; //try adjusting for conveyor pickup
        return offset;
    }

    ROS_WARN("part name not recognized");
    return 0.0; // don't recognize part, so just return zero

}

double RobotMoveActionServer::get_surface_height(Part part) {
    switch (part.location) {
        case Part::QUALITY_SENSOR_1:
        case Part::QUALITY_SENSOR_2:
            return CONVEYOR_HEIGHT; 
            break;
        case Part::BIN1:
        case Part::BIN2:
        case Part::BIN3:
        case Part::BIN4:
        case Part::BIN5:
        case Part::BIN6:
        case Part::BIN7:
        case Part::BIN8:
            return BIN_HEIGHT;
            break;
        default:
            ROS_WARN("surface code not recognized");
            return 0;
    }

}

//assume drop-off poses specify the tray height;
//therefore, need to add part thickness for gripper clearance
double RobotMoveActionServer::get_dropoff_offset(Part part) {
    double offset;
    string part_name(part.name); //a C++ string
    offset = get_pickup_offset(part) + 0.006; //just pad w/ clearance to drop
    return offset;

    //obsolete below here
    if (part_name.compare("gear_part") == 0) {
        offset = get_pickup_offset(part) + 0.006; //assumes frame is at bottom of part, plus add clearance for drop
        //0.01 cm was virtually perfect
        return offset;
    }
    //piston_rod_part
    if (part_name.compare("piston_rod_part") == 0) {
        offset = get_pickup_offset(part) +
                 0.006; //assumes frame is at bottom of part; try 1cm offset; 7mm was not enough
        // 1cm correction looks very good--very small drop
        return offset;
    }
    ROS_WARN("part name not recognized");
    return 0.0; // don't recognize part, so just return zero
}

/*
bool RobotMoveActionServer::bin_cruise_jspace_pose(int8_t location, Eigen::VectorXd &q_vec) {
    //double q_rail;
    
    switch (location) {
        case Part::QUALITY_SENSOR_1:
            q_vec = q_box_Q1_cruise_pose_;
            break;     
        case Part::QUALITY_SENSOR_2:
            q_vec = q_box_Q2_cruise_pose_;
            break;               
        default:
            ROS_WARN("unknown location code");
            return false;
    }
    
    return true;
}

bool RobotMoveActionServer::bin_cruise_jspace_pose(int8_t bin, int8_t box, Eigen::VectorXd &q_vec) {
    double q_rail;
    
    switch (box) {
        case Part::QUALITY_SENSOR_1:
            q_vec = q_box_Q1_cruise_pose_;
            break;     
        case Part::QUALITY_SENSOR_2:
            q_vec = q_box_Q2_cruise_pose_;
            break;               
        default:
            ROS_WARN("unknown location code");
            return false;
    }
    
    q_vec = q_bin_cruise_pose_;
    //now, adjust the rail position to correspond to named bin:
    if (!rail_prepose(bin, q_rail)) {
        ROS_WARN("unknown BIN code");
        return false;
    }
    q_vec[1] = q_rail;
    if (bin==Part::BIN8) {
       q_vec[1]-= 0.6;  //trouble: hits frame when holding pulley part, so move closer to center of rail
     }
    return true;
}
*/

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

//given a rail displacement, compute the corresponding robot base_frame w/rt world coordinates and return as an affine3
Eigen::Affine3d RobotMoveActionServer::affine_base_link(double q_rail) {
    Eigen::Affine3d affine_base_link_wrt_world;
    Eigen::Quaterniond q;
    Eigen::Vector3d Oe;
    Oe[0] = 0.3;
    Oe[1] = q_rail+1.0;
    Oe[2] = 0.8;

    q.x() = 0;
    q.y() = 0;
    q.z() = 0;
    q.w() = 1;
    Eigen::Matrix3d Re(q);
    affine_base_link_wrt_world.linear() = Re;
    affine_base_link_wrt_world.translation() = Oe;
    return affine_base_link_wrt_world;
}

Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail) {
    Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link;
    Eigen::Affine3d affine_part_wrt_base_link;
    //from part, extract pose w/rt world
    geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;
    Eigen::Affine3d affine_part_wrt_world, affine_base_link_wrt_world;
    affine_base_link_wrt_world = affine_base_link(q_rail);
    affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);

    //manual repair of pickup height:
    //Eigen::Vector3d Oe;
    //Oe = affine_part_wrt_world.translation();

    //Oe[2]=get_surface_height(part); //assumes part frame should be flush with target surface
    //affine_part_wrt_world.translation() = Oe;

    affine_part_wrt_base_link = affine_base_link_wrt_world.inverse() * affine_part_wrt_world;
    if (!get_grasp_transform(part, grasp_transform_)) {
        ROS_WARN("did not recognize this part; using identity grasp transform");
    }

    //compute desired gripper pose from part pose and appropriate grasp transform
    //gripper_wrt_base = T_part_wrt_base*T_gripper_wrt_part
    affine_vacuum_gripper_pose_wrt_base_link = affine_part_wrt_base_link * grasp_transform_.inverse();

    //generalize this to use grasp transform!
    //double pickup_offset = get_pickup_offset(part);
    //add this to the z component of the gripper pose:
    // Eigen::Vector3d Oe;
    //Oe = affine_vacuum_gripper_pose_wrt_base_link.translation();
    //Oe[2]=get_surface_height(part)+pickup_offset-BASE_LINK_HEIGHT;
    //affine_vacuum_gripper_pose_wrt_base_link.translation() = Oe;
    return affine_vacuum_gripper_pose_wrt_base_link;
}

Eigen::Affine3d RobotMoveActionServer::affine_vacuum_dropoff_pose_wrt_base_link(Part part, double q_rail) {
    //from part, extract pose w/rt world
    //also, from part name, get vertical offset of grasp pose from agv-dropoff frame;
    //note: agv frame is at BOTTOM of part, but vaccum gripper must be at TOP of part
    //destination pose refers to bottom of part on AGV tray
    //have: agv1_tray_frame_wrt_world_
    Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link;
    Eigen::Affine3d affine_part_wrt_base_link, affine_part_wrt_world;
    //from part, extract pose w/rt world
    //geometry_msgs::PoseStamped part_pose_wrt_agv = part.pose; //this is presumably w/rt tray frame
    geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;  //nope--w/rt world
    string frame_name(part.pose.header.frame_id);
    cout << frame_name << endl;
    ROS_INFO_STREAM("requested part pose w/rt world: " << part_pose_wrt_world);
    //ROS_INFO("part frame: %s",part.pose.header.frame_id);
    Eigen::Affine3d affine_part_wrt_tray, affine_base_link_wrt_world;
    affine_base_link_wrt_world = affine_base_link(q_rail);
    //affine_part_wrt_tray = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_agv);
    affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);

    //manual repair of dropoff height:
    Eigen::Vector3d Oe;
    Oe = affine_part_wrt_world.translation();

    //what is the height of the surface on which we want to place the part?
    Oe[2] = get_surface_height(part); //assumes part frame should be flush with target surface
    bool part_is_up = eval_up_down(part.pose);
        //HACK ADJUSTMENT FOR INVERTED PULLEY:
    // to place an inverted pulley on a surface, its origin should be PULLEY_PART_THICKNESS above the surface;
    string part_name(part.name); //a C++ string
    if (part_name.compare("pulley_part") == 0) {
      if (!part_is_up) {
           ROS_WARN("part is inverted");
           Oe[2] += PULLEY_PART_THICKNESS+PULLEY_PART_GRASP_Z_OFFSET;
      }
    }

    affine_part_wrt_world.translation() = Oe;
    ROS_WARN("I will instead use  part dropoff pose w/rt world of: ");
    xformUtils_.printAffine(affine_part_wrt_world);

    //affine_part_wrt_world = agv1_tray_frame_wrt_world_*affine_part_wrt_tray;
    affine_part_wrt_base_link = affine_base_link_wrt_world.inverse() * affine_part_wrt_world;
    ROS_INFO("dropoff part affine w/rt base link");
    xformUtils_.printAffine(affine_part_wrt_base_link);

    if (!get_grasp_transform(part, grasp_transform_)) {
        ROS_WARN("did not recognize this part; using identity grasp transform");
    }
    affine_vacuum_gripper_pose_wrt_base_link = affine_part_wrt_base_link * grasp_transform_.inverse();
    ROS_INFO("dropoff gripper affine w/rt base link");
    xformUtils_.printAffine(affine_vacuum_gripper_pose_wrt_base_link);
    // affine_vacuum_gripper_pose_wrt_base_link= affine_part_wrt_base_link; //start here, and offset height of gripper

    // double dropoff_offset = get_dropoff_offset(part);
    //add this to the z component of the gripper pose:

    return affine_vacuum_gripper_pose_wrt_base_link;
}

//given desired affine of gripper w/rt base_link, and given an approximate jspace solution, fill in a complete 7dof soln, if possible
//return false if no IK soln, else true
bool RobotMoveActionServer::get_pickup_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,
                                          Eigen::VectorXd approx_jspace_pose, Eigen::VectorXd &q_vec_soln) {
    bool success = true;
    std::vector<Eigen::VectorXd> q6dof_solns;
    Eigen::VectorXd q6dof_ref;
    q6dof_ref = fwd_solver_.map726dof(approx_jspace_pose); //convert to 6dof
    int nsolns = ik_solver_.ik_solve(affine_vacuum_gripper_pose_wrt_base_link, q6dof_solns);
    //std::cout << "number of IK solutions: " << nsolns << std::endl;
    nsolns = fwd_solver_.prune_solns_by_jnt_limits(q6dof_solns);
    if (nsolns == 0) {
        ROS_WARN("NO viable IK solutions found");
        return false; // NO SOLUTIONS
    }
    double q_rail = approx_jspace_pose[1];
    //select the solution that is closest to some reference--try q_6dof_bin6_pickup_pose
    Eigen::VectorXd q_fit;
    q_fit = fwd_solver_.closest_soln(q6dof_ref, q6dof_solns);
    cout << "best fit soln: " << q_fit.transpose() << endl;


    q_vec_soln = fwd_solver_.map627dof(q_rail, q_fit);
    ROS_INFO("q7: [%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]", q_vec_soln[0],
             q_vec_soln[1], q_vec_soln[2], q_vec_soln[3], q_vec_soln[4], q_vec_soln[5], q_vec_soln[6]);

    return success;
}

//function to compute an approach pose:
//specify the Eigen::Affine3d of grasp pose (pickup or dropoff); specify the IK solution to be used for this grasp pose;
//specify the approach vertical standoff distance (e.g. 5cm);
//return (via reference variable) the IK solution for this approach
//return false if no solution exists
bool RobotMoveActionServer::compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,
                                                Eigen::VectorXd approx_jspace_pose, double approach_dist,
                                                Eigen::VectorXd &q_vec_soln) {
    bool success = true;
    std::vector<Eigen::VectorXd> q6dof_solns;
    Eigen::VectorXd q6dof_ref;
    q6dof_ref = fwd_solver_.map726dof(approx_jspace_pose); //convert to 6dof
    //compute the affine of the approach pose:
    Eigen::Matrix3d R_grasp;  //approach pose should have identical orientation
    R_grasp = affine_vacuum_gripper_pose_wrt_base_link.linear();
    Eigen::Vector3d zvec, O_grasp, O_approach;
    zvec = R_grasp.col(2); //approach pose should back off along pure z direction (typically, vertical)
    O_grasp = affine_vacuum_gripper_pose_wrt_base_link.translation();
    O_approach = O_grasp + approach_dist * zvec; //compute offset origin relative to grasp origin
    Eigen::Affine3d approach_affine; //fill in components of approach affine
    approach_affine = affine_vacuum_gripper_pose_wrt_base_link;
    approach_affine.translation() = O_approach;
    //compute IK solutions for approach:

    int nsolns = ik_solver_.ik_solve(approach_affine, q6dof_solns);
    //std::cout << "number of IK solutions: " << nsolns << std::endl;
    nsolns = fwd_solver_.prune_solns_by_jnt_limits(q6dof_solns);
    if (nsolns == 0) {
        ROS_WARN("NO viable IK solutions found for approach IK");
        return false; // NO SOLUTIONS
    }
    double q_rail = approx_jspace_pose[1];
    //select the solution that is closest to the chosen grasp IK soln
    Eigen::VectorXd q_fit;
    q_fit = fwd_solver_.closest_soln(q6dof_ref, q6dof_solns);
    cout << "best fit soln for approach: " << q_fit.transpose() << endl;
    //convert this back to a 7DOF vector, including rail pose
    q_vec_soln = fwd_solver_.map627dof(q_rail, q_fit);
    ROS_INFO("q7: [%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]", q_vec_soln[0],
             q_vec_soln[1], q_vec_soln[2], q_vec_soln[3], q_vec_soln[4], q_vec_soln[5], q_vec_soln[6]);

    return success;
}



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
            //result_.robotState = robotState;
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
            
        case robot_move_as::RobotMoveGoal::DISCARD_GRASPED_PART_Q1:
            ROS_INFO("DISCARD_GRASPED_PART_Q1");
            ROS_INFO("moving to discard pose");
            move_to_jspace_pose(q_box_Q1_hover_pose_,2.0);
            ros::Duration(2.0).sleep();
            move_to_jspace_pose(q_Q1_discard_pose_,2.0);
            ros::Duration(2.0).sleep();
            ROS_INFO("releasing part");
            errorCode = release_fnc(2.0);
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
        case robot_move_as::RobotMoveGoal::TO_PREDEFINED_POSE:
            //ROS_INFO("moving to AGV1 hover pose");
            result_.success = true;
            switch (goal->predfinedPoseCode) {
                //various cases for pre-defined poses go here:
                case robot_move_as::RobotMoveGoal::QUAL_SENSOR_1_CRUISE_POSE:
                    ROS_INFO("moving to QUAL_SENSOR_1_CRUISE_POSE");
                    move_to_jspace_pose(q_box_Q1_cruise_pose_);
                    break;
                case robot_move_as::RobotMoveGoal::QUAL_SENSOR_1_HOVER_POSE:
                    ROS_INFO("moving to QUAL_SENSOR_1_HOVER_POSE ");
                    move_to_jspace_pose(q_box_Q1_hover_pose_);
                    break;
                case robot_move_as::RobotMoveGoal::DISCARD_GRASPED_PART_Q1:
                    ROS_INFO("moving to DISCARD_GRASPED_PART_Q1 ");
                    move_to_jspace_pose(q_Q1_discard_pose_);
                    break;                    
                default:
                    ROS_WARN("predefined move code not implemented!");
                    result_.success = false;
                    result_.errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;
                    as.setAborted(result_);
            }
            if (result_.success == true) {
                joint_trajectory_publisher_.publish(traj_);
                result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
                //result_.robotState = robotState;
                ros::Duration(2.0).sleep();
                as.setSucceeded(result_);
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
