//fncs to help compute key jspace poses

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

bool RobotMoveActionServer::cruise_jspace_pose(int8_t location, Eigen::VectorXd &q_vec) {
    switch (location) {
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

///given a part pose w/rt world, decide if the part is right-side up or up-side down
bool RobotMoveActionServer::eval_up_down(geometry_msgs::PoseStamped part_pose_wrt_world) {
//geometry_msgs::PoseStamped part_pose_wrt_world = part.pose;
 double qx,qy;
 qx =  part_pose_wrt_world.pose.orientation.x;
 qy = part_pose_wrt_world.pose.orientation.y;
 double sum_sqd = qx*qx+qy*qy;
 if (sum_sqd>0.5) return DOWN;
 else  return UP;
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


bool RobotMoveActionServer::get_pickup_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,
                                          Eigen::VectorXd approx_jspace_pose, bool use_wrist_far, Eigen::VectorXd &q_vec_soln) {
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

    if (use_wrist_far) {
       q_fit = fwd_solver_.get_wrist_far_soln(q6dof_solns);
    }

    else {
      q_fit = fwd_solver_.get_wrist_near_soln(q6dof_solns);
    }
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
