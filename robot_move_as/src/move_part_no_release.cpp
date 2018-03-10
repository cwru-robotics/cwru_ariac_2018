//do the following:
//get or compute key poses:
//    Eigen::VectorXd source_hover_pose_,source_cruise_pose_; 
//    Eigen::VectorXd destination_hover_pose_,destination_cruise_pose_;
//bin_hover_jspace_pose  (fixed jspace pose)
//box_hover_pose_        (fixed jspace pose)
//bin_cruise_jspace_pose_
//box_cruise_pose_

unsigned short int RobotMoveActionServer::move_part_no_release(const robot_move_as::RobotMoveGoalConstPtr &goal,double timeout) {
            unsigned short int errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
            ROS_INFO("The part is %s, should be moved from %s to %s, with source pose:", goal->sourcePart.name.c_str(),
                     placeFinder[goal->sourcePart.location].c_str(), placeFinder[goal->targetPart.location].c_str());
            ROS_INFO("part source: ");
            ROS_INFO_STREAM(goal->sourcePart);
            //ROS_INFO_STREAM(goal->sourcePart.pose);
            ROS_INFO("part target:  ");
            ROS_INFO_STREAM(goal->targetPart);
            //ROS_INFO_STREAM(goal->targetPart.pose);
            ROS_INFO("Time limit is %f", timeout);
            // do pick and place
            //anticipate failure, unless proven otherwise;

            errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;  //UNREACHABLE
            //goal->sourcePart

            //compute the necessary joint-space poses:
            if (!hover_jspace_pose(goal->targetPart.location, destination_hover_pose_)) {
                ROS_WARN("bin_hover_jspace_pose() failed for target ");
                return errorCode;
            }
            ROS_INFO_STREAM("destination_hover_pose_: " << destination_hover_pose_.transpose());

            if (!hover_jspace_pose(goal->sourcePart.location, source_hover_pose_)) {
                ROS_WARN("bin_hover_jspace_pose() failed for source bin %d", (int) goal->sourcePart.location);
                return errorCode;
            }
            ROS_INFO_STREAM("source_hover_pose_: " << source_hover_pose_.transpose());

            //cruise pose, adjacent to bin:
            //if (!bin_cruise_jspace_pose(goal->sourcePart.location, goal->targetPart.location,
            //                            destination_cruise_pose_)) {
            //    ROS_WARN("bin_cruise_jspace_pose() failed");
            //    return errorCode;
            //}
            //cruise pose, adjacent to bin:
            //ROS_INFO_STREAM("destination_cruise_pose_: " << destination_cruise_pose_.transpose());

            if (!cruise_jspace_pose(goal->targetPart.location, box_cruise_pose_)) {
                ROS_WARN("box_cruise_jspace_pose() failed");
                return errorCode;
            }
            ROS_INFO_STREAM("dropoff_cruise_pose_: " << box_cruise_pose_.transpose());

            //pick_part_fnc(const cwru_ariac::RobotMoveGoalConstPtr& goal)
            //compute the IK for this pickup pose: pickup_jspace_pose_
            //first, get the equivalent desired affine of the vacuum gripper w/rt base_link;
            //need to provide the Part info and the rail displacement
            //Eigen::Affine3d RobotMoveActionServer::affine_vacuum_pickup_pose_wrt_base_link(Part part, double q_rail)
            affine_vacuum_pickup_pose_wrt_base_link_ = affine_vacuum_pickup_pose_wrt_base_link(goal->sourcePart,
                                                                                               bin_hover_jspace_pose_[1]);
            
            ROS_INFO("pickup pose wrt base link: ");
            xformUtils_.printAffine(affine_vacuum_pickup_pose_wrt_base_link_);
            //provide desired gripper pose w/rt base_link, and choose soln closest to some reference jspace pose, e.g. hover pose
            //note: may need to go to approach pose first; default motion is in joint space
            //if (!get_pickup_IK(cart_grasp_pose_wrt_base_link,approx_jspace_pose,&q_vec_soln);
            if (!get_pickup_IK(affine_vacuum_pickup_pose_wrt_base_link_, bin_hover_jspace_pose_, pickup_jspace_pose_)) {
                ROS_WARN("could not compute IK soln for pickup pose!");
                errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
                return errorCode;
            }
            ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());
            //compute approach_pickup_jspace_pose_
            //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);
            if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
                                     approach_pickup_jspace_pose_)) {
                ROS_WARN("could not compute IK soln for pickup approach pose!");
                errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
                return errorCode;
            }

            affine_vacuum_dropoff_pose_wrt_base_link_ = affine_vacuum_dropoff_pose_wrt_base_link(goal->targetPart,
                                                                                                 box_hover_pose_[1]);
            ROS_INFO("dropoff pose wrt base link: ");
            xformUtils_.printAffine(affine_vacuum_dropoff_pose_wrt_base_link_);
            //ROS_INFO("gripper pose for drop-off: ");
            //std::cout << affine_vacuum_dropoff_pose_wrt_base_link_.translation().transpose() << std::endl;
            if (!get_pickup_IK(affine_vacuum_dropoff_pose_wrt_base_link_, box_hover_pose_, dropoff_jspace_pose_)) {
                ROS_WARN("could not compute IK soln for drop-off pose!");
                errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
                return errorCode;
            }
            ROS_INFO_STREAM("dropoff_jspace_pose_: " << dropoff_jspace_pose_.transpose());
            //compute offset for dropoff
            if (!compute_approach_IK(affine_vacuum_dropoff_pose_wrt_base_link_, dropoff_jspace_pose_, approach_dist_,
                                     approach_dropoff_jspace_pose_)) {
                ROS_WARN("could not compute IK soln for dropoff approach pose!");
                errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
                return errorCode;
            }

            //if all jspace solutions are valid, start the move sequence

            ROS_INFO("moving to bin_cruise_jspace_pose_ ");
            move_to_jspace_pose(bin_cruise_jspace_pose_); //so far, so good, so move to cruise pose in front of bin
            //at this point, have already confired bin ID is good
            ros::Duration(2.0).sleep(); //TUNE ME!!
            //now move to bin hover pose:

            //ROS_INFO("moving to bin_hover_jspace_pose_ ");
            //move_to_jspace_pose(bin_hover_jspace_pose_); //so far, so good, so move to cruise pose in front of bin
            //at this point, have already confired bin ID is good
            //ros::Duration(2.0).sleep(); //TUNE ME!!

            //now move to pickup approach pose:
            ROS_INFO("moving to approach_pickup_jspace_pose_ ");
            move_to_jspace_pose(approach_pickup_jspace_pose_,1.0); //so far, so good, so move to cruise pose in front of bin
            //at this point, have already confired bin ID is good
            ros::Duration(1.0).sleep(); //TUNE ME!!

            //now move to bin pickup pose:
            ROS_INFO("enabling gripper");
            grab(); //do this early, so grasp at first contact

            //now move to bin pickup pose:
            ROS_INFO_STREAM("moving to pickup_jspace_pose_ "<<pickup_jspace_pose_.transpose());
            move_to_jspace_pose(pickup_jspace_pose_,2.0); // try to pick up part
            ros::Duration(2.0).sleep(); //TUNE ME!!

            bool is_attached = false;
            double t_wait=0.0;
            while (!is_attached && (t_wait<1.0)) {
                is_attached = robotInterface.isGripperAttached();
                ros::Duration(0.5).sleep();
                t_wait+=0.4;
                ROS_INFO("waiting for gripper attachment");
            }
            double t_wait_timeout = 3.0;
            double dt_wait=0.1;
            if (!is_attached) {
               ROS_WARN("did not attach; trying lower");
               pickup_jspace_pose_[2]+= 0.05; // lower via shoulder-lift joint
               move_to_jspace_pose(pickup_jspace_pose_,t_wait_timeout);
               t_wait=0.0;
               while (!is_attached && (t_wait<t_wait_timeout)) {
                is_attached = robotInterface.isGripperAttached();
                ros::Duration(dt_wait).sleep();
                t_wait+=dt_wait;
                ROS_INFO("waiting for gripper attachment");
               }
            }
            if(!is_attached) {
                ROS_WARN("could not grasp part; giving up");
                errorCode = robot_move_as::RobotMoveResult::GRIPPER_FAULT;
                return errorCode;
            }
            ROS_INFO("part is attached to gripper");

            ROS_INFO("departing to approach_pickup_jspace_pose_ ");
            move_to_jspace_pose(approach_pickup_jspace_pose_,1.0); //lift part
            ros::Duration(1.0).sleep(); //TUNE ME!!

            //ROS_INFO("moving to bin hover pose");//bin_cruise_jspace_pose_
            //move_to_jspace_pose(bin_hover_jspace_pose_,1.0); //hover pose
            //ros::Duration(1.0).sleep(); //TUNE ME!!

            ROS_INFO("moving to bin_cruise_jspace_pose_ ");
            move_to_jspace_pose(bin_cruise_jspace_pose_,1.0); // move to cruise pose in front of bin
            ros::Duration(1.0).sleep(); //TUNE ME!!

            ROS_INFO("testing if part is still grasped");

            if (!robotInterface.isGripperAttached()) {
                errorCode = robot_move_as::RobotMoveResult::PART_DROPPED;
                ROS_WARN("part dropped!");
                return errorCode;
            }
            //do grasp test; abort if failed
            //ros::Duration(2.0).sleep(); //TUNE ME!!
            //ROS_INFO("I %s got the part", robotInterface.isGripperAttached()? "still": "did not");

            ROS_INFO("moving to dropoff_cruise_pose_");
            move_to_jspace_pose(box_cruise_pose_); //move to agv cruise pose
            ros::Duration(2.0).sleep(); //TUNE ME!!

            if (!robotInterface.isGripperAttached()) {
                errorCode = robot_move_as::RobotMoveResult::PART_DROPPED;
                ROS_WARN("part dropped!");
                return errorCode;
            }


            ROS_INFO("moving to approach_dropoff_jspace_pose_");
            move_to_jspace_pose(approach_dropoff_jspace_pose_,1.0); //move to agv hover pose
            ros::Duration(1.0).sleep(); //TUNE ME!!

            
            ROS_INFO("moving to dropoff_jspace_pose_");
            move_to_jspace_pose(dropoff_jspace_pose_,2.0); //move to dropoff pose
            ros::Duration(2.5).sleep(); //add some settling time

            ROS_INFO("testing if part is still grasped");
            if (!robotInterface.isGripperAttached()) {
                //move back to a safe cruise pose before aborting
                move_to_jspace_pose(box_cruise_pose_,1.0); //
                ros::Duration(1.0).sleep(); //TUNE ME!!
                errorCode = robot_move_as::RobotMoveResult::PART_DROPPED;
                ROS_WARN("part dropped!");
                return errorCode;
            }


            //do grasp test; if failed, return to agv_cruise pose and abort
            //ros::Duration(2.0).sleep(); //TUNE ME!!
            //ROS_INFO("I %s got the part", robotInterface.isGripperAttached()? "still": "did not");

            ROS_INFO("releasing gripper");
            //release gripper
           errorCode = release_fnc(5.0);
            if (errorCode != robot_move_as::RobotMoveResult::NO_ERROR) {
                   ROS_WARN("grasp unsuccessful; timed out");
                    errorCode = errorCode;
                    return errorCode;
                }

            
            ROS_INFO("moving to dropoff_cruise_pose_");
            move_to_jspace_pose(box_cruise_pose_); //move to agv cruise pose
            ros::Duration(2.0).sleep(); //TUNE ME!!

 
            ROS_INFO("part has been placed at target location");

            ROS_INFO("action completed");
            errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
            return errorCode;
}
