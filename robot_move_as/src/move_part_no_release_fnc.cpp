        case robot_move_as::RobotMoveGoal::MOVE:  //Here is the primary function of this server: pick and place
            ROS_INFO("MOVE");
            ROS_INFO("The part is %s, should be moved from %s to %s, with source pose:", goal->sourcePart.name.c_str(),
                     placeFinder[goal->sourcePart.location].c_str(), placeFinder[goal->targetPart.location].c_str());
            ROS_INFO("part source: ");
            ROS_INFO_STREAM(goal->sourcePart);
            //ROS_INFO_STREAM(goal->sourcePart.pose);
            ROS_INFO("part target:  ");
            ROS_INFO_STREAM(goal->targetPart);
            //ROS_INFO_STREAM(goal->targetPart.pose);
            ROS_INFO("Time limit is %f", timeout);
            
            //MOVE THIS TO SHIPMENT_FILLER...and create separate fnc "FLIP_PULLEY"
            /*
            source_is_up = eval_up_down(goal->sourcePart.pose);
            target_is_up = eval_up_down(goal->targetPart.pose);
            flip_part=false;
            if (source_is_up && !target_is_up) flip_part = true;
            if (!source_is_up && target_is_up) flip_part = true;
            if (flip_part) ROS_WARN("need to flip part");

            if (flip_part) {
               ROS_WARN("attempting part flip");
               errorCode = flip_part_fnc(goal);
               ROS_WARN("part-flip rtn code: %d",errorCode);
                if (errorCode != robot_move_as::RobotMoveResult::NO_ERROR) {
                    ROS_INFO("failed to flip part");
                    ROS_INFO("error code: %d", (int) errorCode);
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    as.setAborted(result_);
                    return;
                }
            }
            */

            // if here, do pick and place
            //anticipate failure, unless proven otherwise;

            result_.success = false;
            result_.errorCode = robot_move_as::RobotMoveResult::WRONG_PARAMETER;  //UNREACHABLE
            //goal->sourcePart

            //compute the necessary joint-space poses:
            if (!bin_hover_jspace_pose(goal->targetPart.location, box_hover_pose_)) {
                ROS_WARN("bin_hover_jspace_pose() failed for target ");
                as.setAborted(result_);
            }
            ROS_INFO_STREAM("box_hover: " << box_hover_pose_.transpose());

            if (!bin_hover_jspace_pose(goal->sourcePart.location, bin_hover_jspace_pose_)) {
                ROS_WARN("bin_hover_jspace_pose() failed for source bin %d", (int) goal->sourcePart.location);
                as.setAborted(result_);
                return;
            }
            ROS_INFO_STREAM("bin_hover: " << bin_hover_jspace_pose_.transpose());

            //cruise pose, adjacent to bin:
            if (!bin_cruise_jspace_pose(goal->sourcePart.location, goal->targetPart.location,
                                        bin_cruise_jspace_pose_)) {
                ROS_WARN("bin_cruise_jspace_pose() failed");
                as.setAborted(result_);
                return;
            }
            //cruise pose, adjacent to bin:
            ROS_INFO_STREAM("bin_cruise_jspace_pose_: " << bin_cruise_jspace_pose_.transpose());

            if (!box_cruise_jspace_pose(goal->targetPart.location, box_cruise_pose_)) {
                ROS_WARN("box_cruise_jspace_pose() failed");
                as.setAborted(result_);
                return;
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
                result_.errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
                as.setAborted(result_);
                return;
            }
            ROS_INFO_STREAM("pickup_jspace_pose_: " << pickup_jspace_pose_.transpose());
            //compute approach_pickup_jspace_pose_
            //compute_approach_IK(Eigen::Affine3d affine_vacuum_gripper_pose_wrt_base_link,Eigen::VectorXd approx_jspace_pose,double approach_dist,Eigen::VectorXd &q_vec_soln);
            if (!compute_approach_IK(affine_vacuum_pickup_pose_wrt_base_link_, pickup_jspace_pose_, approach_dist_,
                                     approach_pickup_jspace_pose_)) {
                ROS_WARN("could not compute IK soln for pickup approach pose!");
                result_.errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
                as.setAborted(result_);
                return;
            }

            affine_vacuum_dropoff_pose_wrt_base_link_ = affine_vacuum_dropoff_pose_wrt_base_link(goal->targetPart,
                                                                                                 box_hover_pose_[1]);
            ROS_INFO("dropoff pose wrt base link: ");
            xformUtils_.printAffine(affine_vacuum_dropoff_pose_wrt_base_link_);
            //ROS_INFO("gripper pose for drop-off: ");
            //std::cout << affine_vacuum_dropoff_pose_wrt_base_link_.translation().transpose() << std::endl;
            if (!get_pickup_IK(affine_vacuum_dropoff_pose_wrt_base_link_, box_hover_pose_, dropoff_jspace_pose_)) {
                ROS_WARN("could not compute IK soln for drop-off pose!");
                result_.errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
                as.setAborted(result_);
                return;
            }
            ROS_INFO_STREAM("dropoff_jspace_pose_: " << dropoff_jspace_pose_.transpose());
            //compute offset for dropoff
            if (!compute_approach_IK(affine_vacuum_dropoff_pose_wrt_base_link_, dropoff_jspace_pose_, approach_dist_,
                                     approach_dropoff_jspace_pose_)) {
                ROS_WARN("could not compute IK soln for dropoff approach pose!");
                result_.errorCode = robot_move_as::RobotMoveResult::UNREACHABLE;
                as.setAborted(result_);
                return;
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


            while (!is_attached && (t_wait<1.0)) {
                is_attached = robotInterface.isGripperAttached();
                ros::Duration(0.5).sleep();
                t_wait+=0.4;
                ROS_INFO("waiting for gripper attachment");
            }

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
                result_.errorCode = robot_move_as::RobotMoveResult::GRIPPER_FAULT;

                result_.success = false;
                //result_.robotState = robotState;
                as.setAborted(result_);
                return;
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
                result_.success = false;
                result_.errorCode = robot_move_as::RobotMoveResult::PART_DROPPED;
                //result_.robotState = robotState;
                ROS_WARN("part dropped!");
                as.setAborted(result_);
                return;
            }
            //do grasp test; abort if failed
            //ros::Duration(2.0).sleep(); //TUNE ME!!
            //ROS_INFO("I %s got the part", robotInterface.isGripperAttached()? "still": "did not");

            ROS_INFO("moving to dropoff_cruise_pose_");
            move_to_jspace_pose(box_cruise_pose_); //move to agv cruise pose
            ros::Duration(2.0).sleep(); //TUNE ME!!

            if (!robotInterface.isGripperAttached()) {
                //ROS_INFO("moving to agv_cruise_pose_");
                //move_to_jspace_pose(agv_cruise_pose_,1.0); //move to agv cruise pose
                //ros::Duration(1.0).sleep(); //TUNE ME!!
                result_.success = false;
                result_.errorCode = robot_move_as::RobotMoveResult::PART_DROPPED;
                //result_.robotState = robotState;
                ROS_WARN("part dropped!");
                as.setAborted(result_);
                return;
            }

            //good to here, then messes up...
            int ans;
            //cout<<"enter 1: ";
            //cin>>ans;
            //ROS_INFO("testing if part is still grasped");
            //do grasp test; abort if failed
            // ROS_INFO("I %s got the part", robotInterface.waitForGripperAttach(2.0)? "still": "did not");
            //ROS_INFO("moving to agv_hover_pose_");
            //move_to_jspace_pose(agv_hover_pose_,1.0); //move to agv hover pose
            //ros::Duration(1.0).sleep(); //TUNE ME!!

            ROS_INFO("moving to approach_dropoff_jspace_pose_");
            move_to_jspace_pose(approach_dropoff_jspace_pose_,1.0); //move to agv hover pose
            ros::Duration(1.0).sleep(); //TUNE ME!!
            //could test for grasp...but skip this here
            //cout<<"enter 1: ";
            //cin>>ans;
            
            ROS_INFO("moving to dropoff_jspace_pose_");
            move_to_jspace_pose(dropoff_jspace_pose_,2.0); //move to dropoff pose
            ros::Duration(2.5).sleep(); //add some settling time
            //cout<<"enter 1: ";
            //cin>>ans;
            
            ROS_INFO("testing if part is still grasped");
            if (!robotInterface.isGripperAttached()) {
                //move back to a safe cruise pose before aborting
                move_to_jspace_pose(box_cruise_pose_,1.0); //
                ros::Duration(1.0).sleep(); //TUNE ME!!
                //ROS_INFO("moving to agv_cruise_pose_");
                //move_to_jspace_pose(agv_cruise_pose_,1.0); //move to agv cruise pose
                //ros::Duration(1.0).sleep(); //TUNE ME!!
                result_.success = false;
                result_.errorCode = robot_move_as::RobotMoveResult::PART_DROPPED;
                //result_.robotState = robotState;
                ROS_WARN("part dropped!");
                as.setAborted(result_);
                return;
            }


            //do grasp test; if failed, return to agv_cruise pose and abort
            //ros::Duration(2.0).sleep(); //TUNE ME!!
            //ROS_INFO("I %s got the part", robotInterface.isGripperAttached()? "still": "did not");

            ROS_INFO("releasing gripper");
            //release gripper
           errorCode = release_fnc(5.0);
            if (errorCode != robot_move_as::RobotMoveResult::NO_ERROR) {
                   ROS_WARN("grasp unsuccessful; timed out");
                    result_.success = false;
                    result_.errorCode = errorCode;
                    //result_.robotState = robotState;
                    as.setAborted(result_);
                    return;
                }
            //ros::Duration(2.0).sleep(); //TUNE ME!!
            //ROS_INFO("I %s dropped the part", robotInterface.isGripperAttached()? "did not": "successfully");

            //ROS_INFO("moving to agv_hover_pose_");
            //move_to_jspace_pose(agv_hover_pose_,1.0); //move to agv hover pose
            //ros::Duration(1.0).sleep(); //TUNE ME!!

            //ROS_INFO("moving to agv_cruise_pose_");
            //move_to_jspace_pose(agv_cruise_pose_,1.0); //move to agv cruise pose
            //ros::Duration(1.0).sleep(); //TUNE ME!!
            
            ROS_INFO("moving to dropoff_cruise_pose_");
            move_to_jspace_pose(box_cruise_pose_); //move to agv cruise pose
            ros::Duration(2.0).sleep(); //TUNE ME!!

            //feedback_.robotState = robotState;
            //as.publishFeedback(feedback_);
            //ros::Duration(0.5).sleep();

            //ros::Duration(0.5).sleep();
            //feedback_.robotState = robotState;
            //as.publishFeedback(feedback_);
            //ros::Duration(0.5).sleep();
            ROS_INFO("part has been placed at target location");
            /*dt = ros::Time::now().toSec() - start_time;
            if (dt < timeout) {*/
            ROS_INFO("action completed");
            result_.success = true;
            result_.errorCode = robot_move_as::RobotMoveResult::NO_ERROR;
            //robotState = calcRobotState();
            //result_.robotState = robotState;
            as.setSucceeded(result_);
            /*
        } else {
            ROS_INFO("I am running out of time");
            result_.success = false;
            result_.errorCode = RobotMoveResult::TIMEOUT;
            result_.robotState = robotState;
            as.setAborted(result_);
        }*/
            break;

