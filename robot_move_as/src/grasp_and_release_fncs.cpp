// this fnc returns an error code.  Calling func must provide action-server result
// note: this fnc is not used by PICK, since want to integrate waiting/testing of attachment with robot motion
unsigned short int RobotMoveActionServer::grasp_fnc(double timeout) {
    unsigned short int errorCode = robot_move_as::RobotMoveResult::NO_ERROR; //return this if ultimately successful
    double t_wait = 0.0; //parameters to timeout on waiting for grasp
    double dt_wait = 0.2;
    //double t_wait_timeout = 5.0;
    bool is_attached = false;

    ROS_INFO("enabling gripper");
    grab(); //do this early, so grasp can occur at first contact

    ROS_INFO("waiting for gripper attachment");
    while (!is_attached && (t_wait < timeout)) { //move should be done in 2sec; but wait an extra second
        //grab(); //should not be a problem to repeat this command
        is_attached = robotInterface.isGripperAttached();
        ros::Duration(dt_wait).sleep();
        t_wait += dt_wait;
    }

    if (!is_attached) {
        ROS_WARN("could not grasp part within time limit; giving up");
        errorCode = robot_move_as::RobotMoveResult::GRIPPER_FAULT;
        return errorCode;
    }
    ROS_INFO("part is attached to gripper");
    return errorCode; // if here, return no error
}

unsigned short int RobotMoveActionServer::release_fnc(double timeout) {
    unsigned short int errorCode = robot_move_as::RobotMoveResult::NO_ERROR; //return this if ultimately successful
    double t_wait = 0.0; //parameters to timeout on waiting for grasp
    double dt_wait = 0.2;
    bool is_attached = true;

    ROS_INFO("releasing gripper");
    release(); //invoke release

    ROS_INFO("waiting for gripper release");
    while (is_attached && (t_wait < timeout)) { //move should be done in 2sec; but wait an extra second
        //release(); //may as well keep sending "release" commands
        is_attached = robotInterface.isGripperAttached();
        ros::Duration(dt_wait).sleep();
        ros::spinOnce();
        t_wait += dt_wait;
    }

    if (is_attached) {
        ROS_WARN("release was not successful; giving up");
        errorCode = robot_move_as::RobotMoveResult::GRIPPER_FAULT;
        return errorCode;
    }
    ROS_INFO("part is released from gripper");
    return errorCode; // if here, return no error
}

void RobotMoveActionServer::grab() {
    //ROS_INFO("enable gripper");
    gripper_client.call(attach_);
}

void RobotMoveActionServer::release() {
    //ROS_INFO("release gripper");
    gripper_client.call(detach_);
}
