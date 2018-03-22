// UR10 kinematics implementation file; start w/ fwd kin

#include <ariac_ur_fk_ik/ur_kin.h>

// function for use w/ both fwd and inv kin
// NOTE: q must be q in DH coords!!  use q_vec(i) + DH_q_offsets(i)

double sgn(double x) {
    if (x > 0.0) return 1.0;
    if (x < 0.0) return -1.0;
    return 0.0; //should virtually never happen for floating-point numbers
}

Eigen::Matrix4d compute_A_of_DH(double a, double d, double alpha, double q) {
    Eigen::Matrix4d A;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;

    A = Eigen::Matrix4d::Identity();
    R = Eigen::Matrix3d::Identity();
    //ROS_INFO("compute_A_of_DH: a,d,alpha,q = %f, %f %f %f",a,d,alpha,q);

    double cq = cos(q);
    double sq = sin(q);
    double sa = sin(alpha);
    double ca = cos(alpha);
    R(0, 0) = cq;
    R(0, 1) = -sq*ca; //% - sin(q(i))*cos(alpha);
    R(0, 2) = sq*sa; //%sin(q(i))*sin(alpha);
    R(1, 0) = sq;
    R(1, 1) = cq*ca; //%cos(q(i))*cos(alpha);
    R(1, 2) = -cq*sa; //%	
    //%R(3,1)= 0; %already done by default
    R(2, 1) = sa;
    R(2, 2) = ca;
    p(0) = a * cq;
    p(1) = a * sq;
    p(2) = d;
    A.block<3, 3>(0, 0) = R;
    A.col(3).head(3) = p;
    return A;
}

//alt fnc, just takes q in DH space and index of frame, 0 to 5, and uses global DH vars
Eigen::Matrix4d compute_A_of_DH(int i, double q_DH) {
    Eigen::Matrix4d A;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    double a = DH_a_params[i];
    double d = DH_d_params[i];
    double alpha = DH_alpha_params[i];
    //double q = q_ur + DH_q_offsets[i]; //convert to DH angles

    A = Eigen::Matrix4d::Identity();
    R = Eigen::Matrix3d::Identity();
    //ROS_INFO("compute_A_of_DH: a,d,alpha,q = %f, %f %f %f",a,d,alpha,q);

    double cq = cos(q_DH);
    double sq = sin(q_DH);
    double sa = sin(alpha);
    double ca = cos(alpha);
    R(0, 0) = cq;
    R(0, 1) = -sq*ca; //% - sin(q(i))*cos(alpha);
    R(0, 2) = sq*sa; //%sin(q(i))*sin(alpha);
    R(1, 0) = sq;
    R(1, 1) = cq*ca; //%cos(q(i))*cos(alpha);
    R(1, 2) = -cq*sa; //%	
    //%R(3,1)= 0; %already done by default
    R(2, 1) = sa;
    R(2, 2) = ca;
    p(0) = a * cq;
    p(1) = a * sq;
    p(2) = d;
    A.block<3, 3>(0, 0) = R;
    A.col(3).head(3) = p;
    return A;
}

void UR10FwdSolver::get_joint_names_6dof(vector<string> &jnt_names){
  jnt_names.clear();
  jnt_names.push_back("shoulder_pan_joint");
  jnt_names.push_back("shoulder_lift_joint");
  jnt_names.push_back("elbow_joint");
  jnt_names.push_back("wrist_1_joint");
  jnt_names.push_back("wrist_2_joint");
  jnt_names.push_back("wrist_3_joint");
}

void UR10FwdSolver::get_joint_names_7dof(vector<string> &jnt_names){
    jnt_names.clear();
  jnt_names.push_back("elbow_joint");
  jnt_names.push_back("linear_arm_actuator_joint");  
  jnt_names.push_back("shoulder_lift_joint");
  jnt_names.push_back("shoulder_pan_joint");
  jnt_names.push_back("wrist_1_joint");
  jnt_names.push_back("wrist_2_joint");
  jnt_names.push_back("wrist_3_joint");    
} 

//convert from DH angles to UR angles:
void UR10FwdSolver::q_DH_to_q_UR(Eigen::VectorXd q_soln_DH, Eigen::VectorXd &q_soln_UR){
    for (int i=0;i<6;i++) {
        q_soln_UR[i] = q_soln_DH[i] - DH_q_offsets[i];
    }
    fit_joints_to_range(q_soln_UR);
}

//convert from UR angles to DH angles:
void UR10FwdSolver::q_UR_to_q_DH(Eigen::VectorXd q_soln_UR, Eigen::VectorXd &q_soln_DH){
    for (int i=0;i<6;i++)
        q_soln_DH[i] = q_soln_UR[i] + DH_q_offsets[i];
}

//assumes joints in UR coords; forces values to range -pi to pi
bool UR10FwdSolver::fit_joints_to_range(Eigen::VectorXd &qvec) {
    bool fits = true;
    bool does_fit;
    double q;
    for (int i = 0; i < 6; i++) {
        q = qvec[i];
        does_fit = fit_q_to_range(q_lower_limits[i], q_upper_limits[i], q);
        qvec[i] = q;
        fits = fits&&does_fit;
    }
    if (fits)
        return true;
    else
        return false;
}

bool UR10FwdSolver::fit_q_to_range(double q_min, double q_max, double &q) {
    while (q < q_min) {
        q += 2.0 * M_PI;
    }
    while (q > q_max) {
        q -= 2.0 * M_PI;
    }
    if (q < q_min)
        return false;
    else
        return true;
}


int UR10FwdSolver::prune_solns_by_jnt_limits(vector<Eigen::VectorXd> &q_ik_solns)  {
      vector<Eigen::VectorXd> untrimmed_solns;
      untrimmed_solns = q_ik_solns;
      q_ik_solns.clear();
      int nsolns = untrimmed_solns.size();   
      ROS_INFO("there are %d untrimmed solns",nsolns);
      Eigen::VectorXd q_test;      
      for (int i=0;i<nsolns;i++) {
          q_test= untrimmed_solns[i];
          cout<<"q_soln: "<<q_test.transpose()<<endl;
          if (fit_joints_to_range(q_test)) {
              q_ik_solns.push_back(q_test);
          }
      }
      nsolns = q_ik_solns.size();
      ROS_INFO("and there are %d trimmed solns",nsolns);
      return nsolns;
}    

//given a vector of 6-DOF IK solutions, select the soln that is most consistent with q_nom
//theory: pay attention to elbow [0], shoulder_pan [2]--> righty/lefty
// and q2wrist [4] --> flip/no-flip
Eigen::VectorXd UR10FwdSolver::select_soln_near_qnom(vector<Eigen::VectorXd> q_ik_solns, Eigen::VectorXd q_nom) {

    Eigen::VectorXd q_trial,q_soln_selected;
    int num_solns = q_ik_solns.size();
    if (num_solns<1) {
        ROS_WARN("select_soln_near_qnom: no IK solns!!");
        return q_nom; //this should not happen!
    }
    
    double jspace_dist,jspace_dist_min;
    q_trial = q_ik_solns[0];
    q_soln_selected = q_trial;
    jspace_dist_min = jspace_dist_from_nom(q_nom,q_soln_selected);
    ROS_INFO_STREAM("soln 0: "<<q_trial.transpose()<<endl);
    ROS_INFO_STREAM("dist = "<<jspace_dist_min<<endl);
    for (int i=1;i<num_solns;i++) {
        q_trial = q_ik_solns[i]; 
        jspace_dist = jspace_dist_from_nom(q_nom,q_trial);
        ROS_INFO_STREAM("soln "<<i<<": "<<q_trial.transpose()<<endl);
        ROS_INFO_STREAM("dist = "<<jspace_dist<<endl);        
        if (jspace_dist<jspace_dist_min) {
            jspace_dist_min=jspace_dist;
            q_soln_selected=q_trial;
        }
    }
    return q_soln_selected;  
}

//compute distance between 6-DOF  solns, using elbow, shoulder pan and 2nd wrist ang
double UR10FwdSolver::jspace_dist_from_nom(Eigen::VectorXd q_nom, Eigen::VectorXd q_soln) {
    double dist = (q_nom[0]-q_soln[0])*(q_nom[0]-q_soln[0]);
    dist += (q_nom[2]-q_soln[2])*(q_nom[2]-q_soln[2]);
    dist += (q_nom[4]-q_soln[4])*(q_nom[4]-q_soln[4]);
    return dist;   
}

      
//provide a reference, q_ref, and a vector of q vectors, q_ik_solns;
// return the soln that is closest to q_ref
 Eigen::VectorXd UR10FwdSolver::closest_soln(Eigen::VectorXd q_ref,vector<Eigen::VectorXd> q_ik_solns) {
      int nsolns = q_ik_solns.size();

      //bool UR10FwdSolver::fit_joints_to_range(Eigen::VectorXd &qvec)
      Eigen::VectorXd q_test,q_best;
      fit_joints_to_range(q_ref);
      q_best = q_ik_solns[0];

      fit_joints_to_range(q_best);
      cout<<"trial soln: "<<q_best.transpose()<<endl;
      cout<<"reference pose: "<<q_ref.transpose()<<endl;
      q_ref[5] = q_best[5]; //ignore flange rotation
      double q_err_min = (q_ref-q_best).norm();
      double q_err;
      for (int i=1;i<nsolns;i++) {
          q_test = q_ik_solns[i];
          fit_joints_to_range(q_test);
          cout<<"trial soln: "<<q_test.transpose()<<endl;
          q_ref[5] = q_best[5]; //ignore flange rotation
          q_err = (q_ref-q_test).norm();
          if (q_err<q_err_min) {
              q_best = q_test;
              q_err_min = q_err;
          }
      }
      
      return q_best;
 }
 
 //qw1 near zero is wrist-far (suitable  for avoiding interference w/ near edge of box/bin
 //qw1 near pi is wrist-near (suitable for reaching solns farther away)
  Eigen::VectorXd UR10FwdSolver::get_wrist_near_soln(vector<Eigen::VectorXd> q_ik_solns) {
      int nsolns = q_ik_solns.size();
      double qw1; 
      //bool UR10FwdSolver::fit_joints_to_range(Eigen::VectorXd &qvec)
      Eigen::VectorXd q_best;
      //grab the first soln that qualifies as wrist near
      for (int i=0;i<nsolns-1;i++) {
        q_best = q_ik_solns[i];
        qw1 = q_best[3];
        if (qw1>1.57) return q_best;  //this one will do
      }
      // 
      q_best = q_ik_solns[nsolns-1]; //last possibility, so choose it, regardless
      return q_best;
  }
      
  Eigen::VectorXd UR10FwdSolver::get_wrist_far_soln(vector<Eigen::VectorXd> q_ik_solns) {
      int nsolns = q_ik_solns.size();
      double qw1; 
      //bool UR10FwdSolver::fit_joints_to_range(Eigen::VectorXd &qvec)
      Eigen::VectorXd q_best;
      //grab the first soln that qualifies as wrist near
      for (int i=0;i<nsolns-1;i++) {
        q_best = q_ik_solns[i];
        qw1 = q_best[3];
        if (qw1<1.57) return q_best;  //this one will do
      }
      // 
      q_best = q_ik_solns[nsolns-1]; //last possibility, so choose it, regardless
      return q_best;
  }      
      

//convert 6dof UR10 joints and provided rail displacement to consistent 7dof vector for control
 Eigen::VectorXd UR10FwdSolver::map627dof(double q_linear, Eigen::VectorXd q6dof){
	//q7dof turn is: 0elbow; 1linear; 2shoulder_lift; 3shoulder_pan; 4wrist1; 5wrist2; 6wrist3;
	//q6dof turn is: 0shoulder_pan; 1shoulder_lift; 2elbow; 3wrist1; 4wrist2; 5wrist3;
	 Eigen::VectorXd q7dof;
         q7dof.resize(7);

	q7dof[0] = q6dof[2];
	q7dof[1] = q_linear;
	q7dof[2] = q6dof[1];
	q7dof[3] = q6dof[0];
	q7dof[4] = q6dof[3];
	q7dof[5] = q6dof[4];
	q7dof[6] = q6dof[5];

	return q7dof;
}

//convert 7dof vector to 6dof joint angles of UR10, in order expected by FK/IK fncs
 Eigen::VectorXd UR10FwdSolver::map726dof(Eigen::VectorXd q7dof){
	//q7dof turn is: 0elbow; 1linear; 2shoulder_lift; 3shoulder_pan; 4wrist1; 5wrist2; 6wrist3;
	//q6dof turn is: 0shoulder_pan; 1shoulder_lift; 2elbow; 3wrist1; 4wrist2; 5wrist3;
	 Eigen::VectorXd q6dof;
         q6dof.resize(6);

	q6dof[0]=q7dof[3];
	q6dof[1]=q7dof[2];
	q6dof[2]=q7dof[0];
	q6dof[3]=q7dof[4];
	q6dof[4]=q7dof[5];
	q6dof[5]=q7dof[6];    
	return q6dof;
}

//solve the eqn K = A*cos(q) + B*sin(q) for q; return "true" if at least one soln is valid

bool UR10FwdSolver::solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns) {
    double r, cphi, sphi, phi, gamma;
    double KTOL = 0.000001; //arbitrary tolerance
    r = sqrt(A * A + B * B);
    phi = atan2(B, A);  //ambiguity: phi+M_PI is also a possibility
    q_solns.clear();
    if (fabs(K) > fabs(r)) {
        ROS_WARN("K/r is too large for a cos/sin soln");
        return false; //if |K/r|>1, no solns
    }
    //could still have trouble w/ K=0...
    if (fabs(K) < KTOL) {
        ROS_WARN("K is too small for A,B,K trig soln: user error? ");
        return false; //illegal use of this fnc
    }
    //beta == q_soln - phi 
    gamma = acos(K / r);
    //solns: q = 2npi+/- gamma + phi
    double soln1 = phi + gamma;  //forward soln
    q_solns.push_back(soln1);
    double soln2 = phi - gamma;
    q_solns.push_back(soln2);
    return true;
 

}

UR10FwdSolver::UR10FwdSolver() { //(const hand_s& hs, const atlas_frame& base_frame, double rot_ang) {

    ROS_INFO("fwd_solver constructor");
    // Transform from 6th link to end effector
    //  0, -1,  0,  0
    //  0,  0, -1,  0
    //  1,  0,  0,  0
    //  0,  0,  0,  1    
    
    A_tool = Eigen::MatrixXd::Zero(4, 4);
    A_tool(0, 1) = -1;
    A_tool(1, 2) = -1;
    A_tool(2, 0) = 1;
    A_tool(3, 3) = 1;
    Eigen::Matrix3d R_hand = Eigen:: MatrixXd::Identity(3,3);
    Eigen::Vector3d O_hand= Eigen::MatrixXd::Zero(3,1);
    //Eigen::Affine3d A_tool_wrt_flange_;
    A_tool_wrt_flange_.linear() = R_hand;
    A_tool_wrt_flange_.translation() = O_hand;
    Eigen::Vector3d O_vacuum; //= Eigen::MatrixXd::Zero(3,1);
    O_vacuum<<0,0,g_tool_offset;
    
    Eigen::Matrix3d R_vacuum(g_q_tool);

    Eigen::Affine3d affine_hand;
    affine_hand.linear() = R_vacuum;
    affine_hand.translation() = O_vacuum;  
    //convert the above affine to a 4x4
    g_A_vacuum_wrt_tool0_ = Eigen::MatrixXd::Zero(4, 4);//A_vacuum_wrt_tool0_
    g_A_vacuum_wrt_tool0_.block<3, 3>(0, 0) = R_vacuum;
    g_A_vacuum_wrt_tool0_.block<3, 1>(0, 3) = O_vacuum;
    g_A_vacuum_wrt_tool0_(3, 3) = 1.0;    
}

/*  IN CASE WANT JACOBIAN LATER...finish this
Eigen::MatrixXd irb120_hand_fwd_solver::get_Jacobian(const Vectorq6x1& q_vec) {
    solve(q_vec);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd J_ang = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd J_trans = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd zvecs = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd rvecs = Eigen::MatrixXd::Zero(3, 6);
    Eigen::Matrix4d Apalm = A_mat_products[7];
    Eigen::MatrixXd O_palm = Apalm.block<3, 1>(0, 3);
    Eigen::Matrix4d Ai;
    Eigen::MatrixXd zvec, rvec;
    Eigen::Vector3d t1, t2;
    for (int i = 0; i < 6; i++) {
        Ai = A_mat_products[i];
        zvec = Ai.block<3, 1>(0, 2); //%strip off z axis of each movable frame
        zvecs.block<3, 1>(0, i) = zvec; //%and store them
        rvec = O_palm - Ai.block<3, 1>(0, 3); //%vector from origin of i'th frame to palm 
        rvecs.block<3, 1>(0, i) = rvec;
        J_ang.block<3, 1>(0, i) = zvecs.block<3, 1>(0, i);

        t1 = zvecs.block<3, 1>(0, i);
        t2 = rvecs.block<3, 1>(0, i);
        J_trans.block<3, 1>(0, i) = t1.cross(t2);
    }

    J.block<3, 6>(0, 0) = J_trans;
    J.block<3, 6>(3, 0) = J_ang;
    if (is_lhand(hs_))return mirror_J_to_lhand(J);
    return J;
}

 */

//provide qvec in UR coords; converts to DH coords before FK computation
Eigen::Affine3d UR10FwdSolver::fwd_kin_solve(const Eigen::VectorXd& q_vec_UR) {
    Eigen::Matrix4d M;
    Eigen::VectorXd q_vec_DH;
    q_vec_DH.resize(6);
    q_UR_to_q_DH(q_vec_UR,q_vec_DH);
    M = fwd_kin_solve_(q_vec_DH);
    //add transforms to get to vacuum-gripper frame:
    //M = M*A_tool_wrt_flange_*g_A_vacuum_wrt_tool0_;
    M = M*g_A_vacuum_wrt_tool0_;
    Eigen::Affine3d A(M);
    return A;
}



Eigen::Matrix4d UR10FwdSolver::get_wrist_frame() {
    return A_mat_products[4];
}

//return soln out to tool flange; would still need to account for tool transform for gripper
// assumes jnt angles expressed in DH coords
Eigen::Matrix4d UR10FwdSolver::fwd_kin_solve_(const Eigen::VectorXd& q_vec_DH) {
    Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
    //%compute A matrix from frame i to frame i-1:
    Eigen::Matrix4d A_i_iminusi;
    Eigen::Matrix3d R;
    Eigen::Vector3d p;
    for (int i = 0; i < NJNTS; i++) {
        //A_i_iminusi = compute_A_of_DH(DH_a_params[i],DH_d_params[i],DH_alpha_params[i], q_vec[i] + DH_q_offsets[i] );
        A_i_iminusi = compute_A_of_DH(i, q_vec_DH[i]);
        A_mats[i] = A_i_iminusi;
        //std::cout << "A_mats[" << i << "]:" << std::endl;
        //std::cout << A_mats[i] << std::endl;
    }

    A_mat_products[0] = A_mats[0];
    for (int i = 1; i < NJNTS; i++) {
        A_mat_products[i] = A_mat_products[i - 1] * A_mats[i];
    }

    //Eigen::Matrix4d A5;
    //A5 = A_mat_products[5]*A_tool; //apply a tool transform
    //A_mat_products[5] = A5;
    //std::cout<<" wrist position: "<<A5(0,3)<<", "<<A5(1,3)<<", "<<A5(2,3)<<std::endl;
    return A_mat_products[NJNTS - 1]; //flange frame
}


UR10IkSolver::UR10IkSolver() {
    //constructor: 
    L_humerus = DH_a_params[1];
    L_forearm = DH_a_params[2];
    ROS_INFO("UR10IKSolver constructor");
}

//Given a desired hand orientation, compute origin at intersection of j5 and j4
//this point is not a func of any of the 3 wrist DOFs
//FINISH THIS!!
    Eigen::Vector3d UR10IkSolver::get_O4_fwd(Eigen::Affine3d const& desired_hand_pose) {
        Eigen::Vector3d O4;
        Eigen::Matrix3d R2_wrt_1,R3_wrt_2,R1_wrt_ref,R_hand_wrt_3;
        Eigen::Vector3d n_vec,t_vec,b_vec;
        //define phony wrist frames: 2DOF; first axis in reference x,y plane, in -y direction
        b_vec<<0,-1,0;
        R1_wrt_ref.col(2) = b_vec;
        n_vec<<0,0,1; //have x1 axis point straight up, so (q1,q2)=(0,0) corresponds to z_hand = (0,0,-1)
        t_vec = b_vec.cross(n_vec);
        R1_wrt_ref.col(0) = n_vec;
        R1_wrt_ref.col(1) = t_vec;
        //next frame, frame2, moves as a fnc of phony wrist q1

        t_vec<<0,-1,0; // this axis will remain in this orientation as q1 moves
        //n_vec<<    
        
        
        
        return O4;
    }
    Eigen::Vector3d UR10IkSolver::get_O4_rvrs(Eigen::Affine3d const& desired_hand_pose) {
        Eigen::Vector3d O4;
    }

//-------------------MAIN FUNCTION FOR INVERSE KINEMATICS---------------
//solve IK; return solns, in UR coordinates, in q_ik_solns
//NOTE: ignores "elbow-down" solns
//IF there are 4 solns returned, they will be in the order:
// fwd-far, fwd-near, rvrs-far, rvrs-near
int UR10IkSolver::ik_solve(Eigen::Affine3d const& desired_hand_pose,  vector<Eigen::VectorXd> &q_ik_solns) {
    Eigen::VectorXd q_ik_soln;
    q_ik_soln.resize(NJNTS);
    q_ik_solns.clear();
    Eigen::Vector3d p_des = desired_hand_pose.translation();
    //ROS_INFO_STREAM("init p_des: "<< p_des.transpose()<<endl);
    Eigen::Matrix3d R_des = desired_hand_pose.linear();


    Eigen::Matrix4d T60,T_hand; //target pose, expressed as a 4x4
    //convert affine to 4x4:
    T_hand = Eigen::MatrixXd::Zero(4, 4);
    T_hand.block<3, 3>(0, 0) = R_des;
    T_hand.block<3, 1>(0, 3) = p_des;
    T_hand(3, 3) = 1.0;    
    //ROS_INFO_STREAM("T_hand: "<<endl<<T_hand<<endl);
    
    T60 = T_hand*g_A_vacuum_wrt_tool0_.inverse(); // T_hand = T60*A_vacuum_wrt_tool0_;
//    ROS_INFO_STREAM("T60: "<<std::endl<<T60);
    
    //T60 = Eigen::MatrixXd::Zero(4, 4);
    //T60.block<3, 3>(0, 0) = R_des;
    //T60.block<3, 1>(0, 3) = p_des;
    //T60(3, 3) = 1.0;
    
    //TAKE THIS BACK OUT??
    //p_des = T60.block<3,1>(0,3); //subtracts off distance from O_6 to vacuum gripper
    //ROS_INFO_STREAM("2nd p_des: "<< p_des.transpose()<<endl);
    
    Eigen::Vector3d b6_des = T60.block<3, 1>(0, 2); //R_des.col(2); // direction of desired z-vector
    //ROS_INFO_STREAM("b6_des = "<<b6_des.transpose()<<endl);
    //Eigen::Vector3d j5_axis_des; //NEED TO COMPUTE THIS
    //
  //  ROS_INFO_STREAM("b6_des = "<<b6_des.transpose());

    //w_des is origin O5; wrist is not spherical, but z5 and z6 intersect  
    double L6 = DH_d_params[5];
//    ROS_INFO("L6 = %f",L6);// p_des is O5 at intersection of z5 and z6;
    //note: must consider wrist flip
    Eigen::Vector3d O_5 = p_des - L6*b6_des; // desired wrist position w/rt frame0; watch out for sign! can be defined in vs out
//    ROS_INFO("w_des: %f, %f, %f", w_des[0], w_des[1], w_des[2]);
    std::vector<double> q1_solns;
    std::vector<double> q5_solns_fwd, q5_solns_rvrs;
    std::vector<double> q6_solns_fwd, q6_solns_rvrs;

    q6dof_solns.clear();
    //ROS_INFO_STREAM("IK: p_des = "<<p_des.transpose()<<" ; O_5 = "<<O_5.transpose()<<endl);
    
    
    //solve for q1 first: w_des is O_5
    if (!compute_q1_solns(O_5, q1_solns)) {
        return 0; // no solns
    }
    double q1_fwd = q1_solns[0];
    double q1_rvrs = q1_solns[1];
    //ROS_INFO("q1_fwd,q1_rvrs = %f, %f",q1_fwd,q1_rvrs);
    //compute q5 solutions
    //compute_q5_solns(p_des, q1_solns, q5_solns_1a, q5_solns_1b);

    //given q1 options, can compute target pose w/rt 1 frame(s)
    // transforms to express IK w/rt frames 1_fwd, 1_rvrs
    Eigen::Matrix3d target_R61_fwd, target_R61_rvrs;
    //target_R61a is Target orientation of frame 6 w/rt frame 1, using q1a soln

    Eigen::Matrix4d A1_fwd, A1_rvrs; //4x4 transforms, frames 1 w/rt frame 0

    // T61a is from desired pose;
    //compute_A_of_DH(double a,double d,double q, double alpha); use q_vec(i) + DH_q_offsets(i)
    A1_fwd = compute_A_of_DH(0, q1_fwd); //compute_A_of_DH(DH_a_params[0],DH_d_params[0],DH_alpha_params[0], q1a+ DH_q_offsets[0] );
    A1_rvrs = compute_A_of_DH(0, q1_rvrs); //compute_A_of_DH(DH_a_params[0],DH_d_params[0],DH_alpha_params[0], q1b+ DH_q_offsets[0] );    
    Eigen::Matrix4d T61_fwd, T61_rvrs; //target 4x4 wrt frame 1a, 1b
    Eigen::Vector3d w_wrt_1_fwd, w_wrt_1_rvrs; //wrist point O_5 expressed w/rt frame 1, given q1 options
    Eigen::Vector3d b6_wrt_1_fwd, b6_wrt_1_rvrs; // z-vector of desired flange frame (frame-6), for options q1_fwd, q1_rvrs
    
    
    //Eigen::Vector3d O1_wrt_0,O6_wrt_1; //origin of frame 1 w/rt frame 0
    //R10_fwd = A1_fwd.block<3, 3>(0, 0);
    T61_fwd = A1_fwd.inverse() * T60;
    target_R61_fwd = T61_fwd.block<3, 3>(0, 0);
    //ROS_INFO_STREAM("T61_fwd: "<<endl<<T61_fwd<<endl);
    b6_wrt_1_fwd = target_R61_fwd.col(2);
    //ROS_INFO_STREAM("b6_wrt_1_fwd = "<<b6_wrt_1_fwd.transpose()<<endl);
    
    double q5_UR_fwd_wrist_far = -acos(b6_wrt_1_fwd[2]); 
    double q5_UR_fwd_wrist_near = acos(b6_wrt_1_fwd[2]);
    //ROS_INFO("q5_UR_fwd_wrist_far, q5_UR_fwd_wrist_near = %f, %f",q5_UR_fwd_wrist_far,q5_UR_fwd_wrist_near);
    q5_solns_fwd.resize(2);
    q5_solns_fwd[0] = q5_UR_fwd_wrist_far;
    q5_solns_fwd[1] = q5_UR_fwd_wrist_near;
    
    //R10_rvrs = A1_rvrs.block<3, 3>(0, 0);
    T61_rvrs = A1_rvrs.inverse() * T60;
    target_R61_rvrs = T61_rvrs.block<3, 3>(0, 0);
    b6_wrt_1_rvrs = target_R61_rvrs.col(2);
    double q5_UR_rvrs_wrist_far = acos(b6_wrt_1_rvrs[2]);  //y-component of b_des/1 --> q5
    double q5_UR_rvrs_wrist_near = -acos(b6_wrt_1_rvrs[2]);    
    //ROS_INFO("q5_UR_rvrs_wrist_far, q5_UR_rvrs_wrist_near = %f, %f",q5_UR_rvrs_wrist_far,q5_UR_rvrs_wrist_near);
    q5_solns_rvrs.resize(2);
    q5_solns_rvrs[0] = q5_UR_rvrs_wrist_far;
    q5_solns_rvrs[1] = q5_UR_rvrs_wrist_near;    
    
    
    /*
    O6_wrt_1 = T61a.block<3,1>(0,3); // extract vector from origin of frame 1 to origin frame 6
    O1_wrt_0 = A1a.block<3,1>(0,3); //vector from origin frame 0 to origin frame 1
    // will be same for cases q1a, q1b
    R10b = A1b.block<3, 3>(0, 0);
    T61b = A1b.inverse() * T60;
    target_R61b = T61b.block<3, 3>(0, 0);
    b61b = target_R61b.col(2);
    
    //compute q5 based on orientation of T61a
    compute_q5_solns_from_R(target_R61a, q5_solns_1a);    
    //ROS_INFO("q5_DH solns from q1a via R61a: %f, %f",q5_solns_1a[0],q5_solns_1a[1]);
    ROS_INFO("q5_UR solns from q1a via R61a: %f, %f",q5_solns_1a[0]-DH_q_offsets[4],
            q5_solns_1a[1]-DH_q_offsets[4]);
    
    compute_q5_solns_from_R(target_R61b, q5_solns_1b); //repeat for 2nd q1 option
    ROS_INFO("q5_UR solns from q1b via R61b: %f, %f",q5_solns_1b[0]-DH_q_offsets[4],
            q5_solns_1b[1]-DH_q_offsets[4]);    
    //compute q6 for cases q1a, q1b:
    */
    //ROS_INFO("computing q6 for case q1a: ");
    compute_q6_solns(target_R61_fwd, q5_solns_fwd, q6_solns_fwd);
    //ROS_INFO_STREAM("q6_fwd_far = "<<q6_solns_fwd[0]<<endl);
    //ROS_INFO_STREAM("q6_fwd_near = "<<q6_solns_fwd[1]<<endl);
    //ROS_INFO("computing q6 for case q1b: ");
    compute_q6_solns(target_R61_rvrs, q5_solns_rvrs, q6_solns_rvrs);  
    //ROS_INFO_STREAM("q6_rvrs_far = "<<q6_solns_rvrs[0]<<endl);
    //ROS_INFO_STREAM("q6_rvrs_near = "<<q6_solns_rvrs[1]<<endl);
    
    //compute T41, using q1, q5 and q6
    //there are 4 cases: (q1_fwd, q5_fwd_wrist_far/q6_fwd_wrist_far), (q1_fwd, q5_fwd_wrist_near/q6_fwd_wrist_near) 
    // and               (q1_rvrs, q5_rvrs_wrist_far/q6_rvrs_wrist_far), (q1_rvrs, q5_rvrs_wrist_near/q6_rvrs_wrist_near)
    // each of these has an elbow-up and elbow-down soln
    //compute T41a1:
    Eigen::Matrix4d T64_fwd_far,T64_fwd_near;
    double q5_DH, q6_DH;
    //q_soln_DH[i] = q_soln_UR[i] + DH_q_offsets[i];
    q5_DH = q5_solns_fwd[0] + DH_q_offsets[4];
    q6_DH = q6_solns_fwd[0] + DH_q_offsets[5];   
    T64_fwd_far =  compute_A_of_DH(4, q5_DH)*compute_A_of_DH(5, q6_DH);
    //ROS_INFO_STREAM("T64_fwd_far:"<<endl<<T64_fwd_far<<endl);

    
    q5_DH = q5_solns_fwd[1] + DH_q_offsets[4];
    q6_DH = q6_solns_fwd[1] + DH_q_offsets[5];    
    T64_fwd_near = compute_A_of_DH(4, q5_DH)*compute_A_of_DH(5, q6_DH);
    //ROS_INFO_STREAM("T64_fwd_near:"<<endl<<T64_fwd_near<<endl);
    
    Eigen::Matrix4d T41_fwd_far,T41_fwd_near;
    T41_fwd_far  = T61_fwd*(T64_fwd_far.inverse());
    T41_fwd_near = T61_fwd*(T64_fwd_near.inverse());
    //ROS_INFO_STREAM("T41_fwd_far:"<<endl<<T41_fwd_far<<endl);
    
    double O41x_fwd_far,O41y_fwd_far;
    O41x_fwd_far = T41_fwd_far(0,3);
    O41y_fwd_far = T41_fwd_far(1,3);
    //ROS_INFO("case T41a0: O4 w/rt frame 1: O4x, O4y = %f, %f ", O41xa,O41ya);
    //double reachT41_fwd_far = sqrt(O41x_fwd_far*O41x_fwd_far+O41y_fwd_far*O41y_fwd_far);
    vector<double> elbow_angs_fwd, shoulder_angs_fwd;
    //ROS_INFO("reach = %f",reachT41a0);
        //solve for elbow angles
    double L1 = DH_a_params[1];
    double L2 = DH_a_params[2];  
    
    //first 2 solns (elbow up/dn at q1_fwd, wrist-far q5/q6 soln)
    ROS_INFO_STREAM(endl);
    q_ik_solns.clear();
    //ROS_INFO("compute shoulder and elbow angs for q1_fwd = %f",q1_fwd);
    //ROS_INFO("fwd, wrist-far soln");    
    //ROS_INFO_STREAM("T41_fwd_far: "<<endl<<T41_fwd_far<<endl);
    if(solve_2R_planar_arm(O41x_fwd_far,O41y_fwd_far, L1, L2,shoulder_angs_fwd,elbow_angs_fwd) ) {
        //ROS_INFO("elbow angles: %f, %f ",elbow_angs_fwd[0],elbow_angs_fwd[1]);
        //ROS_INFO("shoulder angles: %f, %f",shoulder_angs_fwd[0],shoulder_angs_fwd[1]);
        
        double elbow_ang, shoulder_ang; //fwd solns, elbow "up"--> elbow angle >0
        if (elbow_angs_fwd[0]>0) {
              elbow_ang = elbow_angs_fwd[0];
              shoulder_ang = shoulder_angs_fwd[0]; 
              //ROS_INFO("elbow ang = %f, shoulder_ang = %f",elbow_ang,shoulder_ang);
        }
       
        else {
            elbow_ang = elbow_angs_fwd[1];
            shoulder_ang = shoulder_angs_fwd[1]; 
            //ROS_INFO("elbow ang = %f, shoulder_ang = %f",elbow_ang,shoulder_ang);
        }
              
        
    
        Eigen::Matrix4d T43_fwd_far,A32,A21;
        //do not need to convert q1,q2,q3 from UR to DH--offsets = 0
        A32 = compute_A_of_DH(2, elbow_ang);
        
        A21 = compute_A_of_DH(1, shoulder_ang);
        T43_fwd_far = A32.inverse()*A21.inverse()*T41_fwd_far;
        double q4_wrist_DH = atan2(T43_fwd_far(1,0),T43_fwd_far(0,0));
        double q4_wrist_UR = q4_wrist_DH- DH_q_offsets[3];
        if (q4_wrist_UR<0) q4_wrist_UR+=2.0*M_PI;       

        Eigen::VectorXd q_soln_UR;
        q_soln_UR.resize(6);
        q_soln_UR[0]= q1_fwd;
        q_soln_UR[1]= shoulder_ang;
        q_soln_UR[2]= elbow_ang;
        q_soln_UR[3] = q4_wrist_UR; //q_soln_UR[i] = q_soln_DH[i] - DH_q_offsets[i]
        q_soln_UR[4] = q5_solns_fwd[0];
        q_soln_UR[5] = q6_solns_fwd[0];        
        //first  soln is FWD, WRIST-FAR
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords
        //ROS_INFO_STREAM("soln fwd, wrist-far: "<<q_soln_UR.transpose()<<endl);
        //other elbow soln: do not include
        /*
        A32 = compute_A_of_DH(2, elbow_angs_a0[1]);
        A21 = compute_A_of_DH(1, shoulder_angs_a0[1]);
        T43a0 = A32.inverse()*A21.inverse()*T41a0;
        q_wrist = atan2(T43a0(1,0),T43a0(0,0));
        //ROS_INFO("wrist angle = %f",q_wrist);  
        q_soln_DH[1]= shoulder_angs_a0[1];
        q_soln_DH[2]=elbow_angs_a0[1];
        q_soln_DH[3] = q_wrist;     
        q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords   
        */    
    }
    // repeat for forward,  wrist-near:
    //ROS_INFO_STREAM(endl);
    //ROS_INFO("----forward, wrist-near case---- ");
    //ROS_INFO_STREAM("T41_fwd_near: "<<endl<<T41_fwd_near<<endl);
    
    double O41x_fwd_near,O41y_fwd_near;
    O41x_fwd_near = T41_fwd_near(0,3);
    O41y_fwd_near = T41_fwd_near(1,3);    
    
    //double reachT41a1 = sqrt(O41x_fwd_near*O41x_fwd_near+O41y_fwd_near*O41y_fwd_near);
    //vector<double> elbow_angs_a1, shoulder_angs_a1;
    //ROS_INFO("reach = %f",reachT41a1);
    if(solve_2R_planar_arm(O41x_fwd_near,O41y_fwd_near, L1, L2,shoulder_angs_fwd,elbow_angs_fwd) ) {
        //ROS_INFO("elbow angles: %f, %f ",elbow_angs_fwd[0],elbow_angs_fwd[1]);
        //ROS_INFO("shoulder angles: %f, %f",shoulder_angs_fwd[0],shoulder_angs_fwd[1]);        
        //ROS_INFO("elbow angles: %f, %f ",elbow_angs_a1[0],elbow_angs_a1[1]);
        //ROS_INFO("shoulder angles: %f, %f",shoulder_angs_a1[0],shoulder_angs_a1[1]);
        double elbow_ang, shoulder_ang; //fwd solns, elbow "up"--> elbow angle >0
        if (elbow_angs_fwd[0]>0) {
              elbow_ang = elbow_angs_fwd[0];
              shoulder_ang = shoulder_angs_fwd[0]; 
              //ROS_INFO("elbow ang = %f, shoulder_ang = %f",elbow_ang,shoulder_ang);
        }
       
        else {
            elbow_ang = elbow_angs_fwd[1];
            shoulder_ang = shoulder_angs_fwd[1]; 
            //ROS_INFO("elbow ang = %f, shoulder_ang = %f",elbow_ang,shoulder_ang);
        }
        
       

        Eigen::Matrix4d T43_fwd_near,A32,A21;
        A32 = compute_A_of_DH(2, elbow_ang);
        A21 = compute_A_of_DH(1, shoulder_ang);
        T43_fwd_near = A32.inverse()*A21.inverse()*T41_fwd_near;
        //cout<<"T43a0: "<<endl;
        //cout<<T43a0<<endl;
        double q4_wrist_DH = atan2(T43_fwd_near(1,0),T43_fwd_near(0,0));
        double q4_wrist_UR = q4_wrist_DH- DH_q_offsets[3];
        if (q4_wrist_UR<0) q4_wrist_UR+=2.0*M_PI;
        //ROS_INFO("wrist angle = %f",q_wrist);
        Eigen::VectorXd q_soln_DH,q_soln_UR;
        q_soln_DH.resize(6);
        q_soln_UR.resize(6);
        q_soln_UR[0]= q1_fwd;
        q_soln_UR[1]= shoulder_ang;
        q_soln_UR[2]= elbow_ang;
        q_soln_UR[3] = q4_wrist_UR; //q_soln_UR[i] = q_soln_DH[i] - DH_q_offsets[i]
        q_soln_UR[4] = q5_solns_fwd[1];
        q_soln_UR[5] = q6_solns_fwd[1];
        //q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        //2nd  soln is FWD, WRIST-NEAR
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords
        //ROS_INFO_STREAM("soln fwd, wrist-near: "<<q_soln_UR.transpose()<<endl);
    
    }
    
    //int ans;
    //cout<<"enter 1: ";
    //cin>>ans;
    //repeat for alternative q1 soln, q1b;
    
    
    
    //ROS_INFO("---------------");
    //ROS_INFO("compute soln for q1 reverse cases, q1_rvrs = %f",q1_rvrs);
     Eigen::Matrix4d T64_rvrs_far,T64_rvrs_near;
 
    q5_DH = q5_solns_rvrs[0] + DH_q_offsets[4];
    q6_DH = q6_solns_rvrs[0] + DH_q_offsets[5];   
    T64_rvrs_far =  compute_A_of_DH(4, q5_DH)*compute_A_of_DH(5, q6_DH);
    //ROS_INFO_STREAM("T64_rvrs_far:"<<endl<<T64_rvrs_far<<endl);     

    q5_DH = q5_solns_rvrs[1] + DH_q_offsets[4];
    q6_DH = q6_solns_rvrs[1] + DH_q_offsets[5];   
    T64_rvrs_near =  compute_A_of_DH(4, q5_DH)*compute_A_of_DH(5, q6_DH);
    //ROS_INFO_STREAM("T64_rvrs_near:"<<endl<<T64_rvrs_near<<endl);       
    
    vector<double> elbow_angs_rvrs, shoulder_angs_rvrs;
    Eigen::Matrix4d T41_rvrs_far,T41_rvrs_near;

    T41_rvrs_far  = T61_rvrs*T64_rvrs_far.inverse();
    T41_rvrs_near = T61_rvrs*T64_rvrs_near.inverse();
    
    //at q1b, first 2 solns (elbow up/dn at q1b, first q5/q6 soln)
    //ROS_INFO("case rvrs, wrist-far");    
    double O41x_rvrs_far,O41y_rvrs_far;
    O41x_rvrs_far = T41_rvrs_far(0,3);
    O41y_rvrs_far = T41_rvrs_far(1,3);
    //ROS_INFO_STREAM("T41_rvrs_far:"<<endl<<T41_rvrs_far<<endl);
    

    if(solve_2R_planar_arm(O41x_rvrs_far,O41y_rvrs_far, L1, L2,shoulder_angs_rvrs,elbow_angs_rvrs) ) {
        //ROS_INFO("elbow angles: %f, %f ",elbow_angs_b0[0],elbow_angs_b0[1]);
        //ROS_INFO("shoulder angles: %f, %f",shoulder_angs_b0[0],shoulder_angs_b0[1]);
        
        //rvrs far case:  q_elbow < 0
       double elbow_ang, shoulder_ang; //fwd solns, elbow "up"--> elbow angle >0
        if (elbow_angs_rvrs[0]<0) {
              elbow_ang = elbow_angs_rvrs[0];
              shoulder_ang = shoulder_angs_rvrs[0]; 
              //ROS_INFO("elbow ang = %f, shoulder_ang = %f",elbow_ang,shoulder_ang);
        }
       
        else {
            elbow_ang = elbow_angs_rvrs[1];
            shoulder_ang = shoulder_angs_rvrs[1]; 
            //ROS_INFO("elbow ang = %f, shoulder_ang = %f",elbow_ang,shoulder_ang);
        }
      
        
        Eigen::Matrix4d T43_rvrs_far,A32,A21;
        A32 = compute_A_of_DH(2, elbow_ang);
        A21 = compute_A_of_DH(1, shoulder_ang);
        T43_rvrs_far = A32.inverse()*A21.inverse()*T41_rvrs_far;

        double q4_wrist_DH = atan2(T43_rvrs_far(1,0),T43_rvrs_far(0,0));
        double q4_wrist_UR = q4_wrist_DH- DH_q_offsets[3];
        if (q4_wrist_UR<0) q4_wrist_UR+=2.0*M_PI;
        //ROS_INFO("wrist angle = %f",q_wrist);
        Eigen::VectorXd q_soln_UR;
        q_soln_UR.resize(6);
        q_soln_UR[0]= q1_rvrs;
        q_soln_UR[1]= shoulder_ang;
        q_soln_UR[2]= elbow_ang;
        q_soln_UR[3] = q4_wrist_UR; //q_soln_UR[i] = q_soln_DH[i] - DH_q_offsets[i]
        q_soln_UR[4] = q5_solns_rvrs[0];
        q_soln_UR[5] = q6_solns_rvrs[0];                
                
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords
        //ROS_INFO_STREAM("soln rvrs, wrist-far: "<<q_soln_UR.transpose()<<endl);

        //skip other elbow soln:
        /*
        A32 = compute_A_of_DH(2, elbow_angs_b0[1]);
        A21 = compute_A_of_DH(1, shoulder_angs_b0[1]);
        T43b0 = A32.inverse()*A21.inverse()*T41b0;
        q_wrist = atan2(T43b0(1,0),T43b0(0,0));
        //ROS_INFO("wrist angle = %f",q_wrist);  
        q_soln_DH[1]= shoulder_angs_b0[1];
        q_soln_DH[2]=elbow_angs_b0[1];
        q_soln_DH[3] = q_wrist;     
        q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords  
         * */     
    }
    
    
    //last 2 solns (elbow up/dn at q1b, second q5/q6 soln)    
    //ROS_INFO("case rvrs, wrist-near");    
    double O41x_rvrs_near,O41y_rvrs_near;
    O41x_rvrs_near = T41_rvrs_near(0,3);
    O41y_rvrs_near = T41_rvrs_near(1,3);
    
    
    if(solve_2R_planar_arm(O41x_rvrs_near,O41y_rvrs_near, L1, L2,shoulder_angs_rvrs,elbow_angs_rvrs) ) {
        //ROS_INFO("elbow angles: %f, %f ",elbow_angs_b1[0],elbow_angs_b1[1]);
        //ROS_INFO("shoulder angles: %f, %f",shoulder_angs_b1[0],shoulder_angs_b1[1]);
       double elbow_ang, shoulder_ang; //fwd solns, elbow "up"--> elbow angle >0
        if (elbow_angs_rvrs[0]<0) {
              elbow_ang = elbow_angs_rvrs[0];
              shoulder_ang = shoulder_angs_rvrs[0]; 
              //ROS_INFO("elbow ang = %f, shoulder_ang = %f",elbow_ang,shoulder_ang);
        }
       
        else {
            elbow_ang = elbow_angs_rvrs[1];
            shoulder_ang = shoulder_angs_rvrs[1]; 
            //ROS_INFO("elbow ang = %f, shoulder_ang = %f",elbow_ang,shoulder_ang);
        }
      
        
        Eigen::Matrix4d T43_rvrs_near,A32,A21;
        A32 = compute_A_of_DH(2, elbow_ang);
        A21 = compute_A_of_DH(1, shoulder_ang);
        T43_rvrs_near = A32.inverse()*A21.inverse()*T41_rvrs_near;

        double q4_wrist_DH = atan2(T43_rvrs_near(1,0),T43_rvrs_near(0,0));
        double q4_wrist_UR = q4_wrist_DH- DH_q_offsets[3];
        if (q4_wrist_UR<0) q4_wrist_UR+=2.0*M_PI;
        //ROS_INFO("wrist angle = %f",q_wrist);
        Eigen::VectorXd q_soln_UR;
        q_soln_UR.resize(6);
        q_soln_UR[0]= q1_rvrs;
        q_soln_UR[1]= shoulder_ang;
        q_soln_UR[2]= elbow_ang;
        q_soln_UR[3] = q4_wrist_UR; //q_soln_UR[i] = q_soln_DH[i] - DH_q_offsets[i]
        q_soln_UR[4] = q5_solns_rvrs[1];
        q_soln_UR[5] = q6_solns_rvrs[1];                
                
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords
        //ROS_INFO_STREAM("soln rvrs, wrist-near: "<<q_soln_UR.transpose()<<endl);

        //other elbow soln:
        /*
        A32 = compute_A_of_DH(2, elbow_angs_b1[1]);
        A21 = compute_A_of_DH(1, shoulder_angs_b1[1]);
        T43b1 = A32.inverse()*A21.inverse()*T41b1;
        q_wrist = atan2(T43b1(1,0),T43b1(0,0));
        //ROS_INFO("wrist angle = %f",q_wrist);  
        q_soln_DH[1]= shoulder_angs_b1[1];
        q_soln_DH[2]=elbow_angs_b1[1];
        q_soln_DH[3] = q_wrist;     
        q_DH_to_q_UR(q_soln_DH,q_soln_UR);
        q_ik_solns.push_back(q_soln_UR); //report result in UR coords     
         * */  
    }            
    return q_ik_solns.size(); 
}

//given desired tool-flange pose, find q1 solns:
// these will  correspond to FWD and RVRS;
// fill  q1_solns w/ q1_solns[0] = FWD and q1_solns[1] = RVRS
bool UR10IkSolver::compute_q1_solns(Eigen::Vector3d w_des, std::vector<double> &q1_solns) {
    q1_solns.clear();
    //ROS_WARN("compute_q1_solns:  ");

    //-s1*w_x + c1*w_y = -d4
    //solve the eqn K = A*cos(q) + B*sin(q) for q; return "true" if at least one soln is valid
    
    //solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns)
    //if (!solve_K_eq_Acos_plus_Bsin(DH_d_params[3], w_des[1], -w_des[0], q1_solns)) {
    double r =sqrt(w_des[0]*w_des[0]+w_des[1]*w_des[1]);
    double phi = atan2(w_des[1],w_des[0]);  //use this for FWD soln
    double gamma = asin(DH_d_params[3]/r);  //for FWD soln, d4 points to the left...so subtract gamma
    //ROS_INFO("d4,r = %f, %f",DH_d_params[3],r);
    //ROS_INFO("phi, gamma = %f, %f",phi,gamma);
    //TEST FOR REACHABILITY; but depends on wrist flip
    double q1_fwd = phi-gamma; //fwd
    if (q1_fwd<0) q1_fwd+=2.0*M_PI; //arbitrarily choose positive solns; can fix later, if desired
    //for reverse soln, add PI to phi, and d4 will point to the right, so ADD gamma and 
    double q1_rvrs = phi+gamma+M_PI; //rvrs
    if (q1_rvrs<0) q1_rvrs+=2.0*M_PI;
    //if (!solve_K_eq_Acos_plus_Bsin(w_des[0], r, DH_d_params[3], q1_solns)) {
        
        //ROS_WARN("compute_q1_solns:  wrist point out of reach");
        //return false; // no solns
    //}
    q1_solns.clear();
    q1_solns.push_back(q1_fwd);
    q1_solns.push_back(q1_rvrs);
    //double q1a = q1_solns[0]; //fwd soln
    //double q1b = q1_solns[1]; //q1a + M_PI; // given q1, q1+pi is also a soln
    //ROS_INFO("q1_fwd, q1_rvrs = %f, %f", q1_fwd, q1_rvrs);
    return true;
}

void UR10IkSolver::compute_q5_solns(Eigen::Vector3d p_des, std::vector<double> q1_solns,
        std::vector<double> &q5_solns_1a, std::vector<double> &q5_solns_1b) {
    //solve for q5:
    // w/rt frame 1, py = d4 + d5*c5
    // -s1*px + c1*py = d4 + d5*c5
    // q5 = +/- arccos[(-px*s1 +py*c1 -d4)/(d6)]
    double cq5;
    double q1a = q1_solns[0];
    double q1b = q1_solns[1];
    q5_solns_1a.clear(); //wrist q5 solns corresponding to first q1 soln
    q5_solns_1b.clear(); //and to second q1 soln

    cq5 = (-p_des[0] * sin(q1a) + p_des[1] * cos(q1a) - DH_d_params[3]) / DH_d_params[5];
//    ROS_INFO("cq5a: %f", cq5);
    if (fabs(cq5) > 1.0) {
        ROS_WARN("no solns for q5a");
    } else {
        q5_solns_1a.push_back(acos(cq5)); // = acos(cq5);
        q5_solns_1a.push_back(-acos(cq5)); // = -q5a_solns[0];
        //ROS_INFO("q5a solns: %f, %f", q5_solns_1a[0], q5_solns_1a[1]);
    }

    //now for other q1 soln:
    cq5 = (-p_des[0] * sin(q1b) + p_des[1] * cos(q1b) - DH_d_params[3]) / DH_d_params[5];

    if (fabs(cq5) > 1.0) {
        ROS_WARN("no solns for q5b");
    } else {
        q5_solns_1b.push_back(acos(cq5));
        q5_solns_1b.push_back(-acos(cq5));
        //ROS_INFO("q5b solns: %f, %f", q5_solns_1b[0], q5_solns_1b[1]);
    }

}

//in this version, have that R61, third col = [c234*s5; s234*s5; -c5]
/* ORIGINAL: */
void UR10IkSolver::compute_q5_solns_from_R(Eigen::Matrix3d R61, std::vector<double> &q5_solns) {
    double c234s5 = R61(0,2);
    double s234s5 = R61(1,2);
    double c5 = -R61(2,2);
    double abs_s5 = sqrt(c234s5*c234s5+s234s5*s234s5);
    double q5a = atan2(abs_s5,c5);
    double q5b = atan2(-abs_s5,c5); //may be off by 2pi
    q5_solns.clear();
    q5_solns.push_back(q5a);
    q5_solns.push_back(q5b);
}

/*
//try alternative
void UR10IkSolver::compute_q5_solns_from_R(Eigen::Matrix3d R61, std::vector<double> &q5_solns) {
    double c234c5 = R61(0,2);
    double s234c5 = R61(2,2);
    double s5 = R61(1,2);
    double abs_c5 = sqrt(c234c5*c234c5+s234c5*s234c5);
    double q5a = atan2(s5,abs_c5);
    double q5b = atan2(s5,-abs_c5);
    q5_solns.clear();
    q5_solns.push_back(q5a);
    q5_solns.push_back(q5b);
}
*/
//compute q6 for given desired orientation of 
bool UR10IkSolver::compute_q6_solns(Eigen::Matrix3d target_R61, std::vector<double> q5_solns,
        std::vector<double> &q6_solns) {
    double sign_s5, q6;
    q6_solns.clear();
    if (q5_solns.size()<1) {
        ROS_WARN("trying to compute q6, but q5_solns is empty");
        return false;
    }

   /* if multiply generic R_2/1(q2)*R_3/2(q3)*R_4/3(q4)*R_5/4(q5)*R_6/5(q6)
      last row of R_6/1(q2,q3,q4,q5,q6) is: [s5*c6; -s5*s6; c5]
    * use this to compute q6 w/ atan2
     */
    double a= target_R61(2,0); //c6 term
    double b = target_R61(2,1); //s6 term
    //compute q6 from first q5 soln:
    sign_s5 = sgn(sin(q5_solns[0]));     
    q6 = atan2(-sign_s5*b,sign_s5*a);
    //ROS_INFO("for first q5:  q6 (in DH coords) = %f",q6);
    //ROS_INFO("q6 in UR coords: %f",q6-DH_q_offsets[5]);
    q6_solns.push_back(q6);
    //repeat for 2nd q5 soln:
    sign_s5 = sgn(sin(q5_solns[1]));     
    q6 = atan2(-sign_s5*b,sign_s5*a);
    //ROS_INFO("for second q5: q6 (in DH coords) = %f",q6);
    q6_solns.push_back(q6);    
    return true;
}



bool UR10IkSolver::solve_2R_planar_arm_elbow_angs(double x_des, double y_des, double L1, double L2,
         vector<double> &q_elbow_solns) {
    const double fit_tol = 0.00001;
    double rsqrd = x_des*x_des +y_des*y_des;
    //ROS_INFO("planar arm: rsqrd, L1, L2 = %f, %f %f",rsqrd,L1,L2);
    //law of cosines: C^2 = A^2 + B^2 - 2ABcos(C_ang)
    double den = 2.0 * L1*L2;
    double num = rsqrd - L1*L1 - L2*L2;
    //test viability here...
    double c_ang = num / den;
    //ROS_INFO("solve_2R_planar_arm_elbow_angs: rsqrd, num, den = %f, %f, %f",rsqrd,num,den);
    if (c_ang > 1.0) {
        ROS_WARN("solve_2R_planar_arm_elbow_angs: destination out of reach at full elbow extension");
        ROS_WARN("x_des,y_des = %f, %f; L1, L2 = %f, %f",x_des,y_des,L1,L2);
        ROS_INFO("c_ang, num, den = %f, %f, %f",c_ang,num,den);

        return false;
    }
    //cout<<"num, den, c4 = "<<num<<", "<<den<<", "<<c4<<endl;
    double s_ang = sqrt(1 - c_ang * c_ang);

    double q_elbow_a = atan2(s_ang, c_ang);  //note: elbow ang in DH coords,  measured from fully extended=0
    //ROS_INFO_STREAM("q_elbow_a = "<<q_elbow_a<<endl);
    q_elbow_solns.clear();
    q_elbow_solns.push_back(q_elbow_a);
    
    double q_elbow_b = -q_elbow_a; //atan2(-s_ang, c_ang);
    q_elbow_solns.push_back(q_elbow_b);
    //ROS_INFO_STREAM("q_elbow_b = "<<q_elbow_b<<endl);
    return true;
}

//for planar arm in x-y plane, link lengths L1, L2, find 2 pairs of solns q1,q2
bool UR10IkSolver::solve_2R_planar_arm(double x_des, double y_des, double L1, double L2,
         vector<double> &q_shoulder_solns,vector<double> &q_elbow_solns) {
    const double fit_tol = 0.00001;
    q_shoulder_solns.clear();
    double q_shoulder;
    //ROS_INFO("solve_2R_planar_arm: x_des,y_des,L1,L2 = %f, %f,  %f,  %f",x_des,y_des,L1,L2);
    //first, solve for elbow-angle solutions: expect 2 (up/dn):
    if (!solve_2R_planar_arm_elbow_angs( x_des,  y_des,  L1,  L2, q_elbow_solns)) {
        return false;
    }
    //given elbow angles, solve for corresponding shoulder angles:
    //ROS_INFO("compute shoulder ang candidates for q_elbow = %f",q_elbow_solns[0]);
    if(!solve_2R_planar_arm_shoulder_ang(x_des,y_des, L1, L2, q_elbow_solns[0], q_shoulder)) {
        return false;
    }
    q_shoulder_solns.push_back(q_shoulder);
    //ROS_INFO("compute shoulder ang candidates for q_elbow = %f",q_elbow_solns[1]);
    if (!solve_2R_planar_arm_shoulder_ang(x_des,y_des, L1, L2, q_elbow_solns[1], q_shoulder)) {
        return false;
    }
    q_shoulder_solns.push_back(q_shoulder);    
    return true;
}

//2R planar robot solution for shoulder angle
//given x_des, y_des, elbow_angle, solve for q_shoulder that points towards x_des,y_des;
//for given elbow angle, there should be a unique shoulder angle, q_shoulder
bool UR10IkSolver::solve_2R_planar_arm_shoulder_ang(double x_des,double y_des, double L1, double L2,
           double q_elbow, double &q_shoulder) {
    //x = (L1+L2*c2)*c1 - (L2*s2)*s1
    double A = L1+L2*cos(q_elbow);  //q_elbow = 0 outstretched
    double B = -L2*sin(q_elbow);  
    const double R2_fit_err_tol = 0.000001;
    std::vector<double> q_solns;
    
    //generic soln: given q_elbow, find q_shoulder to achieve desired x-value of wrist, w/rt frame 1
    solve_K_eq_Acos_plus_Bsin(x_des,  A,  B, q_solns);

    
    double soln1 = q_solns[0]; //have 2 of these,  but only 1 is right
    double soln2 = q_solns[1];
    //ROS_INFO("shoulder angle candidates: %f,%f",soln1,soln2);    
    /*
    q_solns.clear(); //consider more solution options; save the viable ones
    
    if (fit_q_to_range(q_lower_limits[1], q_upper_limits[1], soln1)) {
        q_solns.push_back(soln1);
    }
    soln1+= M_PI;  //possible quadrant  ambiguity in solve_K_eq_Acos_plus_Bsin
     if (fit_q_to_range(q_lower_limits[1], q_upper_limits[1], soln1)) {
        q_solns.push_back(soln1);
    }   
    if (fit_q_to_range(q_lower_limits[1], q_upper_limits[1], soln2)) {
        q_solns.push_back(soln2);
    }    
    soln2+= M_PI;
     if (fit_q_to_range(q_lower_limits[1], q_upper_limits[1], soln2)) {
        q_solns.push_back(soln2);
    }      
    
    int nsolns = q_solns.size();
    ROS_INFO("there are %d shoulder lift solns: ",nsolns);  //lift=0 for arm horizontal in DH coords
    if (nsolns>2) ROS_WARN("MORE THAN 2 SHOULDER SOLNS!");
    ROS_INFO("candidate shoulder solns, to be trimmed: ");
    for (int i=0;i<nsolns;i++) {
        ROS_INFO("q_shoulder = %f",q_solns[i]);
    }
     * */
    //test which solution is correct:
    
    q_shoulder = q_solns[0]; //try the first shoulder option
    double x= L1*cos(q_shoulder) + L2*cos(q_shoulder+q_elbow);
    double y= L1*sin(q_shoulder) + L2*sin(q_shoulder+q_elbow);
    double xtest = A*cos(q_shoulder) + B*sin(q_shoulder);
    //ROS_INFO("solve_2R_planar_arm_shoulder_ang; x_des,y_des = %f, %f",x_des,y_des);
    //ROS_INFO("x,y, xtest = %f, %f, %f ",x,y,xtest);
    double fit_err = (x_des-x)*(x_des-x)+(y_des-y)*(y_des-y);
    //ROS_INFO("shoulder soln= %f;  fit err 1: %f",q_shoulder,fit_err);
    if (fit_err< R2_fit_err_tol) {
        //q_shoulder = q_solns[0];
        //pick periodicity closest to -pi/2
        double q_test1 = fabs(q_shoulder +1.57);
        double q_test2 = fabs(q_shoulder-2.0*M_PI+1.57);
        if (q_test2<q_test1) q_shoulder-= 2.0*M_PI;
        return true; }
    //for (int i=1;i<nsolns;i++) {
      q_shoulder = q_solns[1]; //try the alternative(s)
      x= L1*cos(q_shoulder) + L2*cos(q_shoulder+q_elbow);
      y= L1*sin(q_shoulder) + L2*sin(q_shoulder+q_elbow);
      //ROS_INFO(" shoulder soln 1: ");
      xtest = A*cos(q_shoulder) + B*sin(q_shoulder);
      //ROS_INFO("x,y, xtest = %f, %f, %f ",x,y,xtest);
  
      fit_err = (x_des-x)*(x_des-x)+(y_des-y)*(y_des-y);
      //ROS_INFO("shoulder soln1; fit err: %f",fit_err);
      if (fit_err< R2_fit_err_tol) {
        double q_test1 = fabs(q_shoulder +1.57);
        double q_test2 = fabs(q_shoulder-2.0*M_PI+1.57);
        if (q_test2<q_test1) q_shoulder-= 2.0*M_PI;
         return true; }  
    
    ROS_WARN("could not find shoulder-angle fit within tolerance!");

    return false;
}

 


