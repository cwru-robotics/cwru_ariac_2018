// KUKA kinematics implementation file; start w/ fwd kin

#include <kuka_fk_ik/kuka_kin.h>

// function for use w/ both fwd and inv kin

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

/*
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
*/

//convert from DH angles to UR angles:

//assumes joints in UR coords; forces values to range -pi to pi
bool KukaFwdSolver::fit_joints_to_range(Eigen::VectorXd &qvec) {
    bool fits = true;
    bool does_fit;
    double q;
    for (int i = 0; i < 7; i++) {
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

bool KukaFwdSolver::fit_q_to_range(double q_min, double q_max, double &q) {
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


int KukaFwdSolver::prune_solns_by_jnt_limits(vector<Eigen::VectorXd> &q_ik_solns)  {
      vector<Eigen::VectorXd> untrimmed_solns;
      untrimmed_solns = q_ik_solns;
      q_ik_solns.clear();
      int nsolns = untrimmed_solns.size();   
      //ROS_INFO("there are %d untrimmed solns",nsolns);
      Eigen::VectorXd q_test;      
      for (int i=0;i<nsolns;i++) {
          q_test= untrimmed_solns[i];
          cout<<"q_soln: "<<q_test.transpose()<<endl;
          if (fit_joints_to_range(q_test)) {
              q_ik_solns.push_back(q_test);
          }
      }
      nsolns = q_ik_solns.size();
      //ROS_INFO("and there are %d trimmed solns",nsolns);
      return nsolns;
}    

//given a vector of 6-DOF IK solutions, select the soln that is most consistent with q_nom
//theory: pay attention to elbow [0], shoulder_pan [2]--> righty/lefty
// and q2wrist [4] --> flip/no-flip
Eigen::VectorXd KukaFwdSolver::select_soln_near_qnom(vector<Eigen::VectorXd> q_ik_solns, Eigen::VectorXd q_nom) {

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
    //ROS_INFO_STREAM("soln 0: "<<q_trial.transpose()<<endl);
    //ROS_INFO_STREAM("dist = "<<jspace_dist_min<<endl);
    for (int i=1;i<num_solns;i++) {
        q_trial = q_ik_solns[i]; 
        jspace_dist = jspace_dist_from_nom(q_nom,q_trial);
        //ROS_INFO_STREAM("soln "<<i<<": "<<q_trial.transpose()<<endl);
        //ROS_INFO_STREAM("dist = "<<jspace_dist<<endl);        
        if (jspace_dist<jspace_dist_min) {
            jspace_dist_min=jspace_dist;
            q_soln_selected=q_trial;
        }
    }
    return q_soln_selected;  
}

//compute distance between 6-DOF  solns, using elbow, shoulder pan and 2nd wrist ang
//alt: consider shoulder pan, shoulder elevation, elbow ang, and smaller influence of wrist bend
double KukaFwdSolver::jspace_dist_from_nom(Eigen::VectorXd q_nom, Eigen::VectorXd q_soln) {
    double dist = (q_nom[0]-q_soln[0])*(q_nom[0]-q_soln[0]); //base jnt
    dist += (q_nom[1]-q_soln[1])*(q_nom[1]-q_soln[1]); //shoulder   
    dist += (q_nom[2]-q_soln[2])*(q_nom[2]-q_soln[2]); //elbow
    dist += 0.01*(q_nom[4]-q_soln[4])*(q_nom[4]-q_soln[4]); // care less about wrist soln
    //double dist = (q_nom-q_soln).norm();
    return dist;   
}

      
//provide a reference, q_ref, and a vector of q vectors, q_ik_solns;
// return the soln that is closest to q_ref
/*
 Eigen::VectorXd q_soln::closest_soln(Eigen::VectorXd q_ref,vector<Eigen::VectorXd> q_ik_solns) {
      int nsolns = q_ik_solns.size();

      Eigen::VectorXd q_test,q_best;
      fit_joints_to_range(q_ref);
      q_best = q_ik_solns[0];

      fit_joints_to_range(q_best);
      cout<<"trial soln: "<<q_best.transpose()<<endl;
      cout<<"reference pose: "<<q_ref.transpose()<<endl;
      q_ref[6] = q_best[6]; //ignore flange rotation
      double q_err_min = (q_ref-q_best).norm();
      double q_err;
      for (int i=1;i<nsolns;i++) {
          q_test = q_ik_solns[i];
          fit_joints_to_range(q_test);
          cout<<"trial soln: "<<q_test.transpose()<<endl;
          q_ref[6] = q_best[6]; //ignore flange rotation
          q_err = (q_ref-q_test).norm();
          if (q_err<q_err_min) {
              q_best = q_test;
              q_err_min = q_err;
          }
      }
      
      return q_best;
 }
 */
 //qw1 near zero is wrist-far (suitable  for avoiding interference w/ near edge of box/bin
 //qw1 near pi is wrist-near (suitable for reaching solns farther away)
 /*
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
*/      

//convert 7dof Kuka joints and provided rail displacement to consistent 8dof vector for control
 Eigen::VectorXd KukaFwdSolver::map728dof(double q_linear, Eigen::VectorXd q7dof){
	 Eigen::VectorXd q8dof;
         q8dof.resize(8);
         for (int i=0;i<7;i++) {
             q8dof[i] = q7dof[i];
         }
	q8dof[7] = q_linear;
	return q8dof;
}

//convert 8dof vector to 6dof joint angles of Kuka, in order expected by FK/IK fncs
 Eigen::VectorXd KukaFwdSolver::map827dof(Eigen::VectorXd q8dof){
	 Eigen::VectorXd q7dof;
         q7dof.resize(7);
         for (int i=0;i<7;i++) q7dof[i] = q8dof[i];
  
	return q7dof;
}

//solve the eqn K = A*cos(q) + B*sin(q) for q; return "true" if at least one soln is valid

bool KukaFwdSolver::solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns) {
    double r, cphi, sphi, phi, gamma;
    double KTOL = 0.000001; //arbitrary tolerance
    r = sqrt(A * A + B * B);
    phi = atan2(B, A);  
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
    //ROS_INFO("K, A, B, phi, gamma = %f, %f, %f, %f, %f",K,A,B,phi,gamma);
    //solns: q = 2npi+/- gamma + phi
    double soln1 = phi + gamma;  //forward soln
    q_solns.push_back(soln1);
    double soln2 = phi - gamma;
    q_solns.push_back(soln2);
    return true;
 

}

KukaFwdSolver::KukaFwdSolver() { //(const hand_s& hs, const atlas_frame& base_frame, double rot_ang) {

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
    
    Eigen::Matrix3d R_vacuum(g_quat_tool);

    Eigen::Affine3d affine_hand;
    affine_hand.linear() = R_vacuum;
    affine_hand.translation() = O_vacuum;  
    //convert the above affine to a 4x4
    g_A_vacuum_wrt_tool0_ = Eigen::MatrixXd::Zero(4, 4);//A_vacuum_wrt_tool0_
    g_A_vacuum_wrt_tool0_.block<3, 3>(0, 0) = R_vacuum;
    g_A_vacuum_wrt_tool0_.block<3, 1>(0, 3) = O_vacuum;
    g_A_vacuum_wrt_tool0_(3, 3) = 1.0;    
}


//provide qvec in UR coords; converts to DH coords before FK computation
Eigen::Affine3d KukaFwdSolver::fwd_kin_solve(const Eigen::VectorXd& q_vec_UR) {
    Eigen::Matrix4d M;
    Eigen::VectorXd q_vec_DH;
    q_vec_DH.resize(6);
    q_Kuka_to_q_DH(q_vec_UR,q_vec_DH);
    M = fwd_kin_solve_(q_vec_DH);
    //ROS_INFO_STREAM("ee frame w/rt link0: "<<endl<<M<<endl);
    //add transforms to get to vacuum-gripper frame:
    //M = M*A_tool_wrt_flange_*g_A_vacuum_wrt_tool0_;
    M = M*g_A_vacuum_wrt_tool0_;
    Eigen::Affine3d A(M);
    return A;
}



Eigen::Matrix4d KukaFwdSolver::get_wrist_frame() {
    return A_mat_products[5];
}

//return soln out to tool flange; would still need to account for tool transform for gripper
// assumes jnt angles expressed in DH coords
Eigen::Matrix4d KukaFwdSolver::fwd_kin_solve_(const Eigen::VectorXd& q_vec_DH) {
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
        //ROS_INFO_STREAM("Aprod["<<i<<"] = "<<endl<<A_mat_products[i]<<endl);
    }

    //Eigen::Matrix4d A5;
    //A5 = A_mat_products[5]*A_tool; //apply a tool transform
    //A_mat_products[5] = A5;
    //std::cout<<" wrist position: "<<A5(0,3)<<", "<<A5(1,3)<<", "<<A5(2,3)<<std::endl;
    return A_mat_products[NJNTS - 1]; //flange frame
}


KukaIkSolver::KukaIkSolver() {
    //constructor: 
    L_humerus = DH_a_params[1];
    L_forearm = DH_a_params[2];
    ROS_INFO("KukaIkSolver constructor");
}


//-------------------MAIN FUNCTION FOR INVERSE KINEMATICS---------------
//provide desired hand pose, vacuum-face frame w/rt link-0 frame
//solve IK; return solns in q_ik_solns
//NOTE: ignores "elbow-down" solns and selects q1 closest to 0.
// expect 2 solns: --q1 near zero and elbow up with 2 wrist options
//NOTE: does NOT use redundant DOF; freezes jnt3 at 0

int KukaIkSolver::ik_solve(Eigen::Affine3d const& desired_hand_pose,  vector<Eigen::VectorXd> &q_ik_solns) {
    Eigen::VectorXd q_ik_soln;
    q_ik_soln.resize(NJNTS);
    q_ik_soln<<0,0,0,0,0,0,0;
    q_ik_solns.clear();
    Eigen::Vector3d p_des = desired_hand_pose.translation();
    //ROS_INFO_STREAM("init p_des: "<< p_des.transpose()<<endl);
    Eigen::Matrix3d R_des = desired_hand_pose.linear();  //R_des of vacuum-gripper frame w/rt link0
    //note: define vacuum-gripper frame as having b_des pointing INTO flange, so align part-z w/ flange-z


    Eigen::Matrix4d T70,T_hand; //target pose, expressed as a 4x4
    //convert affine to 4x4:
    T_hand = Eigen::MatrixXd::Zero(4, 4);
    T_hand.block<3, 3>(0, 0) = R_des;
    T_hand.block<3, 1>(0, 3) = p_des;
    T_hand(3, 3) = 1.0;    
    //ROS_INFO_STREAM("T_hand: "<<endl<<T_hand<<endl);
    
    T70 = T_hand*g_A_vacuum_wrt_tool0_.inverse(); // T_hand = T60*A_vacuum_wrt_tool0_;
    Eigen::Matrix3d R_des_70 = T70.block<3,3>(0,0);
    
    Eigen::Vector3d b7_des = T70.block<3, 1>(0, 2); //R_des.col(2); // direction of desired z-vector

    double Lwrist = DH_d_params[6];
    Eigen::Vector3d O_wrist = p_des - Lwrist*b7_des; // desired wrist position w/rt frame0; watch out for sign! can be defined in vs out
    
    //solve for q1 first: w_des is O_wrist
    vector<double> q1_solns;
    if (!compute_q1_solns(O_wrist, q1_solns)) {
        ROS_WARN("ik_solve could not find any q1 solns--something is wrong");
        return 0; // no solns
    }
    int n_q1_solns = q1_solns.size();
    for (int i=0;i<n_q1_solns;i++) {
        ROS_INFO("q1 soln %d = %f",i,q1_solns[i]);
    }
    bool go_on = true;    
    double q1 = q1_solns[0];
    //ROS_INFO("q1 = %f",q1);
           
    // transforms to express IK w/rt frames 1_fwd, 1_rvrs
    Eigen::Matrix3d target_R71;// 
    Eigen::Matrix4d A10; //4x4 transforms, frames 1 w/rt frame 0
    A10 = compute_A_of_DH(0, q1); //compute_A_of_DH(DH_a_params[0],DH_d_params[0],DH_alpha_params[0], q1a+ DH_q_offsets[0] );
    Eigen::Matrix4d T71; //, T61_rvrs; //target 4x4 wrt frame 1a, 1b
    T71 = A10.inverse() * T70;   
    Eigen::Vector3d wrist_wrt_1;
    
    target_R71 = T71.block<3, 3>(0, 0);
    Eigen::Vector3d b7_wrt_1 = target_R71.col(2);
    //ROS_INFO_STREAM("b7_wrt_1 = "<<b7_wrt_1.transpose()<<endl);
    Eigen::Vector3d O7_wrt_1= T71.block<3, 1>(0, 3);
    //wrist pose, w/rt frame 1; expect z-val = 0
    Eigen::Vector3d w_wrt_1 = O7_wrt_1 - b7_wrt_1*Lwrist;
    //ROS_INFO_STREAM("w_wrt_1: "<<w_wrt_1.transpose());
    
    //now, solve a 2-link planar arm for w_wrt_1[0],w_wrt_1[1]
    double L1 = DH_d_params[2];// 2 and 4
    double L2 = DH_d_params[4];
    double q_shoulder,q_elbow;
    std::vector<double> q_elbow_solns;
    if (!solve_2R_planar_arm_elbow_angs(w_wrt_1[0],w_wrt_1[1],  L1,  L2, q_elbow_solns)) {
        return 0; //zero solutions if can't solve for elbow
    }    
    
    
    bool is_fwd = true;  //is goal fwd or rvrs?  --> sign of elbow
    //elbow rotation is negative for forward solns; positive for rvrs solns;
    //  first elbow soln is positive, 2nd is negative
    if (w_wrt_1[0]<0) { 
          is_fwd=false; 
          q_elbow = q_elbow_solns[0];    //positive  elbow soln      
          //ROS_INFO("seeking rvrs soln, using q_elbow = %f",q_elbow);
          if (!compute_shoulder_ang(w_wrt_1[0],w_wrt_1[1],  L1,  L2, q_elbow, q_shoulder)) {
              ROS_WARN("problem computing q_shoulder!");
              return 0;
          }
          
    }
    else {
        //ROS_INFO("seeking fwd soln");
          q_elbow = q_elbow_solns[1];     //negative  elbow soln     
          //ROS_INFO("seeking fwd soln, using q_elbow = %f",q_elbow);
          if (!compute_shoulder_ang(w_wrt_1[0],w_wrt_1[1],  L1,  L2, q_elbow, q_shoulder)) {
              ROS_WARN("problem computing q_shoulder!");
              return 0;  
          }
    }
    //if here, so far, so good:
        q_ik_soln[0] = q1;        
        q_ik_soln[1] = q_shoulder;
        q_ik_soln[2] = 0.0; //arbitrarily freeze this joint
        q_ik_soln[3] = q_elbow;
        //ROS_INFO_STREAM("partial soln: "<<q_ik_soln.transpose()<<endl);
        //now fill in wrist angles in locations 4,5,6
    
    q_ik_solns.clear();
    //Eigen::Matrix4d A21,A32,A43,T40;
    //A21 = compute_A_of_DH(1, q_shoulder);    
    //A32 = compute_A_of_DH(2, 0);
    //A43 = compute_A_of_DH(3, q_elbow);
    //T40 = A10*A21*A32*A43;
    bool is_singular = solve_spherical_wrist(q_ik_soln,R_des_70, q_ik_solns);
    
    //repeat for alternative  q1: 
    //--------------------
    q1 = q1_solns[1]; //180-deg rot of base joint
    A10 = compute_A_of_DH(0, q1); //compute_A_of_DH(DH_a_params[0],DH_d_params[0],DH_alpha_params[0], q1a+ DH_q_offsets[0] );
    T71 = A10.inverse() * T70;   
    
    target_R71 = T71.block<3, 3>(0, 0);
    b7_wrt_1 = target_R71.col(2);
    //ROS_INFO_STREAM("b7_wrt_1 = "<<b7_wrt_1.transpose()<<endl);
    O7_wrt_1= T71.block<3, 1>(0, 3);
    //wrist pose, w/rt frame 1; expect z-val = 0
    w_wrt_1 = O7_wrt_1 - b7_wrt_1*Lwrist;
    //ROS_INFO_STREAM("w_wrt_1: "<<w_wrt_1.transpose());
    
    //now, solve a 2-link planar arm for w_wrt_1[0],w_wrt_1[1]
    // these should be redundant with above
    if (!solve_2R_planar_arm_elbow_angs(w_wrt_1[0],w_wrt_1[1],  L1,  L2, q_elbow_solns)) {
        return 0; //zero solutions if can't solve for elbow
    }    
    
    
    is_fwd = true;  //is goal fwd or rvrs?  --> sign of elbow
    //elbow rotation is negative for forward solns; positive for rvrs solns;
    //  first elbow soln is positive, 2nd is negative
    if (w_wrt_1[0]<0) { 
          is_fwd=false; 
          q_elbow = q_elbow_solns[0];    //positive  elbow soln      
          //ROS_INFO("seeking rvrs soln, using q_elbow = %f",q_elbow);
          if (!compute_shoulder_ang(w_wrt_1[0],w_wrt_1[1],  L1,  L2, q_elbow, q_shoulder)) {
              ROS_WARN("problem computing q_shoulder!");
              return 0;
          }
          
    }
    else {
        //ROS_INFO("seeking fwd soln");
          q_elbow = q_elbow_solns[1];     //negative  elbow soln     
          //ROS_INFO("seeking fwd soln, using q_elbow = %f",q_elbow);
          if (!compute_shoulder_ang(w_wrt_1[0],w_wrt_1[1],  L1,  L2, q_elbow, q_shoulder)) {
              ROS_WARN("problem computing q_shoulder!");
              return 0;  
          }
    }
    //since have not yet considered joint limits, this 2nd soln should also exist
        q_ik_soln[0] = q1;        
        q_ik_soln[1] = q_shoulder;
        q_ik_soln[2] = 0.0; //arbitrarily freeze this joint
        q_ik_soln[3] = q_elbow;
        //ROS_INFO_STREAM("partial soln: "<<q_ik_soln.transpose()<<endl);
        //now fill in wrist angles in locations 4,5,6
    
    //q_ik_solns.clear();
    //Eigen::Matrix4d A21,A32,A43,T40;
    //A21 = compute_A_of_DH(1, q_shoulder);    
    //A32 = compute_A_of_DH(2, 0);
    //A43 = compute_A_of_DH(3, q_elbow);
    //T40 = A10*A21*A32*A43;
    is_singular = solve_spherical_wrist(q_ik_soln,R_des_70, q_ik_solns);    
    
    
    
    //------------------
    
    
    
    int nsolns = q_ik_solns.size();
    if (nsolns<1)
    {
        ROS_WARN("could not find wrist solns!");
    }    

    return nsolns; 
}

//there are 2 q1 solns, ignoring joint limits
// should always have a reachable soln, unless out of reach
bool KukaIkSolver::compute_q1_solns(Eigen::Vector3d w_des, std::vector<double> &q1_solns) {
    q1_solns.clear();
    //ROS_WARN("compute_q1_solns:  ");

    //-s1*w_x + c1*w_y = -d4
    //solve the eqn K = A*cos(q) + B*sin(q) for q; return "true" if at least one soln is valid
    
    //solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns)
    //if (!solve_K_eq_Acos_plus_Bsin(DH_d_params[3], w_des[1], -w_des[0], q1_solns)) {
    double r =sqrt(w_des[0]*w_des[0]+w_des[1]*w_des[1]);
    double phi = atan2(w_des[1],w_des[0]);  
    //ROS_INFO("y,x,phi = %f, %f, %f",w_des[1],w_des[0],phi);
    double q1_near_0 = phi;
    while (q1_near_0<-M_PI/2.0) q1_near_0+=M_PI;
    while (q1_near_0>M_PI/2.0) q1_near_0-=M_PI;
    q1_solns.push_back(q1_near_0);
    double q1;
    if (q1_near_0<0) q1 = q1_near_0+M_PI;
    else q1 = q1_near_0-M_PI;
    q1_solns.push_back(q1);
    return true;
}

//find the + forearm-rotation orientation solution, qw1, qw2, qw3--not concerned with joint limits;
// note: if qw2 is near zero, then at a wrist singularity; 
// inf solutions of qw1+D, qw1-D
// use q1, q2, q3 from q_in; copy these values to q_solns, and tack on the two solutions qw1, qw2, qw3
//CAREFUL: this fnc APPENDS solns to q_solns;  must initialize q_solns in calling pgm
bool KukaIkSolver::solve_spherical_wrist(Eigen::VectorXd q_in,Eigen::Matrix3d R_des, std::vector<Eigen::VectorXd> &q_solns) {
    bool is_singular = false;
    Eigen::Matrix4d A01,A12,A23,A03,A34,A04,A45,A05,A56,A06;
    A01 = compute_A_of_DH(0, q_in[0]);
    A12 = compute_A_of_DH(1, q_in[1]);
    A23 = compute_A_of_DH(2, q_in[2]);
    A34 = compute_A_of_DH(3, q_in[3]);
    A04 = A01*A12*A23*A34;   
    //ROS_INFO_STREAM("R_des: "<<endl<<R_des<<endl);

    Eigen::Vector3d n4,t4,b4; // axes of frame4
    Eigen::Vector3d n5,t5,b5; // axes of frame5; 
    Eigen::Vector3d n6,t6,b6; //axes of frame6
    
    Eigen::Vector3d n_des,b_des; // desired x-axis and z-axis of flange frame
    n4 = A04.col(0).head(3);
    t4 = A04.col(1).head(3);    
    b4 = A04.col(2).head(3);  
    b_des = R_des.col(2);
    n_des = R_des.col(0);
    
    b5 = -b4.cross(b_des); //wrist-bend axis
    double qw1,qw2,qw3;
    
    Eigen::VectorXd q_soln;
    q_soln.resize(7);
      if (b5.norm() <= 0.000001) {
                qw2=0;
                is_singular = true;
      }
      else {
            double cqw1= b5.dot(t4);
            double sqw1= b5.dot(-n4); 
            qw1= atan2(sqw1, cqw1);
        }
    // choose the positive forearm-rotation solution:
    if (qw1>M_PI) {
        qw1-= 2*M_PI;
    }    
    if (qw1<0.0) {
        qw1+= M_PI;
    }
    double qw1b = qw1 -M_PI;
    //ROS_INFO("qw1 options: %f, %f",qw1,qw1b);
           
    //    A34 = compute_A_of_DH(3, q_in[3]);
    // use the + qw1 soln to find qw2, qw3
    A45 = compute_A_of_DH(4, qw1);
    A05 = A04*A45;
    n5 = A05.col(0).head(3);
    t5 = A05.col(1).head(3); 
    double cqw2 = b_des.dot(-t5);
    double sqw2 = b_des.dot(n5);
    qw2 = atan2(sqw2,cqw2);
    //ROS_INFO("for qw1 = %f, --> qw2 = %f",qw1b,qw2);
    //std::cout<<"wrist bend = "<<q5<<std::endl;

    //solve for q6
    A56 = compute_A_of_DH(5, qw2);
    A06 = A05*A56;
    n6 = A06.col(0).head(3);
    t6 = A06.col(1).head(3);   
        
    double cqw3 =n_des.dot(n6);
    double sqw3 =n_des.dot(t6);
    qw3 =atan2(sqw3, cqw3);
    //ROS_INFO("qw1,qw2,qw3 = %f, %f, %f",qw1,qw2,qw3);
    q_soln = q_in;
    q_soln[4] = qw1;
    q_soln[5] = qw2;
    q_soln[6] = qw3;
    while (q_soln[6]>DH_q_max7) q_soln[6]-= 2.0*M_PI;
    while (q_soln[6]<DH_q_min7) q_soln[6]+= 2.0*M_PI;    
    //q_solns.clear();
    q_solns.push_back(q_soln);
    //2nd wrist soln: 
    q_soln[4] = qw1b;
    q_soln[5] *= -1.0; // flip wrist opposite direction
    q_soln[6] += M_PI; // fix the periodicity later; 
    while (q_soln[6]>DH_q_max7) q_soln[6]-= 2.0*M_PI;
    while (q_soln[6]<DH_q_min7) q_soln[6]+= 2.0*M_PI;
    
    //ROS_INFO("alt qw1,qw2,qw3 = %f, %f, %f",q_soln[4],q_soln[5],q_soln[6]);
    q_solns.push_back(q_soln);        
    return is_singular;
}

bool KukaIkSolver::compute_shoulder_ang(double x_des,double y_des,  double L1,  double L2, double q_elbow, double &q_shoulder) {
    //expect x = L1 sin(q_shoulder) + L2 sin(q_shoulder-q_elbow)
    //       y = L1 cos(q_shoulder) + L2 cos(q_shoulder-q_elbow)
    // y = (L1 + L2*cos(q_elbow) * cos(q_shoulder) + (L2*sin(q_elbow))* sin(q_shoulder)
    // K =   A cos(q_shoulder) + B sin(q_shoulder)
    double K = -y_des;
    double A = L1 + L2*cos(q_elbow);
    double B = L2*sin(q_elbow);
    std::vector<double> q_solns;
    //bool KukaFwdSolver::solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns) {
    if(!solve_K_eq_Acos_plus_Bsin(K,A,B,q_solns)) {
        ROS_WARN("problem w/ compute_shoulder_ang...something wrong!");
        return false;
    }
    //if here, have shoulder-ang candidates
    //ROS_INFO("shoulder ang candidates: %f, %f",q_solns[0],q_solns[1]);
    //test which soln is correct:
    q_shoulder = q_solns[0]; // FIX ME!
    double x = L1*sin(q_shoulder) + L2*sin(q_shoulder-q_elbow);
    double y = -L1*cos(q_shoulder) -L2*cos(q_shoulder-q_elbow);
    //ROS_INFO("y_des, y = %f, %f",y_des,y);
    //ROS_INFO("x_des, x = %f, %f",x_des,x);
    if (fabs(x-x_des)< 0.0001) {
        //ROS_INFO("found soln within tolerance");
        return true;
    }
    //try alternative  soln:
    //ROS_INFO("alternative shoulder soln");
    q_shoulder = q_solns[1]; // 
     x = L1*sin(q_shoulder) + L2*sin(q_shoulder-q_elbow);
     y = -L1*cos(q_shoulder) -L2*cos(q_shoulder-q_elbow);
    //ROS_INFO("y_des, y = %f, %f",y_des,y);
    //ROS_INFO("x_des, x = %f, %f",x_des,x);
    if (fabs(x-x_des)< 0.0001) {
        //ROS_INFO("found soln within tolerance");
        return true;
    }    
    
    ROS_WARN("something wrong; no shoulder soln within tolerance!");
    return false;
}

//compute elbow-angle options; return the positive option first, negative option second
bool KukaIkSolver::solve_2R_planar_arm_elbow_angs(double x_des, double y_des, double L1, double L2,
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
        //ROS_WARN("x_des,y_des = %f, %f; L1, L2 = %f, %f",x_des,y_des,L1,L2);
        //ROS_INFO("c_ang, num, den = %f, %f, %f",c_ang,num,den);

        return false;
    }
    //cout<<"num, den, c4 = "<<num<<", "<<den<<", "<<c4<<endl;
    double s_ang = sqrt(1 - c_ang * c_ang);

    //choose the positive soln first:
    double q_elbow_a = fabs(atan2(s_ang, c_ang));  //note: elbow ang in DH coords,  measured from fully extended=0
    
    //ROS_INFO_STREAM("q_elbow_a = "<<q_elbow_a<<endl);
    q_elbow_solns.clear();
    q_elbow_solns.push_back(q_elbow_a);
    
    double q_elbow_b = -q_elbow_a; //atan2(-s_ang, c_ang);
    q_elbow_solns.push_back(q_elbow_b);
    //ROS_INFO("elbow ang solns: %f, %f ",q_elbow_solns[0],q_elbow_solns[1]);
    return true;
}

//for planar arm in x-y plane, link lengths L1, L2, find 2 pairs of solns q1,q2
bool KukaIkSolver::solve_2R_planar_arm(double x_des, double y_des, double L1, double L2,
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
    // try swapping y, x to account for skipping J3
    if(!solve_2R_planar_arm_shoulder_ang(-y_des,x_des, L1, L2, -q_elbow_solns[0], q_shoulder)) {
        return false;
    }
    q_shoulder_solns.push_back(q_shoulder);
    //ROS_INFO("compute shoulder ang candidates for q_elbow = %f",q_elbow_solns[1]);
    // try swapping y, x to account for skipping J3
    if (!solve_2R_planar_arm_shoulder_ang(-y_des,x_des, L1, L2, -q_elbow_solns[1], q_shoulder)) {
        return false;
    }
    q_shoulder_solns.push_back(q_shoulder);    
    return true;
}

//2R planar robot solution for shoulder angle
//given x_des, y_des, elbow_angle, solve for q_shoulder that points towards x_des,y_des;
//for given elbow angle, there should be a unique shoulder angle, q_shoulder
// SPECIALIZED FOR KUKA: freeze J3, home pose is (x,y) = (0,-(L1+L2)) w/rt frame1
// fix this by adding pi/2 to get angles in KUKA coords

bool KukaIkSolver::solve_2R_planar_arm_shoulder_ang(double x_des,double y_des, double L1, double L2,
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
    //ROS_INFO("q1 options: %f, %f",soln1,soln2);
 
    //test which solution is correct:
    
    q_shoulder = soln1; //try the first shoulder option
    double x= L1*cos(q_shoulder) + L2*cos(q_shoulder+q_elbow);
    double y= L1*sin(q_shoulder) + L2*sin(q_shoulder+q_elbow);
    //double xtest = A*cos(q_shoulder) + B*sin(q_shoulder);
    //ROS_INFO("solve_2R_planar_arm_shoulder_ang; x_des,y_des = %f, %f",x_des,y_des);
    //ROS_INFO("x,y = %f, %f ",x,y);
    double fit_err = (x_des-x)*(x_des-x)+(y_des-y)*(y_des-y);
    //ROS_INFO("shoulder soln= %f;  fit err 1: %f",q_shoulder,fit_err);
    if (fit_err< R2_fit_err_tol) {
        //q_shoulder+= M_PI/2;
        // ROS_INFO("q1,q2 = %f, %f fits within tolerance",q_shoulder,q_elbow);
        //q_shoulder = q_solns[0];
        return true; }
      q_shoulder = soln2; //try the alternative(s)
      x= L1*cos(q_shoulder) + L2*cos(q_shoulder+q_elbow);
      y= L1*sin(q_shoulder) + L2*sin(q_shoulder+q_elbow);
      //ROS_INFO(" shoulder soln 1: ");
      //xtest = A*cos(q_shoulder) + B*sin(q_shoulder);
      //ROS_INFO("x,y, xtest = %f, %f, %f ",x,y,xtest);
  
      fit_err = (x_des-x)*(x_des-x)+(y_des-y)*(y_des-y);
      //ROS_INFO("shoulder soln1; fit err: %f",fit_err);
      if (fit_err< R2_fit_err_tol) {
          //q_shoulder+= M_PI/2;
         //ROS_INFO("q1,q2 = %f, %f fits within tolerance",q_shoulder,q_elbow);

         return true; }  
    
    ROS_WARN("could not find shoulder-angle fit within tolerance!");

    return false;
}

 


