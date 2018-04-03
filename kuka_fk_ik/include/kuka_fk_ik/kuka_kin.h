/*********************************************************************
 * Wyatt Newman fk_ik library for UR10 robot
 *********************************************************************/
//test fk/ik with:
// roslaunch ur_gazebo ur10.launch
// rosrun tf tf_echo base_link tool0
// rosrun rosrun ur_fk_ik ur10_fk_ik_test_main

//HACK:  const double DH_q_max2 = 0; //deg2rad*180; //NOT PHYSICAL LIMIT; IMPOSE TO FORCE ELBOW ELEVATED

#ifndef KUKA_KIN_H
#define KUKA_KIN_H

// These kinematics find the transform from the base link to the end effector.
// Though the raw D-H parameters specify a transform from the 0th link to the 6th link,
// offset transforms are specified in this formulation.
// To work with the raw D-H kinematics, use the inverses of the transforms below.

// Transform from base link to 0th link
// -1,  0,  0,  0
//  0, -1,  0,  0
//  0,  0,  1,  0
//  0,  0,  0,  1

// Transform from 6th link to end effector
//  0, -1,  0,  0
//  0,  0, -1,  0
//  1,  0,  0,  0
//  0,  0,  0,  1
//but tool0 frame is same as last DH frame;
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <string>
#include <math.h>
const int NJNTS = 7;
using namespace std;

//KUKA iiwa values:
const double DH_a1 = 0.0;
const double DH_a2 = 0.0; //-0.612;
const double DH_a3 = 0.0; //-0.5723;
const double DH_a4 = 0.0;
const double DH_a5 = 0.0;
const double DH_a6 = 0.0;
const double DH_a7 = 0.0;

const double DH_d1 = 0.360;
const double DH_d2 = 0.0;
const double DH_d3 = 0.420;
const double DH_d4 = 0;
const double DH_d5 = 0.400;
const double DH_d6 = 0;
const double DH_d7 = 0.126;

//hmm...should alpha1 be +pi/2??
const double DH_alpha1 = -M_PI / 2.0; //-M_PI/2.0;
const double DH_alpha2 = M_PI / 2.0;
const double DH_alpha3 = M_PI / 2.0;
const double DH_alpha4 = -M_PI / 2.0;
const double DH_alpha5 = -M_PI / 2.0; //-M_PI/2.0;
const double DH_alpha6 = M_PI / 2.0;
const double DH_alpha7 = 0; 

//robot.DH.theta= '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
const double DH_q_offset1 = 0.0;
const double DH_q_offset2 = 0.0; //-M_PI/2.0; //M_PI; //-M_PI/2.0;
const double DH_q_offset3 = 0.0;
const double DH_q_offset4 = 0; //M_PI/2.0; //0.0;
const double DH_q_offset5 = 0; //0.0;
const double DH_q_offset6 = 0; //0.0;
const double DH_q_offset7 = 0; //0.0;

const double deg2rad = M_PI / 180.0;
//actually, can rotate more than this--but simplify
const double DH_q_max1 = deg2rad * 170; //deg2rad*180;
const double DH_q_max2 = deg2rad * 120; //rad of max practical downward reach  //deg2rad*180; //NOT PHYSICAL LIMIT; IMPOSE TO FORCE ELBOW ELEVATED
const double DH_q_max3 = deg2rad * 170; // beyond 2.9 rad, flange self collides w/ link1; deg2rad*180;
const double DH_q_max4 = deg2rad * 120; //deg2rad*180;//5+M_PI; // TRIM THIS DOWN FOR ARIAC
const double DH_q_max5 = deg2rad * 170; //
const double DH_q_max6 = deg2rad * 120;
const double DH_q_max7 = deg2rad * 175;

const double DH_q_min1 = -deg2rad * 170; //-deg2rad*180;
const double DH_q_min2 = -deg2rad * 120; //rad of max desired shoulder lift--which is beyond vertical//-deg2rad*180;
const double DH_q_min3 = -deg2rad * 170; //0 is fully extended; avoid bending over backwards; -deg2rad*180;
const double DH_q_min4 = -deg2rad * 120; //-deg2rad*180;//1+M_PI; //1+M_PI; //
const double DH_q_min5 = -deg2rad * 170;
const double DH_q_min6 = -deg2rad * 120;
const double DH_q_min7 = -deg2rad * 175;

const double DH_a_params[] = {DH_a1, DH_a2, DH_a3, DH_a4, DH_a5, DH_a6, DH_a7};
const double DH_d_params[] = {DH_d1, DH_d2, DH_d3, DH_d4, DH_d5, DH_d6, DH_d7};
const double DH_alpha_params[] = {DH_alpha1, DH_alpha2, DH_alpha3, DH_alpha4, DH_alpha5, DH_alpha6, DH_alpha7};
const double DH_q_offsets[] = {DH_q_offset1, DH_q_offset2, DH_q_offset3, DH_q_offset4, DH_q_offset5, DH_q_offset6, DH_q_offset7};
const double q_lower_limits[] = {DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6, DH_q_min7};
const double q_upper_limits[] = {DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6, DH_q_max7};

//max velocities unknown?
const double g_qdot_max_vec[] = {5,5,5,5,5,5,5}; //values per URDF on  param server

const double g_tool_offset = 0.008; // vacuum gripper is displaced 8mm from flange (iiwa_link_ee) frame
//Eigen::Quaterniond g_q_tool(0, 1, 0, 0); //quaterion(w,x,y,z))
Eigen::Quaterniond g_quat_tool(0, 1, 0, 0); //quaterion(w,x,y,z))

class KukaFwdSolver {
public:
    KukaFwdSolver(); //constructor;
    Eigen::Affine3d fwd_kin_solve(const Eigen::VectorXd& q_vec); // given vector of q angles, compute fwd kin
    Eigen::Matrix4d get_wrist_frame();
    //Eigen::MatrixXd get_Jacobian(const Vectorq6x1& q_vec);
    //Eigen::Matrix3d test_R61(Eigen::VectorXd q_in);
    void q_Kuka_to_q_DH(Eigen::VectorXd q_soln_Kuka, Eigen::VectorXd &q_soln_DH) {q_soln_DH=q_soln_Kuka;
            for (int i=0;i<NJNTS;i++) q_soln_DH[i]-=DH_q_offsets[i];};
    void q_DH_to_q_Kuka(Eigen::VectorXd q_soln_DH, Eigen::VectorXd &q_soln_Kuka) {q_soln_Kuka=q_soln_DH;
            for (int i=0;i<NJNTS;i++) q_soln_Kuka[i]+=DH_q_offsets[i];};
    bool fit_joints_to_range(Eigen::VectorXd &qvec);
    bool fit_q_to_range(double q_min, double q_max, double &q);
    double jspace_dist_from_nom(Eigen::VectorXd q_nom, Eigen::VectorXd q_soln);
    Eigen::VectorXd select_soln_near_qnom(vector<Eigen::VectorXd> q_ik_solns, Eigen::VectorXd q_nom);

    Eigen::Affine3d get_affine_tool_wrt_flange() {
        return A_tool_wrt_flange_;
    }

    void set_affine_tool_wrt_flange(Eigen::Affine3d A_tool_wrt_flange) {
        A_tool_wrt_flange_ = A_tool_wrt_flange;

    }
    Eigen::MatrixXd g_A_vacuum_wrt_tool0_;
    //convert 7dof Kuka joints (in DH order) and provided rail displacement to consistent 8dof vector for ARIAC control
    Eigen::VectorXd map728dof(double q_linear, Eigen::VectorXd q7dof);

    //convert 8dof vector to 7dof joint angles of Kuka, in order expected by FK/IK fncs
    Eigen::VectorXd map827dof(Eigen::VectorXd q7dof);
    
    
    //std::vector<double> map27dof(double q_linear, Eigen::VectorXd q6dof);
    //void get_joint_names_6dof(vector<string> &jnt_names);
    //void get_joint_names_7dof(vector<string> &jnt_names);
    Eigen::VectorXd closest_soln(Eigen::VectorXd q_ref, vector<Eigen::VectorXd> q_ik_solns);
    //int prune_solns_by_jnt_limits(vector<Eigen::VectorXd> &q_ik_solns);
    //Eigen::VectorXd get_wrist_near_soln(vector<Eigen::VectorXd> q_ik_solns);
    //Eigen::VectorXd get_wrist_far_soln(vector<Eigen::VectorXd> q_ik_solns);
    bool solve_K_eq_Acos_plus_Bsin(double K, double A, double B, std::vector<double> &q_solns);

private:
    Eigen::Affine3d A_tool_wrt_flange_, affine_vacuum_wrt_tool0_;

    Eigen::Matrix4d fwd_kin_solve_(const Eigen::VectorXd& q_vec);
    Eigen::Matrix4d A_mats[7], A_mat_products[7], A_tool; // note: tool A must also handle diff DH vs URDF frame-7 xform
    Eigen::MatrixXd Jacobian;
    int prune_solns_by_jnt_limits(vector<Eigen::VectorXd> &q_ik_solns);
    

};

//class MH5020_IK_solver : public MH5020_fwd_solver
class KukaIkSolver : KukaFwdSolver {
public:
    KukaIkSolver(); //constructor;

    // return the number of valid solutions; actual vector of solutions will require an accessor function
    //int ik_solve(Eigen::Affine3d const& desired_hand_pose); // given vector of q angles, compute fwd kin
    int ik_solve(Eigen::Affine3d const& desired_hand_pose, vector<Eigen::VectorXd> &q_ik_solns);
    //void get_solns(std::vector<Eigen::VectorXd> &q_solns);
    //Eigen::Vector3d get_O4_fwd(Eigen::Affine3d const& desired_hand_pose);
    //Eigen::Vector3d get_O4_rvrs(Eigen::Affine3d const& desired_hand_pose);

    //Eigen::MatrixXd get_Jacobian(const Vectorq6x1& q_vec);
private:
    std::vector<Eigen::VectorXd> q7dof_solns; 
    std::vector<Eigen::VectorXd> q_solns_fit;
    Eigen::Matrix4d A_mats[7], A_mat_products[7], A_tool; // note: tool A must also handle diff DH vs URDF frame-7 xform
    double L_humerus;
    double L_forearm;
    double phi_elbow;
    //given desired flange pose, compute q1 options; expect 2 or 0; return false if out of reach
    bool compute_q1_solns(Eigen::Vector3d w_des, std::vector<double> &q1_solns);
    bool solve_spherical_wrist(Eigen::VectorXd q_in,Eigen::Matrix3d R_des, std::vector<Eigen::VectorXd> &q_solns);
    bool compute_shoulder_ang(double x_des,double y_des,  double L1,  double L2, double q_elbow, double &q_shoulder);


    bool solve_2R_planar_arm_elbow_angs(double x_des, double y_des, double L1, double L2,
            vector<double> &q_elbow_solns);
    bool solve_2R_planar_arm_shoulder_ang(double x_des, double y_des, double L1, double L2,
            double q_elbow, double &q_shoulder);
    bool solve_2R_planar_arm(double x_des, double y_des, double L1, double L2,
            vector<double> &q_shoulder_solns, vector<double> &q_elbow_solns);

};

#endif //UR_KIN_H
