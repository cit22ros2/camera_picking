// Copyright 2023 Keitaro Nakamura 

// Reference:
// https://github.com/ros-planning/moveit2_tutorials/blob
// /a547cf49ff7d1fe16a93dfe020c6027bcb035b51/doc/move_group_interface
// /src/move_group_interface_tutorial.cpp

#include <cmath>
#include <iostream>
#include <array>
#include <math.h>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

double angle(double Rt[],int16_t n);
double degree(double Rt[],int16_t n);
double IK(double L[], double xt[], double Rt[],double psi, double theta[]);
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);
  auto move_group_joint_node = rclcpp::Node::make_shared("joint_values", node_options);
  // For current state monitor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_arm_node);
  executor.add_node(move_group_gripper_node);
  executor.add_node(move_group_joint_node);
  std::thread([&executor]() {executor.spin();}).detach();

 //調整
  MoveGroupInterface move_group_arm(move_group_arm_node, "arm");
  move_group_arm.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_arm.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0

  MoveGroupInterface move_group_gripper(move_group_gripper_node, "gripper");
  move_group_gripper.setMaxVelocityScalingFactor(1.0);  // Set 0.0 ~ 1.0
  move_group_gripper.setMaxAccelerationScalingFactor(1.0);  // Set 0.0 ~ 1.0
  auto gripper_joint_values = move_group_gripper.getCurrentJointValues();
  double GRIPPER_DEFAULT = 0.0;
  double GRIPPER_OPEN = angles::from_degrees(60.0);
  double GRIPPER_CLOSE = angles::from_degrees(20);

  // SRDFに定義されている"home"の姿勢にする
  /*move_group_arm.setNamedTarget("home");
  move_group_arm.move();
*/
  // 何かを掴んでいた時のためにハンドを開く
  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();


  moveit_msgs::msg::JointConstraint joint_constraint;
  joint_constraint.joint_name = "crane_x7_lower_arm_fixed_part_joint";
  joint_constraint.position = 0.0;
  joint_constraint.tolerance_above = angles::from_degrees(30);
  joint_constraint.tolerance_below = angles::from_degrees(30);
  joint_constraint.weight = 1.0;
 
  // 可動範囲を制限する
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "arm_constraints"; 
  constraints.joint_constraints.push_back(joint_constraint);

  joint_constraint.joint_name = "crane_x7_upper_arm_revolute_part_twist_joint";
  joint_constraint.position = 0.0;
  joint_constraint.tolerance_above = angles::from_degrees(30);
  joint_constraint.tolerance_below = angles::from_degrees(30);
  joint_constraint.weight = 0.8;
  constraints.joint_constraints.push_back(joint_constraint);

  move_group_arm.setPathConstraints(constraints);

  geometry_msgs::msg::Pose target_pose;
  tf2::Quaternion q;

  double L[4] = { 105,250,250,60 };
  double xt[3] = { 100,0,200 };
  double Rt[3] = { 90,0,90 };
  double theta[7] = {0};
  double psi;psi=30*M_PI/180;
  psi=0;
  IK(L,xt,Rt,psi,theta);
  // 掴む準備をする
  auto joint_values = move_group_arm.getCurrentJointValues();
  joint_values[0]=angles::from_degrees(0);
  joint_values[1]=angles::from_degrees(23);
  joint_values[2]=angles::from_degrees(0);
  joint_values[3]=angles::from_degrees(-126);
  joint_values[4]=angles::from_degrees(3);
  joint_values[5]=angles::from_degrees(28);
  joint_values[6]=angles::from_degrees(0);
  move_group_arm.setJointValueTarget(joint_values);
  move_group_arm.move();

  psi = 30 * M_PI / 180;
  joint_values[0]=angles::from_degrees(0);
  joint_values[1]=angles::from_degrees(23);
  joint_values[2]=angles::from_degrees(0);
  joint_values[3]=angles::from_degrees(-126);
  joint_values[4]=angles::from_degrees(3);
  joint_values[5]=angles::from_degrees(28);
  joint_values[6]=angles::from_degrees(0);
  move_group_arm.setJointValueTarget(joint_values);
  move_group_arm.move();

  psi = 0;
  joint_values[0]=angles::from_degrees(theta[0]);
  joint_values[1]=angles::from_degrees(theta[1]);
  joint_values[2]=angles::from_degrees(theta[2]);
  joint_values[3]=angles::from_degrees(theta[3]);
  joint_values[4]=angles::from_degrees(theta[4]);
  joint_values[5]=angles::from_degrees(theta[5]);
  joint_values[6]=angles::from_degrees(theta[6]);
  move_group_arm.setJointValueTarget(joint_values);
  move_group_arm.move();

  xt[2]=100;
  IK(L,xt,Rt,psi,theta);
  joint_values[0]=angles::from_degrees(theta[0]);
  joint_values[1]=angles::from_degrees(theta[1]);
  joint_values[2]=angles::from_degrees(theta[2]);
  joint_values[3]=angles::from_degrees(theta[3]);
  joint_values[4]=angles::from_degrees(theta[4]);
  joint_values[5]=angles::from_degrees(theta[5]);
  joint_values[6]=angles::from_degrees(theta[6]);
  move_group_arm.setJointValueTarget(joint_values);
  move_group_arm.move();
  
  xt[0]=110;
  IK(L,xt,Rt,psi,theta);
  joint_values[0]=angles::from_degrees(theta[0]);
  joint_values[1]=angles::from_degrees(theta[1]);
  joint_values[2]=angles::from_degrees(theta[2]);
  joint_values[3]=angles::from_degrees(theta[3]);
  joint_values[4]=angles::from_degrees(theta[4]);
  joint_values[5]=angles::from_degrees(theta[5]);
  joint_values[6]=angles::from_degrees(theta[6]);
  move_group_arm.setJointValueTarget(joint_values);
  move_group_arm.move();

  // ハンドを閉じる
  gripper_joint_values[0] = GRIPPER_DEFAULT;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();
  
  xt[2]=100;
  IK(L,xt,Rt,psi,theta);
  joint_values[0]=angles::from_degrees(theta[0]);
  joint_values[1]=angles::from_degrees(theta[1]);
  joint_values[2]=angles::from_degrees(theta[2]);
  joint_values[3]=angles::from_degrees(theta[3]);
  joint_values[4]=angles::from_degrees(theta[4]);
  joint_values[5]=angles::from_degrees(theta[5]);
  joint_values[6]=angles::from_degrees(theta[6]);
  move_group_arm.setJointValueTarget(joint_values);
  move_group_arm.move();
 
  xt[1] = 100; 
  IK(L,xt,Rt,psi,theta);
  joint_values[0]=angles::from_degrees(theta[0]);
  joint_values[1]=angles::from_degrees(theta[1]);
  joint_values[2]=angles::from_degrees(theta[2]);
  joint_values[3]=angles::from_degrees(theta[3]);
  joint_values[4]=angles::from_degrees(theta[4]);
  joint_values[5]=angles::from_degrees(theta[5]);
  joint_values[6]=angles::from_degrees(theta[6]);
  move_group_arm.setJointValueTarget(joint_values);
  move_group_arm.move();
  
  //ハンドを開く
  gripper_joint_values[0] = GRIPPER_OPEN;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();
  
  joint_values[0]=angles::from_degrees(0);
  joint_values[1]=angles::from_degrees(23);
  joint_values[2]=angles::from_degrees(0);
  joint_values[3]=angles::from_degrees(-126);
  joint_values[4]=angles::from_degrees(3);
  joint_values[5]=angles::from_degrees(28);
  joint_values[6]=angles::from_degrees(0);
  move_group_arm.setJointValueTarget(joint_values);
  move_group_arm.move();
  //制限解除
  move_group_arm.clearPathConstraints();

  // SRDFに定義されている"home"の姿勢にする
  /*move_group_arm.setNamedTarget("vertical");
  move_group_arm.move();
  move_group_arm.setNamedTarget("home");
  move_group_arm.move();
*/
  // ハンドを閉じる
  gripper_joint_values[0] = GRIPPER_DEFAULT;
  move_group_gripper.setJointValueTarget(gripper_joint_values);
  move_group_gripper.move();

  rclcpp::shutdown();
  return 0;
}

double angle(double Rt[],int16_t n){
    for(int16_t i = 0; i < n; i++){
        Rt[i] = Rt[i] * M_PI / 180;
    }
    return 0;
}
double degree(double Rt[],int16_t n){
    for(int16_t i = 0; i < n; i++){
        Rt[i] = Rt[i] * 180 / M_PI;
    }
    return 0;
}
double IK(double L[], double xt[], double Rt[],double psi, double theta[]){
    
    angle(Rt,3);
    
    double r[3][3];
    r[0][0] = cos(Rt[1])*cos(Rt[2]) ;  r[0][1] = -1*sin(Rt[1])*sin(Rt[2]) ;  r[0][2] = sin(Rt[1]);
    r[1][0] = sin(Rt[0])*sin(Rt[1])*cos(Rt[2]) + cos(Rt[1])*sin(Rt[2]);  r[1][1] = -1*sin(Rt[0])*sin(Rt[1])*sin(Rt[2]) + cos(Rt[0])*cos(Rt[2]);  r[1][2] = -1*sin(Rt[0])*cos(Rt[1]);
    r[2][0] = -1*cos(Rt[0])*sin(Rt[1])*cos(Rt[2]) + sin(Rt[0])*sin(Rt[2]);  r[2][1] = cos(Rt[0])*sin(Rt[1])*sin(Rt[2]) + sin(Rt[0])*cos(Rt[2]);  r[2][2] = cos(Rt[0])*cos(Rt[1]);
    double x_sw[3];
    x_sw[0] = xt[0] - r[0][2] * L[3];
    x_sw[1] = xt[1] - r[1][2] * L[3];
    x_sw[2] = xt[2] - L[0] - r[2][2] * L[3];
    double nor_xsw2;
    nor_xsw2 = x_sw[0]*x_sw[0] + x_sw[1]*x_sw[1] + x_sw[2]*x_sw[2];
    double u_sw[3];
    u_sw[0] = x_sw[0]/sqrt(nor_xsw2);
    u_sw[1] = x_sw[1]/sqrt(nor_xsw2);
    u_sw[2] = x_sw[2]/sqrt(nor_xsw2);

    theta[3] = -1*acos((nor_xsw2 - L[1]*L[1] - L[2]*L[2])/(2*L[1]*L[2]));

    double theta20,theta10,S2,C2,M,N;
    theta10 = atan2( x_sw[1] , x_sw[0] );
    N = sin(theta[3])*L[2];
    M = cos(theta[3])*L[2] + L[1];
    S2 = -1*( M*sqrt( x_sw[0]*x_sw[0] + x_sw[1]*x_sw[1] ) + N*x_sw[2] )/( N*N + M*M );
    C2 = ( N*sqrt( x_sw[0]*x_sw[0] + x_sw[1]*x_sw[1] ) - M*x_sw[2] )/( N*N + M*M );
    theta20 = atan2( S2 , C2 );

    double As[3][3],Bs[3][3],Cs[3][3];
    As[0][0] = -1*u_sw[2]*r[1][0] + u_sw[1]*r[2][0];
    As[0][1] = -1*u_sw[2]*r[1][1] + u_sw[1]*r[2][1];
    As[1][0] = u_sw[2]*r[0][0] - u_sw[0]*r[2][0];
    As[1][1] = u_sw[2]*r[0][1] - u_sw[0]*r[2][1];
    As[2][0] = u_sw[0]*r[1][0] - u_sw[1]*r[0][0];
    As[2][1] = u_sw[0]*r[1][1] - u_sw[1]*r[0][1];
    As[2][2] = u_sw[0]*r[1][2] - u_sw[1]*r[0][1];

    Bs[0][0] = -1*u_sw[0]*u_sw[1]*r[1][0] - u_sw[2]*u_sw[0]*r[2][0] + r[0][0]*( u_sw[2]*u_sw[2] + u_sw[1]*u_sw[1] );
    Bs[0][1] = -1*u_sw[0]*u_sw[2]*r[2][1] - u_sw[1]*u_sw[0]*r[1][1] + r[0][1]*( u_sw[2]*u_sw[2] + u_sw[1]*u_sw[1] );
    Bs[1][0] = -1*u_sw[2]*u_sw[1]*r[2][0] - u_sw[1]*u_sw[0]*r[0][0] + r[1][0]*( u_sw[2]*u_sw[2] + u_sw[1]*u_sw[1] );
    Bs[1][1] = -1*u_sw[1]*u_sw[2]*r[2][1] - u_sw[1]*u_sw[0]*r[0][1] + r[1][1]*( u_sw[2]*u_sw[2] + u_sw[1]*u_sw[1] );
    Bs[2][0] = -1*u_sw[0]*u_sw[2]*r[0][0] - u_sw[1]*u_sw[2]*r[1][0] + r[2][0]*( u_sw[0]*u_sw[0] + u_sw[1]*u_sw[1] );
    Bs[2][1] = -1*u_sw[0]*u_sw[2]*r[0][1] - u_sw[1]*u_sw[2]*r[1][1] + r[2][1]*( u_sw[0]*u_sw[0] + u_sw[1]*u_sw[1] );
    Bs[2][2] = -1*u_sw[0]*u_sw[2]*r[0][2] - u_sw[1]*u_sw[2]*r[1][2] + r[2][2]*( u_sw[0]*u_sw[0] + u_sw[1]*u_sw[1] );

    Cs[0][0] = u_sw[0]*u_sw[0]*r[0][0] + u_sw[1]*u_sw[0]*r[1][0] + r[2][0]*u_sw[2]*u_sw[0];
    Cs[0][1] = u_sw[0]*u_sw[0]*r[0][1] + u_sw[1]*u_sw[0]*r[1][1] + r[2][1]*u_sw[2]*u_sw[0];
    Cs[1][0] = u_sw[1]*u_sw[0]*r[0][0] + u_sw[1]*u_sw[1]*r[1][0] + r[2][0]*u_sw[2]*u_sw[1];
    Cs[1][1] = u_sw[0]*u_sw[1]*r[0][1] + u_sw[1]*u_sw[1]*r[1][1] + r[2][1]*u_sw[2]*u_sw[1];
    Cs[2][0] = u_sw[0]*u_sw[2]*r[0][0] + u_sw[1]*u_sw[2]*r[1][0] + r[2][0]*u_sw[2]*u_sw[2];
    Cs[2][1] = u_sw[0]*u_sw[2]*r[0][1] + u_sw[1]*u_sw[2]*r[1][1] + r[2][1]*u_sw[2]*u_sw[2];
    Cs[2][2] = u_sw[0]*u_sw[2]*r[0][2] + u_sw[1]*u_sw[2]*r[1][2] + r[2][2]*u_sw[2]*u_sw[2];

    double theta01,theta02,theta1,theta21,theta22;
    theta01 = -1*( As[1][1]*sin(psi) + Bs[1][1]*cos(psi) + Cs[1][1] );
    theta02 = -1*( As[0][1]*sin(psi) + Bs[0][1]*cos(psi) + Cs[0][1] );
    theta1  = -1*( As[2][1]*sin(psi) + Bs[2][1]*cos(psi) + Cs[2][1] );
    theta21 =      As[2][2]*sin(psi) + Bs[2][2]*cos(psi) + Cs[2][2];
    theta22 = -1*( As[2][0]*sin(psi) + Bs[2][0]*cos(psi) + Cs[2][0] );
    std::cout << "theta01 " << theta01 << " theta02 " << theta02 << " theta1 " << theta1 << " theta21 " << theta21 << " theta22 " << theta22 << std::endl;
    theta[0] = atan2( theta01 , theta02);
    theta[1] = acos( theta1 );
    theta[2] = atan2( theta21 , theta22 );
    if( theta[1] > M_PI/2 ){
        theta[1] = theta[1] - M_PI;
    }

    double Aw[3][3],Bw[3][3],Cw[3][3];
    Aw[0][2] = r[0][2]*( As[0][0]*cos(L[3]) + As[0][1]*sin(L[3]) ) + r[1][2]*( As[1][0]*cos(L[3]) + As[1][1]*sin(L[3]) ) + r[2][2]*( As[2][0]*cos(L[3]) + As[2][1]*sin(L[3]) );
    Aw[1][2] = r[0][2]*As[0][1] + r[1][2]*As[1][1] + r[2][2]*As[2][1];
    Aw[2][0] = r[0][0]*( As[0][0]*sin(L[3]) - As[0][1]*cos(L[3]) ) + r[1][0]*( As[1][0]*sin(L[3]) - As[1][1]*cos(L[3]) ) + r[2][0]*( As[2][0]*sin(L[3]) + As[2][1]*cos(L[3]) );
    Aw[2][1] = r[0][1]*( As[0][0]*sin(L[3]) - As[0][1]*cos(L[3]) ) + r[1][1]*( As[1][0]*sin(L[3]) - As[1][1]*cos(L[3]) ) + r[2][1]*( As[2][0]*sin(L[3]) + As[2][1]*cos(L[3]) );
    Aw[2][2] = r[0][2]*( As[0][0]*sin(L[3]) - As[0][1]*cos(L[3]) ) + r[1][2]*( As[1][0]*sin(L[3]) - As[1][1]*cos(L[3]) ) + r[2][2]*( As[2][0]*sin(L[3]) + As[2][1]*cos(L[3]) );

    Bw[0][2] = r[0][2]*( Bs[0][0]*cos(L[3]) + Bs[0][1]*sin(L[3]) ) + r[1][2]*( Bs[1][0]*cos(L[3]) + Bs[1][1]*sin(L[3]) ) + r[2][2]*( Bs[2][0]*cos(L[3]) + Bs[2][1]*sin(L[3]) );
    Bw[1][2] = r[0][2]*Bs[0][1] + r[1][2]*Bs[1][1] + r[2][2]*Bs[2][1];
    Bw[2][0] = r[0][0]*( Bs[0][0]*sin(L[3]) - Bs[0][1]*cos(L[3]) ) + r[1][0]*( Bs[1][0]*sin(L[3]) - Bs[1][1]*cos(L[3]) ) + r[2][0]*( Bs[2][0]*sin(L[3]) + Bs[2][1]*cos(L[3]) );
    Bw[2][1] = r[0][1]*( Bs[0][0]*sin(L[3]) - Bs[0][1]*cos(L[3]) ) + r[1][1]*( Bs[1][0]*sin(L[3]) - Bs[1][1]*cos(L[3]) ) + r[2][1]*( Bs[2][0]*sin(L[3]) + Bs[2][1]*cos(L[3]) );
    Bw[2][2] = r[0][2]*( Bs[0][0]*sin(L[3]) - Bs[0][1]*cos(L[3]) ) + r[1][2]*( Bs[1][0]*sin(L[3]) - Bs[1][1]*cos(L[3]) ) + r[2][2]*( Bs[2][0]*sin(L[3]) + Bs[2][1]*cos(L[3]) );

    Cw[0][2] = r[0][2]*( Cs[0][0]*cos(L[3]) + Cs[0][1]*sin(L[3]) ) + r[1][2]*( Cs[1][0]*cos(L[3]) + Cs[1][1]*sin(L[3]) ) + r[2][2]*( Cs[2][0]*cos(L[3]) + Cs[2][1]*sin(L[3]) );
    Cw[1][2] = r[0][2]*Cs[0][1] + r[1][2]*Cs[1][1] + r[2][2]*Cs[2][1];
    Cw[2][0] = r[0][0]*( Cs[0][0]*sin(L[3]) - Cs[0][1]*cos(L[3]) ) + r[1][0]*( Cs[1][0]*sin(L[3]) - Cs[1][1]*cos(L[3]) ) + r[2][0]*( Cs[2][0]*sin(L[3]) + Cs[2][1]*cos(L[3]) );
    Cw[2][1] = r[0][1]*( Cs[0][0]*sin(L[3]) - Cs[0][1]*cos(L[3]) ) + r[1][1]*( Cs[1][0]*sin(L[3]) - Cs[1][1]*cos(L[3]) ) + r[2][1]*( Cs[2][0]*sin(L[3]) + Cs[2][1]*cos(L[3]) );
    Cw[2][2] = r[0][2]*( Cs[0][0]*sin(L[3]) - Cs[0][1]*cos(L[3]) ) + r[1][2]*( Cs[1][0]*sin(L[3]) - Cs[1][1]*cos(L[3]) ) + r[2][2]*( Cs[2][0]*sin(L[3]) + Cs[2][1]*cos(L[3]) );

    double theta41,theta42,theta5,theta61,theta62;
    theta41 = Aw[1][2]*sin(psi) + Bw[1][2]*cos(psi) + Cw[1][2] ;
    theta42 = Aw[0][2]*sin(psi) + Bw[0][2]*cos(psi) + Cw[0][2] ;
    theta5  = Aw[2][2]*sin(psi) + Bw[2][2]*cos(psi) + Cw[2][2] ;
    theta61 = Aw[2][1]*sin(psi) + Bw[2][1]*cos(psi) + Cw[2][1] ;
    theta62 = -1 * ( Aw[2][0]*sin(psi) + Bw[2][0]*cos(psi) + Cw[2][0] );
    theta[4] = atan2( theta41 , theta42 );
    theta[5] = acos( theta5 );
    theta[6] = atan2( theta61 , theta62 );
    std::cout << "theta41 " << theta41 << " theta42 " << theta42 << " theta5 " << theta5 << " theta61 " << theta61 << " theta62 " << theta62 << std::endl;
    if( theta[5] > M_PI/2 ){
        theta[5] = theta[5] - M_PI;
    }

    degree(theta,7);
    for(int16_t i = 0; i < 7; i++ ){
        std::cout << i << ":" << theta[i] << ",";
    }
    return 0;
}
