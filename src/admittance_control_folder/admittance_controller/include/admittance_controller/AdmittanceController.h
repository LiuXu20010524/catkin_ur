#ifndef ADMITTANCECONTROLLER_H   
#define ADMITTANCECONTROLLER_H

// ---  ROS --- //
#include "ros/ros.h"

// ---  ROS Interface --- //
// #include "cartesian_state_msgs/PoseTwist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"

// ---  Eigen --- //
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

// ---  UR Kinematics --- //
#include "ur_kinematics/ur_kin.h"

// --- Filter --- //
#include "admittance_controller/onlineFilter.h"
// filter部分都不是自己写的

// --- Constant Value --- //
#define PI 3.1415926536

// --- Velocity Limits --- //
#define MAX_JOINT_SPACE_VEL PI/3.0
#define MAX_TASK_SPACE_LINEAR_VEL 0.3
#define MAX_TASK_SPACE_ANGULAR_VEL pi/3.0

class AdmittanceController
{
private:
  // --- Subscribers --- //
  ros::Subscriber joint_state_sub_;
  ros::Subscriber ft_sensor_sub_;

  // --- Admittance VARIABLES --- //
  Eigen::Matrix3d M_,B_,K_;

  // --- Reference Trajectory --- //
  Eigen::Vector3d xr_,dxr_;

  // --- Init TF Matrix -- //
  void caculateTF();

  // --- Callbcak Functions --- //
  void joint_state_cb(const sensor_msgs::JointState::ConstPtr &msgs);
  void ft_sensor_cb(const geometry_msgs::WrenchStamped::ConstPtr &msgs);

public:
  // ---  ROS VARIABLES --- //
  ros::NodeHandle *nh_;
  ros::Rate *loop_rate_;

  // --- Rate VARIABLES --- //
  double freq_;
  double step_;

  // --- Publishers --- //
  ros::Publisher twist_pub_;
  Eigen::Vector3d twist_;
  geometry_msgs::Twist twist_cmd_;

  // --- External Force --- //
  Eigen::Vector3d Fext_body_frame_,Fext_world_frame_;
  
  // --- Trajectory Variables --- //
  Eigen::Vector3d x_last_,dx_last_;
  Eigen::Vector3d desired_acc_;

  // --- Online Filter --- //
  onlineFilter* butterWorth_;

  // --- Joint State --- //
  Eigen::VectorXd joint_state_position_,joint_state_velocity_;

  // --- TF Matrix --- //
  Eigen::Matrix4d tf_base2world_,tf_tool2ee_;

  AdmittanceController(ros::NodeHandle *nh);
  ~AdmittanceController();

  // --- Forward Kinematics --- //
  Eigen::Matrix4d FK(Eigen::VectorXd joint);

  // --- Handle External Force --- //
  void handleExternalForce();

  // --- Compute Acceleration and Twist --- //
  void computeAdmittance();

  // --- Send Twist Command to Robot --- //
  void sendTwistCommand();

  // --- Set Reference Trajectory --- //
  void setReferenceTrajectory(Eigen::Vector3d xr,Eigen::Vector3d dxr);

  // --- Variable Admittance --- //
  void setAdmittance(Eigen::MatrixX3d M,Eigen::MatrixX3d B,Eigen::MatrixX3d K);

  // --- Start Admittance Control --- //
  void run();

  // --- Move Commands --- //
  bool moveToFrame(Eigen::Vector3d position,Eigen::Vector3d RPY);

  // --- Robot Interface --- //
  Eigen::Matrix4d getCurrentFrame();
  Eigen::VectorXd getCurrentJoint();
  Eigen::Vector3d getReferenceTrajectory();
  void printCurrentFrame();
  void printCurrentJoint();
  void getAdmittance(Eigen::Matrix3d &M,Eigen::Matrix3d &B,Eigen::Matrix3d &K);
  void getStiffness(Eigen::Matrix3d &K);
  void getDamping(Eigen::Matrix3d &B);
  void getInertia(Eigen::Matrix3d &M);
};

#endif