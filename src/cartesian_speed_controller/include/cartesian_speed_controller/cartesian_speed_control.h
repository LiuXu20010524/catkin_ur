#ifndef CARTESIAN_SPEED_CONTROL_H
#define CARTESIAN_SPEED_CONTROL_H

//  --- Standard headfile ---  //
#include <iostream>
#include <vector>
#include <cmath>

//  --- Eigen headfile ---  //
#include <Eigen/Dense>

//  --- Pinocchio headfile ---  //
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

//  --- ROS headfile ---  //
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/MultiArrayLayout.h>

//  --- Constant Value ---  //
#define PI 3.14159265358979323846

//  --- Namespace ---  //
namespace pin = pinocchio;

//  --- Class Definition ---  //
class Cartesian_speed_control
{
private:

public:
//  --- Variables ---  //
    ros::Subscriber twist_sub;
    ros::Subscriber joint_state_sub;
    ros::Publisher joint_pos_pub;
    ros::Publisher joint_speed_pub1;
    ros::Publisher joint_speed_pub2;
    ros::Publisher joint_speed_pub3;
    ros::Publisher joint_speed_pub4;
    ros::Publisher joint_speed_pub5;
    ros::Publisher joint_speed_pub6;
    std::vector<std_msgs::Float64> joint_speed_pub;
    pin::Model model; // Model of the robot
    std::string urdf_filename = "/home/liu_xu/liuxu_Documents/catkin_ur/src/universal_robot/ur_description/urdf/ur5e.urdf"; // URDF file name
    pin::Data data; // Data of the robot
    pin::FrameIndex end_frame_id; // Frame index of the end-effector
    ros::AsyncSpinner *spinner;
    Eigen::Vector<double,6> q; // Joint position
    Eigen::Matrix<double,6,6> pin_J; // Jacobian matrix
    Eigen::Matrix<double,6,6> pin_J_inverse; // Inverse of the jacobian matrix
    bool sub_mark_ = false; // Subscribe mark
    Eigen::Vector<double,6> pos_cmd; // Twist command
    Eigen::Vector<double,6> joint_speed_cmd;

//  --- Functions ---  //
    Cartesian_speed_control(ros::NodeHandle &nh);
    ~Cartesian_speed_control();
    void twist_callback(const geometry_msgs::Twist::ConstPtr &msg);
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void pinocchio_jociobian(); // Calculate the jacobian and jacobian_dot of the robot
    void compute_position();
};

#endif