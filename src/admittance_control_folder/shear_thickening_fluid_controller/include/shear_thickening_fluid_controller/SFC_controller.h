#ifndef SFC_CONTROLLER_H
#define SFC_CONTROLLER_H

//  --- constants ---  //
#define PI 3.14159265358979323846

//  --- standard headers ---  //
#include <iostream>
#include <cmath>
#include <vector>

//  --- ros headers ---  //
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_msgs/TFMessage.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "ur_kinematics/ur_kin.h"

//  --- Eigen headers ---  //
#include <Eigen/Dense>
#include <Eigen/Core>


class SFC_controller
{
private:

public:
//  --- Variables ---  //
    float m , n , g , miu , delta_T;  //parameters of the SFC controller
    Eigen::Vector<double,6> ctrl_cmd_world_frame;
    geometry_msgs::Twist twist_msg;
    ros::Publisher twist_pub;
    ros::Subscriber force_sensor_sub; // subscriber for the force sensor msgs
    ros::Subscriber joint_states_sub; // subscriber for the joint states msgs
    Eigen::Vector<double,6> force_sensor_data; // force sensor data
    Eigen::Vector<double,6> joint_position; // joint position data
    Eigen::Vector<double,6> joint_velocity; // joint velocity data
    Eigen::Matrix<double,3,3> orientation_ee2base; // orientation matrix of the end effector
    Eigen::Matrix<double,4,4> tf_tool2ee; // transform matrix from tool to end effector
    Eigen::Matrix<double,4,4> tf_baselink2world_E402; // transform matrix from base to world
    Eigen::Vector<double,6> force_sensor_data_before_transform;
    bool sub_sensor_mark = false;
    bool sub_joint_mark = false;
    ros::AsyncSpinner *spinner;
    rosbag::Bag bag_ctrl_cmd_world_frame;
    std::string bag_ctrl_cmd_world_frame_path = "./src/admittance_control_folder/shear_thickening_fluid_controller/rosbag_out/SFC_controller_ctrl_cmd_world.bag";
    rosbag::Bag bag_sensor_data_itself;  // 没有经过坐标变换的传感器数据
    std::string bag_sensor_data_itself_path = "./src/admittance_control_folder/shear_thickening_fluid_controller/rosbag_out/SFC_controller_sensor_data_itself.bag";
    rosbag::Bag bag_sensor_data_world;  // 经过坐标变换的传感器数据
    std::string bag_sensor_data_world_path = "./src/admittance_control_folder/shear_thickening_fluid_controller/rosbag_out/SFC_controller_sensor_data_world.bag";
    std_msgs::Float32MultiArray bag_msgs_ctrl_cmd_world_frame;
    std_msgs::Float32MultiArray bag_msgs_sensor_world_frame;

    
//  --- functions ---  //
    SFC_controller(ros::NodeHandle &nh, float m_aim , float delta_T_aim);
    ~SFC_controller();
    void parameters_Auto_tuning(float f_ease, float f_interf, float x_d_bar_dot, float x_c_bar_dot, float w_cease);
    void msg_pub_fun(const Eigen::Vector<double,6> &ctrl_cmd);
    void SFC_main_controller();
    void force_sensor_sub_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg);
    void joint_states_sub_callback(const sensor_msgs::JointState::ConstPtr &msg);
    Eigen::Matrix4d compute_transform_matrix_tool2world();
    void compute_transform_tool2ee();
    void compute_transform_base_link2world_E402();
};

#endif