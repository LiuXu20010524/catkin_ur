#ifndef TRAJ_DEFORM_PHRI_H
#define TRAJ_DEFORM_PHRI_H

// --- includes ---//
    // --- Eigen ---//
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/Sparse>

    // --- ROS ---//
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>

    // --- MoveIt ---//
#include <moveit/move_group_interface/move_group_interface.h>

    // --- C++ Standard---//
#include <iostream>
#include <vector>
#include <cmath>
#include <optional>
#include <algorithm> 

    // --- ur_kinematics ---//
#include "ur_kinematics/ur_kin.h"

    // --- OsqpEigen ---//
#include <OsqpEigen/OsqpEigen.h>

// --- Constants ---//
#define PI 3.14159265358979323846

using namespace std;

// --- Class Definition ---//
class Traj_deform_pHRI
{
public:
// --- Variables ---//
    // --- Variables --- //
    int deform_traj_num_; // Number of deformed trajectory
    int control_times_to_deform_; // how mant control times to deform one time of trajectory
    int loop_rate_; // Loop rate
    double alpha_; // alpha is the weight for the deformation, the larger the alpha, the less the deformation, to ensure the stability of the deformation, alpha should be a very very large number
    // --- subscriber ---//
    ros::Subscriber joint_states_sub;
    ros::Subscriber sensor_sub;
    // --- publisher ---//
    ros::Publisher joint_traj_cmd_pub;
    // --- Subscriber Mark ---//
    bool sub_joint_mark = false; // Mark for joint_states subscriber
    bool sub_sensor_mark = false; // Mark for sensor subscriber
    // --- trajectory Repeat Mark ---//
    bool traj_repeat_mark = false; // if your trajectory is closed, you can set this mark to true
    bool origin_traj_no_repeat_to_end_mark = false; // if your trajectory is not closed, and the trajectory is to the end, you need to set this mark to true.
    // --- Eigen ---//
    Eigen::Vector<double,6> joint_position_; // Joint position
    Eigen::Vector<double,6> joint_velocity_; // Joint velocity
    Eigen::Vector<double,6> sensor_data_; // Sensor data
    Eigen::Matrix4d tf_sensor2flange; // Transformation matrix from the sensor to the end effector(flange)
    Eigen::Matrix<double,4,4> tf_baselink2world_E402; // transform matrix from base to world
    Eigen::VectorXd deform_traj_vector; // Deformed trajectory vector for QP optimization
    Eigen::SparseMatrix<double> P_QP; // QP optimization P matrix
    Eigen::MatrixXd A_paper; // A matrix in the paper "Trajectory Deformations From Physical Human–Robot Interaction" to get the P matrix
    Eigen::SparseMatrix<double> A_QP; // QP optimization A matrix
    Eigen::VectorXd q_QP; // QP optimization q vector
    Eigen::VectorXd l_QP; // QP optimization l vector
    Eigen::VectorXd u_QP; // QP optimization u vector
    std::vector<Eigen::Vector<double,7>,Eigen::aligned_allocator<Eigen::Vector<double,7>>> origin_traj; // Original full trajectory
    Eigen::MatrixXd deform_traj_matrix; // Deformed trajectory matrix, store the deformed trajectory, this matrix will slide to the left and add new trajectory points in the last column
    // --- vector iterator --- //
    std::vector<Eigen::Vector<double,7>,Eigen::aligned_allocator<Eigen::Vector<double,7>>>::iterator origin_traj_iter; // Iterator for original trajectory
    // --- OsqpEigen --- //
    OsqpEigen::Solver solver; // Osqp solver
    // --- rosmsg --- //
    trajectory_msgs::JointTrajectory joint_traj_cmd; // Joint trajectory command
    // --- spinners --- //
    ros::AsyncSpinner *spinner;

// --- Functions ---//
    Traj_deform_pHRI(ros::NodeHandle &nh, int deform_traj_num, int control_times_to_deform, int loop_rate);  // Constructor
    ~Traj_deform_pHRI(); // Destructor
    void joint_states_cb(const sensor_msgs::JointState::ConstPtr &msg); // Callback function for joint_states
    void sensor_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg); // Callback function for sensor
    bool move_to_initial_pose(geometry_msgs::Pose &initial_target_pose); // Move to initial pose
    void compute_tf_sensor2flange(); // Compute the transformation matrix from the sensor to the end effector(flange)
    void define_origin_traj(bool using_custom_traj, std::optional<std::vector<geometry_msgs::Pose>> custom_traj, std::optional<bool> traj_repeat); // Define the original trajectory
    // if using_custom_traj is true, use custom_traj as the original trajectory, otherwise use the default trajectory(circle in cartesian space)
    void traj_deform_pHRI_with_control(); // Trajectory deformation for pHRI
    void traj_deform_main_func(); // Main function for trajectory deformation
    void control_main_func(); // Main function for impedance control
    void compute_transform_base_link2world_E402();
};

#endif