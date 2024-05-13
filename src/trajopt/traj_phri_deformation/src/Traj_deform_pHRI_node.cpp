#include "traj_phri_deformation/Traj_deform_pHRI.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "traj_deform_pHRI_node");
    ros::NodeHandle nh;
    Traj_deform_pHRI traj_deform_pHRI(nh,80,1,125); // create an object of the class Traj_deform_pHRI
    // parameters: nodehandle, deformation trajectory length(points), control_rate/deform_rate(need to be int type) , control loop rate

    //initialize trajectory

    traj_deform_pHRI.define_origin_traj(false,std::nullopt,std::nullopt);
    ROS_INFO("trajectory initialized...");

    // wait for the subscriber to get sensor data and joint data
    ROS_INFO("Waiting for subscribers to get sensor data and joint data...");
    while(traj_deform_pHRI.sub_joint_mark == false || traj_deform_pHRI.sub_sensor_mark == false){ ros::Duration(0.0005).sleep();}
    ROS_INFO("All subscribers are ready!");

    geometry_msgs::Pose pose; // define a initial pose, the initial pose need to be the same as the first point of the trajectory(close to the first point of the trajectory)
    pose.position.x = -0.2;
    pose.position.y = 0.6;
    pose.position.z = 0;
    pose.orientation.x = 0.5;
    pose.orientation.y = -0.5;
    pose.orientation.z = 0.5;
    pose.orientation.w = 0.5;
    bool status = traj_deform_pHRI.move_to_initial_pose(pose); // move to the initial pose using moveit
    // Note: moveit cartesian target point is the tool0 frame, base on the world frame, and the base_link frame is coincident with the world frame
    if(status == false)
    {
        ROS_ERROR("Failed to move to the initial pose!");
        return 0;
    }
    ros::Duration(3.0).sleep(); // sleep for 3 seconds to make sure the robot is stable
    traj_deform_pHRI.traj_deform_pHRI_with_control(); // start the trajectory tracking control with trajectory deformation
    return 0;
}
