#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <iostream>

using namespace std;


int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "moveit_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    arm.setPoseReferenceFrame("base_link");
    arm.setEndEffectorLink("flange");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    arm.setGoalJointTolerance(0.05);
    arm.setGoalPositionTolerance(0.05);
   //允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    geometry_msgs::Pose pose; // define a initial pose, the initial pose need to be the same as the first point of the trajectory(close to the first point of the trajectory)
    pose.position.x = -0.37307;
    pose.position.y = 0.10026;
    pose.position.z = -0.12699;
    pose.orientation.x = 0.859545;
    pose.orientation.y = 0.417114;
    pose.orientation.z = -0.290426;
    pose.orientation.w = 0.053395;

    arm.setPoseTarget(pose);
    // arm.move();
    arm.plan(my_plan);
    cout << arm.execute(my_plan) <<endl;
    // ros::shutdown();
    return 0;
}
