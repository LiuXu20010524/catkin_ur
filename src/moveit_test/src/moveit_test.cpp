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
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    arm.setGoalJointTolerance(0.05);
    arm.setGoalPositionTolerance(0.05);
   //允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);
    arm.setNamedTarget("home");
    // arm.move();
    arm.plan(my_plan);
    cout << arm.execute(my_plan) <<endl;
    ros::Duration(5).sleep();
    arm.setNamedTarget("up");
    arm.move();
    // ros::shutdown();
    return 0;
}
