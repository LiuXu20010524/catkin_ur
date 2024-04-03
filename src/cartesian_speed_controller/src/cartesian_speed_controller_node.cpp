#include "cartesian_speed_controller/cartesian_speed_control.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cartesian_speed_controller_node");
    ros::NodeHandle nh;
    Cartesian_speed_control cartesian_speed_control(nh);
    ros::Rate rate(300);
    while(ros::ok())
    {
        rate.sleep();
    }
    return 0;
}
