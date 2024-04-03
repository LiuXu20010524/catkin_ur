// 注意：E402的UR机械臂好像最多只能接受125Hz的控制频率，在这篇论文中需要考虑稳定性问题。
#include "shear_thickening_fluid_controller/SFC_controller.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "SFC_controller_node");
    ros::NodeHandle nh;
    float freq = 125; // UR机械臂最大接收125Hz的控制频率
    SFC_controller SFC_controller_obj(nh, 1.0, 1/freq); // parameters list: NodeHandle, m_aim, delta_T
    // parameters list: f_ease, f_interf, x_d_bar_dot, x_c_bar_dot, w_cease
    SFC_controller_obj.parameters_Auto_tuning(8, 40, 0.05, 0.15, 2.7);
    ros::Rate rate(freq);
    while(SFC_controller_obj.sub_sensor_mark == false || SFC_controller_obj.sub_joint_mark == false)
    {
        ros::Duration(0.0001).sleep();
    }
    std::cout << "have sensor data and joint data" << std::endl;
    while(ros::ok())
    {
        SFC_controller_obj.SFC_main_controller();
        rate.sleep();
    }
    return 0;
}
