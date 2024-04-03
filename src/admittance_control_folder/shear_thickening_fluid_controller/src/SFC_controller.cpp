#include "shear_thickening_fluid_controller/SFC_controller.h"

SFC_controller::SFC_controller(ros::NodeHandle &nh, float m_aim , float delta_T_aim): m(m_aim),delta_T(delta_T_aim)
{
    ROS_INFO("SFC_controller object is created");
    twist_pub = nh.advertise<geometry_msgs::Twist>("/twist_controller/command", 1); // Note: you need to change the topic name to the one you are using
    force_sensor_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/mcc_1608g_daq", 1, &SFC_controller::force_sensor_sub_callback, this);
    joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &SFC_controller::joint_states_sub_callback, this);
    ctrl_cmd_world_frame.setZero();
    this->compute_transform_tool2ee();
    this->compute_transform_base_link2world_E402();
    bag_ctrl_cmd_world_frame.open(bag_ctrl_cmd_world_frame_path, rosbag::BagMode::Write);
    bag_sensor_data_itself.open(bag_sensor_data_itself_path, rosbag::BagMode::Write);
    bag_sensor_data_world.open(bag_sensor_data_world_path, rosbag::BagMode::Write);
    bag_msgs_ctrl_cmd_world_frame.data.resize(6);
    bag_msgs_sensor_world_frame.data.resize(6);
    spinner = new ros::AsyncSpinner(5);
    spinner->start();
}

SFC_controller::~SFC_controller()
{
    ROS_INFO("SFC_controller object is destroyed");
    bag_ctrl_cmd_world_frame.close();
    bag_sensor_data_itself.close();
    bag_sensor_data_world.close();
    delete spinner;
}

//计算SFC控制器的参数，根据论文《A Shear-Thickening Fluid-Based Controller for Physical Human-Robot Interaction》
//考虑空间是六维的，因此应该六个维度分开单独计算，此处只是用作测试，因此只计算了一个维度的参数，并在之后全都用的这个参数。
//但这样做是不对的，至少对于移动和旋转两个方向的参数应该分开计算。
void SFC_controller::parameters_Auto_tuning(float f_ease, float f_interf, float x_d_bar_dot, float x_c_bar_dot, float w_cease)
{
    n = log(abs(f_interf/f_ease)) / log(abs(x_c_bar_dot/x_d_bar_dot));
    float w_c_max = w_cease * pow(f_interf/f_ease, (n-1)/n);
    if (w_c_max > 1/((n*delta_T*pow(2,(1+n)/(2*n)))))
    {
        w_cease = 2/((n*delta_T)*pow((f_ease/(sqrt(2)*f_interf)),(n-1)/(n)));
        w_c_max = w_cease * pow(f_interf/f_ease, (n-1)/n);
    }
    double psi = 2*sqrt(PI)*tgamma(1+n/2)/tgamma((3+n)/2); // Ψ function in paper
    miu = pow((m*w_cease),n)/(psi*pow(sqrt(2)/f_ease,n-1));
    g = x_d_bar_dot*pow(miu/f_ease,1/n);
    std::cout << "parameters_Auto_tuning" << std::endl << "n: " << n << std::endl << "miu: " << miu << std::endl << "g: " << g << std::endl;
}

//发布控制指令的函数
void SFC_controller::msg_pub_fun(const Eigen::Vector<double,6> &ctrl_cmd)
{
    twist_msg.linear.x = ctrl_cmd(0);
    twist_msg.linear.y = ctrl_cmd(1);
    twist_msg.linear.z = ctrl_cmd(2);
    twist_msg.angular.x = ctrl_cmd(3);
    twist_msg.angular.y = ctrl_cmd(4);
    twist_msg.angular.z = ctrl_cmd(5);
    twist_pub.publish(twist_msg);
}

//接收传感器数据的回调函数，传感器数据是在传感器坐标系下的数据，使用rosbag记录数据(没有经过坐标变换的传感器数据)
void SFC_controller::force_sensor_sub_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    (sub_sensor_mark== true )? :sub_sensor_mark = true;
    // you can use the force sensor data here
    // Note: Pay attention to the coordinate transformation. Generally, the signals collected by the sensor are relative to its own coordinate system.
    force_sensor_data_before_transform << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
    bag_sensor_data_itself.write("/sensor_data_itself_bag", ros::Time::now(), *msg);
}

//接收关节角度数据的回调函数
void SFC_controller::joint_states_sub_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // you can use the joint states data here
    (sub_joint_mark== true )? :sub_joint_mark = true;
    joint_position << msg->position[2], msg->position[1], msg->position[0], msg->position[3], msg->position[4], msg->position[5];
    joint_velocity << msg->velocity[2], msg->velocity[1], msg->velocity[0], msg->velocity[3], msg->velocity[4], msg->velocity[5];
    // Note: 返回的关节角度和关节速度的顺序和UR机械臂结构顺序不一致，第一个关节和第三个关节应该调换
}

//计算SFC控制器的主要控制函数，进行了坐标系变换，同时使用rosbag记录数据
void SFC_controller::SFC_main_controller()
{
    // -----
    Eigen::Matrix3d orientation_baselink2base;
    orientation_baselink2base <<  sqrt(2)/2, -sqrt(2)/2, 0.0,
                                -0.5, -0.5, -sqrt(2)/2,
                                0.5, 0.5, -sqrt(2)/2;
    // -----

    static Eigen::Matrix4d tf_tool2world_;
    static Eigen::Vector<double,6> ctrl_cmd_robot_frame;
    tf_tool2world_ = this->compute_transform_matrix_tool2world();
    force_sensor_data.block(0,0,3,1) = tf_tool2world_.block(0,0,3,3) * force_sensor_data_before_transform.block(0,0,3,1);
    force_sensor_data.block(3,0,3,1) = tf_tool2world_.block(0,0,3,3) * force_sensor_data_before_transform.block(3,0,3,1);
    // std::cout << "force_sensor_data" << std::endl << force_sensor_data << std::endl;
    static Eigen::Vector<double,6> x_double_dot;
    x_double_dot = (1/m)*(force_sensor_data - miu*(ctrl_cmd_world_frame.cwiseAbs().array().pow(n-1).matrix()).cwiseProduct(ctrl_cmd_world_frame));
    ctrl_cmd_world_frame = g*(ctrl_cmd_world_frame+delta_T*x_double_dot); //此处的ctrl_cmd_world_frame是在世界坐标系下的，需要转换到机器人基坐标系下
    ctrl_cmd_world_frame.block(0,0,3,1) = ctrl_cmd_world_frame.block(0,0,3,1) / 10;
    ctrl_cmd_world_frame.block(3,0,3,1) = ctrl_cmd_world_frame.block(3,0,3,1)/1.5;
    ctrl_cmd_robot_frame.block(0,0,3,1) = orientation_baselink2base.transpose() * ctrl_cmd_world_frame.block(0,0,3,1);
    ctrl_cmd_robot_frame.block(3,0,3,1) = orientation_baselink2base.transpose() * ctrl_cmd_world_frame.block(3,0,3,1);
    this->msg_pub_fun(ctrl_cmd_robot_frame);
    std::cout << "ctrl_cmd_world_frame" << std::endl << ctrl_cmd_world_frame << std::endl;
    //使用rosbag记录数据控制指令
    bag_msgs_ctrl_cmd_world_frame.data.clear();
    for(int i = 0;i<6;i++)
    {
        bag_msgs_ctrl_cmd_world_frame.data.push_back(ctrl_cmd_world_frame(i));
    }
    bag_ctrl_cmd_world_frame.write("/ctrl_cmd_world_frame_bag", ros::Time::now(), bag_msgs_ctrl_cmd_world_frame);
    //使用rosbag记录数据传感器转换到世界坐标系下的数据
    bag_msgs_sensor_world_frame.data.clear();
    for(int i = 0; i < 6; i++)
    {
        bag_msgs_sensor_world_frame.data.push_back(force_sensor_data(i));
    }
    bag_sensor_data_world.write("/sensor_data_world_frame_bag", ros::Time::now(), bag_msgs_sensor_world_frame);
}

//计算传感器安装到末端执行器(flange)的变换矩阵
void SFC_controller::compute_transform_tool2ee() // transform from tool(sensor) to end effector, end-effector is link flange in urdf
{
    Eigen::Matrix3d orientation_tool2ee;
    Eigen::Vector3d position_tool2ee;
    Eigen::Vector3d euler_angle_tool2ee(-PI/2, 0.0, -PI/2);  // sensor euler angle to end effector(flange), xyz euler angle
    position_tool2ee << 0.0, 0.0, 0.0;
    orientation_tool2ee = (Eigen::AngleAxisd(euler_angle_tool2ee[2], Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(euler_angle_tool2ee[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(euler_angle_tool2ee[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    tf_tool2ee.block(0,0,3,3) = orientation_tool2ee;
    tf_tool2ee.block(0,3,3,1) = position_tool2ee;
    tf_tool2ee(3,3) = 1.0;
}

//计算base_link到世界坐标系E402的变换矩阵
void SFC_controller::compute_transform_base_link2world_E402()
{
    Eigen::Matrix3d orientation_base2world,orientation_baselink2base;
    Eigen::Vector3d euler_angle_baselink2base(0.0, 0.0, -PI);  // base_link to base xyz euler angle
    Eigen::Vector3d position_base2world(0,0,0);
    orientation_baselink2base = (Eigen::AngleAxisd(euler_angle_baselink2base[2], Eigen::Vector3d::UnitZ())
                                * Eigen::AngleAxisd(euler_angle_baselink2base[1], Eigen::Vector3d::UnitY())
                                * Eigen::AngleAxisd(euler_angle_baselink2base[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    // orientation_baselink2base 是base_link到base的变换矩阵
    orientation_base2world  <<  sqrt(2)/2, -sqrt(2)/2, 0.0,
                                -0.5, -0.5, -sqrt(2)/2,
                                0.5, 0.5, -sqrt(2)/2;  //E402的UR机械臂的base坐标系和世界坐标系的转换矩阵
    tf_baselink2world_E402.block(0,0,3,3) = orientation_base2world*orientation_baselink2base;
    // tf_base2world_E402.block(0,0,3,3) = orientation_base2world;
    tf_baselink2world_E402.block(0,3,3,1) = position_base2world;
    tf_baselink2world_E402(3,3) = 1.0;
}

//计算末端执行器(flange)到世界坐标系的变换矩阵(其中包含了flange到base_link的变换矩阵，以及base_link到base再到世界坐标系的变换矩阵)
Eigen::Matrix4d SFC_controller::compute_transform_matrix_tool2world()
{
    Eigen::Matrix4d tf_tool2world;
    Eigen::Matrix4d tf_ee2base; //note: end-effector is link flange in urdf
    double q_[6] = {joint_position(0), joint_position(1), joint_position(2), joint_position(3), joint_position(4), joint_position(5)};
    double T[16];
    ur_kinematics::forward(q_, T);
    tf_ee2base <<   T[0], T[1], T[2], T[3],
                    T[4], T[5], T[6], T[7],
                    T[8], T[9], T[10], T[11],
                    T[12], T[13], T[14], T[15]; // transform matrix is transform from end effector(flange) to base link
    tf_tool2world = tf_baselink2world_E402 * tf_ee2base * tf_tool2ee;
    // std::cout << "tf_ee2base" << std::endl << tf_ee2base << std::endl;
    // ur_kinematics::forward(q_, T);会给出从base_link到end_effector的变换矩阵
    // 经过这样的变换，世界参考系考虑的是：面向UR机械臂时，竖直向上是Z正，向左是X正，向后是Y正，世界坐标的原点是UR机械臂的Base_link/Base的原点
    return tf_tool2world;
}