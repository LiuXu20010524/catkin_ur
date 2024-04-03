#include "cartesian_speed_controller/cartesian_speed_control.h"

Cartesian_speed_control::Cartesian_speed_control(ros::NodeHandle &nh)
{
    ROS_INFO("Cartesian_speed_control object is created.");
    twist_sub = nh.subscribe<geometry_msgs::Twist>("/twist_controller/command", 1, &Cartesian_speed_control::twist_callback, this);
    joint_state_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &Cartesian_speed_control::joint_state_callback, this);
    joint_pos_pub = nh.advertise<std_msgs::MultiArrayLayout>("/PositionTorqueController/command", 1);
    joint_speed_pub1 = nh.advertise<std_msgs::Float64>("/joint_eff_velocity_controller_1/command", 1);
    joint_speed_pub2 = nh.advertise<std_msgs::Float64>("/joint_eff_velocity_controller_2/command", 1);
    joint_speed_pub3 = nh.advertise<std_msgs::Float64>("/joint_eff_velocity_controller_3/command", 1);
    joint_speed_pub4 = nh.advertise<std_msgs::Float64>("/joint_eff_velocity_controller_4/command", 1);
    joint_speed_pub5 = nh.advertise<std_msgs::Float64>("/joint_eff_velocity_controller_5/command", 1);
    joint_speed_pub6 = nh.advertise<std_msgs::Float64>("/joint_eff_velocity_controller_6/command", 1);

    joint_speed_pub.resize(6);

    pin::urdf::buildModel(urdf_filename,model);
    data = pin::Data(model);
    end_frame_id = model.getFrameId("wrist_3_link");
    joint_speed_cmd.setZero();

    spinner = new ros::AsyncSpinner(10);
    spinner ->start();
    //注：一开始最好让机械臂先移动到一个初始位置，避免奇异点。
}

Cartesian_speed_control::~Cartesian_speed_control()
{
    ROS_INFO("Cartesian_speed_control object is destroyed.");
    delete spinner;
}

void Cartesian_speed_control::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    (sub_mark_== true )? :sub_mark_ = true;
    q << msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5];
}

void Cartesian_speed_control::twist_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    if(sub_mark_ == true)
    {
        static Eigen::Vector<double,6> twist_cmd;
        twist_cmd << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z;
        this->pinocchio_jociobian();
        joint_speed_cmd = pin_J_inverse * twist_cmd;
    }
}

void Cartesian_speed_control::pinocchio_jociobian()
{
    pin_J.setZero(); // Initialize the jacobian matrix
    pin::computeFrameJacobian(model,data,q,end_frame_id,pin::WORLD,pin_J); // Compute the jacobian matrix
    pin_J_inverse = pin_J.inverse(); // Compute the inverse of the jacobian matrix
}

void Cartesian_speed_control::compute_position()
{
    static float T = 1/300;
    pos_cmd = q + T * joint_speed_cmd;
}