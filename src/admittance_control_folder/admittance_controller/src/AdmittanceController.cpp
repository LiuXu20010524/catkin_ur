#include "admittance_controller/AdmittanceController.h"

// 构造函数
AdmittanceController::AdmittanceController(ros::NodeHandle *nh):freq_(125.0),step_(1.0/freq_){
    
    // --- ROS Init --- //
    nh_ =  nh;
    loop_rate_ = new ros::Rate(freq_);

    // --- Subscriber Init --- //
    joint_state_sub_ = nh_->subscribe("/joint_states",10,&AdmittanceController::joint_state_cb,this);
    ft_sensor_sub_ = nh_->subscribe<geometry_msgs::WrenchStamped>("/mcc_1608g_daq", 100, &AdmittanceController::ft_sensor_cb, this);;

    // --- Publisher Init --- //
    twist_pub_ = nh_->advertise<geometry_msgs::Twist>("/twist_controller/command",10);

    // --- Joint State --- //
    joint_state_position_.resize(6);
    joint_state_velocity_.resize(6);

    // --- Admittance VARIABLES Init --- //
    M_ <<   20, 0, 0,
            0, 20, 0,
            0, 0, 20;

    K_ <<   500, 0, 0,
            0, 500, 0,
            0, 0, 500;

    B_ = Eigen::Matrix3d::Zero();
    for(int i=0; i<3; i++){
        B_(i,i) = 2 * sqrt(M_(i,i)*K_(i,i));
    }

    // --- Caculate TF Matrices --- //
    caculateTF();

    // --- Init Fext --- //
    Fext_body_frame_.setZero();
    Fext_world_frame_.setZero();

    // --- Init Online Filter --- //
    butterWorth_ = new onlineFilter(4, 10, 125, 3);    // 4阶10Hz采样频率125Hz ButterWorth滤波器，滤波数据为1×3维

    ros::Duration(2.0).sleep();
}

// 析构函数
AdmittanceController::~AdmittanceController(){
    delete nh_;
    delete loop_rate_;
}

// 计算【基坐标系】到【世界坐标系】以及【工具坐标系】到【末端坐标系】的齐次变换矩阵
void AdmittanceController::caculateTF(){

    // --- TF Matrix --- //
    // tool2ee : RPY = [-PI/2, 0, -PI/2] , Translation = [0 0 0]
    Eigen::Matrix3d rotation_tool2ee;
    Eigen::Vector3d translation_tool2ee;
    Eigen::Vector3d euler_angle_tool2ee(-PI/2, 0.0, -PI/2);
    translation_tool2ee << 0.0, 0.0, 0.0;
    rotation_tool2ee =  Eigen::AngleAxisd(euler_angle_tool2ee[2], Eigen::Vector3d::UnitZ()) * 
                        Eigen::AngleAxisd(euler_angle_tool2ee[1], Eigen::Vector3d::UnitY()) * 
                        Eigen::AngleAxisd(euler_angle_tool2ee[0], Eigen::Vector3d::UnitX());

    tf_tool2ee_ = Eigen::Matrix4d::Zero();
    tf_tool2ee_.block(0,0,3,3) = rotation_tool2ee;
    tf_tool2ee_.block(0,3,3,1) = translation_tool2ee;
    tf_tool2ee_(3,3) = 1.0;

    // base2world 此处的数据不是我计算的，是沿用的师兄的数据
    Eigen::Matrix3d rotation_base2world_1,rotation_base2world_2;
    Eigen::Vector3d euler_angle_base2world_2(0.0, 0.0, -PI),translation_base2world;
    translation_base2world << 0.0, 0.0, 0.0;
    rotation_base2world_1   <<  sqrt(2)/2, -sqrt(2)/2, 0.0,
                                -0.5, -0.5, -sqrt(2)/2,
                                0.5, 0.5, -sqrt(2)/2;
    rotation_base2world_2 = Eigen::AngleAxisd(euler_angle_base2world_2[2], Eigen::Vector3d::UnitZ()) * 
                            Eigen::AngleAxisd(euler_angle_base2world_2[1], Eigen::Vector3d::UnitY()) * 
                            Eigen::AngleAxisd(euler_angle_base2world_2[0], Eigen::Vector3d::UnitX());

    tf_base2world_ = Eigen::Matrix4d::Zero();

    // E402房间的基坐标系到世界坐标系的变换
    // tf_base2world_.block(0,0,3,3) = rotation_base2world_1*rotation_base2world_2;
    // tf_base2world_.block(0,3,3,1) = translation_base2world;
    // tf_base2world_(3,3) = 1.0;

    // E305房间的基坐标系到世界坐标系的变换
    // tf_base2world_.block(0,0,3,3) = Eigen::Matrix3d::Identity();
    tf_base2world_.block(0,0,3,3) = rotation_base2world_2;
    tf_base2world_.block(0,3,3,1) = Eigen::Vector3d::Zero();
    tf_base2world_(3,3) = 1.0;
}

// 关节订阅的回调函数
void AdmittanceController::joint_state_cb(const sensor_msgs::JointState::ConstPtr &msgs){
    // joint position vector
    joint_state_position_(0) = msgs->position[2];
    joint_state_position_(1) = msgs->position[1];
    joint_state_position_(2) = msgs->position[0];
    joint_state_position_(3) = msgs->position[3];
    joint_state_position_(4) = msgs->position[4];
    joint_state_position_(5) = msgs->position[5];

    // joint velocity vector
    joint_state_velocity_(0) = msgs->velocity[2];
    joint_state_velocity_(1) = msgs->velocity[1];
    joint_state_velocity_(2) = msgs->velocity[0];
    joint_state_velocity_(3) = msgs->velocity[3];
    joint_state_velocity_(4) = msgs->velocity[4];
    joint_state_velocity_(5) = msgs->velocity[5];
}

// 力传感器订阅的回调函数
void AdmittanceController::ft_sensor_cb(const geometry_msgs::WrenchStamped::ConstPtr &msgs){
    static Eigen::MatrixXd Fext_raw(1,3),Fext_filtered(1,3);

    Fext_raw(0,0) = msgs->wrench.force.x;
    Fext_raw(0,1) = msgs->wrench.force.y;
    Fext_raw(0,2) = msgs->wrench.force.z;

    butterWorth_->filter(Fext_raw, Fext_filtered);      // 对ATI力传感器的数据进行滤波

    for (int i=0;i<3;i++)
        Fext_body_frame_(i) = Fext_filtered(0,i);
}

// ur前向运动学
Eigen::Matrix4d AdmittanceController::FK(Eigen::VectorXd joint){
    double joint_array[6]={0},tf_ee2base_array[16]={0};
    for (int i=0;i<6;i++)
        joint_array[i] = joint(i);

    ur_kinematics::forward(joint_array,tf_ee2base_array);
    Eigen::Matrix4d tf_ee2base;
    tf_ee2base <<   tf_ee2base_array[0],tf_ee2base_array[1],tf_ee2base_array[2],tf_ee2base_array[3],
                    tf_ee2base_array[4],tf_ee2base_array[5],tf_ee2base_array[6],tf_ee2base_array[7],
                    tf_ee2base_array[8],tf_ee2base_array[9],tf_ee2base_array[10],tf_ee2base_array[11],
                    tf_ee2base_array[12],tf_ee2base_array[13],tf_ee2base_array[14],tf_ee2base_array[15];

    return tf_base2world_*tf_ee2base*tf_tool2ee_;
}

// 获取当前【工具坐标系】相对【世界坐标系】的位姿
Eigen::Matrix4d AdmittanceController::getCurrentFrame(){
    Eigen::VectorXd joint(7);
    joint = joint_state_position_;
    return FK(joint);
}

// 获取当前的关节角度
Eigen::VectorXd AdmittanceController::getCurrentJoint(){
    Eigen::VectorXd joint(7);
    joint = joint_state_position_;
    return joint;
}

Eigen::Vector3d AdmittanceController::getReferenceTrajectory(){
    return xr_;
}


// 打印当前【工具坐标系】相对【世界坐标系】的位姿
void AdmittanceController::printCurrentFrame(){
    Eigen::VectorXd joint(7);
    joint = joint_state_position_;
    
    std::cout << "Current Frame (tool2world) is :" << std::endl;
    FK(joint);
    std::cout << FK(joint)<<std::endl;
}

// 打印当前的关节角度
void AdmittanceController::printCurrentJoint(){
    Eigen::VectorXd joint(7);
    joint = joint_state_position_;
    
    std::cout << "Current Joint is :" << std::endl;
    std::cout << joint << std::endl;
}

// 获取导纳参数
void AdmittanceController::getAdmittance(Eigen::Matrix3d &M,Eigen::Matrix3d &B,Eigen::Matrix3d &K){
    M = M_;
    B = B_;
    K = K_;
}

// 获取刚度矩阵
void AdmittanceController::getStiffness(Eigen::Matrix3d &K){
    K = K_;
}

// 获取阻尼矩阵
void AdmittanceController::getDamping(Eigen::Matrix3d &B){
    B = B_;
}

// 获取惯性矩阵
void AdmittanceController::getInertia(Eigen::Matrix3d &M){
    M = M_;
}

// 移动到指定位姿（未完成）
bool AdmittanceController::moveToFrame(Eigen::Vector3d position,Eigen::Vector3d RPY){
    // PID Controller
    const double TIME_OUT=10.0;
    const double Kp = 20.0,Ki=1.0,Kd=1.0;
    const double tol_pos = 1e-4,tol_vel=1e-3;

    while (ros::ok())
    {
        ros::Duration(0.001).sleep();
    }
    return true;
}

// 将力传感器数据转换到【世界坐标系】下
void AdmittanceController::handleExternalForce(){
    Eigen::Matrix4d current_frame = getCurrentFrame();
    Fext_world_frame_ = current_frame.block(0,0,3,3)*Fext_body_frame_;
    // ROS_INFO("Fext_world_frame_ = %f %f %f \n",Fext_world_frame_(0),Fext_world_frame_(1),Fext_world_frame_(2));
}

// 设定导纳控制的参考轨迹
void AdmittanceController::setReferenceTrajectory(Eigen::Vector3d xr,Eigen::Vector3d dxr){
    xr_ = xr;
    dxr_ = dxr;
}

// 设定导纳参数
void AdmittanceController::setAdmittance(Eigen::MatrixX3d M,Eigen::MatrixX3d B,Eigen::MatrixX3d K){
    M_ = M;
    B_ = B;
    K_ = K;
}

// 计算期望的加速度
void AdmittanceController::computeAdmittance(){
    // calculate desired acceleration
    desired_acc_ = M_.inverse() * (Fext_world_frame_ - B_ * (dx_last_ - dxr_) - K_*(x_last_ - xr_) );

    // update trajectory
    ros::Duration du = loop_rate_->expectedCycleTime();
    dx_last_ = dx_last_ + desired_acc_ *du.toSec();
    x_last_ = x_last_ + dx_last_*du.toSec();

    // calculate twist in body frame 
    Eigen::Matrix3d rotation;
    // E402 使用
    // rotation <<  sqrt(2)/2, -sqrt(2)/2, 0.0,
    //             -0.5, -0.5, -sqrt(2)/2,
    //             0.5, 0.5, -sqrt(2)/2;

    // E305 使用
    rotation = Eigen::Matrix3d::Identity();

    
    twist_ = rotation.transpose()*dx_last_;
}

// 发送运动指令
void AdmittanceController::sendTwistCommand(){
    twist_cmd_.linear.x = twist_(0);
    twist_cmd_.linear.y = twist_(1);
    twist_cmd_.linear.z = twist_(2);
    twist_cmd_.angular.x = 0.0;
    twist_cmd_.angular.y = 0.0;
    twist_cmd_.angular.z = 0.0;

    twist_pub_.publish(twist_cmd_);
}

// 启动导纳控制
void AdmittanceController::run(){

    // --- Init Trajectory Data --- //
    desired_acc_.setZero();
    Eigen::Matrix4d currentFrame = getCurrentFrame();
    x_last_ = currentFrame.block(0,3,3,1);
    xr_ = currentFrame.block(0,3,3,1);
    dx_last_.setZero();
    dxr_.setZero();
    twist_.setZero();

    // --- Variable Reference Trajectory --- //
    Eigen::Vector3d xr,dxr,x_init;
    double t = 0.0;
    x_init = x_last_;
    xr = xr_;
    dxr.setZero();

    ROS_INFO("Start admittance control!");

    while(ros::ok()){
        // handle external force 
        handleExternalForce();

        // set variable reference trajectory
        // setReferenceTrajectory(xr,dxr);

        // compute
        computeAdmittance();

        // send twist command
        sendTwistCommand();

        // loop
        loop_rate_->sleep();
    }
}



