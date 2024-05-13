#include "traj_phri_deformation/Traj_deform_pHRI.h"

Traj_deform_pHRI::Traj_deform_pHRI(ros::NodeHandle &nh, int deform_traj_num, int control_times_to_deform, int loop_rate):deform_traj_num_(deform_traj_num),control_times_to_deform_(control_times_to_deform),loop_rate_(loop_rate)
{
    ROS_INFO("Traj_deform_pHRI Constructor");
    // initialize subscriber and publisher
    joint_states_sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &Traj_deform_pHRI::joint_states_cb, this);
    sensor_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/mcc_1608g_daq", 1, &Traj_deform_pHRI::sensor_cb, this);
    joint_traj_cmd_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command", 1); //E402实机话题
    //joint_traj_cmd_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/scaled_pos_joint_traj_controller/command", 1); 

    // deform_traj.reserve(deform_traj_num_); // reserve the memory for deform_traj
    deform_traj_matrix.resize(7,deform_traj_num_); // resize the deform_traj_matrix
    deform_traj_vector.resize(3*deform_traj_num_); // resize the deform_traj_vector

    // alpha is the weight for the deformation
    alpha_ = 4e11; // alpha_ should be a very very large number in this case, to ensure the stability of the deformation

    // initialize the Osqp-Eigen parameters and solver
    // P_QP is a sparse matrix
    Eigen::MatrixXd P_QP_not_sparse;
    P_QP_not_sparse.resize(3*deform_traj_num_,3*deform_traj_num_);
    P_QP_not_sparse.setZero();
    P_QP.resize(3*deform_traj_num_,3*deform_traj_num_); // resize the P matrix for QP optimization
    A_paper.resize(deform_traj_num_,deform_traj_num_);
    A_paper.setZero();
    for(int i = 0; i < deform_traj_num_-3; i++) // initialize the P matrix for QP optimization
    {
        A_paper.block(i,i,1,4) = Eigen::Vector4d(-1,3,-3,1).transpose();
    }
    A_paper.block(deform_traj_num_-3,deform_traj_num_-4,1,4) = Eigen::Vector4d(-1,3,-3,1).transpose();
    A_paper.block(deform_traj_num_-2,deform_traj_num_-4,1,4) = Eigen::Vector4d(-1,3,-3,1).transpose();
    A_paper.block(deform_traj_num_-1,deform_traj_num_-4,1,4) = Eigen::Vector4d(-1,3,-3,1).transpose();
    P_QP_not_sparse.block(0,0,deform_traj_num_,deform_traj_num_) = alpha_*A_paper.transpose()*A_paper; // you can't change the value of the P matrix and A_paper matrix
    P_QP_not_sparse.block(deform_traj_num_,deform_traj_num_,deform_traj_num_,deform_traj_num_) = alpha_*A_paper.transpose()*A_paper; // you can't change the value of the P matrix and A_paper matrix
    P_QP_not_sparse.block(2*deform_traj_num_,2*deform_traj_num_,deform_traj_num_,deform_traj_num_) = (alpha_/1.5)*A_paper.transpose()*A_paper; // you can't change the value of the P matrix and A_paper matrix
    P_QP = P_QP_not_sparse.sparseView(); // you can't change the value of the P matrix and A_paper matrix
    // set the A matrix for QP optimization, A matrix is a sparse matrix and you can't change the value of the A matrix
    A_QP.resize(12,3*deform_traj_num_); // resize the A matrix for QP optimization, you can't change the value of the A matrix
    A_QP.setZero(); // you can't change the value of the A matrix
    A_QP.insert(0,0) = 1.0; // you can't change the value of the A matrix
    A_QP.insert(1,1) = 1.0; // you can't change the value of the A matrix
    A_QP.insert(2,deform_traj_num_-2) = 1.0; // you can't change the value of the A matrix
    A_QP.insert(3,deform_traj_num_-1) = 1.0; // you can't change the value of the A matrix
    A_QP.insert(4,deform_traj_num_) = 1.0;
    A_QP.insert(5,deform_traj_num_+1) = 1.0;
    A_QP.insert(6,2*deform_traj_num_-2) = 1.0; // you can't change the value of the A matrix
    A_QP.insert(7,2*deform_traj_num_-1) = 1.0; // you can't change the value of the A matrix
    A_QP.insert(8,2*deform_traj_num_) = 1.0; // you can't change the value of the A matrix
    A_QP.insert(9,2*deform_traj_num_+1) = 1.0; // you can't change the value of the A matrix
    A_QP.insert(10,3*deform_traj_num_-2) = 1.0; // you can't change the value of the A matrix
    A_QP.insert(11,3*deform_traj_num_-1) = 1.0; // you can't change the value of the A matrix
    // initialize the q vector for QP optimization, here just initialize the q vector for all the elements are 0, this will be updated in the traj_deform_main_func()
    q_QP.resize(3*deform_traj_num_); // resize the q vector for QP optimization
    q_QP.segment(0,deform_traj_num_) = Eigen::VectorXd::Constant(deform_traj_num_,0); // init the q vector for QP optimization
    q_QP.segment(deform_traj_num_,deform_traj_num_) = Eigen::VectorXd::Constant(deform_traj_num_,0); // init the q vector for QP optimization
    q_QP.segment(2*deform_traj_num_,deform_traj_num_) = Eigen::VectorXd::Constant(deform_traj_num_,0); // init the q vector for QP optimization
    // initialize the l and u vector for QP optimization
    l_QP.resize(12); // resize the l vector for QP optimization
    u_QP.resize(12); // resize the u vector for QP optimization
    l_QP.setZero(); // set the lower bound of the constraints
    u_QP.setZero(); // set the upper bound of the constraints
    // osqp-eigen setting
    solver.data()->setNumberOfVariables(3*deform_traj_num_); // set the number of variables
    solver.data()->setNumberOfConstraints(12); // set the number of constraints
    solver.data()->setGradient(q_QP); // set the q vector for QP optimization
    solver.settings()->setVerbosity(false); // set the verbosity of the solver, if you want to see the details of the solver, you can set it to true
    solver.settings()->setWarmStart(true); // set the warm start of the solver, if you want to use the warm start, you can set it to true
    solver.data()->setLinearConstraintsMatrix(A_QP); // set the A matrix for QP optimization
    solver.data()->setHessianMatrix(P_QP); // set the P matrix for QP optimization
    solver.data()->setLowerBound(l_QP); // set the lower bound of the constraints
    solver.data()->setUpperBound(u_QP); // set the upper bound of the constraints
    // initialize the solver
    if(!solver.initSolver())
    {
        ROS_ERROR("OSQP optimization initialization failed!");
        return;
    }

    this->compute_tf_sensor2flange(); // compute the transformation matrix from the sensor to the end effector(flange)
    // this->compute_tf_flange2tool0();
    this->compute_transform_base_link2world_E402();
    spinner = new ros::AsyncSpinner(10);
    spinner->start();
    ROS_INFO("end of constructor");
}

Traj_deform_pHRI::~Traj_deform_pHRI()
{
    ROS_INFO("Traj_deform_pHRI Destructor");
    spinner->stop();
    delete spinner;
}

// Callback function for joint_states
void Traj_deform_pHRI::joint_states_cb(const sensor_msgs::JointState::ConstPtr &msg)
{
    (sub_joint_mark == true )? :sub_joint_mark = true; // Mark for joint_states subscriber
    // Note: The order of the returned joint angles and joint velocitu is inconsistent with the structural order of the UR manipulator. 
    // The first joint and the third joint should be exchanged.
    joint_position_ << msg->position[2], msg->position[1], msg->position[0], msg->position[3], msg->position[4], msg->position[5];
    joint_velocity_ << msg->velocity[2], msg->velocity[1], msg->velocity[0], msg->velocity[3], msg->velocity[4], msg->velocity[5];
}

// Callback function for sensor
void Traj_deform_pHRI::sensor_cb(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
    (sub_sensor_mark == true )? :sub_sensor_mark = true; // Mark for sensor subscriber
    sensor_data_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
}

// Move to initial pose, using MoveIt
bool Traj_deform_pHRI::move_to_initial_pose(geometry_msgs::Pose &initial_target_pose)
{
    // transform the initial pose from the world frame to the base_link frame
    Eigen::Vector3d position(initial_target_pose.position.x, initial_target_pose.position.y, initial_target_pose.position.z);
    position = tf_baselink2world_E402.block(0,0,3,3).inverse()*position;
    initial_target_pose.position.x = position(0); initial_target_pose.position.y = position(1); initial_target_pose.position.z = position(2);
    Eigen::Quaterniond q(initial_target_pose.orientation.w, initial_target_pose.orientation.x, initial_target_pose.orientation.y, initial_target_pose.orientation.z);
    q = tf_baselink2world_E402.block(0,0,3,3).inverse()*q;
    initial_target_pose.orientation.x = q.x(); initial_target_pose.orientation.y = q.y(); initial_target_pose.orientation.z = q.z(); initial_target_pose.orientation.w = q.w();
    // ----------------
    std::vector<double> joint_group_positions;
    joint_group_positions.push_back(0.5438811462201656);
    joint_group_positions.push_back(-1.462031);
    joint_group_positions.push_back(-1.9216135275443218);
    joint_group_positions.push_back(-0.548961);
    joint_group_positions.push_back(1.741);
    joint_group_positions.push_back(-0.172145771);
    //

    moveit::planning_interface::MoveGroupInterface ur5_move_group("manipulator");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ur5_move_group.setMaxAccelerationScalingFactor(0.1); // Allowable maximum acceleration
    ur5_move_group.setMaxVelocityScalingFactor(0.1); // Allowable maximum velocity
    ur5_move_group.setGoalJointTolerance(.001); // Set the tolerance of the goal joint
    ur5_move_group.setGoalPositionTolerance(.001); // Set the tolerance of the goal position
    ur5_move_group.setGoalOrientationTolerance(.001); // Set the tolerance of the goal orientation
    ur5_move_group.setEndEffectorLink("flange"); // Set the end effector link
    ur5_move_group.setPoseReferenceFrame("base_link"); // Set the reference frame
    // move to the initial position
    
    // ur5_move_group.setPoseTarget(initial_target_pose); // Set target pose
    ur5_move_group.setJointValueTarget(joint_group_positions);
    ur5_move_group.plan(my_plan);
    if(ur5_move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Move to initial pose successfully!");
        return true;
    }
    else
    {
        ROS_ERROR("Move to initial pose failed!");
        return false;
    }
}

// Define the original trajectory
// The original trajectory is a Cartesian coordinate of sensor coordinate
void Traj_deform_pHRI::define_origin_traj(bool using_custom_traj, std::optional<std::vector<geometry_msgs::Pose>> custom_traj,std::optional<bool> traj_repeat)
{
    if (using_custom_traj) // weahter using custom trajectory
    {
        Eigen::Vector<double, 7> temp;
        for (auto &pose : custom_traj.value())
        {
            temp << pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w;
            origin_traj.push_back(temp);
        }
        if (traj_repeat.has_value()) // whether to repeat the trajectory for custom trajectory
        {
            traj_repeat_mark = traj_repeat.value(); // you need to make sure your custom trajectory is a closed loop
        }
    }
    else // if not using custom trajectory
    {
        // Define the original trajectory
        // The original trajectory is a circle in the Cartesian space
        Eigen::Vector<double, 7> temp;
        for (double i = 0; i < 2 * PI; i += 0.004)
        {
            // 我认为ur_kinematics的逆解有相对较大误差，这里做了一个补偿
            temp = Eigen::Vector<double,7>(-0.2*cos(i)-0.02, 0.66, 0.2*sin(i)-0.02 , 0.5, -0.5, 0.5, 0.5);
            // temp = Eigen::Vector<double,7>(-0.2*cos(i), 0.6, 0.2*sin(i), 0.5, -0.5, 0.5, 0.5);
            origin_traj.push_back(temp);
        }
        traj_repeat_mark = true; // default repeat the default trajectory
    }

    origin_traj_iter = origin_traj.begin(); // set the iterator to the beginning of the original trajectory
    deform_traj_matrix.col(0) = *origin_traj_iter; // the first two points of the deformed trajectory are same.
    origin_traj_iter++;
    for(int i = 1; i < deform_traj_num_; i++) // copy the original trajectory to the deformed trajectory
    {
        deform_traj_matrix.col(i) = *origin_traj_iter;
        origin_traj_iter++;
    }
}

// compute the transformation matrix from the sensor to the end effector(flange)
void Traj_deform_pHRI::compute_tf_sensor2flange()
{
    Eigen::Matrix3d orientation_sensor2flange;
    Eigen::Vector3d position_sensor2flange;
    Eigen::Vector3d euler_angle_sensor2flange(-PI/2, 0.0, -PI/2);  // sensor euler angle to end effector(flange), xyz euler angle
    position_sensor2flange << 0.0, 0.0, 0.0;
    orientation_sensor2flange = (Eigen::AngleAxisd(euler_angle_sensor2flange[2], Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(euler_angle_sensor2flange[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(euler_angle_sensor2flange[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    tf_sensor2flange.block(0,0,3,3) = orientation_sensor2flange;
    tf_sensor2flange.block(0,3,3,1) = position_sensor2flange;
    tf_sensor2flange(3,3) = 1.0;
}

void Traj_deform_pHRI::compute_transform_base_link2world_E402()
{
    Eigen::Matrix3d orientation_base2world,orientation_baselink2base;
    // Eigen::Vector3d euler_angle_baselink2base(0.0, 0.0, -PI);  // base_link to base xyz euler angle
    Eigen::Vector3d position_base2world(0,0,0);
    // orientation_baselink2base = (Eigen::AngleAxisd(euler_angle_baselink2base[2], Eigen::Vector3d::UnitZ())
    //                             * Eigen::AngleAxisd(euler_angle_baselink2base[1], Eigen::Vector3d::UnitY())
    //                             * Eigen::AngleAxisd(euler_angle_baselink2base[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
    // orientation_baselink2base 是base_link到base的变换矩阵
    orientation_base2world  <<  sqrt(2)/2, -sqrt(2)/2, 0.0,
                                -0.5, -0.5, sqrt(2)/2,
                                -0.5, -0.5, -sqrt(2)/2;  //E402的UR机械臂的base坐标系和世界坐标系的转换矩阵
    tf_baselink2world_E402.block(0,0,3,3) = orientation_base2world;
    tf_baselink2world_E402.block(0,3,3,1) = position_base2world;
    tf_baselink2world_E402(3,3) = 1.0;
}

// main function for trajectory deformation
void Traj_deform_pHRI::traj_deform_main_func() // so far, the code only deform the xyz position of the trajectory
{
    // In order to improve the scalability of the code, QP optimization is used here
    // Set the q vector for OSQP optimization first
    Eigen::Vector<double,6> F; // sensor msg transformed from sensor frame to base_link(world) frame
    Eigen::Matrix4d tf_flange2baselink;
    double T[16];
    double q[6] = {joint_position_(0),joint_position_(1),joint_position_(2),joint_position_(3),joint_position_(4),joint_position_(5)};
    ur_kinematics::forward(q,T); // calculate the forward kinematics
    tf_flange2baselink << T[0], T[1], T[2], T[3],
                        T[4], T[5], T[6], T[7],
                        T[8], T[9], T[10], T[11],
                        T[12], T[13], T[14], T[15]; // transformation matrix from flange to base_link frame
    F.segment(0,3) = tf_baselink2world_E402.block(0,0,3,3)*tf_flange2baselink.block(0,0,3,3)*tf_sensor2flange.block(0,0,3,3)*sensor_data_.segment(0,3);
    F.segment(3,3) = tf_baselink2world_E402.block(0,0,3,3)*tf_flange2baselink.block(0,0,3,3)*tf_sensor2flange.block(0,0,3,3)*sensor_data_.segment(3,3);
    q_QP.segment(0,deform_traj_num_) = Eigen::VectorXd::Constant(deform_traj_num_,-F(0)); // set the q vector for QP optimization
    q_QP.segment(deform_traj_num_,deform_traj_num_) = Eigen::VectorXd::Constant(deform_traj_num_,-F(1));
    q_QP.segment(2*deform_traj_num_,deform_traj_num_) = Eigen::VectorXd::Constant(deform_traj_num_,-F(2));
    solver.updateGradient(q_QP); // update the q vector for QP optimization
    // start the Osqp optimization
    if(solver.solveProblem()!=OsqpEigen::ErrorExitFlag::NoError)
    {
        ROS_ERROR("OSQP optimization failed!");
        return;
    }
    deform_traj_vector = solver.getSolution(); // get the solution of the QP optimization
    // update the deformed trajectory
    for(int i = 0; i < deform_traj_num_; i++)
    {
        deform_traj_matrix(0,i) += deform_traj_vector(i); // update the x position of the deformed trajectory
        deform_traj_matrix(1,i) += deform_traj_vector(i+deform_traj_num_); // update the y position of the deformed trajectory
        deform_traj_matrix(2,i) += deform_traj_vector(i+2*deform_traj_num_); // update the z position of the deformed trajectory
    }
}

void Traj_deform_pHRI::control_main_func()
{
    trajectory_msgs::JointTrajectoryPoint joint_traj_point;
    Eigen::Matrix4d tf_flange2baselink;
    Eigen::Quaterniond q_targetpose_to_world;
    // note: moveit cartesian target point is the tool0 frame, base on the world frame, and the base_link frame is coincident with the world frame
    double q_sol[48] = {0}; // use ur_kinematics to calculate the inverse kinematics, the q_sol is a pointer to a 8×6 joint angles
    int inverse_return;
    Eigen::Matrix<double,8,6> q_sol_matrix; // joint angles matrix
    // calculate the target point and inverse kinematics
        // convert quaternion to rotation matrix
    q_targetpose_to_world = Eigen::Quaterniond(deform_traj_matrix(6,0),deform_traj_matrix(3,0),deform_traj_matrix(4,0),deform_traj_matrix(5,0));
    tf_flange2baselink.block(0,0,3,3) = (tf_baselink2world_E402.block(0,0,3,3).inverse())*(q_targetpose_to_world);
    tf_flange2baselink.block(3,0,1,3) = Eigen::Matrix<double,1,3>::Zero();
    tf_flange2baselink.block(0,3,3,1) = (tf_baselink2world_E402.block(0,0,3,3).inverse())*deform_traj_matrix.block(0,0,3,1);
    tf_flange2baselink(3,3) = 1.0;
        // calculate the inverse kinematics
    double T[16] = {tf_flange2baselink(0,0), tf_flange2baselink(0,1), tf_flange2baselink(0,2), tf_flange2baselink(0,3),
                    tf_flange2baselink(1,0), tf_flange2baselink(1,1), tf_flange2baselink(1,2), tf_flange2baselink(1,3),
                    tf_flange2baselink(2,0), tf_flange2baselink(2,1), tf_flange2baselink(2,2), tf_flange2baselink(2,3),
                    0, 0, 0, 1};
    inverse_return = ur_kinematics::inverse(T, q_sol); // calculate the inverse kinematics, return the number of solutions
    q_sol_matrix << q_sol[0], q_sol[1], q_sol[2], q_sol[3], q_sol[4], q_sol[5],
                    q_sol[6], q_sol[7], q_sol[8], q_sol[9], q_sol[10], q_sol[11],
                    q_sol[12], q_sol[13], q_sol[14], q_sol[15], q_sol[16], q_sol[17],
                    q_sol[18], q_sol[19], q_sol[20], q_sol[21], q_sol[22], q_sol[23],
                    q_sol[24], q_sol[25], q_sol[26], q_sol[27], q_sol[28], q_sol[29],
                    q_sol[30], q_sol[31], q_sol[32], q_sol[33], q_sol[34], q_sol[35],
                    q_sol[36], q_sol[37], q_sol[38], q_sol[39], q_sol[40], q_sol[41],
                    q_sol[42], q_sol[43], q_sol[44], q_sol[45], q_sol[46], q_sol[47];
        // choose the joint angles which is closest to the current joint angles
        // Note: the q_sol angles returned by the ur_kineamtics::inverse is in [0,2*PI), and not following the same order of change
        // Note: you need to check every angles return by ur_kineamtics::inverse, see weather it is closest to the current joint angles
        //       by adding and minusing 2*PI, PI, 0, -PI, -2*PI. It's not each row adds or minuses the same value, maybe different in each
        //       element of the same row. 
    std::vector<double> position;
    // 原始的代码范围-----------------------------------------------------------
    double min_distance = 1000000;
    int min_index = 0;
    double count =0;
    for(int i = 0; i < inverse_return; i++) // this for loop is to find which row in the q_sol_matrix is closest to the current joint angles
    {   
        count = 0;
        for(int j = 0; j<6;j++)
        {
            double temp = std::min(std::min(std::min(std::min(pow(q_sol_matrix(i,j)+PI-joint_position_(j),2),pow(q_sol_matrix(i,j)-joint_position_(j),2)),pow(q_sol_matrix(i,j)-PI-joint_position_(j),2)),pow(q_sol_matrix(i,j)+2*PI-joint_position_(j),2)),pow(q_sol_matrix(i,j)-2*PI-joint_position_(j),2));
            count+=temp;
        }
        if(count < min_distance)
        {
            min_distance = count;
            min_index = i;
        }
    }
    for(int i=0;i<6;i++) // this for loop is to calculate the joint angles which is closest to the current joint angles in the particular row found in the previous for loop
    {
        std::vector<double> temp_;
        std::vector<double> temp_pow_;
        temp_.push_back(q_sol_matrix(min_index,i));
        temp_.push_back(q_sol_matrix(min_index,i)+PI);
        temp_.push_back(q_sol_matrix(min_index,i)-PI);
        temp_.push_back(q_sol_matrix(min_index,i)+2*PI);
        temp_.push_back(q_sol_matrix(min_index,i)-2*PI);
        temp_pow_.push_back(pow(q_sol_matrix(min_index,i)-joint_position_(i),2));
        temp_pow_.push_back(pow(q_sol_matrix(min_index,i)+PI-joint_position_(i),2));
        temp_pow_.push_back(pow(q_sol_matrix(min_index,i)-PI-joint_position_(i),2));
        temp_pow_.push_back(pow(q_sol_matrix(min_index,i)+2*PI-joint_position_(i),2));
        temp_pow_.push_back(pow(q_sol_matrix(min_index,i)-2*PI-joint_position_(i),2));
        auto min_it = std::min_element(temp_pow_.begin(),temp_pow_.end());
        position.push_back(temp_[std::distance(temp_pow_.begin(),min_it)]);
    }
    // 原始的代码范围-----------------------------------------------------------

    // 优化后的代码范围-----------------------------------------------------------
    // double diff_temp=0.0;
    // double q_diff[8] = {0}; // 逆解到初始解的距离，此值辅助进行选解
    // double min_dis = 1000000;
    // int min_index = 0;
    // for(int i = 0; i < inverse_return; i++) // this for loop is to find which row in the q_sol_matrix is closest to the current joint angles
    // {   
    //     for(int j = 0; j<6;j++)
    //     {
    //         if(q_sol_matrix(i,j) > PI)
    //         {
    //             q_sol_matrix(i,j) -= 2*PI;
    //         }
    //         else if(q_sol_matrix(i,j) < -PI)
    //         {
    //             q_sol_matrix(i,j) += 2*PI;
    //         }
    //         diff_temp = q_sol_matrix(i,j) - joint_position_(j);
    //         q_diff[i] += abs(diff_temp);
    //     }
    //     if(q_diff[i] < min_dis)
    //     {
    //         min_dis = q_diff[i];
    //         min_index = i;
    //     }
    // }
    // for(int i=0;i<6;i++)
    // {
    //     position.push_back(q_sol_matrix(min_index,i));
    // }
    // 优化后的代码范围-----------------------------------------------------------
    joint_traj_cmd.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    joint_traj_cmd.points.clear();
    joint_traj_point.positions = position;
    joint_traj_point.time_from_start = ros::Duration(1/loop_rate_)+ros::Duration(0.01); //need to add a small value to avoid control command out of time
    joint_traj_cmd.points.push_back(joint_traj_point);
    joint_traj_cmd.header.stamp = ros::Time::now(); // set the time stamp
    joint_traj_cmd_pub.publish(joint_traj_cmd);
}

void Traj_deform_pHRI::traj_deform_pHRI_with_control()
{
    // wait for the subscriber to get the data
    // while(sub_joint_mark == false || sub_sensor_mark == false){ ros::Duration(0.0001).sleep();} // wait for the all subscriber to get the data

    ros::Rate loop_rate(loop_rate_); // set the loop rate, note that the ur5 has a highest control rate of 125Hz
    int force_sensor_count = 0; // count the force sensor trigger times
    int count_control_time_to_traj_deform = 0; // count the control times to deform the trajectory
    int count_deform_to_end = deform_traj_num_;
    // main loop
    while(ros::ok())
    {
        count_control_time_to_traj_deform ++;
        (abs(sensor_data_[0])>=0.3 || abs(sensor_data_[1])>=0.3 || abs(sensor_data_[2])>=0.3)? force_sensor_count++ : force_sensor_count = 0;
        // if sensor data is high enough, we count the trigger times
        //only x,y,z force is considered here, you can add the torque if you need

        if(count_control_time_to_traj_deform >= control_times_to_deform_)
        {
            count_control_time_to_traj_deform = 0;
            if(origin_traj_no_repeat_to_end_mark == false) // if the original trajectory is not to the end, you can push the next point to the deformed trajectory
            {
                deform_traj_matrix.leftCols(deform_traj_num_-1) = deform_traj_matrix.rightCols(deform_traj_num_-1); // deformed trajectory matrix left shift
                deform_traj_matrix.rightCols(1) = *origin_traj_iter; // push the next point to the deformed trajectory
            }
            else
            {
                if(count_deform_to_end > 1)
                {
                    deform_traj_matrix.leftCols(deform_traj_num_-1) = deform_traj_matrix.rightCols(deform_traj_num_-1);
                    count_deform_to_end--;
                }
            }
            if(traj_repeat_mark) // if the trajectory is a closed loop, the iterator will return to the beginning
            {
                origin_traj_iter = (++origin_traj_iter == origin_traj.end())? origin_traj.begin() : origin_traj_iter; // if the trajectory is a closed loop, the iterator will return to the beginning
            }
            else // if the trajectory is not a closed loop, the iterator will stop and mark the simbol
            {
                if(origin_traj_no_repeat_to_end_mark != true) // weather the mark had already been set
                {
                    origin_traj_no_repeat_to_end_mark = (++origin_traj_iter == origin_traj.end())? true : false; // if the iterator is to the end at first time, set the mark to true
                }
            }
            // --- Start Trajectory deformation ---//
            if(force_sensor_count >= 4 && origin_traj_no_repeat_to_end_mark == false) // if the force sensor is triggered and the original trajectory is not to the end
            // if sensor data is high enough for 4 times countilously, then deform the trajectory
            {
                ROS_INFO("Trajectory deformation is Triggered!");
                // --- Start Trajectory deformation ---//
                this->traj_deform_main_func(); // trajectory deformation function
            }
        }
        this->control_main_func(); // main control function

        loop_rate.sleep(); // sleep to control the frequency
    }
}