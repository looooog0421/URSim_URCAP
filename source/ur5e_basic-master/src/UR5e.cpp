#include "UR5e.hpp"

UR5e::UR5e(ros::NodeHandle _nh, const std::string &_urdf_param,
           const std::string &_chain_start, const std::string &_chain_end,
           double _timeout, std::string &_UR_prefix, double control_rate) :
        nh_(_nh),
        m_control_rate_(control_rate) {
    //init
    std::string _urdf_param_full = _UR_prefix + _urdf_param;
    p_tracik_solver_ = new TRAC_IK::TRAC_IK(_chain_start, _chain_end, _urdf_param_full, _timeout, 1e-5); //反解
    bool valid = p_tracik_solver_->getKDLChain(chain_);
    p_fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain_);  //正解
    p_jacobian_solver_ = new KDL::ChainJntToJacSolver(chain_);  // jacobian
    joint_size_ = 6;
    joint_names_.push_back("shoulder_pan_joint");
    joint_names_.push_back("shoulder_lift_joint");
    joint_names_.push_back("elbow_joint");
    joint_names_.push_back("wrist_1_joint");
    joint_names_.push_back("wrist_2_joint");
    joint_names_.push_back("wrist_3_joint");

    sub_joint_state_ = nh_.subscribe(_UR_prefix + "/joint_states", 1, &UR5e::subJointStateCB, this);
    sub_wrench_ = nh_.subscribe(_UR_prefix + "/wrench", 1, &UR5e::subWrenchCB, this);
    sub_end_vel_cmd_ = nh_.subscribe("ur5e_end_vel_cmd", 1, &UR5e::subEndVelCmdCB, this);

    pub_hand_cmd_ = nh_.advertise<robotiq_85_msgs::GripperCmd>("gripper/cmd",1);
    
    pub_filtered_wrench_base_ = nh_.advertise<geometry_msgs::WrenchStamped>("filtered_wrench_base", 1);

    client_servoj_ = new Client(_UR_prefix + "/pos_joint_traj_controller/follow_joint_trajectory", true);
    ROS_INFO("wait for UR server");
    client_servoj_->waitForServer(ros::Duration());
    ROS_INFO("server connect! ");

    force_ur_update();
    //防止转速过大　初始化
    // start_frame_ = current_frame_;
    start_JntArr_ = current_JntArr_;
    b_speed_ = true;
    // repair_wrench();

    nh_.param("end_admittance_control", end_admittance_control_, false);
    ROS_INFO("initialized variables ");

    source_dir_ = boost::filesystem::path(__FILE__).parent_path().parent_path().string();
}

void UR5e::subJointStateCB(const sensor_msgs::JointState &state) {
    //get joint array
    KDL::JntArray jntArr;
    KDL::JntArray jntSpeed;
    KDL::JntArray jntCur;
    jntArr.resize(joint_size_);
    jntSpeed.resize(joint_size_);
    jntCur.resize(joint_size_);
    int n = state.name.size();
    for (int i = 0; i < joint_size_; ++i) {
        int x = 0;
        for (; x < n; ++x){
            if (state.name[x] == (joint_names_[i])) {
                jntArr(i) = state.position[x];
                jntSpeed(i) = state.velocity[x];
                jntCur(i) = state.effort[x];
                break;
            }
        }
        if (x == n) {
            ROS_ERROR_STREAM("Error,  joint name : " << joint_names_[i] << " , not found.  ");
            return;
        }
    }
    current_JntArr_ = jntArr;
    current_JntCur_ = jntCur;
    // forward kinematic
    p_fk_solver_->JntToCart(current_JntArr_, current_frame_);
    bur_sub_ = true;
}

void UR5e::subWrenchCB(const geometry_msgs::WrenchStamped &wrench) {
    //repair for sensor installation
    KDL::Wrench wrench_ori(KDL::Vector(wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z), 
        KDL::Vector(wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z));

    wrench_ee_ = wrench_ori - repair_wrench_;
    wrench_ee_.force -= current_frame_.M.Inverse() * gravity_;
    wrench_ee_.torque -= mass_center_ * (current_frame_.M.Inverse() * gravity_);
    // 计算相对于基坐标系的力矩信息
    // 传感器坐标系和tool0即正解坐标系一直
    wrench_base_ = current_frame_ * wrench_ee_;
    filtered_wrench_base_ = (1 - rc_filter_ft_) * last_wrench_base_ + rc_filter_ft_ * wrench_base_;
    last_wrench_base_ = filtered_wrench_base_;

    // 测试滤波效果
    geometry_msgs::WrenchStamped filtered_wrench_msg;
    filtered_wrench_msg.header.stamp = ros::Time::now();
    filtered_wrench_msg.wrench.force.x = filtered_wrench_base_.force.x();
    filtered_wrench_msg.wrench.force.y = filtered_wrench_base_.force.y();
    filtered_wrench_msg.wrench.force.z = filtered_wrench_base_.force.z();
    filtered_wrench_msg.wrench.torque.x = filtered_wrench_base_.torque.x();
    filtered_wrench_msg.wrench.torque.y = filtered_wrench_base_.torque.y();
    filtered_wrench_msg.wrench.torque.z = filtered_wrench_base_.torque.z();
    pub_filtered_wrench_base_.publish(filtered_wrench_msg);
    bwrench_sub_ = true;
}

void UR5e::subEndVelCmdCB(const geometry_msgs::TwistStamped &twist){
    double time_delay = ros::Time::now().toSec() - twist.header.stamp.toSec();
    // 如果时间延迟超过两个控制周期则不发送控制指令
    double damp_ratio = 0.01;
    double control_rate = 100;
    ros::Rate loop_rate(control_rate);
    if(time_delay < 0.02){
        force_wrench_update();
        force_ur_update();
        twist_cmd_ = KDL::Twist(KDL::Vector(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z), 
                                KDL::Vector(twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z));
    }else
        twist_cmd_ = KDL::Twist::Zero();
}

void UR5e::run(){
    if(end_admittance_control_)
        repair_wrench();
    // move to start
    std::vector<double> start_joints;
    nh_.getParam("start_joints", start_joints);
    if (start_joints.size() != 6)
        throw std::runtime_error("Wrong start_joints size!");

    KDL::JntArray start_jnt(6);
    for (int i = 0; i < 6; ++i)
        start_jnt(i) = start_joints[i] * M_PI / 180;
    servoj_moveto(start_jnt, 5, true);


    // 准备接受命令
    double damp_ratio = 0.001;
    nh_.param("damp_ratio", damp_ratio, 0.03);
    double control_rate = 100;
    ros::Rate loop_rate(control_rate);
    KDL::Jacobian jacobian(6);
    Eigen::VectorXd end_vel(6);
    KDL::JntArray target_jnt(6);
    
    std::vector<double> ft_dead_area = {-1.5, 1.5};
    // 这是 tool0 到 grasp frame 坐标原点的向量
    KDL::Vector tool02grasp_vec(0, 0, -0.14);
    while (ros::ok())
    {
        ros::spinOnce();
        KDL::Twist twist_cmd = twist_cmd_;
        // 转换到tool0 坐标系 对应雅克比矩阵
        twist_cmd.vel += twist_cmd.rot * (current_frame_.M * tool02grasp_vec);

        if(end_admittance_control_){
            KDL::Wrench wrench_base = getEndWrenchToBase();
            // 增加滤波的死区
            filter_ft_dead_area(wrench_base, ft_dead_area, ft_dead_area);
            KDL::Twist force_ext_twist;
            force_ext_twist.vel = damp_ratio * wrench_base.force;
            twist_cmd += force_ext_twist;
        }
        
        getJacobian(jacobian);
        end_vel << twist_cmd.vel.x(), twist_cmd.vel.y(), twist_cmd.vel.z(),
                twist_cmd.rot.x(), twist_cmd.rot.y(), twist_cmd.rot.z();
        Eigen::VectorXd jnt_vel = jacobian.data.inverse() * end_vel;
        
        KDL::JntArray current_jnt = getCurrentJointState();
        for(int i = 0; i < 6; ++i)
            target_jnt(i) = current_jnt(i) + jnt_vel(i) * 1.0 / control_rate; 
        servoj_moveto(target_jnt, 1.0 / control_rate, false);
        loop_rate.sleep();
    }
}

void UR5e::servoj_moveto(KDL::JntArray target, double time, bool wait_for_D) {
    trajectory_msgs::JointTrajectoryPoint p0;
    trajectory_msgs::JointTrajectoryPoint p1;
    control_msgs::FollowJointTrajectoryGoal g;
    g.trajectory.header.stamp = ros::Time::now();
    g.trajectory.joint_names.push_back("shoulder_pan_joint");
    g.trajectory.joint_names.push_back("shoulder_lift_joint");
    g.trajectory.joint_names.push_back("elbow_joint");
    g.trajectory.joint_names.push_back("wrist_1_joint");
    g.trajectory.joint_names.push_back("wrist_2_joint");
    g.trajectory.joint_names.push_back("wrist_3_joint");

    if (!is_jntspeed_large(start_JntArr_, target, time, 1)) {
        for (int x = 0; x < 6; ++x) {
            p0.positions.push_back(current_JntArr_(x));
            p0.velocities.push_back(0);
        }
        p0.time_from_start = ros::Duration(0);

        for (int x = 0; x < 6; ++x) {
            p1.positions.push_back(target(x));
            p1.velocities.push_back(0);
        }
        p1.time_from_start = ros::Duration(time);
        g.trajectory.points.push_back(p1);
        client_servoj_->sendGoal(g);
        if (wait_for_D)
            ros::Duration(time).sleep();
        start_JntArr_ = target;
    }
    else
        ROS_WARN("jntarray speed is large");
}

void UR5e::servoj_moveto(KDL::Frame target, double time, bool wait_for_D) {
    KDL::JntArray target_jntarr(6);
    int ret = p_tracik_solver_->CartToJnt(current_JntArr_, target, target_jntarr);
    if(ret == 1)
        servoj_moveto(target_jntarr, time, wait_for_D);
    else
        ROS_INFO_STREAM(ret);
}

void UR5e::move_line_vel(KDL::Vector end_point, double vel){
    force_ur_update();
    KDL::Vector start_point = current_frame_.p;
    double dis_start_end = (end_point - start_point).Norm();
    KDL::Frame start_frame = current_frame_;
    KDL::Frame next_frame = start_frame;
    ros::Rate loop_rate(m_control_rate_);
    int step_num = static_cast<int>(dis_start_end / vel * m_control_rate_);
    KDL::Vector step_vec = (end_point - start_point) / step_num;
    for(int i = 1; i <= step_num; ++i){
        next_frame.p = start_frame.p + i * step_vec;
        servoj_moveto(next_frame, 1.0/m_control_rate_, false);
        loop_rate.sleep();
    }
}

bool UR5e::is_jntspeed_large(KDL::JntArray start_jnt, KDL::JntArray end_jnt, double duration, double speed_scale){
    b_speed_ = false;
    KDL::JntArray delta_array(joint_size_);
    std::vector<double> sizeArr;
    for (int i = 0; i < joint_size_; i++){
        delta_array(i) = end_jnt(i) - start_jnt(i);
        double dsize = double(std::abs(delta_array(i) * 180 / M_PI));
        sizeArr.push_back(dsize);
    }
    double delta_jnt_max = 0;
    for (int i = 0; i < sizeArr.size(); i++)
        delta_jnt_max = (sizeArr[i] > delta_jnt_max) ? sizeArr[i] : delta_jnt_max;
    double jntspeed = delta_jnt_max / duration;
    if(jntspeed > max_jnt_speed_ * speed_scale)
        b_speed_ = true;
    return b_speed_;
}

void UR5e::force_ur_update() {
    bur_sub_ = false;
    while(ros::ok() && !bur_sub_)
        ros::spinOnce();
}

void UR5e::force_wrench_update() {
    bwrench_sub_ = false;
    while(ros::ok() && !bwrench_sub_)
        ros::spinOnce();
}

void UR5e::repair_wrench() {
    // bool use_last_end_mass_center;
    // nh_.param("use_last_end_mass_center", use_last_end_mass_center, false);
    // if(!use_last_end_mass_center){

    // }
    recordGravity();
}

void UR5e::recordGravity() {
    // ur5e 末端力传感器和tool0 坐标系一致
    ROS_INFO_STREAM("begin recording gravity");
    ROS_INFO_STREAM("waiting for ur and wrench to update");
    ROS_WARN_STREAM("The method is only suit for axisymmetric body");
    // TODO (重力补偿针对任意物体)
    force_ur_update();
    ROS_INFO_STREAM("updating ur and wrench is ok");

    // 初始化重力补偿关节位置变量
    std::vector<double> gravity_up_joints, gravity_down_joints, middle_joints;
    nh_.getParam("gravity_up_joints", gravity_up_joints);
    nh_.getParam("gravity_down_joints", gravity_down_joints);
    nh_.getParam("middle_joints", middle_joints);

    if (gravity_up_joints.size() != 6 || gravity_down_joints.size() != 6 || middle_joints.size() != 6)
        throw std::runtime_error("Wrong gravity_up_joints size for recording gravity!");

    KDL::JntArray gravity_up_jnt(6), gravity_down_jnt(6), middle_jnt(6);
    for (int i = 0; i < 6; ++i) {
        gravity_up_jnt(i) = gravity_up_joints[i] * M_PI / 180;
        gravity_down_jnt(i) = gravity_down_joints[i] * M_PI / 180;
        middle_jnt(i) = middle_joints[i] * M_PI / 180;
    }

    // up
    servoj_moveto(gravity_up_jnt, 5, true);
    ros::Duration(0.8).sleep();
    force_wrench_update();
    KDL::Wrench up_wrench = wrench_ee_;

    //down
    servoj_moveto(gravity_down_jnt, 3, true);
    ros::Duration(0.8).sleep();
    force_wrench_update();
    KDL::Wrench down_wrench = wrench_ee_;

    gravity_ = (up_wrench.force - down_wrench.force) / 2;
    repair_wrench_ = (down_wrench + up_wrench) / 2;
    ROS_INFO_STREAM("gravity: " << gravity_.x() << ", " << gravity_.y() << ", " << gravity_.z());

    // middle
    // 默认末端质量中心在z轴上
    servoj_moveto(middle_jnt, 2, true);
    ros::Duration(0.8).sleep();
    force_wrench_update();
    KDL::Wrench middle_wrench = wrench_ee_;

    mass_center_.data[2] = middle_wrench.torque.x() / gravity_.z();
    ROS_INFO_STREAM("gravity position z =  " << mass_center_.data[2]);

    servoj_moveto(gravity_down_jnt, 2, true);
}

void UR5e::robotiq_hand_move(float position, float vel, float force){
    ROS_INFO_STREAM("hand move, position="<<position<<"  vel="<<vel<<"  force="<<force);
    robotiq_85_msgs::GripperCmd hand_cmd;
    position = position/1000;
    vel = vel/1000;
    force = force/1000;
    if (position < 0 || position>0.086 || vel>0.02 ||vel<-0.02 || force>255 ||force<0){
        ROS_ERROR("The command of the Robotiq is out of range");
        return;
    }
    force_ur_update();
    if(true ){
        hand_cmd.position = position;   // scale 0-0.085mm
        hand_cmd.speed = vel;
        hand_cmd.force = force;
        pub_hand_cmd_.publish(hand_cmd);
    }
}

void UR5e::filter_ft_dead_area(KDL::Wrench &wrench, const std::vector<double> &force_dead, const std::vector<double> &torque_dead){
    // 判断数据有效性
    if(force_dead.size() != 2 || torque_dead.size() != 2)
        throw std::runtime_error("The input size of dead area is invalid!");
    
    for(int i = 0; i < 3; ++i){
        // force dead area
        if(wrench.force.data[i] < force_dead[0]){
            wrench.force.data[i] -= force_dead[0];
        }else if(force_dead[1] < wrench.force.data[i]){
            wrench.force.data[i] -= force_dead[1];
        }else{
            wrench.force.data[i] = 0;
        }
        // torque dead area
        if(wrench.torque.data[i] < torque_dead[0]){
            wrench.torque.data[i] -= torque_dead[0];
        }else if(torque_dead[1] < wrench.torque.data[i]){
            wrench.torque.data[i] -= torque_dead[1];
        }else{
            wrench.torque.data[i] = 0;
        }
    }
}