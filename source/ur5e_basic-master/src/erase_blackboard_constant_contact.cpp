#include "ros/ros.h"
#include "UR5e.hpp"

// sub wrench variable
// double force_con_ati_g = 0;
double rc_filter_ati = 0.06;
bool b_wrench = false;
bool b_end_vel = false;
KDL::Wrench wrench_ati_g = KDL::Wrench::Zero();

// sub ur5e end velocity varialble
KDL::Twist twist_cmd_g = KDL::Twist::Zero();

void subATI(geometry_msgs::WrenchStamped wrench_msg){
    // force_con_ati_g = (1 - rc_filter_ati) * force_con_ati_g + rc_filter_ati * fabs(wrench_msg.wrench.force.z);
    wrench_ati_g.force.data[0] = (1 - rc_filter_ati) * wrench_ati_g.force.x() + rc_filter_ati * wrench_msg.wrench.force.x;
    wrench_ati_g.force.data[1] = (1 - rc_filter_ati) * wrench_ati_g.force.y() + rc_filter_ati * wrench_msg.wrench.force.y;
    wrench_ati_g.force.data[2] = (1 - rc_filter_ati) * wrench_ati_g.force.z() + rc_filter_ati * wrench_msg.wrench.force.z;
    wrench_ati_g.torque.data[0] = (1 - rc_filter_ati) * wrench_ati_g.torque.x() + rc_filter_ati * wrench_msg.wrench.torque.x;
    wrench_ati_g.torque.data[1] = (1 - rc_filter_ati) * wrench_ati_g.torque.y() + rc_filter_ati * wrench_msg.wrench.torque.y;
    wrench_ati_g.torque.data[2] = (1 - rc_filter_ati) * wrench_ati_g.torque.z() + rc_filter_ati * wrench_msg.wrench.torque.z;
    b_wrench = true;
}

void subEndVelCmdCB(const geometry_msgs::TwistStamped &twist){
    double time_delay = ros::Time::now().toSec() - twist.header.stamp.toSec();
    // 如果时间延迟超过两个控制周期则不发送控制指令
    if(time_delay < 0.02){
        twist_cmd_g = KDL::Twist(KDL::Vector(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z), 
                                KDL::Vector(twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z));
    }else
        twist_cmd_g = KDL::Twist::Zero();
    b_end_vel = true;
}

void pub_contact_msg(ros::Publisher &pub, KDL::Wrench &wrench){
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.wrench.force.x = wrench.force.x();
    msg.wrench.force.y = wrench.force.y();
    msg.wrench.force.z = wrench.force.z();
    msg.wrench.torque.x = wrench.torque.x();
    msg.wrench.torque.y = wrench.torque.y();
    msg.wrench.torque.z = wrench.torque.z();
    pub.publish(msg);
}

// 使用时一定要注意传感器数据清零
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5e_server");
    ros::NodeHandle nh;

    double timeout = 0.005;
    double control_rate = 200;
    std::string urdf_param, UR_prefix;
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));
    nh.param("ur_prefix", UR_prefix, std::string("UR5e"));
    UR5e *ur5e = new UR5e(nh, urdf_param, "base_link", "tool0", timeout, UR_prefix, control_rate);

    ros::Subscriber sub_ati = nh.subscribe("netft_data", 1, subATI);
    ros::Subscriber sub_end_vel_cmd = nh.subscribe("ur5e_end_vel_cmd", 1, subEndVelCmdCB);
    ros::Publisher pub_contact_force = nh.advertise<geometry_msgs::WrenchStamped>("contact_force", 1);
    ROS_INFO_STREAM("Waiting for wrench...");
    while(ros::ok() && !b_wrench){
        ros::spinOnce();
    }
    ROS_INFO_STREAM("ATI wrench is ok!");

    // TODO 检查一下robotiq不动的原因
    // ur5e->robotiq_hand_move(0.0, 5, 20);
    // ros::spin();


    // 擦黑板
    // move to start
    ROS_INFO_STREAM("Move to start position");
    std::vector<double> start_joints;
    nh.getParam("erase_blackboard_start_joints_board_up", start_joints);
    nh.param("rc_filter_ati", rc_filter_ati, 0.06);
    if (start_joints.size() != 6)
        throw std::runtime_error("Wrong start_joints size!");

    KDL::JntArray start_jnt(6);
    for (int i = 0; i < 6; ++i)
        start_jnt(i) = start_joints[i] * M_PI / 180;
    ur5e->servoj_moveto(start_jnt, 2, true);
    ros::Duration(0.5).sleep();

    bool pidOrAdmittance = true;
    nh.getParam("pidOrAdmittance", pidOrAdmittance);
    if(pidOrAdmittance){
        // PI 恒力控制 目前只是P控制
        ROS_INFO_STREAM("Ready for constant contact");
        double kp, ki;
        nh.param("kp", kp, 0.0);
        nh.param("ki", ki, 0.0);
        double integration = 0;
        // 在10N附近0.05领域保持1S 认为完成恒力接触
        double force_con_threhold = 0.05;
        double force_d = 10;
        double time_con = 1;
        std::vector<double> dead_area_ati = {-0.5, 0.5};
        KDL::Wrench wrench_d(KDL::Vector(0, 0, force_d), KDL::Vector(0, 0, 0));
        bool finish_con = false;

        KDL::Jacobian jacobian(6);
        Eigen::VectorXd end_vel(6);
        KDL::JntArray target_jnt(6);
        ros::Rate loop_rate(control_rate);
        uint64_t nsec0 = ros::Time::now().toNSec();
        KDL::Wrench wrench_ati = wrench_ati_g;
        while(ros::ok()){
            ros::spinOnce();
            if(30 < wrench_ati_g.force.Norm() || 30 < wrench_ati_g.torque.Norm())
                throw std::runtime_error("Stop! contact force is too big! ");
            // double force_con = ur5e->getEndWrenchToBase().force.z();  // 接触力
            wrench_ati = wrench_d + wrench_ati_g;
            // ur5e->filter_ft_dead_area(wrench_ati, dead_area_ati, dead_area_ati);
            
            double force_error = wrench_ati.force.z();
            integration += force_error * 1 / control_rate;
            double vel_z = kp * force_error + ki * integration;
            // saturate 需要验证
            if(0.4 < fabs(vel_z)){
                ROS_INFO_STREAM("PID gain is too big! vel = " << vel_z);
                vel_z = vel_z / fabs(vel_z) * 0.4;
            }
            KDL::Twist twist_cmd = KDL::Twist::Zero();
            twist_cmd.vel.data[2] = -vel_z;
            twist_cmd += twist_cmd_g;

            end_vel << twist_cmd.vel.x(), twist_cmd.vel.y(), twist_cmd.vel.z(),
                    twist_cmd.rot.x(), twist_cmd.rot.y(), twist_cmd.rot.z();
            ur5e->getJacobian(jacobian);
            Eigen::VectorXd jnt_vel = jacobian.data.inverse() * end_vel;
            
            KDL::JntArray current_jnt = ur5e->getCurrentJointState();
            for(int i = 0; i < 6; ++i)
                target_jnt(i) = current_jnt(i) + jnt_vel(i) * 1.0 / control_rate; 
            ur5e->servoj_moveto(target_jnt, 1.0 / control_rate, false);
            // 判断是否完成恒力接触
            double time_duration = (ros::Time::now().toNSec() - nsec0) / 1e9;
            // if(fabs(force_error) < force_con_threhold){
            //     if(time_con < time_duration){
            //         ROS_INFO_STREAM("Finished constant force contact");
            //         break;
            //     }
            // }else{
            //     nsec0 = ros::Time::now().toNSec();
            // }
            pub_contact_msg(pub_contact_force, wrench_ati_g);
            loop_rate.sleep();
        }
    }else{
        // 变刚度控制
        ROS_INFO_STREAM("Variable admittance control");
        double damp_ratio_x = 0.03;
        double damp_ratio_y = 0.03;
        double damp_ratio_z = 0.03;
        nh.param("damp_ratio", damp_ratio_y, 0.03);
        nh.param("damp_ratio", damp_ratio_z, 0.03);
        double control_rate = 100;
        ros::Rate loop_rate(control_rate);
        KDL::Jacobian jacobian(6);
        Eigen::VectorXd end_vel(6);
        KDL::JntArray target_jnt(6);
        
        std::vector<double> ft_dead_area = {-1, 1};
        // 这是 tool0 到 grasp frame 坐标原点的向量
        KDL::Vector tool02grasp_vec(0, 0, -0.14);
        KDL::Frame T_ati_tool0 = KDL::Frame::Identity();
        T_ati_tool0.M = KDL::Rotation::RPY(0, 0, M_PI_2);
        while (ros::ok())
        {
            ros::spinOnce();
            // 获取yz方向的刚度
            KDL::Frame T_tool0_base = ur5e->getCurrentFrame();
            KDL::Twist twist_cmd = twist_cmd_g;
            twist_cmd.vel += twist_cmd.rot * (T_tool0_base.M * tool02grasp_vec);

            KDL::Wrench wrench_base = wrench_ati_g;
            // 增加滤波的死区 保证不推机械臂时静止
            ur5e->filter_ft_dead_area(wrench_base, ft_dead_area, ft_dead_area);
            // 转换到tool0 坐标系 对应雅克比矩阵
            KDL::Vector force_ati_base = T_tool0_base.M * T_ati_tool0.M * wrench_base.force;  // base_link 下的受力

            
            KDL::Twist force_ext_twist;
            force_ext_twist.vel.data[0] = damp_ratio_x * force_ati_base.x();
            force_ext_twist.vel.data[1] = damp_ratio_y * force_ati_base.y();
            force_ext_twist.vel.data[2] = damp_ratio_z * force_ati_base.z();
            twist_cmd += force_ext_twist;
            
            ur5e->getJacobian(jacobian);
            end_vel << twist_cmd.vel.x(), twist_cmd.vel.y(), twist_cmd.vel.z(),
                    twist_cmd.rot.x(), twist_cmd.rot.y(), twist_cmd.rot.z();
            Eigen::VectorXd jnt_vel = jacobian.data.inverse() * end_vel;
            
            KDL::JntArray current_jnt = ur5e->getCurrentJointState();
            for(int i = 0; i < 6; ++i)
                target_jnt(i) = current_jnt(i) + jnt_vel(i) * 1.0 / control_rate; 
            ur5e->servoj_moveto(target_jnt, 1.0 / control_rate, false);
            loop_rate.sleep();
        }
    }
}