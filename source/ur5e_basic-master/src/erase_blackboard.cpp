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
    KDL::Vector contact_start(0.2703, 0.5568, 0.2863);
    KDL::Vector contact_end(0.2708, 0.3151, 0.2439);
    std::vector<std::vector<double> > data_record;

    // 擦黑板
    // move to start
    ROS_INFO_STREAM("Move to start position");
    ros::Rate loop_rate(control_rate);
    std::vector<double> start_joints;
    nh.getParam("erase_blackboard_start_joints_board_up", start_joints);
    nh.param("rc_filter_ati", rc_filter_ati, 0.06);
    if (start_joints.size() != 6)
        throw std::runtime_error("Wrong start_joints size!");

    KDL::JntArray start_jnt(6);
    for (int i = 0; i < 6; ++i)
        start_jnt(i) = start_joints[i] * M_PI / 180;
    uint64_t nsec0 = ros::Time::now().toNSec();  // 起始时间
    ur5e->servoj_moveto(start_jnt, 4, false);
    // 记录数据
    int num = 4 * control_rate;
    for(int i = 0; i < num; ++i){
        std::vector<double> data_tmp;
        uint64_t time_now = ros::Time::now().toNSec() - nsec0;
        double time_now_d = time_now / 1e9;
        data_tmp.push_back(time_now_d);
        KDL::JntArray jnt_cur = ur5e->getCurrentCurrent();
        for(int i = 0; i < 6; ++i)
            data_tmp.push_back(jnt_cur(i));
        data_tmp.push_back(wrench_ati_g.force.z());
        data_record.push_back(data_tmp);
        loop_rate.sleep();
    }
    // ros::Duration(4).sleep();

    // PI 恒力控制 目前只是P控制
    ROS_INFO_STREAM("Ready for constant contact");
    double kp, ki, kd;
    nh.param("kp", kp, 0.0);
    nh.param("ki", ki, 0.0);
    nh.param("kd", kd, 0.0);
    double integration = 0;
    // 在10N附近0.05领域保持1S 认为完成恒力接触
    double force_con_threhold = 0.6;
    double force_d = 10;
    double time_con = 1;
    std::vector<double> dead_area_ati = {-0.5, 0.5};
    KDL::Wrench wrench_d(KDL::Vector(0, 0, force_d), KDL::Vector(0, 0, 0));
    bool finish_con = false;



    KDL::Wrench wrench_ati = wrench_ati_g;
    KDL::Vector target_point(0, 0, 0);
    KDL::Frame target_frame = ur5e->getCurrentFrame();
    KDL::Frame start_frame = ur5e->getCurrentFrame();

    // y 方向运动距离
    double step_count = 1;
    int step_num = static_cast<int>(fabs(contact_end.y() - contact_start.y()) / 0.02 * control_rate);
    double step_dis = (contact_end.y() - contact_start.y()) / step_num;
    uint64_t nsec0_con_start = ros::Time::now().toNSec();  // 起始时间
    double last_error = 0;
    while(ros::ok()){
        ros::spinOnce();
        if(30 < wrench_ati_g.force.Norm() || 30 < wrench_ati_g.torque.Norm())
            throw std::runtime_error("Stop! contact force is too big! ");
        wrench_ati = wrench_d + wrench_ati_g;
        
        double del_z = 0 ;
        double force_error = wrench_ati.force.z();
        // saturate 需要验证  0.4 * 1 / 200 = 0.002 2mm
        if(1 < fabs(wrench_ati_g.force.z())){ // 开始加入恒力控制
            integration += force_error * 1 / control_rate;
            del_z = kp * force_error + ki * integration + kd * (force_error - last_error) * control_rate;
            last_error = force_error;
            if(0.002 < fabs(del_z)){  // 增益太大
                ROS_INFO_STREAM("PID gain is too big! vel = " << del_z);
                del_z = del_z / fabs(del_z) * 0.002;
            }
        }else{
            del_z = 0.0005;  // 直线下降
        }

        target_frame = ur5e->getCurrentFrame();
        target_frame.p.data[2] -= del_z;

        // 判断是否完成恒力接触
        double time_duration = (ros::Time::now().toNSec() - nsec0_con_start) / 1e9;
        if(finish_con){
            target_frame.p.data[1] = start_frame.p.y() + step_count * step_dis;
            ++step_count;
            if(step_num < step_count){
                ROS_INFO_STREAM("finished erase blackboard");
                break;
            }
        }else{
            if(fabs(force_error) < force_con_threhold){
                if(time_con < time_duration){
                    ROS_INFO_STREAM("Finished constant force contact");
                    // finish_con = true;
                }
            }else{
                nsec0_con_start = ros::Time::now().toNSec();
            }
        }

        ur5e->servoj_moveto(target_frame, 1.0 / control_rate, false);
        pub_contact_msg(pub_contact_force, wrench_ati_g);

        // record
        std::vector<double> data_tmp;
        uint64_t time_now = ros::Time::now().toNSec() - nsec0;
        double time_now_d = time_now / 1e9;
        data_tmp.push_back(time_now_d);
        KDL::JntArray jnt_cur = ur5e->getCurrentCurrent();
        for(int i = 0; i < 6; ++i)
            data_tmp.push_back(jnt_cur(i));
        data_tmp.push_back(wrench_ati_g.force.z());
        data_record.push_back(data_tmp);
        loop_rate.sleep();
    }

    // record data
    ROS_INFO_STREAM("Recording data");
    std::ofstream data_file;
    std::string filename =  "erase_blackboard_pid.txt";
    std::string file = ur5e->getSourceCodePath() + "/data/" + filename;
    data_file.open(file.c_str());
    for(unsigned int i = 0; i < data_record.size(); ++i){
        for(unsigned int j = 0; j < data_record[i].size(); ++j)
            data_file << data_record[i][j] << " ";
        data_file << std::endl;
    }
    data_file.close();
    ROS_INFO_STREAM("Recording data is finished");
}