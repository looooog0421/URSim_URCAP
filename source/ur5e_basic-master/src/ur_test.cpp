#include "ros/ros.h"
#include "UR5e.hpp"


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

    // 运动到初始位置
    std::vector<double> gravity_down_joints;
    nh.getParam("gravity_down_joints", gravity_down_joints);  //[180, -90, -90, -90, 90, 90]
    KDL::JntArray gravity_down_jnt(6);
    for(unsigned int i = 0; i < 6; ++i)
        gravity_down_jnt(i) = gravity_down_joints[i] * M_PI / 180;
    ROS_INFO_STREAM("Move start joint position");
    ur5e->servoj_moveto(gravity_down_jnt, 5, true);
    ros::Duration(0.5).sleep();

    // 直线运动测试
    ROS_INFO_STREAM("Move line test");
    KDL::Frame start_frame = ur5e->getCurrentFrame();
    KDL::Frame line_frame = start_frame;
    line_frame.p.data[1] += 0.05;
    ur5e->move_line_vel(line_frame.p, 0.05);
    ros::Duration(0.5).sleep();

    // sin 曲线测试跟踪性能
    ROS_INFO_STREAM("Ready to sin test");
    ur5e->force_ur_update();
    KDL::Frame original_P = ur5e->getCurrentFrame();
    KDL::Frame target_frame = original_P;
    KDL::JntArray target_jnt(6);

    ros::Rate loop_rate(control_rate);
    uint64_t nsec0 = ros::Time::now().toNSec();
    std::vector<std::vector<double> > data_record;
    while (ros::ok()) {
        ros::spinOnce();
        // ROS_INFO_STREAM("z: " << ur5e->getCurrentFrame().p.z());
        uint64_t time_now = ros::Time::now().toNSec() - nsec0;
        double time_now_d = time_now / 1e9;

        target_frame.p.data[2] = original_P.p.data[2] + 0.05 * sin(2 * M_PI / 5 * time_now_d);
        ur5e->servoj_moveto(target_frame, 1 / control_rate, false);

        // data
        std::vector<double> data_tmp;
        data_tmp.push_back(time_now_d);
        for(unsigned int i = 0; i < 3; ++i)
            data_tmp.push_back(target_frame.p.data[i]);
        for(unsigned int i = 0; i < 3; ++i)
            data_tmp.push_back(ur5e->getCurrentFrame().p.data[i]);
        data_record.push_back(data_tmp);

        if (20 < time_now_d)
            break;
        loop_rate.sleep();
    }
    // record data
    ROS_INFO_STREAM("Recording data");
    std::ofstream data_file;
    std::string filename =  "sin_test.txt";
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