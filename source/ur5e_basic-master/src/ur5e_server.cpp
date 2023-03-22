#include "ros/ros.h"
#include "UR5e.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5e_server");
    ros::NodeHandle nh;

    double timeout = 0.005;
    double control_rate = 100;
    std::string urdf_param, UR_prefix;
    nh.param("urdf_param", urdf_param, std::string("/robot_description"));
    nh.param("ur_prefix", UR_prefix, std::string("UR5e"));
    UR5e *ur5e = new UR5e(nh, urdf_param, "base_link", "tool0", timeout, UR_prefix, control_rate);

    // 擦黑板
    ur5e->run();
}