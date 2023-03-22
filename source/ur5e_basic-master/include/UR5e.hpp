#ifndef UR5e_BASIC_UR5E_HPP
#define UR5e_BASIC_UR5E_HPP

#include <iostream>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen_conversions/eigen_kdl.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <trac_ik/trac_ik.hpp>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "robotiq_85_msgs/GripperCmd.h"
#include "robotiq_85_msgs/GripperStat.h"

#include <boost/filesystem.hpp>

class UR5e {
  typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> Client;
public:
    UR5e(ros::NodeHandle nh, const std::string &urdf_param,
            const std::string & _chain_start, const std::string & _chain_end,
            double _timeout, std::string &prefix, double control_rate = 100);
    ~UR5e() {
        delete client_servoj_;
        delete p_fk_solver_;
        delete p_tracik_solver_;
    }

    /**
    * \brief Get current joint state datas
    *
    * \return current joint state data array
    *  - Count and order match joint names defined in robot data
    */
    KDL::JntArray getCurrentJointState(){ros::spinOnce(); return current_JntArr_;}

    /**
    * \brief Get current position of the robot end link defined in robot data based on robot start link coordinate system( base_link most of the time)
    *
    * \return current position of the robot end link
    */
    KDL::Frame getCurrentFrame(){ros::spinOnce(); return current_frame_;}

    /**
    * \brief Get current current of the robot joint
    *
    * \return current current of the robot joint
    */
    KDL::JntArray getCurrentCurrent(){ros::spinOnce(); return current_JntCur_;}

    /**
    * \brief Get wrench realtive to base
    *
    * \return current position of the robot end link
    */
    KDL::Wrench getEndWrenchToBase(bool filter = true){ return filter ? filtered_wrench_base_ : wrench_base_; }

    void getJacobian(KDL::Jacobian &jacobian){ p_jacobian_solver_->JntToJac(current_JntArr_, jacobian);}

    /**
    * \brief Get path for source code, usually for locate path for recording data
    *
    * \return source code path. For example $HOME/project/ur_ws
    */
    std::string getSourceCodePath(){ return source_dir_; }

    /**
    * 移动到目标位置
    */
    void servoj_moveto(KDL::JntArray target, double time, bool wait_for_D = true);

    void servoj_moveto(KDL::Frame target, double time, bool wait_for_D = true);

    void move_line_vel(KDL::Vector target, double vel);

    /**
    * \brief Force UR5e of force/torque sensor update value
    */
    void force_wrench_update();
    void force_ur_update();


    bool is_jntspeed_large(KDL::JntArray start_jnt, KDL::JntArray end_jnt, double duration, double speed_scale = 0.5);

    void repair_wrench();
    void recordGravity();
    
    /**
    * \brief Receive the velocity of end effector to control UR5e
    */
    void run();

    /**
    * \brief control robotiq with position, velocity and force
    */
    void robotiq_hand_move(float position, float vel = 5, float force = 20);
    
    /**
    * \brief filter data with dead area, the return data is in wrench
    */
    void filter_ft_dead_area(KDL::Wrench &wrench, const std::vector<double> &force_dead = {-0.5, 0.5}, const std::vector<double> &torque_dead = {-0.5, 0.5});

private:
    void subJointStateCB(const sensor_msgs::JointState &state);
    void subWrenchCB(const geometry_msgs::WrenchStamped &wrench);
    void subEndVelCmdCB(const geometry_msgs::TwistStamped &twist);
    
    ros::NodeHandle nh_;

    int joint_size_;

    std::vector<std::string> joint_names_; //定义一个大小可变的数组保存字符串

    KDL::JntArray start_JntArr_;
    KDL::Frame start_frame_;

    KDL::JntArray current_JntArr_;
    KDL::JntArray current_JntCur_;  // 关节电流
    KDL::Frame current_frame_;  //fk result

    KDL::Frame target_frame_;
    KDL::JntArray target_JntArr_;

    double m_control_rate_ = 100;
    double max_jnt_speed_ = 180;    // 最大关节角速度:单位度
    bool b_speed_;

    // 作为更新关节角度和力传感器信息的flag
    bool bur_sub_;
    bool bwrench_sub_;
    
    KDL::Wrench repair_wrench_;
    KDL::Wrench wrench_base_;
    KDL::Wrench wrench_ee_;
    KDL::Wrench last_wrench_base_;
    KDL::Wrench filtered_wrench_base_;
    double rc_filter_ft_ = 0.06;  // 0.1

    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_wrench_;
    ros::Subscriber sub_end_vel_cmd_;
    ros::Publisher pub_filtered_wrench_base_;
    ros::Publisher pub_hand_cmd_;  // 控制robotiq
    Client *client_servoj_;

    KDL::Chain chain_;
    TRAC_IK::TRAC_IK *p_tracik_solver_;
    KDL::ChainFkSolverPos_recursive *p_fk_solver_;  // KDL正运动学求解器
    KDL::ChainJntToJacSolver *p_jacobian_solver_;   // 求解雅克比矩阵

    // record data
    std::string source_dir_;

    // 重力补偿
    KDL::Vector gravity_ = KDL::Vector(0, 0, 0);
    KDL::Vector mass_center_ = KDL::Vector(0, 0, 0);
    bool end_admittance_control_ = false;

    // 末端速度指令
    KDL::Twist twist_cmd_;
};

#endif // UR5e_BASIC_UR5E_HPP