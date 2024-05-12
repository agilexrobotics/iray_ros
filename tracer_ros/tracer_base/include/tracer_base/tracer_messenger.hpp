/* 
 * Tracer_messenger.hpp
 * 
 * Created on: Jun 14, 2019 10:24
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef TRACER_MESSENGER_HPP
#define TRACER_MESSENGER_HPP

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

#include "tracer_msgs/TracerLightCmd.h"
#include "tracer_msgs/ClearErr.h"
#include "tracer_msgs/ResetOdom.h"
#include "tracer_msgs/BatteryState.h"

#include "ugv_sdk/mobile_robot/tracer_robot.hpp"
namespace westonrobot
{

class TracerROSMessenger: protected TracerRobot
{
public:
    explicit TracerROSMessenger(ros::NodeHandle *nh);
    TracerROSMessenger(TracerRobot *Tracer, ros::NodeHandle *nh);

    std::string odom_frame_;
    std::string base_frame_;
    std::string odom_topic_name_;
    int update_rate_;
    bool publish_odom_tf_;
    bool use_encoder_;

    bool simulated_robot_ = false;
    int sim_control_rate_ = 50;

    void SetupSubscription();

    void PublishStateToROS();
    void PublishUartStateToROS();
    void PublishSimStateToROS(double linear, double angular);

    void GetCurrentMotionCmdForSim(double &linear, double &angular);
    void DetachRobot();

private:
    TracerRobot *tracer_;
    ros::NodeHandle *nh_;

    std::mutex twist_mutex_;
    geometry_msgs::Twist current_twist_;

    ros::Publisher odom_publisher_;
    ros::Publisher status_publisher_;
    ros::Publisher status_uart_publisher_;
    ros::Publisher battery_state_pub_;
    ros::Subscriber clear_err_subscriber_;
    ros::Subscriber reset_odom_subscriber_;
    ros::Subscriber motion_cmd_subscriber_;
    ros::Subscriber light_cmd_subscriber_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // speed variables
    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double last_linear_speed = 0.0;
    double last_angular_speed = 0.0;
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    //wheel encoder odomentry, mm
    float last_left_wheel_odom = 0;
    float last_right_wheel_odom = 0;
    float left_wheel_odom;
    float right_wheel_odom;
    
    ros::Time last_time_;
    ros::Time current_time_;

    void TwistCmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
    void LightCmdCallback(const tracer_msgs::TracerLightCmd::ConstPtr &msg);
    void ClearCallback(const tracer_msgs::ClearErr::ConstPtr &msg);
    void ResetOdomCallback(const tracer_msgs::ResetOdom::ConstPtr &msg);
    void PublishOdometryToROS(double linear, double angular, double dt);
};
} // namespace wescore

#endif /* TRACER_MESSENGER_HPP */
