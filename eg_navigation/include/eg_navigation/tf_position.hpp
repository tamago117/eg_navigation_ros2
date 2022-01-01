/**
* @file tf_position.h
* @brief  read frame position from tf(ros2 version)
* @author Michikuni Eguchi
* @date 2021.12.31
* @details Read position from transformations in tf
*/

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono> //時間計測系
#include <string>
#include <iostream>

class tf_position : public rclcpp::Node
{
public:
    tf_position(std::string base_id, std::string child_id, double rate);
    geometry_msgs::msg::PoseStamped getPoseStamped();
    geometry_msgs::msg::Pose getPose();
    double getRoll();
    double getPitch();
    double getYaw();
    double norm();

private:
    rclcpp::TimerBase::SharedPtr timer;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    std::string base_id, child_id;
    geometry_msgs::msg::TransformStamped tfStamp;
    geometry_msgs::msg::PoseStamped poseStamp;
    void quat2rpy(double& roll, double& pitch, double& yaw);

};

tf_position::tf_position(std::string base_id_, std::string child_id_, double rate) : Node("tf_position"), base_id(base_id_), child_id(child_id_), tfBuffer(this->get_clock()), tfListener(tfBuffer)
{
    //lambda式で関数定義
    auto read_tf = [&]() -> void
    {
        try
        {
            tfStamp = tfBuffer.lookupTransform(base_id, child_id, tf2::TimePointZero);
            poseStamp.header = tfStamp.header;
            poseStamp.pose.position.x = tfStamp.transform.translation.x;
            poseStamp.pose.position.y = tfStamp.transform.translation.y;
            poseStamp.pose.position.z = tfStamp.transform.translation.z;
            poseStamp.pose.orientation = tfStamp.transform.rotation;
        }
        catch(tf2::TransformException& ex)
        {
            std::cout<<ex.what()<<std::endl;
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }
    };

    timer = this->create_wall_timer(std::chrono::milliseconds((int)(1.0/rate*1000)), read_tf);
    

}

geometry_msgs::msg::PoseStamped tf_position::getPoseStamped()
{
    return poseStamp;
}

geometry_msgs::msg::Pose tf_position::getPose()
{
    return poseStamp.pose;
}

double tf_position::getRoll(){
    double roll,pitch,yaw;
    quat2rpy(roll, pitch, yaw);
    return roll;
}

double tf_position::getYaw(){
    double roll,pitch,yaw;
    quat2rpy(roll, pitch, yaw);
    return yaw;
}

double tf_position::getPitch(){
    double roll,pitch,yaw;
    quat2rpy(roll, pitch, yaw);
    return pitch;
}

void tf_position::quat2rpy(double& roll, double& pitch, double& yaw)
{
    //tf::Quaternion quat;
    tf2::Quaternion quat(
        poseStamp.pose.orientation.x,
        poseStamp.pose.orientation.y,
        poseStamp.pose.orientation.z,
        poseStamp.pose.orientation.w);
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

double tf_position::norm()
{
    double x = poseStamp.pose.position.x;
    double y = poseStamp.pose.position.y;
    double z = poseStamp.pose.position.z;
    return sqrt(x*x + y*y + z*z);
}