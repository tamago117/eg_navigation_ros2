/**
* @file wp_load.cpp
* @brief csv to path node (ros2)
* @author Michikuni Eguchi
* @date 2022.1.1
* @details wpをcsvから読み込んでpathとmarkerarrayで配信する
*/


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <chrono> //時間計測系
#include <iostream>
#include <memory> //smart pointer

#include "eg_wptool/csv_input.hpp"

class wpLoad : public rclcpp::Node
{
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Clock ros_clock;

    std::string filePath, map_id;
    int rate;

public:
    explicit wpLoad(const std::string& node_name)
     : Node(node_name),
       ros_clock(RCL_ROS_TIME),
       map_id("map"),
       rate(2)
    {
        //declare parameter
        declare_parameter<std::string>("filePath", filePath);
        declare_parameter<std::string>("map_frame_id", map_id);
        declare_parameter<int>("loop_rate", rate);

        //get parameter
        get_parameter("filePath", filePath);
        get_parameter("map_frame_id", map_id);
        get_parameter("loop_rate", rate);

        auto publish_path = [this]() -> void
        {
            static nav_msgs::msg::Path path;
            //csv読み込み
            csv::csv_input csv(filePath);

            int lineNum = csv.lineNum();
            for(int i = 0; i < lineNum; ++i)
            {
                static geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = map_id;
                pose.header.stamp = ros_clock.now();
                pose.pose.position.x = csv.readCSV(i, 0);
                pose.pose.position.y = csv.readCSV(i, 1);
                pose.pose.position.z = csv.readCSV(i, 2);
                pose.pose.orientation.x = csv.readCSV(i, 3);
                pose.pose.orientation.y = csv.readCSV(i, 4);
                pose.pose.orientation.z = csv.readCSV(i, 5);
                pose.pose.orientation.w = csv.readCSV(i, 6);

                path.poses.push_back(pose);
            }
            path.header.frame_id = map_id;
            path.header.stamp = ros_clock.now();


            path_pub->publish(path);
        };

        path_pub = create_publisher<nav_msgs::msg::Path>("wayPoint/path",10);

        timer = create_wall_timer(std::chrono::microseconds(1000/rate), publish_path);
    }

    ~wpLoad(){};

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wpLoad>("wpLoad"));
    rclcpp::shutdown();

    return 0;
}