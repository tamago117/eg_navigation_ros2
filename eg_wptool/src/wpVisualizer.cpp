/**
* @file wpVisualizer.cpp
* @brief way point visualizer node for Rviz (ros2)
* @author Michikuni Eguchi
* @date 2022.1.1
* @details pathを読んでwaypointの番号をmarkerarrayで配信する可視化用ノード This node visualize waypoint number reading path.
*          And change marker color in accordance with now waypoint number
*/


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <iostream>
#include <chrono> //時間計測系
#include <memory>
#include <string>

namespace wp{

static const double marker_diameter = 0.1;
static const double marker_height = 0.03;
static const double text_size = 0.1;

class wpVisualizer : public rclcpp::Node
{
private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerText_pub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr targetWp_sub;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Clock ros_clock;
    //color
    std_msgs::msg::ColorRGBA green;
    std_msgs::msg::ColorRGBA red;
    std_msgs::msg::ColorRGBA gray;
    std_msgs::msg::ColorRGBA black;

    int targetWp;
    nav_msgs::msg::Path path;
    double markerSize;
    int rate;

    //function
    void publish_marker();
    void targetWp_callback(const std_msgs::msg::Int32::UniquePtr targetWp_num);
    void path_callback(const nav_msgs::msg::Path::UniquePtr path_message);

public:
    explicit wpVisualizer(const std::string& node_name);

    ~wpVisualizer(){};

    std_msgs::msg::ColorRGBA set_color(double r, double g, double b, double a);

};

wpVisualizer::wpVisualizer(const std::string& node_name)
: Node(node_name),
  green(set_color(0.0, 1.0, 0.0, 1.0)),
  red(set_color(1.0, 0.0, 0.0, 1.0)),
  gray(set_color(0.3, 0.3, 0.3, 1.0)),
  black(set_color(0.0, 0.0, 0.0, 1.0)),
  targetWp(0),
  markerSize(1),
  rate(10)
{
    //declare parameter
    declare_parameter<int>("loop_rate", rate);
    declare_parameter<double>("markerSize", markerSize);

    get_parameter("loop_rate", rate);
    get_parameter("markerSize", markerSize);

    //set publisher
    marker_pub = create_publisher<visualization_msgs::msg::MarkerArray>("wayPoint/marker", 10);
    markerText_pub = create_publisher<visualization_msgs::msg::MarkerArray>("wayPoint/markerText", 10);
    //set subscriber
    targetWp_sub = create_subscription<std_msgs::msg::Int32>(
        "wayPoint/target", 10, std::bind(&wpVisualizer::targetWp_callback, this, std::placeholders::_1));
    path_sub = create_subscription<nav_msgs::msg::Path>(
        "wayPoint/path", 10, std::bind(&wpVisualizer::path_callback, this, std::placeholders::_1));



    timer = create_wall_timer(std::chrono::milliseconds(1000/rate), std::bind(&wpVisualizer::publish_marker, this));
}

void wpVisualizer::publish_marker()
{
    static const double marker_diameter = 0.1 * markerSize;
    static const double marker_height = 0.03 * markerSize;
    static const double text_size = 0.1 * markerSize;

    //marker
    visualization_msgs::msg::MarkerArray marker_array, markerText_array;
    int path_size = path.poses.size();
    for(int i=0; i < path_size; ++i){
        visualization_msgs::msg::Marker marker, markerText;
        marker.header.frame_id = markerText.header.frame_id = path.header.frame_id;
        marker.header.stamp = markerText.header.stamp = ros_clock.now();
        marker.ns = markerText.ns = "waypoint_marker";
        marker.id = markerText.id = i;
        //marker.id = 0;
        //markerText.id = 1;
        marker.lifetime = markerText.lifetime = rclcpp::Duration(1);
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        markerText.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = markerText.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = marker_diameter;
        marker.scale.y = marker_diameter;
        marker.scale.z = marker_height;
        markerText.scale.x = text_size;
        markerText.scale.y = text_size;
        markerText.scale.z =text_size;
        marker.pose = markerText.pose = path.poses.at(i).pose;
        markerText.pose.position.z += marker_height;
        markerText.text= std::to_string(i).c_str();
        //color select
        if(targetWp > i){
            marker.color = gray;
        }
        if(targetWp == i){
            marker.color = red;
        }
        if(targetWp < i){
            marker.color = green;
        }
        markerText.color = black;

        marker_array.markers.push_back(marker);
        markerText_array.markers.push_back(markerText);
    }

    marker_pub->publish(marker_array);
    markerText_pub->publish(markerText_array);
}


std_msgs::msg::ColorRGBA wpVisualizer::set_color(double r, double g, double b, double a)
{
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;

    return color;
}


void wpVisualizer::targetWp_callback(const std_msgs::msg::Int32::UniquePtr targetWp_num)
{
    targetWp = targetWp_num->data;
}


void wpVisualizer::path_callback(const nav_msgs::msg::Path::UniquePtr path_message)
{
    path = *path_message;
}

}//namespace wp

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wp::wpVisualizer>("wpVisualizer"));
    rclcpp::shutdown();

    return 0;
}