#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class Robot
{
private:
  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::TwistStamped vel_cmd, ang_vel_cmd;
  rclcpp::Time last_execute_time;
  geometry_msgs::msg::PointStamped home, destination, no_fly_zone;
  std_msgs::msg::Float32 no_fly_zone_radius;
  visualization_msgs::msg::Marker no_fly_zone_marker, no_fly_zone_label, home_label, home_sphere, destination_label, destination_sphere;
  visualization_msgs::msg::MarkerArray marker_array;
  
  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_sub;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr ang_vel_cmd_sub;

  // publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_odom_pub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr home_pub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr destination_pub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr no_fly_zone_pub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr no_fly_zone_radius_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr no_fly_zone_vis_pub;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Node* node;
  
public:
  Robot(rclcpp::Node* node)
    : node(node){
    // initialization
    home.header.frame_id = "world";
    home.point.x = 0;
    home.point.y = 0;
    home.point.z = 0;

    destination.header.frame_id = "world";
    destination.point.x = 10;
    destination.point.y = 0;
    destination.point.z = 0;

    no_fly_zone.header.frame_id = "world";
    no_fly_zone.point.x = 5;
    no_fly_zone.point.y = 0;
    no_fly_zone.point.z = 0;

    no_fly_zone_radius.data = 2.f;

    float marker_height = 10;
    no_fly_zone_marker.header.frame_id = "world";
    no_fly_zone_marker.ns = "no_fly_zone";
    no_fly_zone_marker.id = 0;
    no_fly_zone_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    no_fly_zone_marker.action = visualization_msgs::msg::Marker::ADD;
    no_fly_zone_marker.pose.orientation.w = 1;
    no_fly_zone_marker.pose.position.x = no_fly_zone.point.x;
    no_fly_zone_marker.pose.position.y = no_fly_zone.point.y;
    no_fly_zone_marker.pose.position.z = marker_height/2;
    no_fly_zone_marker.scale.x = 2*no_fly_zone_radius.data;
    no_fly_zone_marker.scale.y = 2*no_fly_zone_radius.data;
    no_fly_zone_marker.scale.z = marker_height;
    no_fly_zone_marker.color.r = 1;
    no_fly_zone_marker.color.g = 0;
    no_fly_zone_marker.color.b = 0;
    no_fly_zone_marker.color.a = 0.3;
  
    no_fly_zone_label.header.frame_id = "world";
    no_fly_zone_label.ns = "no_fly_zone_label";
    no_fly_zone_label.id = 0;
    no_fly_zone_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    no_fly_zone_label.action = visualization_msgs::msg::Marker::ADD;
    no_fly_zone_label.pose.orientation.w = 1;
    no_fly_zone_label.pose.position.x = no_fly_zone.point.x;
    no_fly_zone_label.pose.position.y = no_fly_zone.point.y;
    no_fly_zone_label.pose.position.z = marker_height/2;
    no_fly_zone_label.scale.x = 1;//2*no_fly_zone_radius.data;
    no_fly_zone_label.scale.y = 1;//2*no_fly_zone_radius.data;
    no_fly_zone_label.scale.z = 1;//marker_height;
    no_fly_zone_label.color.r = 1;
    no_fly_zone_label.color.g = 1;
    no_fly_zone_label.color.b = 1;
    no_fly_zone_label.color.a = 1;
    no_fly_zone_label.text = "No Fly Zone";

    home_label.header.frame_id = "world";
    home_label.ns = "home_label";
    home_label.id = 0;
    home_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    home_label.action = visualization_msgs::msg::Marker::ADD;
    home_label.pose.orientation.w = 1;
    home_label.pose.position.x = home.point.x;
    home_label.pose.position.y = home.point.y;
    home_label.pose.position.z = home.point.z;
    home_label.scale.x = 0.5;
    home_label.scale.y = 0.5;
    home_label.scale.z = 0.5;
    home_label.color.r = 1;
    home_label.color.g = 1;
    home_label.color.b = 1;
    home_label.color.a = 1;
    home_label.text = "Home";

    home_sphere.header.frame_id = "world";
    home_sphere.ns = "home_sphere";
    home_sphere.id = 0;
    home_sphere.type = visualization_msgs::msg::Marker::SPHERE;
    home_sphere.action = visualization_msgs::msg::Marker::ADD;
    home_sphere.pose.orientation.w = 1;
    home_sphere.pose.position.x = home.point.x;
    home_sphere.pose.position.y = home.point.y;
    home_sphere.pose.position.z = home.point.z;
    home_sphere.scale.x = 0.5;
    home_sphere.scale.y = 0.5;
    home_sphere.scale.z = 0.5;
    home_sphere.color.r = 0;
    home_sphere.color.g = 1;
    home_sphere.color.b = 0;
    home_sphere.color.a = 1;
  
    destination_label.header.frame_id = "world";
    destination_label.ns = "destination_label";
    destination_label.id = 0;
    destination_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    destination_label.action = visualization_msgs::msg::Marker::ADD;
    destination_label.pose.orientation.w = 1;
    destination_label.pose.position.x = destination.point.x;
    destination_label.pose.position.y = destination.point.y;
    destination_label.pose.position.z = destination.point.z;
    destination_label.scale.x = 0.5;
    destination_label.scale.y = 0.5;
    destination_label.scale.z = 0.5;
    destination_label.color.r = 1;
    destination_label.color.g = 1;
    destination_label.color.b = 1;
    destination_label.color.a = 1;
    destination_label.text = "Destination";

    destination_sphere.header.frame_id = "world";
    destination_sphere.ns = "destination_sphere";
    destination_sphere.id = 0;
    destination_sphere.type = visualization_msgs::msg::Marker::SPHERE;
    destination_sphere.action = visualization_msgs::msg::Marker::ADD;
    destination_sphere.pose.orientation.w = 1;
    destination_sphere.pose.position.x = destination.point.x;
    destination_sphere.pose.position.y = destination.point.y;
    destination_sphere.pose.position.z = destination.point.z;
    destination_sphere.scale.x = 0.5;
    destination_sphere.scale.y = 0.5;
    destination_sphere.scale.z = 0.5;
    destination_sphere.color.r = 0;
    destination_sphere.color.g = 0;
    destination_sphere.color.b = 1;
    destination_sphere.color.a = 1;

    marker_array.markers.push_back(no_fly_zone_marker);
    marker_array.markers.push_back(no_fly_zone_label);
    marker_array.markers.push_back(home_label);
    marker_array.markers.push_back(home_sphere);
    marker_array.markers.push_back(destination_label);
    marker_array.markers.push_back(destination_sphere);
  
    odom.header.frame_id = "world";
    odom.child_frame_id = "world";
    odom.pose.pose.orientation.w = 1;
  
    last_execute_time = node->get_clock()->now();

    // init publishers
    robot_odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
    home_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("home", 10);
    destination_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("destination", 10);
    no_fly_zone_pub = node->create_publisher<geometry_msgs::msg::PointStamped>("no_fly_zone", 10);
    no_fly_zone_radius_pub = node->create_publisher<std_msgs::msg::Float32>("no_fly_zone_radius", 10);
    no_fly_zone_vis_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("no_fly_zone_vis", 10);
  }

  void update(){
    // keep track of time
    rclcpp::Time now = node->get_clock()->now();
    double dt = (now - last_execute_time).seconds();
    last_execute_time = now;

    // update the robot's position based on the velocity command
    odom.pose.pose.position.x += vel_cmd.twist.linear.x*dt;
    odom.pose.pose.position.y += vel_cmd.twist.linear.y*dt;
    odom.pose.pose.position.z += vel_cmd.twist.linear.z*dt;

  
    // update the robot's orientation based on the velocity command
    double roll, pitch, yaw;
    tf2::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    roll += ang_vel_cmd.twist.angular.x*dt;
    pitch += ang_vel_cmd.twist.angular.y*dt;
    yaw += ang_vel_cmd.twist.angular.z*dt;
    q.setRPY(roll, pitch, yaw);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // publish
    odom.header.stamp = home.header.stamp = destination.header.stamp = no_fly_zone.header.stamp = no_fly_zone_marker.header.stamp = node->get_clock()->now();
    robot_odom_pub->publish(odom);
    home_pub->publish(home);
    destination_pub->publish(destination);
    no_fly_zone_pub->publish(no_fly_zone);
    no_fly_zone_radius_pub->publish(no_fly_zone_radius);
    no_fly_zone_vis_pub->publish(marker_array);//no_fly_zone_marker);
  }

  void set_velocity(float x, float y, float z){
    vel_cmd.twist.linear.x = x;
    vel_cmd.twist.linear.y = y;
    vel_cmd.twist.linear.z = z;
  }

  bool is_at_flight_altitude(){
    return odom.pose.pose.position.z > 0.99;
  }

  bool is_on_ground(){
    return odom.pose.pose.position.z < 0.01;
  }

  bool is_over_home(){
    float distance = sqrt(pow(odom.pose.pose.position.x - home.point.x, 2) +
			  pow(odom.pose.pose.position.y - home.point.y, 2));
    return distance < 0.01;
  }

  bool is_in_no_fly_zone(){
    float distance = sqrt(pow(odom.pose.pose.position.x - no_fly_zone.point.x, 2) +
			  pow(odom.pose.pose.position.y - no_fly_zone.point.y, 2));
    return distance < no_fly_zone_radius.data;
  }
  
  bool is_over_destination(){
    float distance = sqrt(pow(odom.pose.pose.position.x - destination.point.x, 2) +
			  pow(odom.pose.pose.position.y - destination.point.y, 2));
    return distance < 0.01;
  }

  
  
};
