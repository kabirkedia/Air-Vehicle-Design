#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <behavior_tree_example/robot.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"

#include <behavior_tree/behavior_tree.h> // Include the behavior tree library containing the Action and Condition classes.


class BehaviorTreeExample : public rclcpp::Node
{
private:
  bool got_robot_odom, got_home, got_destination, got_no_fly_zone, got_no_fly_zone_radius;
  nav_msgs::msg::Odometry robot_odom;
  float flight_z, ground_z;
  geometry_msgs::msg::PointStamped home, destination;
  float acceptance_radius;
  geometry_msgs::msg::PointStamped no_fly_zone;
  float no_fly_zone_radius;
  
  // Condition variables
  bt::Condition* at_flight_altitude_condition;
  bt::Condition* on_ground_condition;
  bt::Condition* visited_destination_condition;
  bt::Condition* at_home_condition;
  bt::Condition* in_fly_zone_condition;
  bt::Condition* in_no_fly_zone_condition;
  std::vector<bt::Condition*> conditions;

  // Action variables
  bt::Action* land_action;
  bt::Action* takeoff_action;
  bt::Action* go_to_destination_action;
  bt::Action* go_home_action;
  std::vector<bt::Action*> actions;
  
  // subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr home_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr destination_sub;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr no_fly_zone_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr no_fly_zone_radius_sub;
  
  // publishers
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_pub;

  // timers
  rclcpp::TimerBase::SharedPtr timer;

  Robot robot;
  
public:
  BehaviorTreeExample()
    : Node("behavior_tree_example")
    , robot(this){
    

    on_ground_condition = new bt::Condition("On Ground", this);
    at_flight_altitude_condition = new bt::Condition("At Flight Altitude", this);
    visited_destination_condition = new bt::Condition("Visited Destination", this);
    at_home_condition = new bt::Condition("At Home", this);
    in_fly_zone_condition = new bt::Condition("In Fly Zone", this);
    in_no_fly_zone_condition = new bt::Condition("In No Fly Zone", this);
    conditions.push_back(on_ground_condition);
    conditions.push_back(at_flight_altitude_condition);
    conditions.push_back(visited_destination_condition);
    conditions.push_back(at_home_condition);
    conditions.push_back(in_fly_zone_condition);
    conditions.push_back(in_no_fly_zone_condition);
    
    land_action = new bt::Action("Land", this);
    takeoff_action = new bt::Action("Takeoff", this);
    go_to_destination_action = new bt::Action("Go To Destination", this);
    go_home_action = new bt::Action("Go Home", this);
    actions.push_back(land_action);
    actions.push_back(takeoff_action);
    actions.push_back(go_to_destination_action);
    actions.push_back(go_home_action);
    
    // init timer
    timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&BehaviorTreeExample::timer_callback, this));
  }
  
  void timer_callback(){
    at_flight_altitude_condition->set(robot.is_at_flight_altitude());
    on_ground_condition->set(robot.is_on_ground());
    at_home_condition->set(robot.is_over_home());
    in_no_fly_zone_condition->set(robot.is_in_no_fly_zone());
    in_fly_zone_condition->set(!robot.is_in_no_fly_zone());
    if(!visited_destination_condition->get())
      visited_destination_condition->set(robot.is_over_destination());
    
    if(land_action->is_active()){
      if(on_ground_condition->get()){
	land_action->set_success();
	robot.set_velocity(0, 0, 0);
      }
      else{
	land_action->set_running();
	robot.set_velocity(0, 0, -0.3);
      }
    }

    if(go_home_action->is_active()){
      if(at_home_condition->get()){
	go_home_action->set_success();
	robot.set_velocity(0, 0, 0);
      }
      else{
	go_home_action->set_running();
	robot.set_velocity(-0.3, 0, 0);
      }
    }
  
    if(go_to_destination_action->is_active()){   
      if(robot.is_over_destination()){
	go_to_destination_action->set_success();
	robot.set_velocity(0, 0, 0);
      }
      else{
	go_to_destination_action->set_running();
	robot.set_velocity(0.3, 0, 0);
      }
    }

    if(takeoff_action->is_active()){
      takeoff_action->set_running();
      robot.set_velocity(0, 0, 0.3);
    }
    else{
      if(takeoff_action->active_has_changed())
	robot.set_velocity(0, 0, 0);
    }
    
    robot.update();
    
    for(bt::Condition* condition : conditions)
      condition->publish();
    for(bt::Action* action : actions)
      action->publish();
  }
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BehaviorTreeExample>());
  rclcpp::shutdown();
  return 0;
}
