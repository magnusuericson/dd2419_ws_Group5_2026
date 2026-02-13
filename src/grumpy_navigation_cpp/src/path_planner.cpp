// path_planner_node.cpp
//
// MS2 path planner: straight-line path (2 waypoints) from current robot pose
// to a standoff pose near the goal.
//
// Subscribes:  /planner/goal   (geometry_msgs/PoseStamped)  [goal pose in map frame]
// Publishes:   /planner/path   (nav_msgs/Path)
//
// Params:
//  - map_frame (string) default "map"
//  - base_frame (string) default "base_link"
//  - standoff_distance (double) default 0.35   [meters]
//  - plan_rate_hz (double) default 5.0
//  - yaw_at_goal (bool) default true           [face the goal at last waypoint]

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

using namespace std::chrono_literals;

static double yaw_from_quat(const geometry_msgs::msg::Quaternion &q)
{
  tf2::Quaternion tq;
  tf2::fromMsg(q, tq);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}

static geometry_msgs::msg::Quaternion quat_from_yaw(double yaw)
{
  tf2::Quaternion tq;
  tq.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(tq);
}

class PathPlannerNode : public rclcpp::Node
{
public:
  PathPlannerNode() : Node("path_planner_node")
  {
    // ---- Params
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("standoff_distance", 0.35);
    declare_parameter<double>("plan_rate_hz", 5.0);
    declare_parameter<bool>("yaw_at_goal", true);

    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    standoff_distance_ = get_parameter("standoff_distance").as_double();
    plan_rate_hz_ = get_parameter("plan_rate_hz").as_double();
    yaw_at_goal_ = get_parameter("yaw_at_goal").as_bool();

    // ---- TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Pub/Sub
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planner/path", 10);

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planner/goal", 10,
      std::bind(&PathPlannerNode::on_goal, this, std::placeholders::_1));

    // ---- Timer
    const double hz = std::max(0.1, plan_rate_hz_);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / hz));

    timer_ = create_wall_timer(period, std::bind(&PathPlannerNode::tick, this));

    RCLCPP_INFO(get_logger(),
                "MS2 PathPlanner ready. map_frame=%s base_frame=%s standoff=%.2f rate=%.1fHz",
                map_frame_.c_str(), base_frame_.c_str(), standoff_distance_, plan_rate_hz_);
  }

private:
  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Accept goal in map_frame_ only (keep MS2 simple & unambiguous)
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Goal frame_id is '%s' but planner expects '%s'. Ignoring goal.",
                           msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    last_goal_ = *msg;
    last_goal_.header.frame_id = map_frame_;
    have_goal_ = true;

    RCLCPP_INFO(get_logger(), "Received goal: (%.2f, %.2f)",
                last_goal_.pose.position.x, last_goal_.pose.position.y);
  }

  bool lookup_robot_pose(geometry_msgs::msg::PoseStamped &out_pose_map)
  {
    // We want base_frame pose expressed in map_frame
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "TF lookup failed (%s -> %s): %s",
                           map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }

    out_pose_map.header.stamp = now();
    out_pose_map.header.frame_id = map_frame_;
    out_pose_map.pose.position.x = tf.transform.translation.x;
    out_pose_map.pose.position.y = tf.transform.translation.y;
    out_pose_map.pose.position.z = tf.transform.translation.z;
    out_pose_map.pose.orientation = tf.transform.rotation;
    return true;
  }

  geometry_msgs::msg::PoseStamped compute_standoff(const geometry_msgs::msg::PoseStamped &robot_map,
                                                   const geometry_msgs::msg::PoseStamped &goal_map) const
  {
    const double rx = robot_map.pose.position.x;
    const double ry = robot_map.pose.position.y;
    const double gx = goal_map.pose.position.x;
    const double gy = goal_map.pose.position.y;

    const double dx = gx - rx;
    const double dy = gy - ry;
    const double dist = std::hypot(dx, dy);

    // Direction from robot -> goal
    double heading = std::atan2(dy, dx);

    geometry_msgs::msg::PoseStamped standoff;
    standoff.header.frame_id = map_frame_;
    standoff.header.stamp = now();

    // If we are extremely close, just keep current position and set yaw
    if (dist < 1e-6) {
      standoff.pose.position.x = rx;
      standoff.pose.position.y = ry;
    } else {
      const double s = std::clamp(standoff_distance_, 0.0, dist);
      // Stop s meters before the goal along the line
      standoff.pose.position.x = gx - s * std::cos(heading);
      standoff.pose.position.y = gy - s * std::sin(heading);
    }

    standoff.pose.position.z = 0.0;

    if (yaw_at_goal_) {
      // face the goal at the end
      standoff.pose.orientation = quat_from_yaw(heading);
    } else {
      // keep robot yaw
      standoff.pose.orientation = robot_map.pose.orientation;
    }

    return standoff;
  }

  void publish_path(const geometry_msgs::msg::PoseStamped &start_map,
                    const geometry_msgs::msg::PoseStamped &goal_standoff_map)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = map_frame_;

    geometry_msgs::msg::PoseStamped p0 = start_map;
    p0.header.stamp = path.header.stamp;
    p0.header.frame_id = map_frame_;

    geometry_msgs::msg::PoseStamped p1 = goal_standoff_map;
    p1.header.stamp = path.header.stamp;
    p1.header.frame_id = map_frame_;

    path.poses.push_back(p0);
    path.poses.push_back(p1);

    path_pub_->publish(path);
  }

  void tick()
  {
    if (!have_goal_) return;

    geometry_msgs::msg::PoseStamped robot_map;
    if (!lookup_robot_pose(robot_map)) return;

    // last_goal_ is already required to be in map_frame_
    auto standoff = compute_standoff(robot_map, last_goal_);

    publish_path(robot_map, standoff);
  }

private:
  // Params
  std::string map_frame_;
  std::string base_frame_;
  double standoff_distance_{0.35};
  double plan_rate_hz_{5.0};
  bool yaw_at_goal_{true};

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Pub/Sub
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  geometry_msgs::msg::PoseStamped last_goal_;
  bool have_goal_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
