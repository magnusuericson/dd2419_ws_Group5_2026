
#include <chrono>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class FakeGoalPublisher : public rclcpp::Node
{
public:
  FakeGoalPublisher() : Node("fake_goal_publisher")
  {
    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/planner/goal", 10);

    timer_ = create_wall_timer(
      3s,
      std::bind(&FakeGoalPublisher::publish_goal, this));

    rng_.seed(std::random_device()());
  }

private:
  void publish_goal()
  {
    std::uniform_real_distribution<double> dist(-2.0, 2.0);

    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";

    msg.pose.position.x = dist(rng_);
    msg.pose.position.y = dist(rng_);
    msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);   // yaw = 0 for now
    msg.pose.orientation = tf2::toMsg(q);

    pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Published fake goal: (%.2f, %.2f)",
                msg.pose.position.x,
                msg.pose.position.y);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mt19937 rng_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeGoalPublisher>());
  rclcpp::shutdown();
  return 0;
}
