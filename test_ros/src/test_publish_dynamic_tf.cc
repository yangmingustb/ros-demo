#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"

#include <chrono>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

using namespace std::chrono_literals;

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
      : Node("vehicle_base_tf_pub"), count_(0)
  {

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
        500ms, std::bind(&FramePublisher::handle_turtle_pose, this));
  }

private:
  void handle_turtle_pose()
  {
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Pose pose;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id ="vehicle";

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = 1000.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    // For the same reason, turtle can only rotate around one axis
    // and this why we set rotation in x and y to 0 and obtain
    // rotation in z axis from the message
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(this->get_logger(), "Publishing vehicle tf ");

    RCLCPP_INFO(this->get_logger(), "Publishing: %.4f, %.2f, %.2f ", t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}