#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticFramePublisher : public rclcpp::Node
{
public:
    explicit StaticFramePublisher()
        : Node("static_turtle_tf2_broadcaster")
    {
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms once at startup
        this->make_transforms();
    }

private:
    void make_transforms()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "vehicle";

        t.transform.translation.x = 10;
        t.transform.translation.y = 10;
        t.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(
            0,
            0,
            0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_static_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char *argv[])
{
    auto logger = rclcpp::get_logger("logger");

    // Pass parameters and initialize node
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<StaticFramePublisher>());
    rclcpp::shutdown();
    return 0;
}