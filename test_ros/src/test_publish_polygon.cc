#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("minimal_publisher"), count_(0)
    {
        base_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("base_polygon", 10);
        veh_polygon_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("veh_polygon", 10);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::PolygonStamped();

        geometry_msgs::msg::Point32 pt;
        pt.x = 0;
        pt.y = 0;
        pt.z = 0;

        message.polygon.points.push_back(pt);

        pt.x = 10;
        pt.y = 0;
        pt.z = 0;


        message.polygon.points.push_back(pt);

        pt.x = 10;
        pt.y = 5;
        pt.z = 0;


        message.polygon.points.push_back(pt);

        pt.x = 0;
        pt.y = 5;
        pt.z = 0;


        message.polygon.points.push_back(pt);

        message.header.frame_id = "map";

        RCLCPP_INFO(this->get_logger(), "Publishing: ");
        base_polygon_publisher_->publish(message);


        // veh polygon position
        message.polygon.points.clear();
        pt.x = 1000;
        pt.y = 0;
        pt.z = 0;

        message.polygon.points.push_back(pt);

        pt.x = 1000;
        pt.y = 5;
        pt.z = 0;


        message.polygon.points.push_back(pt);

        pt.x = 999;
        pt.y = 5;
        pt.z = 0;


        message.polygon.points.push_back(pt);

        pt.x = 999;
        pt.y = 0;
        pt.z = 0;


        message.polygon.points.push_back(pt);

        message.header.frame_id = "map";
        veh_polygon_publisher_->publish(message);


    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr base_polygon_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr veh_polygon_publisher_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}