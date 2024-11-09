#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <gps_msgs/msg/gps_fix.hpp>
#include <cmath>
#include <sstream>
#include <chrono>

class IntersectionSubscriber : public rclcpp::Node
{
public:
    IntersectionSubscriber() : Node("intersection_collision_warning_node")
    {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&IntersectionSubscriber::timer_callback, this));

        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/Vehicle_Map_Intersection", 10,
            std::bind(&IntersectionSubscriber::topic_callback, this, std::placeholders::_1));

        subscription_other_message_ = this->create_subscription<gps_msgs::msg::GPSFix>(
            "/other_car_message", 10,
            std::bind(&IntersectionSubscriber::other_gps_callback, this, std::placeholders::_1));

        EMERGENCY = this->create_publisher<std_msgs::msg::String>("/scenes", 10);

        has_local_message_ = false;
        has_other_message_ = false;
    }

private:
    void timer_callback()
    {
        // 检查是否有其他消息超时未收到，3秒未更新则认为过期
        if (this->now() - last_other_msg_time_ > rclcpp::Duration::from_seconds(3.0))
        {
            has_other_message_ = false;
        }
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到本地车辆地图交叉口信息: '%s'", msg->data.c_str());

        sscanf(msg->data.c_str(), "GPS位置: 纬度: %lf, 经度: %lf, 航向角: %lf",
               &local_lat_, &local_lon_, &local_heading_);

        has_local_message_ = true;

        check_and_warn();
    }

    void other_gps_callback(const gps_msgs::msg::GPSFix::SharedPtr msg)
    {
        last_other_msg_time_ = this->now();

        other_lat_ = msg->latitude;
        other_lon_ = msg->longitude;
        other_heading_ = msg->track;

        RCLCPP_INFO(this->get_logger(),
                    "收到GPS数据: 纬度=%.6f, 经度=%.6f, 方向=%.1f",
                    other_lat_, other_lon_, other_heading_);

        has_other_message_ = true;

        check_and_warn();
    }

    void check_and_warn()
    {
        if (has_local_message_ && has_other_message_)
        {
            double distance = haversine(local_lat_, local_lon_, other_lat_, other_lon_);

            // 30米内，航向角差在70-100度之间
            if (distance <= 30.0)
            {
                double heading_diff = std::abs(local_heading_ - other_heading_);
                if (heading_diff > 180.0)
                {
                    heading_diff = 360.0 - heading_diff;
                }

                if (heading_diff >= 70.0 && heading_diff <= 100.0)
                {
                    RCLCPP_WARN(this->get_logger(), "警告: 交叉路口碰撞风险！！: 距离=%.2f米, 航向角差=%.1f度", distance, heading_diff);

                    std::string fault_alarm_msg = "交叉路口碰撞预警";
                    auto message = std_msgs::msg::String();
                    message.data = fault_alarm_msg;
                    EMERGENCY->publish(message);

                    // 重置标志以防止重复触发
                    has_local_message_ = false;
                    has_other_message_ = false;
                }
            }
        }
    }

    double haversine(double lat1, double lon1, double lat2, double lon2) const
    {
        const double R = 6371000.0; // 地球半径，单位：米
        double dLat = (lat2 - lat1) * M_PI / 180.0;
        double dLon = (lon2 - lon1) * M_PI / 180.0;

        lat1 = lat1 * M_PI / 180.0;
        lat2 = lat2 * M_PI / 180.0;

        double a = sin(dLat / 2) * sin(dLat / 2) +
                   cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));

        return R * c;
    }

    bool has_local_message_;
    bool has_other_message_;
    double local_lat_, local_lon_, local_heading_;
    double other_lat_, other_lon_, other_heading_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr subscription_other_message_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr EMERGENCY;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_other_msg_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntersectionSubscriber>());
    rclcpp::shutdown();
    return 0;
}
