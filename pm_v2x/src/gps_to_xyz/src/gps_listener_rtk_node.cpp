#include <chrono>
#include <memory>
#include <string>
#include <iomanip>
// #include <serial/serial.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
// #include "sensor_msgs/msg/nav_sat_fix.hpp"  // 包含 NavSatFix 消息类型
#include "geometry_msgs/msg/twist.hpp"
#include "raw_sensor_msgs/msg/chanavgnss.hpp"


/*
raw_sensor_msgs/msg/Chanavgnss  /gnss_ins

header:
  stamp:
    sec: 1727535076
    nanosec: 265968164
  frame_id: base_link
latitude: 36.7523151
longitude: 117.25573636
altitude: 34.96
utm_east: 0.0
utm_north: 0.0
north_velocity: -0.001
east_velocity: 0.003
up_velocity: 0.002
velocity: 0.004
roll: 0.26
pitch: 0.67
yaw: 0.61
x: 0.018883335902141307
y: 0.34771355790252284
z: 0.24055424390140992
w: 0.9060200643567432
roll_rate: 0.0
pitch_rate: 0.0
yaw_rate: 0.0
ax: -0.0047
ay: 0.0122
az: 1.0334
none_gps_data: 0
none_vehicle_data: 0
error_gyroscope: 0
error_accelerometer: 0
nsv1_num: 24
nsv2_num: 34
age: 0
*/
using namespace std::chrono_literals;

class RtkGpsListener : public rclcpp::Node{
public:
    RtkGpsListener() : Node("rtk_gps_listener_node"){
        // rclcpp::QoS qos_profile(10);
        // qos_profile.reliable();
        // subscription_ = this->create_subscription<msg_interfaces::msg::String>(
        //     "/chcnav/nmea_sentence", qos_profile,
        //     std::bind(&RtkGpsListener::topic_callback, this, std::placeholders::_1));

        langyi_subscription_ = this->create_subscription<raw_sensor_msgs::msg::Chanavgnss>(
            "/gnss_ins",10,
            std::bind(&RtkGpsListener::topic_callback, this, std::placeholders::_1));

        
        gps_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("/local_messages_b", 10);
        speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/local_speed_b", 10);
        // udp_publisher_ = this->create_publisher<std_msgs::msg::String>("/send_to_udp_topic1", 10);
        UI_llahs_publisher_ = this->create_publisher<gps_msgs::msg::GPSFix>("/ui_datapublisher_llahs", 10);
    }

private:

    void topic_callback(const raw_sensor_msgs::msg::Chanavgnss::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(),"Received: '%s'",msg->sentence.c_str());

        // std::string nmea_sentence = msg->sentence;
        // if (nmea_sentence.find("$GPCHC") == 0){
        //     std::vector<std::string> fields;
        //     std::stringstream ss(nmea_sentence);
        //     std::string field;
        //     while (std::getline(ss, field, ',')) {
        //         fields.push_back(field);
        //     }
        //     if (fields.size() >= 14){
        //         double latitude  = std::stod(fields[12]);
        //         double longitude = std::stod(fields[13]);
        //         // double altitude  = std::stod(fields[14]);
        //         double altitude  = 0.0;
        //         double heading   = std::stod(fields[3]);
        //         double speed_ms = std::stod(fields[18]);
        //         double speed_kmh = speed_ms * 3.6;

            double latitude = msg->latitude;
            double longitude = msg->longitude;
            double altitude = 0.0;
            double speed_ms = msg->velocity;
            double speed_kmh = speed_ms * 3.6;
            double heading = msg->yaw;
        

            // 创建并发布 GPSFix 消息
            auto gps_msg = gps_msgs::msg::GPSFix();
            gps_msg.latitude = latitude;
            gps_msg.longitude = longitude; 
            gps_msg.altitude = altitude;
            gps_msg.track = heading;
            gps_msg.speed = speed_kmh;
            gps_publisher_->publish(gps_msg);
            UI_llahs_publisher_->publish(gps_msg);

            // 创建并发布 Twist 消息
            auto speed_msg = geometry_msgs::msg::Twist();
            speed_msg.linear.x = speed_ms; // m/s
            speed_publisher_->publish(speed_msg);

            // Publish data to UDP topic
            // auto udp_msg = std_msgs::msg::String();
            // std::ostringstream oss;
            // oss << std::fixed << std::setprecision(7);  // 设置小数点后保留7位
            // oss << "Lat:" << latitude << ",Lon:" << longitude << ",Heading:" << heading << ",Speed:" << speed_kmh;
            // udp_msg.data = oss.str();
            // RCLCPP_INFO(this->get_logger(), "Publishing to UDP topic: %s", udp_msg.data.c_str());
            // udp_publisher_->publish(udp_msg);
        
    }


    // rclcpp::Subscription<msg_interfaces::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<raw_sensor_msgs::msg::Chanavgnss>::SharedPtr langyi_subscription_;

    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr udp_publisher_;
    rclcpp::Publisher<gps_msgs::msg::GPSFix>::SharedPtr UI_llahs_publisher_;
};

int main(int argc,char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RtkGpsListener>());
    rclcpp::shutdown();
    return 0;
}