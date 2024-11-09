// 场景：紧急车辆

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <sstream>

class FaultAlarmReceiverNode : public rclcpp::Node {
public:
    FaultAlarmReceiverNode()
      : Node("vehicle_emergency_node") {
        // 订阅紧急车辆
        fault_alarm_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/emergency_alarm", 10, std::bind(&FaultAlarmReceiverNode::fault_alarm_callback, this, std::placeholders::_1));

        // 订阅本车位姿
        local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/local_pose_b", 10,
            std::bind(&FaultAlarmReceiverNode::localPoseCallback, this, std::placeholders::_1));

        // 订阅其他车辆位姿
        other_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/other_pose_b", 10,
            std::bind(&FaultAlarmReceiverNode::otherPoseCallback, this, std::placeholders::_1));

        // 订阅速度消息
        local_speed_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/local_speed_b", 10, 
            std::bind(&FaultAlarmReceiverNode::localSpeedCallback, this, std::placeholders::_1));

        other_speed_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/other_speed_b", 10, 
            std::bind(&FaultAlarmReceiverNode::otherSpeedCallback, this, std::placeholders::_1));

        //  发布
        EMERGENCY = this->create_publisher<std_msgs::msg::String>("/scenes", 10); 
        
    }

private:
    void fault_alarm_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received emergency vehicle message: %s", msg->data.c_str());

        // 提示驾驶员注意附近异常车辆
        if (!local_pose_received_ || !other_pose_received_) {
            RCLCPP_WARN(this->get_logger(), "注意紧急车辆，注意避让！");
            return;
        }


        std::string fault_alarm_msg;
        std::string direction;
        double delta_x = other_pose_.pose.position.x - local_pose_.pose.position.x;
        double delta_y = other_pose_.pose.position.y - local_pose_.pose.position.y;

        if (delta_y > -1 && delta_y < 1 && delta_x > 0) {
            direction = "正前方";
        } else if (delta_y > -1 && delta_y < 1 && delta_x < 0) {
            direction = "正后方";
        } else if (delta_y < -1 && delta_x > 1) {
            direction = "右前方";
        } else if (delta_y < -1 && delta_x < 1 && delta_x > -1) {
            direction = "右侧";
        } else if (delta_y < -1 && delta_x < -1) {
            direction = "右后方";
        } else if (delta_y > 1 && delta_x > 1) {
            direction = "左前方";
        } else if (delta_y > 1 && delta_x < 1 && delta_x > -1) {
            direction = "左侧";
        } else if (delta_y > 1 && delta_x < -1) {
            direction = "左后方";
        } else {
            direction = "未知方向";
        }
        RCLCPP_WARN(this->get_logger(), "注意 %s 紧急车辆，注意避让！", direction.c_str());

        // std::ostringstream oss;
        // oss<<"注意"<<direction<<"紧急车辆，注意避让！";
        // auto message=std_msgs::msg::String();
        // std::string fault_alarm_msg = oss.str();
        // message.data = fault_alarm_msg;
        // EMERGENCY->publish(message);

        fault_alarm_msg = "紧急车辆提醒";
        RCLCPP_WARN(this->get_logger(), fault_alarm_msg.c_str());
        auto message = std_msgs::msg::String();
        message.data = fault_alarm_msg;
        EMERGENCY->publish(message);
        
    }

    void localPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        local_pose_ = *msg;
        local_pose_received_ = true;
    }

    void otherPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        other_pose_ = *msg;
        other_pose_received_ = true;
    }

    void localSpeedCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        local_speed_kph_ = msg->linear.x * 3.6;
        // RCLCPP_INFO(this->get_logger(), "本车速度: %f km/h", local_speed_kph_);
    }

    void otherSpeedCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        other_speed_kph_ = msg->linear.x * 3.6;
        // RCLCPP_INFO(this->get_logger(), "远车速度: %f km/h", other_speed_kph_);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_alarm_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr other_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr local_speed_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr other_speed_subscriber_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr EMERGENCY;
    std::ostringstream oss;
    
    geometry_msgs::msg::PoseStamped local_pose_;
    geometry_msgs::msg::PoseStamped other_pose_;
    double local_speed_kph_ = 0.0;
    double other_speed_kph_ = 0.0;

    bool local_pose_received_ = false;
    bool other_pose_received_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaultAlarmReceiverNode>());
    rclcpp::shutdown();
    return 0;
}