// 场景：盲区预警

// BlindSpotWarningNode
// blind_spot_warning_node
// 取消了打印其他东西，只在盲区预警触发时候打印

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 包含tf2相关头文件、
#include <geometry_msgs/msg/twist.hpp> // 接收速度消息
#include <std_msgs/msg/string.hpp>


using namespace std::chrono_literals;

class BlindSpotWarningNode : public rclcpp::Node {
public:
  BlindSpotWarningNode() : Node("blind_spot_warning_node") {
    // 订阅本车位姿
    local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/local_pose_b", 10,
        std::bind(&BlindSpotWarningNode::localPoseCallback, this, std::placeholders::_1));

    // 订阅其他车辆位姿
    other_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/other_pose_b", 10,
        std::bind(&BlindSpotWarningNode::otherPoseCallback, this, std::placeholders::_1));

    // 订阅速度消息
    local_speed_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/local_speed_b", 10, 
        std::bind(&BlindSpotWarningNode::localSpeedCallback, this, std::placeholders::_1));

    other_speed_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/other_speed_b", 10, 
        std::bind(&BlindSpotWarningNode::otherSpeedCallback, this, std::placeholders::_1));

    
    EMERGENCY = this->create_publisher<std_msgs::msg::String>("/scenes", 10); 

  }

private:
  void localPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    local_pose_ = *msg;
  }

  void otherPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    other_pose_ = *msg;

    if (local_pose_.header.stamp.sec == 0) {
      RCLCPP_WARN(this->get_logger(), "Waiting for local pose...");
      return;
    }

    // 计算相对位置
    std::string direction;
    double delta_x = other_pose_.pose.position.x - local_pose_.pose.position.x;
    double delta_y = other_pose_.pose.position.y - local_pose_.pose.position.y;
    double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);

    // if (delta_y>-1 && delta_y<1 && delta_x>0 && delta_x<5){
    //   RCLCPP_WARN(this->get_logger(), "注意前方有车！小心碰撞！");
    // }else if (delta_y>-1&&delta_y<1 && delta_x>0){
    //   direction = "正前方";
    //   double relative_speed = other_speed_kph_ - local_speed_kph_;
    //   RCLCPP_INFO(this->get_logger(), "Relative speed: %.2f km/h", relative_speed);
    //   if (relative_speed != 0){
    //   double TTC = distance / (std::abs(relative_speed) * 1000.0 / 3600.0);
    //   RCLCPP_INFO(this->get_logger(), "TTC: %f seconds", TTC);
    //   if (TTC < 5){
    //     RCLCPP_WARN(this->get_logger(), "注意前方有车！小心碰撞！");
    //   }
    // }

    std::string fault_alarm_msg;

    if (delta_y > -1 && delta_y < 1 && delta_x > 0) {
      direction = "正前方";
    } else if (delta_y > -1 && delta_y < 1 && delta_x < 0) {
      direction = "正后方";
    } else if (delta_y < -1 && delta_x > 1) {
      direction = "右前方";
    } else if (delta_y < -1 && delta_x < 1 && delta_x > -1) {
      direction = "右侧";
    } else if (delta_y < -1 && delta_y > -3 && delta_x < -1) {


        // fault_alarm_msg = "注意右后方有车请勿变道！";
        fault_alarm_msg = "盲区预警";
        RCLCPP_WARN(this->get_logger(), fault_alarm_msg.c_str());
        auto message = std_msgs::msg::String();
        message.data = fault_alarm_msg;
        EMERGENCY->publish(message);


      direction = "右后方"; // 也需要设置 direction
    } else if (delta_y < -1 && delta_x < -1) {
      direction = "右后方";
    } else if (delta_y > 1 && delta_x > 1) {
      direction = "左前方";
    } else if (delta_y > 1 && delta_x < 1 && delta_x > -1) {
      direction = "左侧"; 
    } else if(delta_y > 1 && delta_y < 3 && delta_x < -1){

      // fault_alarm_msg = "注意左后方有车请勿变道！";
      fault_alarm_msg = "盲区预警";
      RCLCPP_WARN(this->get_logger(), fault_alarm_msg.c_str());
      auto message = std_msgs::msg::String();
      message.data = fault_alarm_msg;
      EMERGENCY->publish(message);

      direction = "左后方"; // 也需要设置 direction
    }else if (delta_y > 1 && delta_x < -1) {
      direction = "左后方";
    };
    
    
    // 打印结果
    // RCLCPP_INFO(this->get_logger(), "远车方位: [%s] ,相隔距离： %.2f meters", 
    //             direction.c_str(),distance);
  }

  void localSpeedCallback(const geometry_msgs::msg::Twist::SharedPtr msg) { // 去掉了const
    local_speed_kph_ = msg->linear.x * 3.6;
    // RCLCPP_INFO(this->get_logger(), "本车速度: %f km/h",local_speed_kph_);
  }

  void otherSpeedCallback(const geometry_msgs::msg::Twist::SharedPtr msg) { //去掉了const
    other_speed_kph_ = msg->linear.x * 3.6;
    // RCLCPP_INFO(this->get_logger(), "远车速度：%f km/h",other_speed_kph_);
    // RCLCPP_INFO(this->get_logger(), "Linear x: %f km/h", msg->linear.x);
  }

  // ... 成员变量 ...
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr other_pose_sub_;
  geometry_msgs::msg::PoseStamped local_pose_;
  geometry_msgs::msg::PoseStamped other_pose_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr local_speed_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr other_speed_subscriber_;


  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr EMERGENCY;

  double local_speed_kph_=0.0;
  double other_speed_kph_=0.0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BlindSpotWarningNode>());
  rclcpp::shutdown();
  return 0;
}