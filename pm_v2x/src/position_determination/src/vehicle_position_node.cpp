// 场景：前项碰撞预警
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 包含tf2相关头文件、
#include <geometry_msgs/msg/twist.hpp> // 接收速度消息
#include <std_msgs/msg/string.hpp>
#include <sstream>

using namespace std::chrono_literals;

class VehiclePositionNode : public rclcpp::Node {
public:
  VehiclePositionNode() : Node("vehicle_position_node") {
    // 订阅本车位姿
    local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/local_pose_b", 10,
        std::bind(&VehiclePositionNode::localPoseCallback, this, std::placeholders::_1));

    // 订阅其他车辆位姿
    other_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/other_pose_b", 10,
        std::bind(&VehiclePositionNode::otherPoseCallback, this, std::placeholders::_1));

    // 订阅速度消息
    local_speed_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/local_speed_b", 10, 
        std::bind(&VehiclePositionNode::localSpeedCallback, this, std::placeholders::_1));

    other_speed_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/other_speed_b", 10, 
        std::bind(&VehiclePositionNode::otherSpeedCallback, this, std::placeholders::_1));

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

    // 场景一：前项碰撞预警
    std::string fault_alarm_msg;
    if (delta_y>-1 && delta_y<1 && delta_x>0 && delta_x<10){
      // RCLCPP_WARN(this->get_logger(), "注意前方有车！小心碰撞！");

      // fault_alarm_msg = "注意前方有车！小心碰撞！";
      // RCLCPP_WARN(this->get_logger(), fault_alarm_msg.c_str());
      // auto message = std_msgs::msg::String();
      // message.data = fault_alarm_msg;
      // EMERGENCY->publish(message);

      fault_alarm_msg = "前项碰撞预警";
      RCLCPP_WARN(this->get_logger(), fault_alarm_msg.c_str());
      auto message = std_msgs::msg::String();
      message.data = fault_alarm_msg;
      EMERGENCY->publish(message);

    }else if (delta_y>-1&&delta_y<1 && delta_x>0){
      direction = "正前方";
      double relative_speed = other_speed_kph_ - local_speed_kph_;
      RCLCPP_INFO(this->get_logger(), "Relative speed: %.2f km/h", relative_speed);
      if (relative_speed != 0){
      double TTC = distance / (std::abs(relative_speed) * 1000.0 / 3600.0);
      RCLCPP_INFO(this->get_logger(), "TTC: %f seconds", TTC);
      if (TTC < 5){
        // RCLCPP_WARN(this->get_logger(), "注意前方有车！小心碰撞！");
        //   fault_alarm_msg = "注意前方有车！小心碰撞！";
        // RCLCPP_WARN(this->get_logger(), fault_alarm_msg.c_str());
        // auto message = std_msgs::msg::String();
        // message.data = fault_alarm_msg;
        // EMERGENCY->publish(message);

      fault_alarm_msg = "前项碰撞预警";
      RCLCPP_WARN(this->get_logger(), fault_alarm_msg.c_str());
      auto message = std_msgs::msg::String();
      message.data = fault_alarm_msg;
      EMERGENCY->publish(message);
      
      }
    }

    }else if (delta_y>-1 && delta_y<1 && delta_x<0){
      direction = "正后方";
    }
    else if (delta_y<-1 && delta_x>1){
      direction = "右前方";
    }else if (delta_y<-1 && delta_x<1 && delta_x>-1){
      direction = "右侧";
    }else if (delta_y<-1 && delta_x<-1){
      direction = "右后方";
    }
    else if (delta_y>1 && delta_x>1){
      direction = "左前方";
    }
    else if (delta_y>1 && delta_x<1 && delta_x>-1){
      direction = "左前方";
    }else if (delta_y>1 && delta_x<1 && delta_x>-1){
      direction = "左侧";
    }else if (delta_y>1 && delta_x<-1){
      direction = "左后方";
    };
    

    // // 计算相对角度
    // tf2::Quaternion q_local, q_other, q_relative; // 声明了三个四元数对象，分别表示本车位姿、其他车辆位姿和相对位姿
    // tf2::convert(local_pose_.pose.orientation, q_local); // 将本车位姿的四元数转换为 tf2::Quaternion 对象储存在 q_local 中
    // tf2::convert(other_pose_.pose.orientation, q_other);
    // q_relative = q_local.inverse() * q_other; // q.local.inverse() 表示 q_local 的逆矩阵，乘以 q_other 得到相对位姿的四元数

    // // 使用 tf2::Matrix3x3 获取 yaw 角
    // tf2::Matrix3x3 m(q_relative); // 使用相对四元数q_relative初始化了一个tf2::Matrix3x3对象m,这个矩阵表示了与q_relative相同的旋转。
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw); // 将矩阵m表示的旋转转换为Roll、Pitch和Yaw角

    // // 将弧度yaw转为角度
    // double angle_deg = yaw * 180.0 / M_PI; // yaw是一个表示偏航角的变量,绕Z轴的旋转
    // RCLCPP_WARN(this->get_logger(), "angle_deg%.2f",angle_deg);  
    // // 判断是否在“正前方”3米内  
    // if (distance < 3.0 && std::abs(angle_deg) <= 5.0) {  
    //     RCLCPP_WARN(this->get_logger(), "注意前方有慢车！");  
    // }  

    // // 判断方向
    // // std::string direction;
    // if (std::abs(angle_deg) <= 5.0) {
    //   direction = "正前方";
    // } else if (std::abs(angle_deg) >= 170.0) {
    //   direction = "正后方";
    // } else if (angle_deg > 5.0 && angle_deg <= 60.0) {
    //   direction = "右前方";
    // } else if (angle_deg > 60.0 && angle_deg <= 120.0) {
    //   direction = "右侧";
    // } else if (angle_deg > 120.0 && angle_deg <= 170.0) {
    //   direction = "右后方";
    // } else if (angle_deg < -5.0 && angle_deg >= -60.0) {
    //   direction = "左前方";
    // } else if (angle_deg < -60.0 && angle_deg >= -120.0) {
    //   direction = "左侧";
    // } else {
    //   direction = "左后方";
    // }

    // 打印结果
    RCLCPP_INFO(this->get_logger(), "远车方位: [%s] ,相隔距离： %.2f meters", 
                direction.c_str(),distance);
  }

  void localSpeedCallback(const geometry_msgs::msg::Twist::SharedPtr msg) { // 去掉了const
    local_speed_kph_ = msg->linear.x * 3.6;
    RCLCPP_INFO(this->get_logger(), "本车速度: %f km/h",local_speed_kph_);
  }

  void otherSpeedCallback(const geometry_msgs::msg::Twist::SharedPtr msg) { //去掉了const
    other_speed_kph_ = msg->linear.x * 3.6;
    RCLCPP_INFO(this->get_logger(), "远车速度：%f km/h",other_speed_kph_);
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
  rclcpp::spin(std::make_shared<VehiclePositionNode>());
  rclcpp::shutdown();
  return 0;
}