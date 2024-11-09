#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <eigen3/Eigen/Geometry> // 四元数
#include <eigen3/Eigen/Dense>
// #include <gps_msgs/msg/GPSFix.hpp>
#include "gps_msgs/msg/gps_fix.hpp"

#include<std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class GpsToXyzNode : public rclcpp::Node {
public:
  GpsToXyzNode() : Node("gps_to_xyz_node") {
    // 订阅本车 GPS 消息
    subscription_ = this->create_subscription<gps_msgs::msg::GPSFix>(
      "/local_messages_b", 10,
      std::bind(&GpsToXyzNode::gps_callback, this, std::placeholders::_1));

    // 订阅其他车辆 GPS 消息
    // other_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    //   "/other_messages_b", 10,
    //   std::bind(&GpsToXyzNode::other_gps_callback, this, std::placeholders::_1));

    other_subscription_ = this->create_subscription<gps_msgs::msg::GPSFix>("/other_car_message",10,
      std::bind(&GpsToXyzNode::other_gps_callback, this, std::placeholders::_1));

    // 发布本地笛卡尔坐标系坐标
    local_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/local_pose_b", 10);
    other_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/other_pose_b", 10);


    // 发布 RPY 数据
    rpy_publisher_ = this->create_publisher<std_msgs::msg::String>("/RPYdata", 10);

    // 设置WGS84椭球模型
    earth_ = std::make_shared<GeographicLib::Geocentric>(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
  }

private:

  void gps_callback(const gps_msgs::msg::GPSFix::SharedPtr msg) {
    double latitude = msg->latitude;
    double longitude = msg->longitude;
    // heading = msg->position_covariance[8];
    heading = msg->track;

    std::cout<<"接过来的heading:"<<heading<<std::endl;

        // 检查纬度和经度范围
    if (latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
        RCLCPP_WARN(this->get_logger(), "Received invalid GPS data: Lat=%f, Long=%f", latitude, longitude);
        return;
    }

    // 更新本地坐标系原点
    local_cartesian_.Reset(latitude, longitude, 0);
    

    // 本车坐标转换后始终为原点 (0, 0, 0)
    geometry_msgs::msg::PoseStamped local_pose;
    local_pose.header.stamp = this->get_clock()->now();
    local_pose.header.frame_id = "map";
    local_pose.pose.position.x = 0.0;
    local_pose.pose.position.y = 0.0;
    local_pose.pose.position.z = 0.0;

    // 计算旋转四元数
    Eigen::AngleAxisd angle_axis(heading * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q(angle_axis);
    local_pose.pose.orientation.x = q.x();
    local_pose.pose.orientation.y = q.y();
    local_pose.pose.orientation.z = q.z();
    local_pose.pose.orientation.w = q.w();

    local_pose_pub_->publish(local_pose);

    auto rpy_msg = std_msgs::msg::String();
    double yaw = heading; // 假设 `yaw` 直接等于航向角
    rpy_msg.data = "roll: 0, pitch: 0, yaw: " + std::to_string(yaw);
    // RCLCPP_INFO(this->get_logger(), "Publishing RPY data: %s", rpy_msg.data.c_str());
    rpy_publisher_->publish(rpy_msg);

    // // 更新旋转矩阵
    // rotation_matrix_ = q.toRotationMatrix();
    // RCLCPP_INFO(this->get_logger(), "Received GPS data: Lat=%f, Long=%f, Heading=%f", latitude, longitude, heading);
  }

  // Roll-Pitch-Yaw to Rotation Matrix
  Eigen::Matrix3d eulerAnglesToRotationMatrix(double roll, double pitch, double yaw) {
      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
      Eigen::Matrix3d rotationMatrix = q.matrix();

      return rotationMatrix;
  }

  void other_gps_callback(const gps_msgs::msg::GPSFix::SharedPtr msg) {
    double other_latitude = msg->latitude;
    double other_longitude = msg->longitude;

        // 检查其他车辆的纬度和经度范围
    if (other_latitude < -90.0 || other_latitude > 90.0 || other_longitude < -180.0 || other_longitude > 180.0) {
        RCLCPP_WARN(this->get_logger(), "Received invalid GPS data from other vehicle: Lat=%f, Long=%f", other_latitude, other_longitude);
        return;
    }

    // 将其他车辆的经纬度转换为本地坐标系中的 x_other, y_other, z_other
    double x_other, y_other, z_other;
    local_cartesian_.Forward(other_latitude, other_longitude, 0, x_other, y_other, z_other);

    double yaw;
    if(0<=heading && heading<=90){
      yaw = -(90-heading);
      yaw = yaw*M_PI/180;
    }
    else if(90<heading && heading<360){
      yaw = (heading-90);
      yaw = yaw*M_PI/180;
    }

    Eigen::Matrix3d rotation = eulerAnglesToRotationMatrix(0,0,yaw);
    Eigen::Vector3d vector_other;
    vector_other << x_other, y_other, z_other;
    Eigen::Vector3d vector_transformed = rotation * vector_other;

    double delta_x = vector_transformed.x();
    double delta_y = vector_transformed.y();
    double distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
    RCLCPP_INFO(this->get_logger(), "distance: %f", distance);


    // // 将旋转矩阵应用到目标车辆的坐标上
    // Eigen::Vector3d other_local_coords(x_other, y_other, z_other);
    // Eigen::Vector3d other_rotated_coords = rotation_matrix_ * other_local_coords;

    // // 发布其他车辆坐标
    geometry_msgs::msg::PoseStamped other_pose;
    other_pose.header.stamp = this->get_clock()->now();
    other_pose.header.frame_id = "map";
    other_pose.pose.position.x = vector_transformed[0];
    other_pose.pose.position.y = vector_transformed[1];
    other_pose.pose.position.z = vector_transformed[2];
    other_pose_pub_->publish(other_pose);

    RCLCPP_INFO(this->get_logger(), "Other Vehicle Transformed Local data: x=%f, y=%f, z=%f",
                vector_transformed.x(), vector_transformed.y(), vector_transformed.z());
  }

  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr subscription_;
  // rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr other_subscription_;
  rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr other_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr other_pose_pub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rpy_publisher_; // 新增 RPY 话题发布器

  // GeographicLib对象
  GeographicLib::LocalCartesian local_cartesian_;
  double heading;
  Eigen::Matrix3d rotation_matrix_; // 旋转矩阵
  std::shared_ptr<GeographicLib::Geocentric> earth_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsToXyzNode>());
  rclcpp::shutdown();
  return 0;
}
