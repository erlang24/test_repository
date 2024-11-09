// // 测试用的节点

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>

// class GPSToXYZNode : public rclcpp::Node
// {
// public:
//   GPSToXYZNode() : Node("gps_to_xyz_node")
//   {
//     // 创建订阅者,订阅 /local_pose 话题
//     local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "/local_pose", 10, std::bind(&GPSToXYZNode::localPoseCallback, this, std::placeholders::_1));

//     // 创建订阅者,订阅 /other_pose 话题
//     other_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "/other_pose", 10, std::bind(&GPSToXYZNode::otherPoseCallback, this, std::placeholders::_1));
//   }

// private:
//   void localPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//   {
//     // 打印 /local_pose 话题的内容
//     RCLCPP_INFO(this->get_logger(), "Received local pose:");
//     RCLCPP_INFO(this->get_logger(), "  Position: (%.3f, %.3f, %.3f)", 
//                 msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     RCLCPP_INFO(this->get_logger(), "  Orientation: (%.3f, %.3f, %.3f, %.3f)",
//                 msg->pose.orientation.x, msg->pose.orientation.y, 
//                 msg->pose.orientation.z, msg->pose.orientation.w);
//   }

//   void otherPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
//   {
//     // 打印 /other_pose 话题的内容
//     RCLCPP_INFO(this->get_logger(), "Received other pose:");
//     RCLCPP_INFO(this->get_logger(), "  Position: (%.3f, %.3f, %.3f)",
//                 msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     RCLCPP_INFO(this->get_logger(), "  Orientation: (%.3f, %.3f, %.3f, %.3f)",
//                 msg->pose.orientation.x, msg->pose.orientation.y,
//                 msg->pose.orientation.z, msg->pose.orientation.w);
//   }

//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr other_pose_sub_;
// };

// // [INFO]  Received local pose:
// // [INFO]    Position: (0.000, 0.000, 0.000) 
// // [INFO]    Orientation: (-0.000, -0.000, -0.643, 0.766)
// // [INFO]  Received other pose:
// // [INFO]    Position: (13.630, 1.296, -0.000)
// // [INFO]    Orientation: (0.000, 0.000, 0.000, 1.000)

// /*
// local_pose:
//   位置：xyz都为0
//   方向：四元数表示旋转，由实部和三个虚部组成，x,y,z分别表示绕x,y,z轴旋转的角度，w表示旋转的方向。

// other_pose:
//   位置：x,y,z
//   方向：在任何轴上没有旋转，实部是1，x，y，z轴的旋转分量都为0。

// */

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<GPSToXYZNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }