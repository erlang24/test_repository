
// 我忘了这个文件是干啥的了，当时写的接受这个json进行处理，然后呢，这个仅仅就是接收然后处理？？？？？
// 知道了，看.md写了

#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <openssl/md5.h>
#include <iomanip>
#include <sstream>

using json = nlohmann::json;

class UDPMessageReceiver : public rclcpp::Node {
public:
    UDPMessageReceiver() : Node("udp_message_receiver") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/receive_UDPstring_messages",
            10,
            std::bind(&UDPMessageReceiver::listener_callback, this, std::placeholders::_1)
        );
    }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string message = msg->data;
        std::cout << "收到消息: " << message << std::endl;

        if (!is_valid_json(message)) {
            std::cout << "收到的数据不是完整的JSON格式，忽略..." << std::endl;
            return;
        }

        try {
            json json_data = json::parse(message);
            std::string received_hash = calculate_md5(message);

            std::cout << "收到有效的JSON数据: " << json_data.dump() << std::endl;
            std::cout << "接收到的数据哈希值: " << received_hash << std::endl;

            parse_asn_info(json_data);
        } catch (json::parse_error& e) {
            std::cout << "解析JSON时发生错误: " << e.what() << std::endl;
        }
    }

    bool is_valid_json(const std::string& message) {
        return message.front() == '{' && message.back() == '}';
    }

    std::string calculate_md5(const std::string& message) {
        unsigned char hash[MD5_DIGEST_LENGTH];
        MD5(reinterpret_cast<const unsigned char*>(message.c_str()), message.length(), hash);
        std::ostringstream ss;
        for (int i = 0; i < MD5_DIGEST_LENGTH; ++i) {
            ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hash[i]);
        }
        return ss.str();
    }

    void parse_asn_info(const json& json_data) {
        try {
            auto nodes = json_data["MessageFrame"]["mapFrame"]["nodes"]["Node"];
            std::string node_name = nodes["name"];
            auto ref_pos = nodes["refPos"];
            double lat = ref_pos["lat"];
            double long_val = ref_pos["long"];

            std::cout << "十字路口名称: " << node_name << ", 参考位置: (纬度: " << lat << ", 经度: " << long_val << ")" << std::endl;

            for (const auto& link : nodes["inLinks"]["Link"]) {
                parse_link_info(link);
            }
        } catch (const std::exception& e) {
            std::cout << "解析ASN信息时发生错误: " << e.what() << std::endl;
        }
    }

    void parse_link_info(const json& link) {
        std::string link_name = link["name"];
        auto lanes = link["lanes"]["Lane"];
        int speed_limit = link["speedLimits"]["RegulatorySpeedLimit"]["speed"];
        double max_speed_kmh = (speed_limit > 0) ? round((speed_limit * 0.02 * 3.6)) : 0;

        std::cout << "链接名称: " << link_name << ", 速度限制: " << max_speed_kmh << " km/h" << std::endl;

        for (const auto& lane : lanes) {
            parse_lane_info(lane);
        }
    }

    void parse_lane_info(const json& lane) {
        std::string lane_id = lane["laneId"];
        double lane_width = lane["laneWidth"];
        auto maneuvers = lane["maneuvers"];
        std::cout << "  车道ID: " << lane_id << ", 宽度: " << lane_width << ", 操作: " << maneuvers << std::endl;

        for (const auto& connection : lane["connectsTo"]["Connection"]) {
            parse_connection_info(connection);
        }
    }

    void parse_connection_info(const json& connection) {
        auto remote_intersection = connection["remoteIntersection"];
        std::string phase_id = connection["phaseId"];
        std::string region_id = remote_intersection["region"];
        std::string road_id = remote_intersection["id"];
        std::string connecting_lane = connection["connectingLane"]["lane"];
        std::string connecting_maneuver = connection["connectingLane"]["maneuver"];

        std::cout << " 连接到下游路段: " << road_id << " (区域: " << region_id << "), 当前相位ID: " << phase_id
                  << ", 连接车道: " << connecting_lane << ", 操作: " << connecting_maneuver << std::endl;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDPMessageReceiver>());
    rclcpp::shutdown();
    return 0;
}


