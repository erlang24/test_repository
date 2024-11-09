#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include <cmath>
#include <thread>
#include <mutex>



struct Intersection{ // 路口
    double latitude;
    double longitude;
};

class GreenWaveNode : public rclcpp::Node
{
public:
    GreenWaveNode() : Node("green_wave_node")
    {

        // receive_UDPstring_messages
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/light_json_string", 10,
            std::bind(&GreenWaveNode::udp_callback, this, std::placeholders::_1));

        gps_subscription_ = this->create_subscription<gps_msgs::msg::GPSFix>(
            "/ui_datapublisher_llahs", 10,
            std::bind(&GreenWaveNode::gps_callback, this, std::placeholders::_1));

        lukou_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/lukou_center",10,
            std::bind(&GreenWaveNode::lukou_callback, this, std::placeholders::_1));

        car_lane_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/vehicle_on_road_info", 10,
            std::bind(&GreenWaveNode::car_lane_callback, this, std::placeholders::_1));

        EMERGENCY = this->create_publisher<std_msgs::msg::String>("/scenes", 10);
        traffic_light_pub_ = this->create_publisher<std_msgs::msg::String>("/traffic_light_json", 10);

        road_condition_valid_ = false; // 初始化为false
        last_lane_msg_time_ = std::chrono::steady_clock::now();

        // 设置定时器，每100毫秒发布一次 50
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&GreenWaveNode::publish_traffic_light_json, this));
    }

private:

    void lukou_callback(const std_msgs::msg::String::SharedPtr msg) {
        double lat, lon;
        int matched = sscanf(msg->data.c_str(), "十字路口经度:%lf,十字路口纬度:%lf", &lon, &lat);
        // RCLCPP_INFO(this->get_logger(), "解析经纬度成功: %s", msg->data.c_str());
        if (matched == 2) {
            intersection_.latitude = lat;
            intersection_.longitude = lon;
        } else {
            // 输出调试信息
            RCLCPP_WARN(this->get_logger(), "解析经纬度失败: %s", msg->data.c_str());
            intersection_.latitude = intersection_.longitude = std::numeric_limits<double>::quiet_NaN();
        }
    }


    void udp_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::thread json_thread([this, msg]() {
            std::lock_guard<std::mutex> lock(mutex_); // 加锁

            try
            {
                spat_json_ = nlohmann::json::parse(msg->data);
                current_light_ = 0; // 每次收到新消息时重置

                for (const auto &intersection : spat_json_["intersections"])
                {
                    int intersection_id = intersection["intersectionId"]["id"];
                    phase_id = intersection_id;
                    std::cout << "@@@phase_id: " << phase_id << std::endl;

                    for (const auto &phase : intersection["phases"])
                    {
                        // phase_id = phase["id"];
                        // std::cout << "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@phase_id: " << phase_id << std::endl;

                        for (const auto &state : phase["phaseStates"])
                        {
                            int light = state["light"];
                            int start_time = state["timing"]["counting"]["startTime"];
                            int likely_end_time = state["timing"]["counting"]["likelyEndTime"];

                            // 判断绿灯 (light == 6)，并且没有检测过
                            
                            if (light == 6 && start_time == 0)
                            // if (light == 6 && start_time == 0 && !first_light_detected_)
                            {
                                current_light_ = light;
                                likely_end_time_ = likely_end_time;
                                first_light_detected_ = true;
                                RCLCPP_INFO(this->get_logger(), "Detected green light: %d, Likely end time: %d", current_light_, likely_end_time_);
                            }
                        }
                    }
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
            }
        });
        json_thread.detach();
    }

    bool has_published_speed = false; // 是否已经发布速度的标志
    std::chrono::steady_clock::time_point last_publish_time;
    double fixed_speed_ms = 0.0; // 固定发布的速度


    void car_lane_callback(const std_msgs::msg::String::SharedPtr msg){
        last_lane_msg_time_ = std::chrono::steady_clock::now();
        std::string carlane_phaseID = msg->data.c_str();

        size_t pos = carlane_phaseID.find(":");
        if(pos != std::string::npos){
            std::string car_lane_phaseID_number = carlane_phaseID.substr(pos + 1);

            std::cout<<"!!!!!!!!!!!carlane_light_phaseID : " << carlane_phaseID << std::endl;
            std::cout<<"!!!!!!!!!!!phase_id : " << std::to_string(phase_id) << std::endl;


            if (car_lane_phaseID_number == std::to_string(phase_id)){
                road_condition_valid_ = true;
            }else{
                road_condition_valid_ = false;
            }
        }
    }



    void gps_callback(const gps_msgs::msg::GPSFix::SharedPtr msg)
    {
        // auto now = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        if(std::chrono::duration_cast<std::chrono::seconds>(
            current_time - last_lane_msg_time_).count() > 1) {
            road_condition_valid_ = false;
        }

        
        // 检查是否为绿灯且第一次检测到绿灯或距上次发布时间超过阈值
        if (current_light_ == 6 && first_light_detected_ && 
            (has_published_speed == false || std::chrono::duration_cast<std::chrono::seconds>(current_time - last_publish_time).count() >= 2)) 
        {
            double lat1 = msg->latitude;
            double lon1 = msg->longitude;
            // double lat2 = 35.999827;
            // double lon2 = 116.999580;
            // double lat2 = 36.75198239810366;
            // double lon2 = 117.25557561999338;
            double lat2 = intersection_.latitude;
            double lon2 = intersection_.longitude;

            double distance = haversine(lat1, lon1, lat2, lon2);
            double time_in_seconds = likely_end_time_ * 0.1;

            // std::cout<<"likel+++++++++++++++++++++_end_time_: "<<likely_end_time_<<std::endl;
            fixed_speed_ms = distance / time_in_seconds; // 保存固定速度
            double required_speed_kmh = fixed_speed_ms * 3.6;

            RCLCPP_INFO(this->get_logger(), "距离红绿灯: %.2f 米, 最低: %.2f km/h 可通过红绿灯", distance, required_speed_kmh);
            RCLCPP_INFO(this->get_logger(), "距离红绿灯: %.2f 米, 最低: %.2f m/s 可通过红绿灯", distance, fixed_speed_ms);

            last_publish_time = current_time; // 更新发布时间
            has_published_speed = true; // 设置已发布标志
        }

        std::cout << "***has_published_speed: " << has_published_speed << std::endl;
        std::cout << "***road_condition_valid_: " << road_condition_valid_ << std::endl;
        std::cout << "***time diff: " << std::chrono::duration_cast<std::chrono::seconds>(current_time - last_publish_time).count() << std::endl;
        
        // 持续两秒发布固定速度
        if (has_published_speed && road_condition_valid_ && std::chrono::duration_cast<std::chrono::seconds>(current_time - last_publish_time).count() < 2)
        {
            std::string fault_alarm_msg;
            fault_alarm_msg = "绿波车速引导," + std::to_string(fixed_speed_ms);
            RCLCPP_WARN(this->get_logger(), fault_alarm_msg.c_str());
            
            auto message = std_msgs::msg::String();
            message.data = fault_alarm_msg;
            EMERGENCY->publish(message);
        }
        else if (current_light_ != 6) {
            has_published_speed = false; // 如果不是绿灯，重置标志
        }
    }


    double haversine(double lat1, double lon1, double lat2, double lon2) const
    {
        const double R = 6371000;
        double dlat = (lat2 - lat1) * M_PI / 180.0;
        double dlon = (lon2 - lon1) * M_PI / 180.0;

        lat1 = lat1 * M_PI / 180.0;
        lat2 = lat2 * M_PI / 180.0;

        double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::sin(dlon / 2) * std::sin(dlon / 2) * std::cos(lat1) * std::cos(lat2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

        return R * c;
    }

    void publish_traffic_light_json()
    {
        std::lock_guard<std::mutex> lock(mutex_); // 加锁
        if (first_light_detected_ && road_condition_valid_) // ==========================
        // if (first_light_detected_)
        {
            auto traffic_light_message = std_msgs::msg::String();
            traffic_light_message.data = spat_json_.dump();
            traffic_light_pub_->publish(traffic_light_message);
        }
    }

    std::mutex mutex_; // 用于保护共享资源
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lukou_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr car_lane_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr EMERGENCY;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr traffic_light_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string vehicle_lane_;
    int phase_id;
    bool road_condition_valid_ = false; // 新增变量，用于判断是否在目标道路上

    std::chrono::steady_clock::time_point last_lane_msg_time_;

    Intersection intersection_; // 成员变量

    nlohmann::json spat_json_;
    int current_light_ = 0;
    int likely_end_time_ = 0;
    bool first_light_detected_  = false;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GreenWaveNode>());
    rclcpp::shutdown();
    return 0;
}