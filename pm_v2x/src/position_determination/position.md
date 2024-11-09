position_determinnation 这个包用来判断车辆位置场景等

其中 receive_handle_topic_json_node.cpp 这个文件用来接收处理json数据

但是没有修改好，不影响整个包的编译和运行

我打算重新起一个python的包，专门用来处理json数据，


Haversine公式，也称为球面余弦公式。它是一种用于计算两个GPS坐标点之间大圆距离的算法。
  
  公式连接：[CSDN Haversine公式](https://blog.csdn.net/gaocuisheng/article/details/126060795)
![Haversine公式](/img/haversine.jpg)

## 绿波车速引导思路：
- 第一步：接收车辆的信息（经纬度、速度），接收rsu透传过来的map信息，接收rsu透传过来的spat红绿灯信息
- 第二步：判断车辆在哪个车道上（车辆点与打的点之间的距离，距离在一定范围内，则认为车辆在车道上）
- 第三步：判断完车道，可以通过 map 拿到当前车道上的 phaseID ，将phaseID 与 spat 中的 ID 进行匹配，匹配成功再进行绿波判断
- 第四步：当 ID 成功匹配，使用当前车辆的gps坐标与红绿灯的坐标进行距离计算得到距离（m），将距离（m）/ 当前绿灯的剩余时间（s）得到速度（m/s），将速度（m/s）转换为 km/h 
- 第五步：将 km/h 与 当前车速 进行比较，如果 当前车速 >= km/h ，则可以通过当前红绿灯，如果 当前车速 < km/h ，则不能通过当前红绿灯

<!-- 一个rsu可以发布多红绿灯信息，或者只需要匹配 ID 即可。 -->

BUT 后期可以进行更改

简单做法：
因为现在rsu透传的路口只有一个红绿灯，我们让车辆在固定车道行驶，所以暂时不需要进行前三步（可以做），拿到当前rsu透传过来的spat红绿灯信息和接收，先不用管相位，直接进行第四步和第五步（无法判断车辆在哪个车道上，也就是说所有的车道都会进行绿波判断）【可以后期加上前三步】

绿波：
// 绿波车速引导
// green_wave_node.cpp

/*
{
    'msgCnt': 101, 
    'intersections': 
    [
        {
        'intersectionId': {'region': 500, 'id': 85}, 
        'status': ['0000000000100000'], 
        'phases': 
            [
                {'id': 6, 
                    'phaseStates': [{
                        'light': 3, 
                        'timing': {
                            'counting': {
                                'startTime': 0, 
                                'likelyEndTime': 290, 
                                'nextDuration': 800
                                }
                            }
                        }, 
                        {'light': 6, 
                         'timing': {
                             'counting': {
                                 'startTime': 0, 
                                 'likelyEndTime': 20, 
                                 'nextDuration': 500
                                }
                            }
                        }
                    ]
                }
            ]
        }
    ]
}
*/

// 11、id：定义信号灯相位ID，用于地图匹配。
// 一辆车想知道自己前方的红绿灯，得先通过定位得知自己位于这个路口的哪一条路上，
// 知道自己在哪一条路上了就可以拿到对应的pahseid，就可以倒这来拿信号灯的灯态了。（1、可以通过地图消息中link中的Movement拿phaseId；2、也可以通过link中的lane来拿phaseId）


<!-- 绿波车速引导场景 -->
```c++
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nlohmann/json.hpp"
#include "gps_msgs/msg/gps_fix.hpp"
#include <cmath>
#include <thread>
#include <mutex>


// =========== 里面的红绿灯坐标需要修改一下，修改好了/traffic_light_json 话题发布为4hz的问题 ===========


class GreenWaveNode : public rclcpp::Node
{
public:
    GreenWaveNode() : Node("green_wave_node")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/receive_UDPstring_messages", 10,
            std::bind(&GreenWaveNode::udp_callback, this, std::placeholders::_1));

        gps_subscription_ = this->create_subscription<gps_msgs::msg::GPSFix>(
            "/ui_datapublisher_llahs", 10,
            std::bind(&GreenWaveNode::gps_callback, this, std::placeholders::_1));

        // car_lane_subscription_ = this->create_subscription<std_msgs::msg::String>(
        //     "/vehicle_on_road_info", 10,
        //     std::bind(&GreenWaveNode::car_lane_callback, this, std::placeholders::_1));

        EMERGENCY = this->create_publisher<std_msgs::msg::String>("/scenes", 10);
        traffic_light_pub_ = this->create_publisher<std_msgs::msg::String>("/traffic_light_json", 10);

        // 设置定时器，每100毫秒发布一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GreenWaveNode::publish_traffic_light_json, this));
    }

private:

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

                    for (const auto &phase : intersection["phases"])
                    {
                        int phase_id = phase["id"];

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



    // void car_lane_callback(const std_msgs::msg::String::SharedPtr msg){
    //     vehicle_lane_ = msg->data;
    //     std::string link_name, lane_id, phase_id;
    //     // messages = f"连接名称{link_name}连接车道id{lane_id}相位id{phase_id}"
    //     // [INFO] [1726142168.588222397] [obu_handle_node]: 发布到/vehicle_on_road_info: 连接名称west连接车道id2相位id9

    //     size_t link_pos = vehicle_lane_.find("连接名称");
    //     size_t lane_pos = vehicle_lane_.find("连接车道id");
    //     size_t phase_pos = vehicle_lane_.find("相位id");

    //     if (link_pos != std::string::npos && lane_pos != std::string::npos && phase_pos != std::string::npos){ // 检查这三个字符串是否在有效位置
    //         link_name = vehicle_lane_.substr(link_pos +4,lane_pos-(link_pos +4));
    //         lane_id = vehicle_lane_.substr(lane_pos + 6, phase_pos-(lane_pos + 6));
    //         phase_id = vehicle_lane_.substr(phase_pos + 4);

    //         if (link_name == "west" && lane_id == "2" && phase_id == "9")
    //         {
    //             RCLCPP_INFO(this->get_logger(), "车辆在目标道路上，准备发布红绿灯信息...");
    //             road_condition_valid_ = true;
    //         }else
    //         {
    //             RCLCPP_WARN(this->get_logger(), "车辆不在目标道路上，停止发布红绿灯信息");
    //             road_condition_valid_ = false; 
    //         }
    //     }
    // }


    // void gps_callback(const gps_msgs::msg::GPSFix::SharedPtr msg)
    // {
    //     // if (current_light_ == 6 && first_light_detected_ && road_condition_valid_) 
    //     if (current_light_ == 6 && first_light_detected_) 
    //     {
    //         double lat1 = msg->latitude;
    //         double lon1 = msg->longitude;
    //         double lat2 = 35.999827;
    //         double lon2 = 116.999580;

    //         // latitude: 36.666470078006434
    //         // longitude: 117.66625297699605

    //         double distance = haversine(lat1, lon1, lat2, lon2);
    //         double time_in_seconds = likely_end_time_*0.01;
    //         double required_speed_ms = distance / time_in_seconds;
    //         double required_speed_kmh = required_speed_ms * 3.6;

    //         RCLCPP_INFO(this->get_logger(), "距离红绿灯: %.2f 米, 最低: %.2f km/h 可通过红绿灯",distance, required_speed_kmh);
    //         RCLCPP_INFO(this->get_logger(), "距离红绿灯: %.2f 米, 最低: %.2f m/s 可通过红绿灯",distance, required_speed_ms);


    //         std::string fault_alarm_msg; 
    //         fault_alarm_msg = "绿波车速引导," + std::to_string(required_speed_ms);
    //         // fault_alarm_msg = "绿波车速引导";
    //         RCLCPP_WARN(this->get_logger(), fault_alarm_msg.c_str());
    //         auto message = std_msgs::msg::String();
    //         message.data = fault_alarm_msg;
    //         EMERGENCY->publish(message);
    //         current_light_ = 0;
    //         // first_light_detected_ = false;
    //     }
    // }

    bool has_published_speed = false; // 新增变量，上面的函数与这个一样，
    std::chrono::steady_clock::time_point last_publish_time;

    void gps_callback(const gps_msgs::msg::GPSFix::SharedPtr msg)
    {
        auto now = std::chrono::steady_clock::now();
        
        // 检查是否为绿灯且第一次检测到绿灯或距上次发布时间超过阈值
        if (current_light_ == 6 && first_light_detected_ && 
            (has_published_speed == false || std::chrono::duration_cast<std::chrono::seconds>(now - last_publish_time).count() >= 2)) 
        {
            double lat1 = msg->latitude;
            double lon1 = msg->longitude;
            double lat2 = 35.999827;
            double lon2 = 116.999580;

            double distance = haversine(lat1, lon1, lat2, lon2);
            double time_in_seconds = likely_end_time_ * 0.01;
            double required_speed_ms = distance / time_in_seconds;
            double required_speed_kmh = required_speed_ms * 3.6;

            RCLCPP_INFO(this->get_logger(), "距离红绿灯: %.2f 米, 最低: %.2f km/h 可通过红绿灯", distance, required_speed_kmh);
            RCLCPP_INFO(this->get_logger(), "距离红绿灯: %.2f 米, 最低: %.2f m/s 可通过红绿灯", distance, required_speed_ms);

            std::string fault_alarm_msg;
            fault_alarm_msg = "绿波车速引导," + std::to_string(required_speed_ms);
            RCLCPP_WARN(this->get_logger(), fault_alarm_msg.c_str());
            
            auto message = std_msgs::msg::String();
            message.data = fault_alarm_msg;
            EMERGENCY->publish(message);

            last_publish_time = now; // 更新发布的时间
            has_published_speed = true; // 设置已发布标志
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
        // if (first_light_detected_ && road_condition_valid_) ==========================
        if (first_light_detected_)
        {
            auto traffic_light_message = std_msgs::msg::String();
            traffic_light_message.data = spat_json_.dump();
            traffic_light_pub_->publish(traffic_light_message);
        }
    }

    std::mutex mutex_; // 用于保护共享资源
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr gps_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr car_lane_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr EMERGENCY;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr traffic_light_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string vehicle_lane_;
    bool road_condition_valid_ = true; // 新增变量，用于判断是否在目标道路上

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


// [INFO] [1726146109.141526849] [green_wave_node]: Received message: {"msgCnt": 101, "intersections": [{"intersectionId": {"region": 500, "id": 85}, "status": ["0000000000100000"], "phases": [{"id": 6, "phaseStates": [{"light": 6, "timing": {"counting": {"startTime": 0, "likelyEndTime": 370, "nextDuration": 500}}}, {"light": 7, "timing": {"counting": {"startTime": 370, "likelyEndTime": 570, "nextDuration": 300}}}]}]}]}

// [INFO] [1726145760.147275529] [green_wave_node]: Received message: 
// {"msgCnt": 101, "intersections": [{
//     "intersectionId": {"region": 500, "id": 85}, 
//     "status": ["0000000000100000"], 
//     "phases": [{"id": 6, 
//                 "phaseStates": [{"light": 3, "timing": {"counting": {"startTime": 0, "likelyEndTime": 580, "nextDuration": 800}}}, 
//                                 {"light": 6, "timing": {"counting": {"startTime": 580, "likelyEndTime": 840, "nextDuration": 500}}}]}]}]}


