#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <raw_sensor_msgs/msg/chanavgnss.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/quaternion.hpp>

// #include <tf/transform_broadcaster.h>
// #include <tf/transform_datatypes.h>
// #include <geometry_msgs/Quaternion.h>

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <algorithm>

#define GPSWeek   2
#define GPSTime   3
#define Heading   4
#define Pitch     5
#define Roll      6
#define Gyro_x    7
#define Gyro_y    8
#define Gyro_z    9
#define Acc_x     10
#define Acc_y     11
#define Acc_z     12
#define Latitude  13
#define Longitude 14
#define Altitude  15
#define VE        16
#define VN        17
#define VU        18
#define Velocity  19
#define NSV1      20
#define NSV2      21
#define Status    22
#define Age       23
#define Space     24

serial::Serial ser;

sensor_msgs::msg::Imu imu_msg;
sensor_msgs::msg::NavSatFix gps_msg;
raw_sensor_msgs::msg::Chanavgnss gnssimuinfo;

class ChanavNode : public rclcpp::Node
{
public:
    ChanavNode() : Node("chanav_node")
    {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        declare_parameter<int>("baud_rate", 115200);

        port_ = get_parameter("port").as_string();
        baud_rate_ = get_parameter("baud_rate").as_int();

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 200);
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 200);
        gnss_ins_pub_ = this->create_publisher<raw_sensor_msgs::msg::Chanavgnss>("gnss_ins", 1000);

        openSerialPort();
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ChanavNode::readSerialData, this));
    }

private:
    void openSerialPort()
    {
        try
        {
            ser.setPort(port_);
            ser.setBaudrate(baud_rate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port %s. Retrying...", port_.c_str());
            rclcpp::sleep_for(std::chrono::seconds(5));
            openSerialPort();
        }
        if (ser.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "Serial port %s initialized.", port_.c_str());
        }
    }

    void readSerialData()
    {
        if (ser.isOpen() && ser.available())
        {
            std::string read = ser.read(ser.available());
            str_.append(read);
            if (str_.size() > 350)
            {
                unsigned int loc1 = str_.find("$GPCHC", 0);
                if (loc1 > 350)
                {
                    RCLCPP_INFO(this->get_logger(), "Read GPCHC fail.");
                    str_.clear();
                    return;
                }
                unsigned int loc3 = str_.find("$GPCHC", loc1 + 1);
                if (loc3 > 350)
                {
                    str_.clear();
                    return;
                }

                parseData(str_.substr(loc1, loc3 - loc1));
                str_.erase(0, loc3);
            }
        }
    }

    void parseData(const std::string &data)
    {
        char *serial_Ctype;
        int len = data.length();
        serial_Ctype = (char *)malloc((len + 1) * sizeof(char));
        data.copy(serial_Ctype, len, 0);
        serial_Ctype[len] = '\0';

        char *ptr = strtok(serial_Ctype, ",");
        int i = 0;
        double gpsweek, gpstime, yaw, pitch, roll, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, latitude, longitude, altitude, ve, vn, vu, velocity, nsv1, nsv2, status, age, space;

        while (ptr != NULL && i < Space + 1)
        {
            i++;
            switch (i)
            {
            case GPSWeek:
                gpsweek = atoi(ptr);
                break;
            case GPSTime:
                gpstime = atof(ptr);
                break;
            case Heading:
                yaw = atof(ptr);
                break;
            case Pitch:
                pitch = atof(ptr);
                break;
            case Roll:
                roll = atof(ptr);
                break;
            case Gyro_x:
                gyro_x = atof(ptr);
                break;
            case Gyro_y:
                gyro_y = atof(ptr);
                break;
            case Gyro_z:
                gyro_z = atof(ptr);
                break;
            case Acc_x:
                acc_x = atof(ptr);
                break;
            case Acc_y:
                acc_y = atof(ptr);
                break;
            case Acc_z:
                acc_z = atof(ptr);
                break;
            case Latitude:
                latitude = atof(ptr);
                break;
            case Longitude:
                longitude = atof(ptr);
                break;
            case Altitude:
                altitude = atof(ptr);
                break;
            case VE:
                ve = atof(ptr);
                break;
            case VN:
                vn = atof(ptr);
                break;
            case VU:
                vu = atof(ptr);
                break;
            case Velocity:
                velocity = atof(ptr);
                break;
            case NSV1:
                nsv1 = atof(ptr);
                break;
            case NSV2:
                nsv2 = atof(ptr);
                break;
            case Status:
                status = atoi(ptr);
                break;
            case Age:
                age = atof(ptr);
                break;
            case Space:
                space = atof(ptr);
                break;
            }
            ptr = strtok(NULL, ",");
        }
        free(serial_Ctype);

        publishImu(roll, pitch, yaw, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z);
        publishGps(latitude, longitude, altitude);
        publishGnss(latitude, longitude, altitude, ve, vn, vu, velocity, acc_x, acc_y, acc_z, roll, pitch, yaw, nsv1, nsv2);
    }

    void publishImu(double roll, double pitch, double yaw, double gyro_x, double gyro_y, double gyro_z, double acc_x, double acc_y, double acc_z)
    {
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";

        tf2::Quaternion imu_orientation;
        imu_orientation.setRPY(roll, pitch, yaw);
        imu_msg.orientation = tf2::toMsg(imu_orientation);

        imu_msg.angular_velocity.x = gyro_x;
        imu_msg.angular_velocity.y = gyro_y;
        imu_msg.angular_velocity.z = gyro_z;

        imu_msg.linear_acceleration.x = acc_x;
        imu_msg.linear_acceleration.y = acc_y;
        imu_msg.linear_acceleration.z = acc_z;

        imu_pub_->publish(imu_msg);
    }

    void publishGps(double latitude, double longitude, double altitude)
    {
        gps_msg.header.stamp = this->get_clock()->now();
        gps_msg.header.frame_id = "gps_link";
        gps_msg.latitude = latitude;
        gps_msg.longitude = longitude;
        gps_msg.altitude = altitude;

        gps_pub_->publish(gps_msg);
    }

void publishGnss(double latitude, double longitude, double altitude,
                 double ve, double vn, double vu, double velocity,
                 double acc_x, double acc_y, double acc_z,
                 double roll, double pitch, double yaw,
                 uint32_t nsv1, uint32_t nsv2)
{
    gnssimuinfo.none_gps_data = 0;
    gnssimuinfo.none_vehicle_data = 0;
    gnssimuinfo.error_gyroscope = 0;
    gnssimuinfo.error_accelerometer = 0;

    gnssimuinfo.header.stamp = this->get_clock()->now(); // 使用ROS2的时间库
    gnssimuinfo.header.frame_id = "base_link";
    gnssimuinfo.latitude = latitude;
    gnssimuinfo.longitude = longitude;
    gnssimuinfo.altitude = altitude;
    gnssimuinfo.east_velocity = ve;
    gnssimuinfo.north_velocity = vn;
    gnssimuinfo.up_velocity = vu;
    gnssimuinfo.velocity = velocity;
    gnssimuinfo.ax = acc_x;
    gnssimuinfo.ay = acc_y;
    gnssimuinfo.az = acc_z;
    gnssimuinfo.roll = roll;
    gnssimuinfo.pitch = pitch;
    gnssimuinfo.yaw = yaw;
    gnssimuinfo.x = imu_msg.orientation.x;
    gnssimuinfo.y = imu_msg.orientation.y;
    gnssimuinfo.z = imu_msg.orientation.z;
    gnssimuinfo.w = imu_msg.orientation.w;
    gnssimuinfo.nsv1_num = nsv1;
    gnssimuinfo.nsv2_num = nsv2;
    gnss_ins_pub_->publish(gnssimuinfo);
}

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<raw_sensor_msgs::msg::Chanavgnss>::SharedPtr gnss_ins_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string port_;
    int baud_rate_;
    std::string str_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChanavNode>());
    rclcpp::shutdown();
    return 0;
}
