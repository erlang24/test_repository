"""--1--obu_handlee_node.py 接收 /receive_UDPstring_messages 主题的消息(只提取Map json数据)进行解析
        /receive_UDPstring_messages 这个话题里面是透传的数据，有"json",还有透传的其他车辆的消息
"""

"""--2--增加 另一个功能 接收 /local_messages_b 话题消息(提取gps位置信息)"""

"""--3--根据OBU的位置信息,确定车辆所在的车道(哈弗辛公式)  可以进一步确定车辆在该车道上的相对位置(如接近路口、处于中间或离开路口)
        (当车辆在第四个或第五个点附近的时候，发送透传信息，发送内容相同当前车辆的经纬度和航向角)
        (/send_to_udp_topic1 话题 发到透传)
        (/Vehicle_Map_Intersection 发到场景判断)
"""


# ros2 run obu_json obu_handle_node

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from sensor_msgs.msg import NavSatFix
from gps_msgs.msg import GPSFix
import json
import math
import hashlib


class UDPMessageReceiver(Node):
    def __init__(self):
        super().__init__('obu_handle_node')
        self.subscription = self.create_subscription(String, '/receive_UDPstring_messages', self.listener_callback, 10)
        self.gps_subscription = self.create_subscription(GPSFix, '/ui_datapublisher_llahs', self.gps_callback, 10)
        self.publisher_intersection = self.create_publisher(String, '/Vehicle_Map_Intersection', 10)
        self.publisher_udp = self.create_publisher(String, '/send_to_udp_topic1', 10)
        self.road_info_pub = self.create_publisher(String, '/vehicle_on_road_info', 10)  # 发布在指定道路上的车辆信息

        self.publisher_lukou_ = self.create_publisher(String, '/lukou_center',10)

        self.global_json_data = None
        self.phase_id = None
        self.timer = self.create_timer(0.05,self.publish_road_info)

        self.current_gps = None
        self.current_heading = None
        

        self.get_logger().info("节点已启动，等待消息连接...")

    def listener_callback(self, msg):
        message = msg.data
        self.get_logger().info(f"收到透传过来的消息: {message}")
        try:
            self.global_json_data = json.loads(message)  # 尝试解析JSON数据
            received_hash = hashlib.md5(message.encode('utf-8')).hexdigest()
            self.get_logger().info(f"接收到的数据哈希值: {received_hash}")

            # 解析ASN
            self.parse_asn_info(self.global_json_data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"收到的消息无法解析为JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"处理消息时发生错误: {e}")

    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        self.current_heading = msg.track
        latitude = msg.latitude
        longitude = msg.longitude
        heading = msg.track
        self.get_logger().info(f"本车OBU车辆GPS位置: 纬度: {latitude}, 经度: {longitude}, 航向角: {heading}")


# [INFO] [1725090778.886460748] [obu_handle_node]: OBU车辆GPS位置: 纬度: 36.665985486986514, 经度: 117.66627453112797, 航向角: 2.952406044685487
# [INFO] [1725090778.891974724] [obu_handle_node]: 收到消息: Lat:36.666042,Lon:117.666264,Heading:0.486583,Speed:0.000000



    def parse_asn_info(self, json_data):
        """解析ASN编码的十字路口信息。"""
        try:
            nodes = json_data.get("MessageFrame", {}).get("mapFrame", {}).get("nodes", {}).get("Node", {})
            node_name = nodes.get("name")
            ref_pos = nodes.get("refPos", {})
            lat = ref_pos.get("lat")
            long = ref_pos.get("long")

            if lat is not None and long is not None:
                lat = self.convert_lat_long(lat)
                long = self.convert_lat_long(long, is_longitude=True)

                self.get_logger().info(f"十字路口名称: {node_name}, 参考位置: (纬度: {lat}, 经度: {long})")
                log_message = f"十字路口经度:{long},十字路口纬度:{lat}"
                self.publisher_lukou_.publish(String(data = log_message))
            
            else:
                self.get_logger().warning("十字路口经纬度信息未获取到。")


            in_links = nodes.get("inLinks", {}).get("Link", [])
            closest_lane = None
            closest_lane_info = {}

            for link in in_links:
                link_name = link.get("name")
                # upstreamNodeId 
                # linkWidth
                # movements
                # points

                # lanes = link.get("lanes", {}).get("Lane", {})
                lanes = link.get("lanes", {}).get("Lane", [])
                if isinstance(lanes, dict):
                    lanes = [lanes]
                    
                speed_limit = link.get("speedLimits", {}).get("RegulatorySpeedLimit", {}).get("speed")
                max_speed_kmh = round((int(speed_limit) * 0.02 * 3.6)) if speed_limit else None  # 转换为 km/h

                self.get_logger().info(f"链接名称: {link_name}, 速度限制: {max_speed_kmh} km/h")

                for lane in lanes:
                    lane_id = lane.get("laneId")
                    lane_width = lane.get("laneWidth")
                    maneuvers = lane.get("maneuvers")
                    self.get_logger().info(f"  车道ID: {lane_id}, 宽度: {lane_width}, 操作: {maneuvers}")

                    closest_point = None
                    closest_distance = float('inf')
                    point_index = None

                    points = lane.get("points", {}).get("RoadPoint", [])
                    for idx, roadpoint in enumerate(points, start=1):
                        position = roadpoint.get("posOffset", {}).get("offsetLL", {}).get("position-LatLon", {})
                        lon = position.get("lon")
                        lat = position.get("lat")

                        if lon and lat:
                            lon = self.convert_lat_long(lon, is_longitude=True)
                            lat = self.convert_lat_long(lat)

                            # 计算距离
                            distance = self.calculate_distance(self.current_gps, (lat, lon))
                            if distance < closest_distance:
                                closest_distance = distance
                                closest_point = (lat, lon)
                                point_index = idx  # 正确的点索引

                            # 打印每个车道的点和距离
                            self.get_logger().info(f"    车道ID: {lane_id}, 第{idx}个点位置: (纬度: {lat}, 经度: {lon}), 距离: {distance:.2f} 米")

                            # 可以将固定的点范围设置成得到的款的范围 lane_width
                            # 检查车辆是否在第5个或第4个点附近
                            # if (idx == 5 and distance < 10) or (idx == 4 and distance < 5):
                            if (idx == 5 and distance < lane_width/100) or (idx == 4 and lane_width/100):
                                self.publish_vehicle_info()

                            # 车辆在车道上
                            if (distance < lane_width/100):
                                self.publish_road_info()

                    if closest_point and closest_distance < 10:
                        if not closest_lane or closest_distance < closest_lane_info.get('distance', float('inf')):
                            closest_lane = lane_id
                            closest_lane_info = {
                                'lane_id': lane_id,
                                'lane_name': link_name,
                                'point_index': point_index,  # 使用正确的点索引
                                'distance': closest_distance,
                                'position': closest_point
                            }
                    
                    connects_to = lane.get("connectsTo", {}).get("Connection", [])
                    for connection in connects_to:
                        remote_intersection = connection.get("remoteIntersection", {})
                        phase_id = connection.get("phaseId")
                        region_id = remote_intersection.get("region")
                        road_id = remote_intersection.get("id")
                        connecting_lane = connection.get("connectingLane", {}).get("lane")
                        connecting_maneuver = connection.get("connectingLane", {}).get("maneuver")
                        self.phase_id = phase_id
                        self.get_logger().info(f"    连接到下游路段: {road_id} (区域: {region_id}), 当前相位ID: {phase_id}, 连接车道: {connecting_lane}, 操作: {connecting_maneuver}")


                    # # 判断是否位于 west 连接的 车道2 
                    # if link_name == "west" and lane_id == "2":
                    #     messages = f"连接名称{link_name}连接车道id{lane_id}相位id{phase_id}"
                    #     self.publish_road_info(messages)
                    #     print(f"发布的messages::{messages}")
                        
                        

            if closest_lane:
                self.get_logger().info(f"OBU车辆位于车道: {closest_lane_info['lane_name']} (车道ID: {closest_lane_info['lane_id']}), 在第{closest_lane_info['point_index']}个点附近, 距离: {closest_lane_info['distance']:.2f} 米")
            else:
                self.get_logger().info("OBU车辆未找到匹配的车道")

        except KeyError as e:
            self.get_logger().error(f"解析ASN信息时发生错误: {e}")

    def publish_vehicle_info(self):
        """发布车辆的GPS位置和航向角信息"""
        if self.current_gps and self.current_heading is not None:
            latitude, longitude = self.current_gps
            heading = self.current_heading
            message = f"GPS位置: 纬度: {latitude}, 经度: {longitude}, 航向角: {heading}"

            # 发布到 /Vehicle_Map_Intersection
            msg = String()
            msg.data = message
            self.publisher_intersection.publish(msg)
            self.get_logger().info(f"发布到/Vehicle_Map_Intersection: {message}")
            # [INFO] [1726142168.588222397] [obu_handle_node]: 发布到/vehicle_on_road_info: 连接名称west连接车道id2相位id9

            # 发布到 /send_to_udp_topic1
            self.publisher_udp.publish(msg)
            self.get_logger().info(f"发布到/send_to_udp_topic1: {message}")


    def publish_road_info(self):
        """发布车辆所在指定道路的信息"""
        if self.phase_id is not None:
            carlane_phase_id = self.phase_id
            message = f"车道相位id:{carlane_phase_id}"
            msg = String()
            msg.data = message
            self.road_info_pub.publish(msg.data)
            self.get_logger().info("aaaaaaaaaaaaaaa车辆在指定道路上")

    

    def calculate_distance(self, gps1, gps2):
        """计算两点之间的距离（米）"""
        if gps1 and gps2:
            lat1, lon1 = gps1
            lat2, lon2 = gps2
            R = 6371000  # 地球半径（米）
            phi1 = math.radians(lat1)
            phi2 = math.radians(lat2)
            delta_phi = math.radians(lat2 - lat1)
            delta_lambda = math.radians(lon2 - lon1)
            a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
            return R * c  # 返回距离（米）
        return float('inf')


    # # 直接将 json 里面的经纬度进行了小数点偏移，比如说(纬度: 35.99978, 经度: 116.999175)
    # # 但是需要将经纬度格式进行对应


    def convert_lat_long(self, value, is_longitude=False):
        if is_longitude:
            return float(f"{value[:3]}.{value[3:]}")
        else:
            return float(f"{value[:2]}.{value[2:]}")

    # def convert_lat_long(self, value, is_longitude=False):
    #     if is_longitude:
    #         # 先进行小数点偏移5位
    #         value = float(f"{value[:5]}.{value[5:]}")
    #         if value > 117:
    #             return math.floor(value / 100) + (value - math.floor(value / 100) * 100) / 60
    #         else:
    #             return math.floor(value / 100) + ((value - math.floor(value / 100) * 100) - 40) / 100
    #     else:
    #         # 先进行小数点偏移4位
    #         value = float(f"{value[:4]}.{value[4:]}")
    #         if value > 36:
    #             return math.floor(value / 100) + (value - math.floor(value / 100) * 100) / 60
    #         else:
    #             return math.floor(value / 100) + ((value - math.floor(value / 100) * 100) - 40) / 100


def main(args=None):
    rclpy.init(args=args)
    udp_message_receiver = UDPMessageReceiver()

    try:
        rclpy.spin(udp_message_receiver)
    except KeyboardInterrupt:
        pass
    finally:
        udp_message_receiver.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()




# ***************************************************************************************************************************************

