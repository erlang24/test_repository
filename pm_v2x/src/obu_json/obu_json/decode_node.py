# 接收 /Uint8_encode 话题消息，解码提取 Map Light vehicle

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import String
import asn1tools
from gps_msgs.msg import GPSFix
import math
import json
from geometry_msgs.msg import Twist


class UdpArrayDecoderNode(Node):
    def __init__(self):
        super().__init__('decode_node')
        
        # Create a subscriber for the UInt8MultiArray messages
        # Uint8_encode  encoded_bsm_data rsu_map
        self.subscription = self.create_subscription(UInt8MultiArray,'/Uint8_encode',self.listener_callback,10)  # Uint8_encode
        self.gps_subscription = self.create_subscription(GPSFix, '/ui_datapublisher_llahs', self.gps_callback, 10)
        self.publisher_intersection = self.create_publisher(String, '/Vehicle_Map_Intersection', 10)
        # self.publisher_udp = self.create_publisher(String, '/send_to_udp_topic1', 10)
        # self.publihser_gpsfix_five = self.create_publisher(GPSFix, '/vehicleToencode', 10)
        self.road_info_pub = self.create_publisher(String, '/vehicle_on_road_info', 10)

        self.publisher_lukou_ = self.create_publisher(String, '/lukou_center',10)

        self.publisher_car_message = self.create_publisher(GPSFix, '/other_car_message', 10) # 其他车辆的经纬度换成这个话题
        self.publisher_speed_ms = self.create_publisher(Twist, '/other_speed_b', 10)

        self.light_string_publisher = self.create_publisher(String, '/light_json_string', 10)

        self.publisher_fault = self.create_publisher(String, '/fault_alarm', 10) # 解码完成发送故障报警
        self.publisher_emergency = self.create_publisher(String, '/emergency_alarm', 10) # 解码完成发送紧急报警

        # Compile ASN.1 files
        self.asn1_spec = asn1tools.compile_files([
            '/home/erlang/vehicleToEverything/asn/BSM.asn',
            '/home/erlang/vehicleToEverything/asn/MapSpeedLimit.asn',
            '/home/erlang/vehicleToEverything/asn/VehClass.asn',
            '/home/erlang/vehicleToEverything/asn/RSM.asn',
            '/home/erlang/vehicleToEverything/asn/DefPosition.asn',
            '/home/erlang/vehicleToEverything/asn/MapLink.asn',
            '/home/erlang/vehicleToEverything/asn/MapPoint.asn',
            '/home/erlang/vehicleToEverything/asn/MapNode.asn',
            '/home/erlang/vehicleToEverything/asn/DefTime.asn',
            '/home/erlang/vehicleToEverything/asn/DefMotion.asn',
            '/home/erlang/vehicleToEverything/asn/MapLane.asn',
            '/home/erlang/vehicleToEverything/asn/VehSize.asn',
            '/home/erlang/vehicleToEverything/asn/MsgFrame.asn',
            '/home/erlang/vehicleToEverything/asn/Map.asn',
            '/home/erlang/vehicleToEverything/asn/VehSafetyExt.asn',
            '/home/erlang/vehicleToEverything/asn/DefPositionOffset.asn',
            '/home/erlang/vehicleToEverything/asn/RSI.asn',
            '/home/erlang/vehicleToEverything/asn/VehStatus.asn',
            '/home/erlang/vehicleToEverything/asn/DefAcceleration.asn',
            '/home/erlang/vehicleToEverything/asn/VehEmgExt.asn',
            '/home/erlang/vehicleToEverything/asn/VehBrake.asn',
            '/home/erlang/vehicleToEverything/asn/SPATIntersectionState.asn',
            '/home/erlang/vehicleToEverything/asn/SignalPhaseAndTiming.asn',

            # '/home/promote/vehicleToEverything/asn/BSM.asn',
            # '/home/promote/vehicleToEverything/asn/MapSpeedLimit.asn',
            # '/home/promote/vehicleToEverything/asn/VehClass.asn',
            # '/home/promote/vehicleToEverything/asn/RSM.asn',
            # '/home/promote/vehicleToEverything/asn/DefPosition.asn',
            # '/home/promote/vehicleToEverything/asn/MapLink.asn',
            # '/home/promote/vehicleToEverything/asn/MapPoint.asn',
            # '/home/promote/vehicleToEverything/asn/MapNode.asn',
            # '/home/promote/vehicleToEverything/asn/DefTime.asn',
            # '/home/promote/vehicleToEverything/asn/DefMotion.asn',
            # '/home/promote/vehicleToEverything/asn/MapLane.asn',
            # '/home/promote/vehicleToEverything/asn/VehSize.asn',
            # '/home/promote/vehicleToEverything/asn/MsgFrame.asn',
            # '/home/promote/vehicleToEverything/asn/Map.asn',
            # '/home/promote/vehicleToEverything/asn/VehSafetyExt.asn',
            # '/home/promote/vehicleToEverything/asn/DefPositionOffset.asn',
            # '/home/promote/vehicleToEverything/asn/RSI.asn',
            # '/home/promote/vehicleToEverything/asn/VehStatus.asn',
            # '/home/promote/vehicleToEverything/asn/DefAcceleration.asn',
            # '/home/promote/vehicleToEverything/asn/VehEmgExt.asn',
            # '/home/promote/vehicleToEverything/asn/VehBrake.asn',
            # '/home/promote/vehicleToEverything/asn/SPATIntersectionState.asn',
            # '/home/promote/vehicleToEverything/asn/SignalPhaseAndTiming.asn',
        ], numeric_enums=True)

        self.current_gps = None
        self.current_heading = None

        self.phase_id = None
        # self.timer = self.create_timer(0.05,self.publish_road_info)

    def listener_callback(self, msg):
        try:
            data = msg.data
            decoded_data = self.asn1_spec.decode("MessageFrame", data)

            # 使用索引访问tuple数据
            if decoded_data[0] == 'bsmFrame':
                self.car_message(decoded_data)
            elif decoded_data[0] == 'mapFrame': 
                self.extract_map_info(decoded_data)
            elif decoded_data[0] == 'spatFrame':
                self.traffic_light(decoded_data)

            self.get_logger().info(f'Decoded data: {decoded_data}')
        except Exception as e:
            self.get_logger().error(f'Error decoding data: {e}')



    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        self.current_heading = msg.track
        latitude = msg.latitude
        longitude = msg.longitude
        heading = msg.track
        self.get_logger().info(f"本车OBU车辆GPS位置: 纬度: {latitude}, 经度: {longitude}, 航向角: {heading}")
        # 
        # self.extract_map_info()

    def car_message(self,decoded_data):
        bsm_data = decoded_data[1]  # 使用索引1获取字典内容
        gps_msg = GPSFix()
        gps_msg.latitude = bsm_data['pos']['lat'] / 10000000.0
        gps_msg.longitude = bsm_data['pos']['long'] / 10000000.0
        gps_msg.speed = bsm_data['speed'] * 0.02 * 3.6
        gps_msg.track = bsm_data['heading'] * 0.0125
        print("其他车辆的透传过来的信息：", gps_msg)
        self.publisher_car_message.publish(gps_msg)

        speed_ms = Twist()
        speed_ms.linear.x = bsm_data['speed'] * 0.02
        self.publisher_speed_ms.publish(speed_ms)

        # 解析安全扩展信息
        if 'safetyExt' in bsm_data:
            fault_string = String()
            fault_string.data = str(bsm_data['safetyExt']['events'])
            print("故障信息：", fault_string.data)
            if '00000000000100' in fault_string.data:
                message = String()
                message.data = "异常车辆"
                self.publisher_fault.publish(message)

        if 'emergencyExt' in bsm_data:
            emergency_data = String()
            emergency_data.data = str(bsm_data['emergencyExt']['responseType'])
            print("紧急信息：", emergency_data.data)
            if '1' in emergency_data.data:
                message = String()
                message.data = "紧急车辆"
                self.publisher_emergency.publish(message)
                print("发布紧急车辆")
                

    def traffic_light(self, decoded_data):
        print("解码出来的红绿灯：", decoded_data)
        
        spat_data = decoded_data[1]
        # 构建简化的JSON格式数据
        json_data = {
            'msgCnt': spat_data['msgCnt'],
            'intersections': []
        }
        
        for intersection in spat_data['intersections']:
            # 直接获取status的bytearray部分并解码为字符串
            status_str = intersection['status'][0].decode('ascii')
            
            intersection_data = {
                'intersectionId': {
                    'region': intersection['intersectionId']['region'],
                    'id': intersection['intersectionId']['id']
                },
                'status': [status_str],  # 直接使用解码后的字符串
                'phases': []
            }
            
            for phase in intersection['phases']:
                phase_data = {
                    'id': phase['id'],
                    'phaseStates': []
                }
                
                for state in phase['phaseStates']:
                    timing_data = state['timing'][1]
                    phase_state = {
                        'light': state['light'],
                        'timing': {
                            'counting': {
                                'startTime': timing_data['startTime'],
                                'likelyEndTime': timing_data['likelyEndTime'],
                                'nextDuration': timing_data['nextDuration']
                            }
                        }
                    }
                    phase_data['phaseStates'].append(phase_state)
                
                intersection_data['phases'].append(phase_data)
            
            json_data['intersections'].append(intersection_data)
        # 打印或发送填充完成的JSON数据
        print(f"填写的json数据:{json_data}")

        msg = String()
        # msg.data = str(json_data)
        msg.data = json.dumps(json_data)
        self.light_string_publisher.publish(msg)


    # @staticmethod
    # def msg():
    #     return (('mapFrame', {'msgCnt': 0, 'timeStamp': 18, 'nodes': [{'name': 'JG_center_es', 'id': {'region': 500, 'id': 5}, 'refPos': {'lat': 36751988036, 'long': 117255482743}, 'inLinks': [{'name': 'bei_dong', 'upstreamNodeId': {'region': 500, 'id': 2}, 'speedLimits': [{'type': 5, 'speed': 416}], 'linkWidth': 300, 'points': [{'posOffset': {'offsetLL': ('position-LL2', {'lon': 61, 'lat': 3441})}}, {'posOffset': {'offsetLL': ('position-LL2', {'lon': 48, 'lat': 2909})}}, {'posOffset': {'offsetLL': ('position-LL2', {'lon': 35, 'lat': 2377})}}, {'posOffset': {'offsetLL': ('position-LL1', {'lon': 20, 'lat': 1757})}}, {'posOffset': {'offsetLL': ('position-LL1', {'lon': 7, 'lat': 1225})}}, {'posOffset': {'offsetLL': ('position-LL1', {'lon': -7, 'lat': 604})}}], 'movements': [{'remoteIntersection': {'region': 500, 'id': 3}, 'phaseId': 27}, {'remoteIntersection': {'region': 500, 'id': 4}, 'phaseId': 26}, {'remoteIntersection': {'region': 500, 'id': 1}, 'phaseId': 0}], 'lanes': [{'laneID': 1, 'laneWidth': 300, 'maneuvers': (bytearray(b'110000000000'), 96), 'connectsTo': [{'remoteIntersection': {'region': 500, 'id': 4}, 'connectingLane': {'lane': 1, 'maneuver': (bytearray(b'100000000000'), 96)}, 'phaseId': 26}, {'remoteIntersection': {'region': 500, 'id': 3}, 'connectingLane': {'lane': 1, 'maneuver': (bytearray(b'010000000000'), 96)}, 'phaseId': 27}], 'speedLimits': [{'type': 5, 'speed': 416}], 'points': [{'posOffset': {'offsetLL': ('position-LL2', {'lon': 5091, 'lat': -107})}}, {'posOffset': {'offsetLL': ('position-LL2', {'lon': 4217, 'lat': -98})}}, {'posOffset': {'offsetLL': ('position-LL2', {'lon': 3343, 'lat': -90})}}, {'posOffset': {'offsetLL': ('position-LL2', {'lon': 2468, 'lat': -82})}}, {'posOffset': {'offsetLL': ('position-LL1', {'lon': 1594, 'lat': -73})}}, {'posOffset': {'offsetLL': ('position-LL1', {'lon': 938, 'lat': -67})}}]}]}, {'name': 'south', 'upstreamNodeId': {'region': 500, 'id': 3}, 'speedLimits': [{'type': 5, 'speed': 416}], 'linkWidth': 300, 'points': [{'posOffset': {'offsetLL': ('position-LL2', {'lon': 61, 'lat': 3441})}}, {'posOffset': {'offsetLL': ('position-LL2', {'lon': 48, 'lat': 2909})}}, {'posOffset': {'offsetLL': ('position-LL2', {'lon': 35, 'lat': 2377})}}, {'posOffset': {'offsetLL': ('position-LL1', {'lon': 20, 'lat': 1757})}}, {'posOffset': {'offsetLL': ('position-LL1', {'lon': 7, 'lat': 1225})}}, {'posOffset': {'offsetLL': ('position-LL1', {'lon': -7, 'lat': 604})}}], 'movements': [{'remoteIntersection': {'region': 500, 'id': 4}, 'phaseId': 37}, {'remoteIntersection': {'region': 500, 'id': 1}, 'phaseId': 36}, {'remoteIntersection': {'region': 500, 'id': 2}, 'phaseId': 0}], 'lanes': [{'laneID': 1, 'laneWidth': 300, 'maneuvers': (bytearray(b'110000000000'), 96), 'connectsTo': [{'remoteIntersection': {'region': 500, 'id': 1}, 'connectingLane': {'lane': 1, 'maneuver': (bytearray(b'100000000000'), 96)}, 'phaseId': 36}, {'remoteIntersection': {'region': 500, 'id': 4}, 'connectingLane': {'lane': 1, 'maneuver': (bytearray(b'010000000000'), 96)}, 'phaseId': 37}], 'speedLimits': [{'type': 5, 'speed': 416}], 'points': [{'posOffset': {'offsetLL': ('position-LL2', {'lon': -38, 'lat': -3509})}}, {'posOffset': {'offsetLL': ('position-LL2', {'lon': -20, 'lat': -2885})}}, {'posOffset': {'offsetLL': ('position-LL2', {'lon': -2, 'lat': -2260})}}, {'posOffset': {'offsetLL': ('position-LL1', {'lon': 14, 'lat': -1636})}}, {'posOffset': {'offsetLL': ('position-LL1', {'lon': 32, 'lat': -1011})}}, {'posOffset': {'offsetLL': ('position-LL1', {'lon': 50, 'lat': -387})}}]}]}]}]}))


    def extract_map_info(self,decoded_data):
        # decoded_data = self.msg()
        if decoded_data[0] == 'mapFrame':
            map_data = decoded_data[1]
            extracted_info = {
                'msgCnt': map_data.get('msgCnt'),
                'timeStamp': map_data.get('timeStamp'),
                'nodes': []
            }

            closest_lane = None
            closest_lane_info = {}

            # 提取每个节点的详细信息
            for node in map_data.get('nodes', []):
                ref_pos = node.get('refPos', {})
                ref_lat = ref_pos.get('lat')
                ref_lon = ref_pos.get('long')
                center_lat = self.convert_lat_long(ref_lat)
                center_lon = self.convert_lat_long(ref_lon, is_longitude=True)
                # print(f"！！！！中心点经纬度: {center_lat}, {center_lon}")
                self.publisher_lukou_.publish(String(data = f"十字路口经度:{center_lon},十字路口纬度:{center_lat}"))
                
                node_id = node.get('id', {})
                node_info = {
                    'name': node.get('name'),
                    'region': node_id.get('region'),
                    'id': node_id.get('id'),
                    'refPos': {'lat': center_lat, 'lon': center_lon},
                    'inLinks': []
                }
                print(f"十字路口名称: {node_info['name']}, 区域: {node_info['region']}, 节点ID: {node_info['id']}, 中心点经纬度: {center_lat}, {center_lon}")

                # 提取 inLinks 信息
                for link in node.get('inLinks', []):
                    link_name = link.get('name')
                    upstream_node_id = link.get('upstreamNodeId', {})
                    upstream_id = upstream_node_id.get('id')
                    upstream_region = upstream_node_id.get('region')
                    link_info = {
                        'name': link_name,
                        # 'upstreamNodeId': link.get('upstreamNodeId'),
                        'upstreamNodeId': {'id': upstream_id,'region': upstream_region},
                        'speedLimits': link.get('speedLimits'),
                        'linkWidth': link.get('linkWidth'),
                        'movements': link.get('movements'),
                        'lanes': [],
                        'points': []
                    }

                    # 道路 ID upstream_id

                    is_vehicle_on_lane = False

                    # 提取每个 point 的经纬度，标记顺序
                    for idx, point in enumerate(link.get('points', []), start=1):
                        pos_offset = point.get('posOffset', {}).get('offsetLL', (None, {}))
                        offset_type, offset_vals = pos_offset

                        if offset_type in ['position-LL1', 'position-LL2']:
                            offset_lat = offset_vals.get('lat', 0)
                            offset_lon = offset_vals.get('lon', 0)
                            actual_lat = center_lat + offset_lat * 10**-7
                            actual_lon = center_lon + offset_lon * 10**-7
                            link_info['points'].append({'lat': actual_lat, 'lon': actual_lon})
                            # print(f"道路第{idx}个点经纬度: {actual_lat}, {actual_lon}")

                    # 提取 lanes 信息
                    for lane in link.get('lanes', []):
                        maneuvers = lane.get('maneuvers', (bytearray(b''), 0))
                        lane_maneuver = maneuvers[0].decode().strip('b').strip("'")  # 去掉多余字符

                        lane_info = {
                            'laneID': lane.get('laneID'),
                            'laneWidth': lane.get('laneWidth'),
                            'maneuvers': lane_maneuver,
                            'connectsTo': [],
                            'speedLimits': lane.get('speedLimits'),
                            'lane_points': []
                        }
                        
                        print(f"中心点链接道路名称：{link_info['name']},车道ID: {lane_info['laneID']}, 宽度: {lane_info['laneWidth']}, 操作: {lane_maneuver}")


                        closest_point = None
                        closest_distance = float('inf')
                        point_index = None 


                        # 提取 lane_points 的经纬度
                        # for lane_point in lane.get('points', []):
                        for idx, lane_point in enumerate(lane.get('points', []), start=1):
                            lane_pos_offset = lane_point.get('posOffset', {}).get('offsetLL', (None, {}))
                            lane_offset_type, lane_offset_vals = lane_pos_offset

                            if lane_offset_type in ['position-LL1', 'position-LL2']:
                                lane_offset_lat = lane_offset_vals.get('lat', 0)
                                lane_offset_lon = lane_offset_vals.get('lon', 0)
                                
                                actual_lane_lat = center_lat + lane_offset_lat * 10**-7
                                actual_lane_lon = center_lon + lane_offset_lon * 10**-7
                                # print(f"中心点：{center_lat},{center_lon}")
                                # print(f"偏差:{lane_offset_lat},{lane_offset_lon}")
                                lane_info['lane_points'].append({'lat': actual_lane_lat, 'lon': actual_lane_lon})
                                # print(f"道路点经纬度: {actual_lane_lat}, {actual_lane_lon}")
                                print(f"道路第{idx}个点经纬度: {actual_lane_lat}, {actual_lane_lon}")

                                distance = self.calculate_distance(self.current_gps,(actual_lane_lat,actual_lane_lon))
                                if distance < closest_distance:
                                    closest_distance = distance
                                    closest_point = (actual_lane_lat, actual_lane_lon)
                                    point_index = idx
                                # if (idx ==5 and distance < lane_info['laneWidth']/100):
                                if (idx == 5 or idx == 6) and distance < lane_info['laneWidth'] / 100:
                                    self.publish_vehicle_info()

                                # 检查车辆是否在车道范围内并更新标记
                                # if self.current_gps and (distance < lane_info['laneWidth'] / 100):
                                if self.current_gps and (distance < 4):
                                    is_vehicle_on_lane = True

                        # 如果车辆在该车道上，调用发布道路信息函数
                        if is_vehicle_on_lane:
                            self.phase_id = link.get('movements', [{}])[0].get('phaseId', None)
                            print(f"车辆在车道上,当前车道红绿灯相位ID: {self.phase_id}")
                            self.publish_road_info()
                        else:
                            # 重置 `self.phase_id` 避免多余发布
                            self.phase_id = None
                                        

                        if closest_point and closest_distance < 10:
                            if not closest_lane or closest_distance < closest_lane_info.get('distance', float('inf')):
                                closest_lane = lane_info['laneID']
                                closest_lane_info = {
                                    'lane_id': lane_info['laneID'],
                                    'lane_name': link_name,
                                    'point_index': point_index,  # 使用正确的点索引
                                    'distance': closest_distance,
                                    'position': closest_point
                                }


                        # 提取 connectsTo 信息
                        for conn in lane.get('connectsTo', []):
                            remote_intersection = conn.get('remoteIntersection', {})
                            region_id = remote_intersection.get('region')
                            road_id = remote_intersection.get('id')
                            connecting_lane = conn.get('connectingLane', {}).get('lane')
                            connecting_maneuver = conn.get('connectingLane', {}).get('maneuver', (bytearray(b''), 0))[0].decode().strip('b').strip("'")
                            phase_id = conn.get('phaseId')
                            self.phase_id = phase_id

                            connects_to_info = {
                                'road_id': road_id,
                                'region_id': region_id,
                                'phase_id': phase_id,
                                'connecting_lane': connecting_lane,
                                'connecting_maneuver': connecting_maneuver
                            }
                            lane_info['connectsTo'].append(connects_to_info)
                            print(f"连接到下游路段: {road_id} (区域: {region_id}), 当前相位ID: {phase_id}, 连接车道: {connecting_lane}, 操作: {connecting_maneuver}")

                        link_info['lanes'].append(lane_info)
                    
                    node_info['inLinks'].append(link_info)
                extracted_info['nodes'].append(node_info)

                if closest_lane:
                    self.get_logger().info(f"OBU车辆位于车道: {closest_lane_info['lane_name']} (车道ID: {closest_lane_info['lane_id']}), 在第{closest_lane_info['point_index']}个点附近, 距离: {closest_lane_info['distance']:.2f} 米")
                else:
                    self.get_logger().info("OBU车辆未找到匹配的车道")

            return extracted_info
        else:
            self.get_logger().warning("Decoded data is not in 'mapFrame' format.")
            return {}



    def publish_vehicle_info(self):
        """发布车辆的GPS位置和航向角信息"""
        if self.current_gps and self.current_heading is not None:
            latitude, longitude = self.current_gps
            heading = self.current_heading
            message = f"GPS位置: 纬度: {latitude}, 经度: {longitude}, 航向角: {heading}"

            # 发布到 /Vehicle_Map_Intersection
            msg = String()
            msg.data = message
            print("本车已到达路口")
            print("本车已到达路口")
            print("本车已到达路口")
            print("本车已到达路口")
            self.publisher_intersection.publish(msg)
            self.get_logger().info(f"发布到/Vehicle_Map_Intersection: {message}")
            # [INFO] [1726142168.588222397] [obu_handle_node]: 发布到/vehicle_on_road_info: 连接名称west连接车道id2相位id9

            # 发布到 /send_to_udp_topic1
            # ====================================修改后应该发布到固定节点，编码成asn，再进行透传
            # self.publisher_udp.publish(msg)
            # self.get_logger().info(f"发布到/send_to_udp_topic1: {message}")

            # gpsfix_five = GPSFix()
            # gpsfix_five.latitude = latitude
            # gpsfix_five.longitude = longitude
            # gpsfix_five.track = heading
            # self.publihser_gpsfix_five.publish(gpsfix_five)
            # self.get_logger().info(f"发布到/vehicleToencode: {message}")


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


    def publish_road_info(self):
        """发布车辆所在指定道路的信息"""
        if self.phase_id is not None and self.current_gps is not None:
            carlane_phase_id = self.phase_id
            message = f"车道相位id:{carlane_phase_id}"
            msg = String()
            msg.data = message
            self.road_info_pub.publish(msg)
            self.get_logger().info("aaaaaaaaaaaaaaa车辆在指定道路上")



    def convert_lat_long(self, value, is_longitude=False):
        value_str = str(value)
        if is_longitude:
            return float(f"{value_str[:3]}.{value_str[3:]}")
        else:
            return float(f"{value_str[:2]}.{value_str[2:]}")


def main(args=None):
    rclpy.init(args=args)
    node = UdpArrayDecoderNode()

    # # 测试解析
    # extracted_info = node.extract_map_info()
    # node.get_logger().info(f'Extracted test data: {extracted_info}')


    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

