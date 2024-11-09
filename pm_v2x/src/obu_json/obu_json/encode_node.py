import rclpy
from rclpy.node import Node
from gps_msgs.msg import GPSFix
from std_msgs.msg import String
import asn1tools
from datetime import datetime
from std_msgs.msg import UInt8MultiArray

class GPSDataProcessor(Node):
    def __init__(self):
        super().__init__('encode_node')
        
        # 接收 GPS 数据 进行编码然后发向udp透传节点 ui_datapublisher_llahs

        # self.subscription = self.create_subscription(
        #     GPSFix, '/vehicleToencode', self.gps_callback, 10) # 车辆到达第五第六个点的时候才能收到
        

        self.subscription_ui_llahs = self.create_subscription(GPSFix, '/ui_datapublisher_llahs', self.gps_callback, 10)


        # 发布编码的 ASN 消息到指定话题  encoded_bsm_data
        self.encoded_publisher = self.create_publisher(UInt8MultiArray, '/encoded_bsm_data', 10)
        
        self.vehicle_event_subscription = self.create_subscription(String, '/vehicle_event_fault', self.vehicle_event_callback, 10)
        self.vehicle_event_publisher = self.create_publisher(UInt8MultiArray, '/encoded_bsm_data', 10)

        self.vehicle_emergency_subscription = self.create_subscription(String, '/vehicle_emergency', self.vehicle_emergency_callback, 10)
        self.vehicle_emergency_publisher = self.create_publisher(UInt8MultiArray, '/encoded_bsm_data', 10)
        
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

        self.total_ms = None
        self.micro_latitude = None
        self.micro_longitude = None
        self.speed_0_02 = None
        self.heading_0_0125 = None
        
    
    def gps_callback(self, msg):
        # 接收到的GPS数据
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        heading = msg.track
        speed = msg.speed

        longitude_6decimal = float(format(longitude, '.6f'))
        self.micro_longitude = int(longitude_6decimal * 10000000)
        latitude_6decimal = float(format(latitude, '.6f'))
        self.micro_latitude = int(latitude_6decimal * 10000000)

        elevation_in_10cm = int(round(altitude * 10))
        if elevation_in_10cm < -4096:
            elevation_in_10cm = -4096 
        elif elevation_in_10cm > 61439:
            elevation_in_10cm = 61439

        speed_ms = speed * (1000.0/3600.0)
        self.speed_0_02 = int(round(speed_ms/0.02))
        if self.speed_0_02 < 0:
            self.speed_0_02 = 0
        elif self.speed_0_02 > 8191:
            self.speed_0_02 = 8191

        self.heading_0_0125 = int(round(heading / 0.0125))
        if self.heading_0_0125 < 0:
            self.heading_0_0125 = 0
        elif self.heading_0_0125 > 28800:
            self.heading_0_0125 = self.heading_0_0125 % 28800

        now = datetime.now()
        formatted_time = now.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        print(f"当前系统时间: {formatted_time}")
        self.total_ms = ((now.hour * 3600 + now.minute * 60 + now.second) * 1000 + int(now.microsecond/1000)) % 65536




        # 创建并填充ASN.1结构
        basic_safety_message = {
            'msgCnt': 1,
            'id': b'00000001',
            'secMark': self.total_ms,
            'pos': {
                'lat': self.micro_latitude,
                'long': self.micro_longitude,
                'elev': elevation_in_10cm
            },
            'transmission': 2,
            'speed': self.speed_0_02,
            'heading': self.heading_0_0125,
            'accelSet': {'long': 0, 'lat': 0, 'vert': 0, 'yaw': 0},
            'brakes': {'abs': 0, 'scs': 0},
            'size': {'width': 200, 'length': 400},
            'vehicleClass':{'classification':10}
        }
        
        # ASN.1编码
        encoded_bsm = self.asn1_spec.encode("MessageFrame",('bsmFrame', basic_safety_message))
        print(f"Encoded BSM: {encoded_bsm}")

        decoded_bsm = self.asn1_spec.decode('MessageFrame', encoded_bsm)
        print(f"Decoded BSM: {decoded_bsm}")

        # 直接使用UInt8MultiArray发送
        encoded_msg = UInt8MultiArray()
        encoded_msg.data = encoded_bsm
        self.encoded_publisher.publish(encoded_msg)
    

    def vehicle_event_callback(self,msg):
        # 异常车辆
        vehicle_event_data = msg.data
        print(f"Received vehicle event data: {vehicle_event_data}")
        if self.micro_latitude and vehicle_event_data:
            basic_safety_message = {
                'msgCnt': 1,
                'id': b'00000001',
                'secMark': self.total_ms,
                'pos': {
                    'lat': self.micro_latitude,
                    'long': self.micro_longitude,
                    # 'elev': elevation_in_10cm
                },
                'transmission': 2,
                'speed': self.speed_0_02,
                'heading': self.heading_0_0125,
                'accelSet': {'long': 0, 'lat': 0, 'vert': 0, 'yaw': 0},
                'brakes': {'abs': 0, 'scs': 0},
                'size': {'width': 200, 'length': 400},
                'vehicleClass':{'classification':10},
                'safetyExt':{'events':(b'00000000000100',14*8)}
            }
            encoded_bsm = self.asn1_spec.encode("MessageFrame",('bsmFrame', basic_safety_message))
            print(f"8888888888888: {encoded_bsm}")

            encoded_msg = UInt8MultiArray()
            encoded_msg.data = encoded_bsm
            self.vehicle_event_publisher.publish(encoded_msg)

            decoded_bsm = self.asn1_spec.decode('MessageFrame', encoded_bsm)
            print(f"Decoded 8888888: {decoded_bsm}")
        
        else :
            print("没有接收到GPS数据")


    def vehicle_emergency_callback(self,msg):
        # 紧急车辆
        vehicle_emergency_data = msg.data
        print(f"Received vehicle emergency data: {vehicle_emergency_data}")
        if self.micro_latitude and vehicle_emergency_data:
            basic_safety_message = {
                'msgCnt': 1,
                'id': b'00000001',
                'secMark': self.total_ms,
                'pos': {
                    'lat': self.micro_latitude,
                    'long': self.micro_longitude,
                    # 'elev': elevation_in_10cm
                },
                'transmission': 2,
                'speed': self.speed_0_02,
                'heading': self.heading_0_0125,
                'accelSet': {'long': 0, 'lat': 0, 'vert': 0, 'yaw': 0},
                'brakes': {'abs': 0, 'scs': 0},
                'size': {'width': 200, 'length': 400},
                'vehicleClass':{'classification':10},
                # 'safetyExt':{'events':(b'00000000000100',14*8)}
                'emergencyExt': {'responseType': 1,'sirenUse': 2,'lightsUse': 2   }
            }
            encoded_bsm = self.asn1_spec.encode("MessageFrame",('bsmFrame', basic_safety_message))
            print(f"99999999999: {encoded_bsm}")

            encoded_msg = UInt8MultiArray()
            encoded_msg.data = encoded_bsm
            self.vehicle_emergency_publisher.publish(encoded_msg)

            decoded_bsm = self.asn1_spec.decode('MessageFrame', encoded_bsm)
            print(f"Decoded 9999999999999999: {decoded_bsm}")



def main(args=None):
    rclpy.init(args=args)
    gps_data_processor = GPSDataProcessor()
    rclpy.spin(gps_data_processor)
    gps_data_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
