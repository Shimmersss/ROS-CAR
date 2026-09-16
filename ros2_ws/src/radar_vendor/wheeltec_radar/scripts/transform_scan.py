#! /usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from radar_msgs.msg import RadarScan
from radar_msgs.msg import RadarReturn
import math

class LaserScanModifier(Node):
    def __init__(self):
        super().__init__('transform_scan')
        
        # 订阅LaserScan话题
        self.subscription = self.create_subscription(RadarScan,'/radarscan', self.listener_callback,10)
        
        # 发布修改后的LaserScan话题
        self.publisher = self.create_publisher(LaserScan, '/scan2', 10)

        # 数据
        self.modified_scan = LaserScan()
        self.modified_scan.angle_min = 0.0
        self.modified_scan.angle_max = 2*math.pi
        self.modified_scan.angle_increment = 2*math.pi/720.0
        #self.modified_scan.time_increment = msg.time_increment
        #self.modified_scan.scan_time = msg.scan_time
        self.modified_scan.range_min = 0.0
        self.modified_scan.range_max = 30.0
        self.modified_scan.ranges = [float('inf')] * 720
        
        
    def listener_callback(self, msg):
        self.modified_scan.ranges = [float('inf')] * 720
        self.modified_scan.header = msg.header
        for obj in msg.returns:
            # 计算目标角度在扫描中的索引
            angle_diff = obj.azimuth - self.modified_scan.angle_min
            index = int(round(angle_diff / self.modified_scan.angle_increment ))
            for i in range(-3,3):
                if self.modified_scan.ranges[index+i]>obj.range:
                    self.modified_scan.ranges[index+i] = obj.range  
        # 发布修改后的数据
        self.publisher.publish(self.modified_scan)

def main(args=None):
    rclpy.init(args=args)
    laser_scan_modifier = LaserScanModifier()
    rclpy.spin(laser_scan_modifier)
    laser_scan_modifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
