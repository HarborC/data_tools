import os
import cv2
import rosbag
import rospy
from sensor_msgs.msg import Image, Imu, MagneticField, PointCloud2
from cv_bridge import CvBridge
from std_msgs.msg import Header
import glob
from datetime import datetime
import pytz
import numpy as np
from sensor_msgs.point_cloud2 import create_cloud
import std_msgs.msg
import open3d as o3d

root_folder = '/mnt/g/data/AVD/2024-11/2024-11-24_15-34-54/'
output_bag = os.path.join(root_folder, 'data.bag')
g = 9.81

start_time = 1732433732799336910 * 1e-9
end_time = -1

bridge = CvBridge()

with rosbag.Bag(output_bag, 'w') as bag:
    # horizon lidar
    horizon_lidar_folder = os.path.join(root_folder, 'horizon_lidar')
    lidar_files = os.listdir(horizon_lidar_folder)
    if len(lidar_files) == 0:
        for lidar_filename in sorted(os.listdir(horizon_lidar_folder)):
            lidar_path = os.path.join(horizon_lidar_folder, lidar_filename)
            
            timestamp = float(lidar_filename.split('.')[0]) * 1e-9
            
            if start_time > 0 and timestamp < start_time:
                continue
            
            if end_time > 0 and timestamp > end_time:
                break

            print(f"timestamp: {timestamp}")
            
            pcd = o3d.io.read_point_cloud(lidar_path)
            points = np.asarray(pcd.points)
            
            dtype = np.dtype([
                ('x', 'f4'),
                ('y', 'f4'),
                ('z', 'f4'),
                ('intensity', 'f4'),
                ('time', 'f4'),
                ('ring', 'u2'),
            ])

            structured_points = np.zeros(points.shape[0], dtype=dtype)
            structured_points['x'] = points[:, 0]
            structured_points['y'] = points[:, 1]
            structured_points['z'] = points[:, 2]
            structured_points['intensity'] = points[:, 3]
            structured_points['time'] = points[:, 4]
            structured_points['ring'] = points[:, 5]

            header = std_msgs.msg.Header()
            header.frame_id = "livox_horizon"
            header.stamp = rospy.Time.from_sec(timestamp)

            # 定义 PointCloud2 消息的字段
            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1),
                PointField('intensity', 12, PointField.FLOAT32, 1),
                PointField('time', 16, PointField.FLOAT32, 1),
                PointField('ring', 20, PointField.UINT16, 1),
            ]

            # 创建 PointCloud2 消息
            cloud_msg = create_cloud(header, fields, structured_points.tolist())

            bag.write('/horizon_lidar/data', cloud_msg, cloud_msg.header.stamp)

        print(f"horizon_lidar done.")
        
    # horizon_imu
    horizon_imu_file = os.path.join(root_folder, 'horizon_imu.txt')
    horizon_imu = open(horizon_imu_file, 'r', encoding='utf-8', errors='ignore')
    
    for line in horizon_imu:
        timestamp, gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z = line.strip().split(' ')
        timestamp = float(timestamp) * 1e-9
        
        if start_time > 0 and timestamp < start_time:
            continue
        
        if end_time > 0 and timestamp > end_time:
            break
        
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.from_sec(timestamp)
        imu_msg.linear_acceleration.x = float(acc_x) * g
        imu_msg.linear_acceleration.y = float(acc_y) * g
        imu_msg.linear_acceleration.z = float(acc_z) * g
        imu_msg.angular_velocity.x = float(gyr_x)
        imu_msg.angular_velocity.y = float(gyr_y)
        imu_msg.angular_velocity.z = float(gyr_z)
        
        bag.write('/imu/horizon_data', imu_msg, imu_msg.header.stamp)
        
    horizon_imu.close()
    
    print(f"horizon_imu done.")
    
    # hkvison image
    hkvison_image_folder = os.path.join(root_folder, 'hkvison_image')
    image_files = os.listdir(hkvison_image_folder)
    if len(image_files) == 0:
        for image_filename in sorted(image_files):
            image_path = os.path.join(hkvison_image_folder, image_filename)
            
            timestamp = float(image_filename.split('.')[0]) * 1e-9
            
            if start_time > 0 and timestamp < start_time:
                continue
            
            if end_time > 0 and timestamp > end_time:
                break

            print(f"timestamp: {timestamp}")

            # read image
            image = cv2.imread(image_path)
            if image is None:
                continue

            # # raw image
            # image_msg = bridge.cv2_to_imgmsg(image, encoding='mono8')
            # image_msg.header = Header()
            # image_msg.header.stamp = rospy.Time.from_sec(timestamp)
            
            # compressed image
            image_msg = bridge.cv2_to_compressed_imgmsg(image)
            image_msg.header = Header()
            image_msg.header.stamp = rospy.Time.from_sec(timestamp)
            image_msg.format = 'jpeg'
            
            # 写入bag文件
            bag.write('/hkvison_camera/image_compressed', image_msg, image_msg.header.stamp)

        print(f"hkvison_image done.")

    bag.close()
    
print(f"Bag file {output_bag} created successfully.")
