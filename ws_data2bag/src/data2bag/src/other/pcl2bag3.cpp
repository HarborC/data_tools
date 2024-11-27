#include "ros/ros.h"
#include "opencv2/highgui.hpp"
#include "flag.h"
#include "io_tools.h"
#include <iostream>
#include <vector>
#include <glob.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace pandar_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      double, timestamp, timestamp)(uint16_t, ring, ring))

namespace hx_slam {
// FIELDS x y z intensity channel roi facet is_2nd_return multi_return confid_level flag elongation timestamp scanline scan_idx frame_id ring_id
// SIZE 4 4 4 2 2 2 2 2 2 2 2 2 8 2 2 4 4
// TYPE F F F U U U U U U U U U F U U U U
// COUNT 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1
struct PointRobinw {
  float x, y, z;
  uint16_t intensity;
  uint16_t channel;
  uint16_t roi;
  uint16_t facet;
  uint16_t is_2nd_return;
  uint16_t multi_return;
  uint16_t confid_level;
  uint16_t flag;
  uint16_t elongation;
  double timestamp;
  uint16_t scanline;
  uint16_t scan_idx;
  uint32_t frame_id;
  uint32_t ring_id;
};
}  // namespace hx_slam

POINT_CLOUD_REGISTER_POINT_STRUCT(
    hx_slam::PointRobinw,
    (float, x, x)(float, y, y)(float, z, z)(uint16_t, intensity, intensity)(uint16_t, channel,
                                                                            channel)(
        uint16_t, roi, roi)(uint16_t, facet, facet)(uint16_t, is_2nd_return, is_2nd_return)(
        uint16_t, multi_return, multi_return)(uint16_t, confid_level, confid_level)(
        uint16_t, flag, flag)(uint16_t, elongation, elongation)(double, timestamp, timestamp)(
        uint16_t, scanline, scanline)(uint16_t, scan_idx,
                                      scan_idx)(uint32_t, frame_id,
                                                frame_id)(uint32_t, ring_id, ring_id))

typedef hx_slam::PointRobinw PointT;
typedef pcl::PointCloud<PointT>::Ptr PointPtr;

ros::Publisher pcd_pub;
void ReadDirectory(int argc, char** argv)
{


    //读取文件夹
    ros::init(argc,argv,"image_publisher");
    ros::NodeHandle nh;
    std::string imgPath="/mnt/g/projects/hx_droid_slam_ws/AC9723__orin-1_20240313114300__lidar300_pcd";
    nh.getParam("pcl_folder", imgPath);
    std::cout<< "read pcl path success!"<<imgPath<<std::endl;

    //-----------
    pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_pcl", 100);

    //读取文件
    ros::Rate loopRate(10);


    std::string pcd_dir = std::string(imgPath);
    std::vector<std::pair<double, std::string>> pcd_times_files;
    std::vector<std::string> pcd_files = hx_slam::io::GetFileList(pcd_dir);
    for (size_t i = 0; i < pcd_files.size(); i++) {
        std::string root, ext;
        hx_slam::io::SplitFileExtension(pcd_files[i], &root, &ext);
        if (ext != ".pcd") {
            std::cout << "ext: " << ext << std::endl;
            continue;
        }

        std::string base_name = hx_slam::io::GetPathBaseName(root);
        std::vector<std::string> items = hx_slam::io::StringSplit(base_name, "_");
        double pcd_time = std::stod(items.back()) * 1e-3;
        pcd_times_files.push_back(std::make_pair(pcd_time, pcd_files[i]));
    }  

    std::sort(pcd_times_files.begin(), pcd_times_files.end(), [](const std::pair<double, std::string> &a, const std::pair<double, std::string> &b) {
        return a.first < b.first;
    });

    for (size_t i = 3; i < pcd_times_files.size(); i++) {
        std::string pcd_file = pcd_times_files[i].second;
        double pcd_time = pcd_times_files[i].first;

        if(!nh.ok()){
            break;
        }

        std::cout << std::fixed << pcd_times_files[i].first << " : " << pcd_times_files[i].second << std::endl;

        pcl::PointCloud<pandar_ros::Point> pl_orig;

        PointPtr cloud(new pcl::PointCloud<PointT>);
        if (pcl::io::loadPCDFile(pcd_file, *cloud) == -1)
        {
            std::cout << "点云数据读取失败: "<< pcd_file << std::endl;
        }
        for(int index=0 ; index < cloud->points.size() ; index++){

            pandar_ros::Point added_pt;
            added_pt.x = cloud->points[index].x;
            added_pt.y = cloud->points[index].y;
            added_pt.z = cloud->points[index].z;
            added_pt.intensity = cloud->points[index].intensity;
            added_pt.timestamp = cloud->points[index].timestamp;
            added_pt.ring = 0;

            // ROS_INFO("int=%0.20f, decimal=%0.20f \n", tmp_cur, added_pt.time);

            pl_orig.points.push_back(added_pt);
            
        }

        sensor_msgs::PointCloud2 laserCloud;
        pcl::toROSMsg(pl_orig, laserCloud);


        // double h;
        // h = 0.31231242314135;
        ros::Time timestamp_ros = ros::Time(pcd_time);
        laserCloud.header.stamp = timestamp_ros;
        laserCloud.header.frame_id = "/scan";
  
        // flag_xx.show_flag();
        pcd_pub.publish(laserCloud);

        std::cout << "publish pcd file: " << pcd_file << std::endl;

        ros::spinOnce();
        std::cout << "publish pcd file: " << pcd_file << std::endl;

        // loopRate.sleep();
        // std::cout << "publish pcd file: " << pcd_file << std::endl;
    }


}


int main(int argc, char** argv)
{
    ReadDirectory(argc,argv);

    return 0;
}
