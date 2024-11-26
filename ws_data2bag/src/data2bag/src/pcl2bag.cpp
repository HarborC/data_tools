#include "ros/ros.h"
#include "opencv2/highgui.hpp"
#include "flag.h"
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




std::vector<std::string> GetFiles(std::string dirPath)
{
    std::vector<std::string> files;
    struct dirent *ptr;
    char base[1000];
    DIR *dir;
    dir = opendir(dirPath.c_str());
    while((ptr = readdir(dir)) != nullptr){
        if(ptr->d_type == 8){
            files.push_back(dirPath + '/' + ptr->d_name);
        }
    }
    closedir(dir);
    std::sort(files.begin(),files.end());
    //for(size_t idx = 0; idx <files.size(); idx++){
        //std::cout<< files[idx] <<std::endl;
    //}
    return files;
}

std::vector<std::string> StringSplit(std::string str,std::string pattern)
{
  std::string::size_type pos;
  std::vector<std::string> result;
  str+=pattern;
  int size=str.size();
 
  for(int i=0; i<size; i++)
  {
    pos=str.find(pattern,i);
    if(pos<size)
    {
      std::string s=str.substr(i,pos-i);
      result.push_back(s);
      i=pos+pattern.size()-1;
    }
  }
  return result;
}

ros::Time TimestampToRosTime(std::string timestamp)
{
    size_t len = timestamp.length();
    size_t secLen = len - 9;
    std::string sec_string = timestamp.substr(0,secLen);
    std::string nsec_string = timestamp.substr(secLen,9);
    return ros::Time(std::stoi(sec_string),std::stoi(nsec_string));
}

typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT>::Ptr PointPtr;

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

ros::Publisher pcd_pub;
void ReadDirectory(int argc, char** argv)
{


    //读取文件夹
    ros::init(argc,argv,"image_publisher");
    ros::NodeHandle nh;
    std::string imgPath="/mnt/g/projects/test_data/1/horizon_lidar";
    nh.getParam("pcl_folder", imgPath);
    std::cout<< "read pcl path success!"<<imgPath<<std::endl;

    //-----------
    pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_pcl", 100);

    //读取文件
    std::vector<std::string> imgPaths = GetFiles(imgPath);//argv[1]
    std::cout <<  imgPaths.size();
    ros::Rate loopRate(10);


    for(size_t imgIdx = 0; imgIdx < imgPaths.size(); imgIdx++){

        if(!nh.ok()){
            break;
        }

        pcl::PointCloud<velodyne_ros::Point> pl_orig;

        //文件名、时间戳获取
        std::vector<std::string> pathSplits = StringSplit(imgPaths[imgIdx], "/");
        std::string timestamp = StringSplit(pathSplits.back(),".").front();
        std::string stringStream;
        stringStream = imgPaths[imgIdx];

        // std::cout << stringStream << std::endl;

        PointPtr cloud(new pcl::PointCloud<PointT>);
        if (pcl::io::loadPCDFile(stringStream, *cloud) == -1)
        {
            std::cout << "点云数据读取失败: "<< stringStream << std::endl;
        }
        for(int index=0 ; index < cloud->points.size() ; index++){

            velodyne_ros::Point added_pt;
            added_pt.x = cloud->points[index].x;
            added_pt.y = cloud->points[index].y;
            added_pt.z = cloud->points[index].z;

            added_pt.intensity = cloud->points[index].intensity;
            auto tmp_cur =  cloud->points[index].curvature;
            added_pt.time = (cloud->points[index].curvature-static_cast<int>(tmp_cur))*10.0;

            // ROS_INFO("int=%0.20f, decimal=%0.20f \n", tmp_cur, added_pt.time);

            pl_orig.points.push_back(added_pt);
            added_pt.ring = 1;


        }

        sensor_msgs::PointCloud2 laserCloud;
        pcl::toROSMsg(pl_orig, laserCloud);


        // double h;
        // h = 0.31231242314135;
        ros::Time timestamp_ros = TimestampToRosTime(timestamp);
        laserCloud.header.stamp = timestamp_ros;
        laserCloud.header.frame_id = "scan";
        // std::cout<<std::fixed<< std::setprecision(9)<<timestamp_ros.toSec()<<std::endl;
        // std::cout<<timestamp_ros.toNSec()<<std::endl;
        // std::cout << h <<std::endl;
        // cv::Mat img = cv::imread(stringStream.str(), CV_LOAD_IMAGE_COLOR);
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", laserCloud).toImageMsg();
        //msg->header.stamp = ros::Time(timestampInt);
        // msg->header.stamp = TimestampToRosTime(timestamp);
        //std::cout<<std::fixed<<std::endl<<"laser："<<laserCloud.header.stamp<<std::endl;

  
        // flag_xx.show_flag();
        pcd_pub.publish(laserCloud);

        ros::spinOnce();
        loopRate.sleep();
    }
}


int main(int argc, char** argv)
{
    ReadDirectory(argc,argv);

    return 0;
}
