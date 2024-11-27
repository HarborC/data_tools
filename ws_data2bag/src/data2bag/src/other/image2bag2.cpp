#include "flag.h"
#include "opencv2/highgui.hpp"
#include "ros/ros.h"
#include <dirent.h>
#include <glob.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <vector>

std::vector<std::string> GetFiles(std::string dirPath) {
  std::vector<std::string> files;
  struct dirent *ptr;
  char base[1000];
  DIR *dir;
  dir = opendir(dirPath.c_str());
  while ((ptr = readdir(dir)) != nullptr) {
    if (ptr->d_type == 8) {
      files.push_back(dirPath + '/' + ptr->d_name);
    }
  }
  closedir(dir);
  std::sort(files.begin(), files.end());
  // for(size_t idx = 0; idx <files.size(); idx++){
  // std::cout<< files[idx] <<std::endl;
  //}
  return files;
}

std::vector<std::string> StringSplit(std::string str, std::string pattern) {
  std::string::size_type pos;
  std::vector<std::string> result;
  str += pattern;
  int size = str.size();

  for (int i = 0; i < size; i++) {
    pos = str.find(pattern, i);
    if (pos < size) {
      std::string s = str.substr(i, pos - i);
      result.push_back(s);
      i = pos + pattern.size() - 1;
    }
  }
  return result;
}

ros::Time TimestampToRosTime(std::string timestamp) {
  size_t len = timestamp.length();
  size_t secLen = len - 9;
  std::string sec_string = timestamp.substr(0, secLen);
  std::string nsec_string = timestamp.substr(secLen, 9);
  return ros::Time(std::stoi(sec_string), std::stoi(nsec_string));
}

bool map_init = false;
cv::Mat map_x, map_y;

cv::Mat imgProcess(cv::Mat img) {
  cv::Mat imgGray;
  if (map_init) {
    cv::remap(img, imgGray, map_x, map_y, cv::INTER_LINEAR);
    
    cv::Mat img_crop = imgGray(cv::Rect(0, 0, 640, 480));

    cv::Mat resized_img;
    cv::resize(imgGray, resized_img, cv::Size(640, 480));

    return resized_img;
  }

  
  
  return imgGray;
}

void initRemap() {
  Eigen::Matrix3d K;
  K << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  Eigen::Matrix3d Knew;
  Knew = K;
  double focal_scale = 0.75;
  Knew(0, 0) = focal_scale * K(0, 0);
  Knew(1, 1) = focal_scale * K(1, 1);
  cv::Mat K_cv, K_new_cv;
  cv::eigen2cv(K, K_cv);
  cv::eigen2cv(Knew, K_new_cv);

  cv::Size img_size(640, 480);
  cv::fisheye::initUndistortRectifyMap(K_new_cv, cv::Mat(), cv::Mat::eye(3, 3, CV_32F), K_cv, img_size, CV_16SC2, map_x, map_y);

  map_init = true;
}

ros::Publisher pcd_pub;
void ReadDirectory(int argc, char **argv) {

  //读取文件夹
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  std::string imgPath = "/mnt/g/projects/test_data/1/horizon_lidar";
  nh.getParam("pcl_folder", imgPath);
  std::cout << "read image path success!" << imgPath << std::endl;

  //-----------
  pcd_pub = nh.advertise<sensor_msgs::Image>("/image", 100);

  //读取文件
  std::vector<std::string> imgPaths = GetFiles(imgPath); // argv[1]
  std::cout << imgPaths.size();
  ros::Rate loopRate(10);

  initRemap();

  for (size_t imgIdx = 0; imgIdx < imgPaths.size(); imgIdx++) {
    if (!nh.ok()) {
      break;
    }

    //文件名、时间戳获取
    std::vector<std::string> pathSplits = StringSplit(imgPaths[imgIdx], "/");
    std::string timestamp = StringSplit(pathSplits.back(), ".").front();
    std::string stringStream;
    stringStream = imgPaths[imgIdx];

    // std::cout << stringStream << std::endl;

    cv::Mat img = cv::imread(stringStream, 0);
    if (img.empty()) {
      std::cout << "image data read failed: " << stringStream << std::endl;
      continue;
    }

    cv::Mat imgGray = imgProcess(img);

    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    // msg->header.stamp = ros::Time(timestampInt);

    // double h;
    ros::Time timestamp_ros = TimestampToRosTime(timestamp);
    msg.header.stamp = timestamp_ros;
    msg.header.frame_id = "scan";

    // flag_xx.show_flag();
    pcd_pub.publish(msg);

    ros::spinOnce();
    loopRate.sleep();
  }
}

int main(int argc, char **argv) {
  ReadDirectory(argc, argv);

  return 0;
}
