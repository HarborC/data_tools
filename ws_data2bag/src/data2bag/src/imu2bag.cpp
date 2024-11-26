#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <fstream>
#include <iostream>
#include <vector>
#include "flag.h"



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
    //while(nsec_string.length() < 9){
        //nsec_string += "0";
    //}
    return ros::Time(std::stoi(sec_string),std::stoi(nsec_string));
}

double StringToDouble(std::string strData)

{
    return std::stod(strData);
}




int main(int argc, char** argv)
{
    if (argc<2){
        std::cout<<"input imu path!"<<std::endl;
        return -1;
    }

    ros::init(argc,argv, "imu_publisher");
    ros::NodeHandle nh;
    //读取文件
    std::string imuFilePath="/mnt/g/projects/test_data/1/horizon_imu.txt";
    nh.getParam("imu_file",imuFilePath);
    std::cout<<"input imu path success!"<<imuFilePath<<std::endl;
    ros::Publisher imuPub = nh.advertise<sensor_msgs::Imu>("/imu_data", 1000000);
    ros::Rate loopRate(200);

    std::fstream imuFile(imuFilePath);
    std::string readLine;
    // std::getline(imuFile,readLine);

    while(ros::ok() && std::getline(imuFile,readLine)){
        //std::cout<<readLine<<std::endl;
        std::vector<std::string> lineSplit = StringSplit(readLine," ");
        // if(lineSplit.size()<6){
        //     continue;
        // }

        sensor_msgs::Imu imuData;
        std::string timestamp = lineSplit[0];
        imuData.header.stamp = TimestampToRosTime(timestamp);
        

        imuData.header.frame_id = "scan";
        imuData.angular_velocity.x = StringToDouble(lineSplit[1]);
        imuData.angular_velocity.y = StringToDouble(lineSplit[2]);
        imuData.angular_velocity.z = StringToDouble(lineSplit[3]);

        imuData.linear_acceleration.x = StringToDouble(lineSplit[4]);
        imuData.linear_acceleration.y = StringToDouble(lineSplit[5]);
        imuData.linear_acceleration.z = StringToDouble(lineSplit[6]);

      // std::cout<<std::fixed<<"imu："<<imuData.header.stamp<<std::endl;

        // flag_xx.add_flag();
        // flag_xx.show_flag();
        imuPub.publish(imuData);
        ros::spinOnce();
        loopRate.sleep();
    }

    imuFile.close();
    return 0;
}
