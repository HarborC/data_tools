#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
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
        std::cout<<"input gps path!"<<std::endl;
        return -1;
    }

    ros::init(argc,argv, "gps_publisher");
    ros::NodeHandle nh;
    //读取文件
    std::string gpsFilePath="/mnt/g/projects/test_data/1/gps.txt";
    nh.getParam("gps_file",gpsFilePath);
    std::cout<<"input gps path success!"<<gpsFilePath<<std::endl;
    ros::Publisher gpsPub = nh.advertise<sensor_msgs::NavSatFix>("/gps", 1000000);
    ros::Rate loopRate(200);

    std::fstream gpsFile(gpsFilePath);
    std::string readLine;
    // std::getline(gpsFile,readLine);

    while(ros::ok() && std::getline(gpsFile,readLine)){
      if(readLine.empty())
          continue;
        //std::cout<<readLine<<std::endl;
        std::vector<std::string> lineSplit = StringSplit(readLine," ");
        // if(lineSplit.size()<6){
        //     continue;
        // }

        // std::cout<<readLine<<std::endl;

        sensor_msgs::NavSatFix gpsData;
        std::string timestamp = lineSplit[0];
        gpsData.header.stamp = TimestampToRosTime(timestamp);
        
        double lat = std::stod(lineSplit[1]);
        int lat_1 = int(lat/100);
        double lat_2 = (lat/100 - lat_1) * 100 / 60;
        lat = lat_1 + lat_2;

        double lon = std::stod(lineSplit[2]);
        int lon_1 = int(lon/100);
        double lon_2 = (lon/100 - lon_1) * 100 / 60;
        lon = lon_1 + lon_2;

        gpsData.header.frame_id = "scan";
        gpsData.latitude = lat;
        gpsData.longitude = lon;
        gpsData.altitude = StringToDouble(lineSplit[3]);
        

        if (std::abs(gpsData.altitude) < 1e-9)
          continue;

        // std::cout<<std::fixed<<"gps："<<gpsData.header.stamp<<std::endl;

        gpsData.position_covariance = {0.5, 0, 0, 0, 0.5, 0, 0, 0, 1};

        // flag_xx.add_flag();
        // flag_xx.show_flag();
        gpsPub.publish(gpsData);
        ros::spinOnce();
        loopRate.sleep();
    }

    gpsFile.close();
    return 0;
}
