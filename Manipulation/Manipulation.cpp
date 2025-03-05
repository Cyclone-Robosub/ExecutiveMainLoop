// Created By Tanishq Dwivedi
#include <iostream>
#include <fstream>
#include <ctime>
#include <string>
#include <thread>
#include "GetSensorData.cpp"
#include <memory>
#include "SetupManipulation.hpp"
#include <thread>
//#include <ros/ros.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


// Start an object to create the DataSet
// Research File for the currentdate and assign 
// the string back for GetData.
void setupfunc(std::string &selectedFile){
  Setup startsending = Setup();
  selectedFile = startsending.start();
}
/*Get Data gets the data to the file selectedFile
and uses rclcpp spin in order to start appending data
to the file*/
void GetData(std::string &selectedFile, int argc, char * argv[]){
  rclcpp::init(argc, argv);
  std::shared_ptr<GetSensorData> get_sensor_data = std::make_shared<GetSensorData>(selectedFile);
  
  rclcpp::spin(get_sensor_data);
  
}
int main(int argc, char * argv[]) {
    std::string selectedFile;

    //Courtsey of DillRaj
    std::cerr << "NOT sigma sigma boy" << std::endl;

    setupfunc(selectedFile);
    GetData(selectedFile, argc, argv);
    
  //  ros::spin();
    return 0;

}


