#include <iostream>
#include <filesystem>
#include "SetupConfig/Setup.cpp"
#include <thread>
#include <utility>
#include <string>
#include <chrono>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::literals;
namespace fs = std::filesystem;
//start the executive Loop
class ExecutiveMainLoop{
    public:
    //Setup for all the functions should be done here.
           //Maybe we could code each function to setup on its own.
           //The functions run assuming that the inital first iteration
           //of the loop starts stage by stage with no wait. 
        ExecutiveMainLoop(int argc, char * argv[]){
            rclcpp::init(argc, argv);
            research_sensor_Subscription = this->create_subscription<std__msgs::msg::String>("researchSensorsData", 5);
            
        }
        //Need to think about this and draw it out. Think about using ROS or not for everything and anything. Need to think about how we should the standard of creating nodes and reading messages. There are multiple implementations, but one of them has to be the best.
        void ReadInputs(): Node("ReadInputsNode"){

        }
        void UpdateState(){std::cout << "Done";}
        void ExecuteDecisionLoop(){std::cout << "Done";}
        //Sends Commands to Thruster Queue
        void SendThrusterCommands(){std::cout << "Done";}
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr research_sensor_Subscription_;
};
int main(int argc, char * argv[]){
    //setup Robot during inilization.
    Setup initStateandConfig = Setup();
    ExecutiveMainLoop mainLoop(argc, argv);

    //these threads functions will have loops that go on for ever
    //these functions will have wait functions just in case with a queue system.
    //The main thread will continue doing other things to ensure these threads can keep going.
    std::jthread ReadInputsThread(&ExecutiveMainLoop::ReadInputs, &mainLoop);
    std::jthread UpdateStateThread(&ExecutiveMainLoop::UpdateState, &mainLoop);
    std::jthread ExecutiveDecisionLoopThread(&ExecutiveMainLoop::ExecuteDecisionLoop, &mainLoop);
    std::jthread SendThrusterCommandsThread(&ExecutiveMainLoop::SendThrusterCommands, &mainLoop);


    ReadInputsThread.join();
    UpdateStateThread.join();
    ExecutiveDecisionLoopThread.join();
    SendThrusterCommandsThread.join();

    return 0;
}