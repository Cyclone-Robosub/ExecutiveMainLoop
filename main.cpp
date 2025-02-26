#include <iostream>
#include <filesystem>
#include "SetupConfig/Setup.cpp"
#include <thread>
#include <utility>
#include <string>
#include <chrono>
using namespace std::literals;
namespace fs = std::filesystem;
//start the executive Loop
class ExecutiveMainLoop{
    public:
        ExecutiveMainLoop() {
            std::cout << "Done";
        }
        void ReadInputs(){std::cout << "Done";}
        void UpdateState(){std::cout << "Done";}
        void ExecuteDecisionLoop(){std::cout << "Done";}
        //Sends Commands to Thruster Queue
        void SendThrusterCommands(){std::cout << "Done";}
private:
};
int main(){
    //setup Robot during inilization.
    Setup initStateandConfig = Setup();
    ExecutiveMainLoop mainLoop;

    //can't get std::jthread to work
    std::thread ReadInputsThread(&ExecutiveMainLoop::ReadInputs, &mainLoop);
    std::thread UpdateStateThread(&ExecutiveMainLoop::UpdateState, &mainLoop);
    std::thread ExecutiveDecisionLoopThread(&ExecutiveMainLoop::ExecuteDecisionLoop, &mainLoop);
    std::thread SendThrusterCommandsThread(&ExecutiveMainLoop::SendThrusterCommands, &mainLoop);


    ReadInputsThread.join();
    UpdateStateThread.join();
    ExecutiveDecisionLoopThread.join();
    SendThrusterCommandsThread.join();

    return 0;
}