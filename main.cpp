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