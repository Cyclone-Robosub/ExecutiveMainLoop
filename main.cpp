#include <iostream>
#include <filesystem>
#include "SetupConfig/Setup.cpp"
#include <thread>
#include <utility>

namespace fs = std::filesystem;
//start the executive Loop
class ExecutiveMainLoop{
    public:
        ExecutiveMainLoop();
        void ReadInputs();
        void UpdateState();
        void ExecuteDecisionLoop();
        //Sends Commands to Thruster Queue
        void SendThrusterCommands();
    private:
};
int main(){
    //setup Robot during inilization.
    Setup initStateandConfig = Setup();
    ExecutiveMainLoop mainLoop = ExecutiveMainLoop();
    std::jthread ReadInputsThread(&ExecutiveMainLoop::ReadInputs, &mainLoop);
    std::jthread UpdateState(&ExecutiveMainLoop::UpdateState, &mainLoop);
    std::jthread ExecutiveDecisionLoop(&ExecutiveMainLoop::ExecuteDecisionLoop, &mainLoop);
    std::jthread SendThrusterCommands(&ExecutiveMainLoop::SendThrusterCommands, &mainLoop);


    return 0;
}