#include "WaypointExecutive.hpp"
#include <chrono>
#include <fstream>
#include <memory>

void WaypointExecutive::SetupROS() {
  WaypointPublisher =
      this->create_publisher<std_msgs::msg::Float32Array>("waypoint_topic", 10);
  SOCINTSub = this->create_subscription<std_msgs::msg::Bool>("SOCIntTopic", 10);
  VisionSub =
      this->create_subscription<std_msgs::msg::String>("VisionTopic", 10);
}

// Need to think about a better StopWorking mechanism.
void WaypointExecutive::Controller() {
  MissionQueue.parseJSONForMission();
  while (!MissionQueue.allTasksComplete()) {
    getNewMissionTask();
    while (!CurrentTask.steps.empty()) {
      getNewMissionStep();
      SendCurrentWaypoint();
      while (!isCurrentStepCompleted()) {
        // if (CurrentTask.isInterruptable) { //Think about Hard vs Soft INT
        CheckINTofStep(); // Potentially Conditional Unresponsive Function
                          // (should not be as development continues)
        if (!Current_Interrupts.empty()) {
          ServiceINTofStep();
        }
        //}
      }
    }
  }
}
///@brief O(1) Algo and no conditional waiting.
void WaypointExecutive::SendCurrentWaypoint() {
  CurrentWaypointPtr = CurrentStep.WaypointPointer;
  // Use Custom msg.
  WaypointPublisher->publish((CurrentWaypointPtr.get_array_copy()));
  // start the timer
}
///@brief O(1) and no conditional waiting. Returns True if the task should still
/// run.
bool WaypointExecutive::isCurrentStepCompleted() {
  if (!MetPositionandTimeReq()) {
    return false;
  }

  //use the vision condition option to check if done?

  if (ManipulationCodeandStatus.has_value()) {
    if (!ManipulationCodeandStatus.second) {
      return false;
    }
  }
  // else
  return true;
}
///@brief Conditional waiting for some and for now. Pushes INT instance to the queue of Current Queue.
void WaypointExecutive::CheckINTofStep() {
  Interrupts generateINT;
  // check vision if needed -> Manipulation Tasks can be coded apart and along
  // side this vision requriment along with position and altitude.
  if (CurrentStep.VisionCommand.has_value()) {
    switch (VisionCommand) {
    case "BINS_SPOTTED":
      // Check Bool
      // Reset the Flag.
      // interrupt.BINSSPOTTED = true;
      break;
    case "DROP_INTO_BINS":
      /*if(Altitude is good)
          {
            Vision reset flag.
            interrupt.DROPTHEBINS;
          }*/
      break;
    default:
      break;
    }
  }
  // listen to the vision topic;

  // check battery
  if (isSOCINT.has_value()) {
    isSOCINT.reset();
    generateINT.SOCDanger = true;
    Current_Interrupts.push(generateINT);
  }
}

///@brief O(1) Algo and no conditional waiting. Service the INT. Clear the
/// Current Interrupt at the end.
void WaypointExecutive::ServiceINTofStep() {
  Interrupts ServiceINT = Current_Interrupts.top();
  if (ServiceINT.SOCDanger) {
    // Battery WayPoint
    CurrentTask = {.WaypointPointer =
                       std::make_shared<waypointPtr>()}; // Creating a new.
    SendCurrentWaypoint();
    StopWorking = true;
    EndReport();
  }
  if(ServiceINT.BINS_SPOTTED){
    //
  }
  if (ServiceINT.ManipulationCodeSend) {
    // Manipulation_Publisher ->publish(
    // CurrentTask.ManipulationCodeandStatus.first);
    CurrentTask.ManipulationCodeandStatus.second = true;
  }
  Current_Interrupts.pop();
}

void WaypointExecutive::getNewMissionTask() {
  CurrentTask = MissionQueue.popNextTask();
}

///@brief O(1) Algo and no conditional waiting. Has authority of changing
/// CurrentTask.
void WaypointExecutive::getNewMissionStep() {
  // fetch or predetermined waypoints.
  // Predetermined -> Waypoint Objects?
  CurrentStep = CurrentTask.pop();
}

void WaypointExecutive::SOCIntCallback(const std_msgs::Bool::SharedPtr msg) {
  isSOCINT = msg->data;
}

void WaypointExecutive::ManipulationTask() {
  // Send Manipulation Code over Publisher.
}

///@brief O(1) Algo and no conditional waiting.
bool WaypointExecutive::MetPositionandTimeReq() {
  // check position var (include tolerance) with CurrentWaypointPtr
  // if position not met -> if(optional) CurrentStep.StopTimer(); //Missed after reaching
  // it. then return false; else if position met {
  if (CurrentStep.HoldWaypTime_TimeElapsed.has_value()) {
    if (!CurrentStep.isTimerOn) {
     CurrentStep.StartTimer();
    }
  }
  //}

  // Time Req
  if (CurrentStep.HoldWaypTime_TimeElapsed.has_value()) {
    CurrentStep.CalcTimer();
    if (CurrentStep.HoldWaypTime_TimeElapsed.first >
        CurrentStep.HoldWaypTime_TimeElapsed.second) {
      return false;
    }
  }
  return true;
}


/// @brief: Will Exit itself after creating Report
void WaypointExecutive::EndReport(){
  std::ofstream ReportFile("End_Report");
  ReportFile << "___________START OF REPORT__________" << std::endl;
  ReportFile << "Reason for Report : ";
  if(StopWorking){
    ReportFile << "A predetermined flag to stop the controller." << std::endl;
  }else{
    ReportFile << "Finished all tasks." << std::endl;
  }
  ReportFile << "___________END OF REPORT ___________" << std::endl;
  ReportFile.close();
  ~WaypointExecutive();
}