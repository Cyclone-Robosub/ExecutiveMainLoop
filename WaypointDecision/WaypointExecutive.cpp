#include "WaypointExecutive.hpp"
#include <chrono>
#include <memory>

void WaypointExecutive::SetupROS() {
  WaypointPublisher =
      this->create_publisher<std_msgs::msg::Float32Array>("waypoint_topic", 10);
  SOCINTSub = this->create_subscription<std_msgs::msg::Bool>("SOCIntTopic", 10);
  VisionSub =
      this->create_subscription<std_msgs::msg::String>("VisionTopic", 10);
}

//Need to think about a better StopWorking mechanism. 
void WaypointExecutive::Controller() {
  MissionQueue.parseJSONForMission();
  while (!MissionQueue.allTasksComplete() || !StopWorking) {
    getNewMissionTask();
    while (!CurrentTask.steps.empty() || !StopWorking) {
      getNewMissionStep();
      SendCurrentWaypoint();
      while (!isCurrentStepCompleted() || !StopWorking) {
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
  //Use Custom msg.
  WaypointPublisher->publish((CurrentWaypointPtr.get_array_copy()));
  // start the timer
}
///@brief O(1) and no conditional waiting. Returns True if the task should still
/// run.
bool WaypointExecutive::isCurrentStepCompleted() {
  if (!MetPositionandTimeReq()) {
    return false;
  }
  if (ManipulationCodeandStatus.has_value()) {
    if (!ManipulationCodeandStatus.second) {
      return false;
    }
  }
  // else
  return true;
}
///@brief Conditional waiting for some and for now. Returns a optional if there
/// is a condition met such as position or vision has sent some data over.
void WaypointExecutive::CheckINTofStep() {
  Interrupts generateINT;
  // check vision if needed -> Manipulation Tasks can be coded apart and along
  // side this vision requriment along with position and altitude.
  if (CurrentStep.NeedsVision) {
    /*if(VisionChangedorSentSomethingOveroneofTheTopics)
    {
     return true;
    }*/

    /*
    if(Vision I See the Bins Flag){
      Reset the Flag.
      interrupt.BINSSPOTTED = true;
    }
    */
    /*
        if(Vision Drop Flag and Manipulation Code for this Current Task)
          {
            if(Altitude is good)
            {
              Vision reset flag.
              interrupt.DROPTHEBINS;
            }
        }
    */
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
  }

  if (ServiceINT.ManipulationCodeSend) {
    // Manipulation_Publisher ->publish(
    // CurrentTask.ManipulationCodeandStatus.first);
    CurrentTask.ManipulationCodeandStatus.second = true;
  }
  Current_Interrupts.pop();
}


void WaypointExecutive::getNewMissionTask(){
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
  // if position not met -> if(optional) StopTimer(); //Missed after reaching
  // it. then return false; else if position met {
  if (CurrentStep.HoldWaypTime_TimeElapsed.has_value()) {
    if (!isTimerOn) {
      StartTimer();
    }
  }
  //}

  // Time Req
  if (CurrentStep.HoldWaypTime_TimeElapsed.has_value()) {
    CalcTimer();
    if (CurrentStep.HoldWaypTime_TimeElapsed.first >
        CurrentStep.HoldWaypTime_TimeElapsed.second) {
      return false;
    }
  }
  return true;
}



//These Timer functions should be put into the Step Struct to avoid confusion of a timer for WaypointExecutive vs timer for Current Step.

///@brief O(1) Algo and no conditional waiting. Save the current time in
/// timeInital.
void WaypointExecutive::StartTimer() {
  timeInital = std::chrono::steady_clock::now();
  isTimerOn = true;
}

///@brief O(1) Algo and no conditional waiting.
void WaypointExecutive::StopTimer() { isTimerOn = false; }
///@brief O(1) Algo and no conditional waiting. Add time to the Elapsed Time of
/// Current Step.
void WaypointExecutive::CalcTimer() {
  auto deltaTime = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - timeInital)
                       .count();
  CurrentTask.HoldWaypTime_TimeElapsed.second += deltaTime;
  timeInital = std::chrono::steady_clock::now();
}