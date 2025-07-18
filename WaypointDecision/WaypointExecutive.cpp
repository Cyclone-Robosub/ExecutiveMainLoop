#include "WaypointExecutive.h"
#include <memory>


void WaypointExecutive::Controller() {
  while (true) {
    getNewMissionCommand();
    SendCurrentWaypoint();
    while (isTaskNotCompleted()) {
      if (CurrentTask.isInterruptable) { //Think about Hard vs Soft INT
        INT_Flag = CheckINTofTask(); // Potential Conditional Unresponsive Function
        if (INT_Flag.has_value()) {
          ServiceINTofTask();
        }
      }
    }
  }
}
///@brief O(1) Algo and no conditional waiting.
void WaypointExecutive::SendCurrentWaypoint() {
  CurrentWaypointPtr = CurrentTask.WaypointPointer;
  // Publisher-> publish the ptr->Array of Float;
}
///@brief O(1) and no conditional waiting. Returns True if the task should still
/// run.
bool WaypointExecutive::isTaskNotCompleted() {
  // check position var (include tolerance) with CurrentWaypointPtr
  // if position not met -> return true
  // else
  if (HoldWaypTime_TimeElapsed.has_value()) {
    (HoldWaypTime_TimeElapsed.first > HoldWaypTime_TimeElapsed.second) ? true
                                                                       : false;
  }
  // else
  return false;
}
///@brief Conditional waiting. Returns a value in the optional if there is a
/// condition met such as position or vision has sent some data over.
std::optional<Interrupt> WaypointExecutive::CheckINTofTask() {

  // check vision if needed -> Manipulation Tasks can be coded apart and along side this vision requriment along with position and altitude.
  if (CurrentTask.NeedsVision) {
    /*if(VisionChanged)
    {
     return true;
    }*/
  }
  // listen to the vision topic;

  // check battery
  if (isSOCINT.has_value()) {
    isSOCINT.reset();
    return {.SOCDanger = true}// INT_CODE;
  }
  return std::nullopt;
}

///@brief Service the INT. Clear the INT_Flag at the end.
void WaypointExecutive::ServiceINTofTask() {
  if(INT_Flag.SOCDanger){
    //Battery WayPoint
    CurrentTask = {.WaypointPointer = };
  }
  SendCurrentWaypoint();
  INT_Flag.reset();
}

///@brief O(1) Algo and no conditional waiting. Has authority of changing
/// CurrentTask.
void WaypointExecutive::getNewMissionCommand() {
  // fetch or predetermined waypoints.

  // Fetch
  // std::getline or what ever the JSON equivlent.
  // make new Task Object
  // Fill in Data
  // Return

  // Predetermined -> Waypoint Objects?
}

void WaypointExecutive::SOCIntCallback(const std_msgs::Bool::SharedPtr msg) {
  isSOCINT = msg->data;
}

void WaypointExecutive::ManipulationTask(){
  //Send Manipulation Code over Publisher.
}