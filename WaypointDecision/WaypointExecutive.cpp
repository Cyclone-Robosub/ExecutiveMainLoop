#include "WaypointExecutive.h"
#include <memory>


void WaypointExecutive::Controller() {
  while (!AllTasksCompleted || !StopWorking) {
    getNewMissionCommand();
    SendCurrentWaypoint();
    while (!isCurrentTaskCompleted()) {
     // if (CurrentTask.isInterruptable) { //Think about Hard vs Soft INT
        CheckINTofTask(); // Potentially Conditional Unresponsive Function (should not be as development continues)
        if (!Current_Interrupts.empty()) {
          ServiceINTofTask();
        }
      //}
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
bool WaypointExecutive::isCurrentTaskCompleted() {
  // check position var (include tolerance) with CurrentWaypointPtr
  // if position not met -> return false;
  if (HoldWaypTime_TimeElapsed.has_value()) {
    if(HoldWaypTime_TimeElapsed.first > HoldWaypTime_TimeElapsed.second){
      return false;
    }
  }
  if(ManipulationCodeandStatus.has_value()){
    if(!ManipulationCodeandStatus.second){
      return false;
    }
  }
  // else
  return true;
}
///@brief Conditional waiting for some and for now. Returns a optional if there is a condition met such as position or vision has sent some data over.
void WaypointExecutive::CheckINTofTask() {
  Interrupts generateINT;
  // check vision if needed -> Manipulation Tasks can be coded apart and along side this vision requriment along with position and altitude.
  if (CurrentTask.NeedsVision) {
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
///Current Interrupt at the end.
void WaypointExecutive::ServiceINTofTask() {
  Interrupts ServiceINT = Current_Interrupts.top();
  if(ServiceINT.SOCDanger){
    //Battery WayPoint
    CurrentTask = {.WaypointPointer = std::make_shared<waypointPtr>()}; //Creating a new.
    SendCurrentWaypoint();
    StopWorking = true;
  }

  if(ServiceINT.ManipulationCodeSend){
    //Manipulation_Publisher ->publish( CurrentTask.ManipulationCodeandStatus.first);
    CurrentTask.ManipulationCodeandStatus.second = true;
  }
  Current_Interrupts.pop();
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