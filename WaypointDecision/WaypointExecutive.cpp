#include "WaypointExecutive.h"
#include <memory>

///
void WaypointExecutive::Controller() {
  while (true) {
    getMissionCommand();
    while (isTaskNotCompleted()) {
      if (CheckINTofTask()) {
        ServiceINTofTask();
      } else {
        SetCurrentWaypoint();
      }
    }
  }
}

///@brief : O(1) Algo and no conditional waiting.
void WaypointExecutive::SetCurrentWaypoint() {
  // check position
  MissionWaypoint.WaypointPointer;

  // check vision if needed
  // listen to the vision topic;

  // check heading and position

  // check battery
  if (isSOCINT) {
    // Pre-determined Waypoint; //Change the Mission Waypoint
  }
  // check real-time requirement
  CurrentWaypointPtr = std::make_shared<waypointPtr>();
}

///@brief O(1) Algo and no conditional waiting. Has authority of changing CurrentTask.
WaypointMission WaypointExecutive::getMissionCommand() {
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