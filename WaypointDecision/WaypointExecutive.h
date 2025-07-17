#include "Task.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>
#include <iostream>
#include <fstream>

class WaypointExecutive {
  WaypointExecutive() {
    Controller();
  }
  void Controller();
  void SetCurrentWaypoint();
  void getMissionCommand();
  bool isTaskNotCompleted();
  bool CheckINTofTask();
  void ServiceINTofTask();
  // publisher of CurrentWaypointPtr topic.
  private:
  bool isSOCINT{false};
  waypointPtr CurrentWaypointPtr;
  Task CurrentTask;
  //Task MissionWaypoint;

};