#include "Task.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>
#include <iostream>
#include <fstream>

/*
Notes:
  INT stands for interrupts.
  The Interrupt code right now is rededaunt.
  
*/

struct Interrupt {
  bool SOCDanger{false};
};

class WaypointExecutive {
  WaypointExecutive() {
    Controller();
  }

private:
  void Controller();
  void SendCurrentWaypoint();
  void getNewMissionCommand();
  bool isTaskNotCompleted();
  std::optional<Interrupt> CheckINTofTask();
  void ServiceINTofTask();
  // publisher of CurrentWaypointPtr topic.
  bool isSOCINT{false};
  waypointPtr CurrentWaypointPtr;
  Task CurrentTask;
  std::optional<Interrupt> INT_Flag;
  //Need to resolve the Time Elapsed and Counting.

  //callback ROS2 functions
  /*
    positionCallback->positionvar
    visionCallback->Vision Data Fields
  */
};
