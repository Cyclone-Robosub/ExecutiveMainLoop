#include "Task.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>
#include <iostream>
#include <fstream>

/*
Notes:
  INT stands for interrupts.
  Theoretically, interrupts that occur while another interrupt is being serviced could be never attended to as the order in which the CheckINTofTask checks all the interrupts specifies priority. This could be solved by a queue of Interrupts, but then there is a loss of actual priority. Thus a data structure of the interrupts, perferably a max heap, that has the ability to sort itself (This will be a task for the controller to do so) is a solution. 


  Vision and Manipulation INT should be able to work together and apart. There will probably be functions, but understanding the outputs and inputs from Vision and Manipulation should solve this implementation problem. 
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
  void ManipulationTask();
  std::optional<Interrupt> CheckINTofTask();
  void ServiceINTofTask();
  // publisher of CurrentWaypointPtr topic.
  std::optional<bool> isSOCINT;
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
