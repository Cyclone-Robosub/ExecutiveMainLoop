#include "Task.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cstddef>
#include <memory>
#include <iostream>
#include <queue>
#include <fstream>

/*
Notes:
  INT stands for interrupts.
  Theoretically, interrupts that occur while another interrupt is being serviced or checked could be never attended to as the order in which the ServiceINTofTask or services all the interrupts specifies priority. This could be solved by a queue of Interrupts, but then there is a loss of actual priority. Thus a data structure of the interrupts, perferably a max heap, that sorts itself is a solution. Multithreading is another solution.


  Vision and Manipulation INT should be able to work together and apart. There will probably be functions, but understanding the outputs and inputs from Vision and Manipulation should solve this implementation problem. 
*/

struct Interrupts {
  bool SOCDanger{false};
  //Vision I See The Bins
  //Vision Drop Objects;
  //ManipulationSendCode
};

class WaypointExecutive {
  WaypointExecutive() {
    Controller();
  }

private:
  void Controller();
  void SendCurrentWaypoint();
  void getNewMissionCommand();
  bool isCurrentTaskCompleted();
  void ManipulationTask();
  void CheckINTofTask();
  void ServiceINTofTask();
  // publisher of CurrentWaypointPtr topic.
  std::optional<bool> isSOCINT;
  waypointPtr CurrentWaypointPtr;
  Task CurrentTask;
  std::queue<Interrupts> Current_Interrupts; //Review the priority queue < opreator between two elements with void pointers.
  //Need to resolve the Time Elapsed and Counting.

  //callback ROS2 functions
  /*
    positionCallback->positionvar
    visionCallback->Vision Data Fields
  */
  bool StopWorking{false};
  bool AllTasksCompleted{false};
  void StartTimer();
  void StopTimer();
  std::chrono::steady_clock::time_point timeInital;

};
