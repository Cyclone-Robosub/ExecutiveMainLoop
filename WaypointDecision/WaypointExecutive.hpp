#include "JSON_Parser/MissionAnalyser.hpp"
#include "Task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cstddef>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <queue>

/*
Notes:
  INT stands for interrupts.
  Theoretically, interrupts that occur while another interrupt is being serviced
or checked could be never attended to as the order in which the ServiceINTofTask
or services all the interrupts specifies priority. This could be solved by a
queue of Interrupts, but then there is a loss of actual priority. Thus a data
structure of the interrupts, perferably a max heap, that sorts itself is a
solution. Multithreading is another solution.


  Vision and Manipulation INT should be able to work together and apart. There
will probably be functions, but understanding the outputs and inputs from Vision
and Manipulation should solve this implementation problem.


*/
///@TODO: 
// Tasks to Complete 
// Interrupt Handling 
//    Controller vs Task Interrupts Handling and Implementaion Location 
//    Then Finish up CheckINTofStep and ServiceINTofStep
//    Upload Design Image.
// ROS2
//     Vision and Manipulation ROS Topics 
//     Custom MSG for Float or just get_x ...
// Manipulation Code Checkup with ManipulationTask() updated.
// StopWorking mechanism in Controller 
// Timer Implementation Location Change to Task.hpp



struct Interrupts {
  bool SOCDANGER{false};
  bool BINS_SPOTTED{false};
  bool DROP_INTO_BINS{false};
  bool TriggerManipSendCode{false};
};

class WaypointExecutive {
  WaypointExecutive() : MissionQueue("JSON_Parser/MissionPath.JSON") {
    SetupROS();
    Controller();
  }

private: 
  void SetupROS();
  void Controller();
  void SendCurrentWaypoint();
  void getNewMissionStep();
  void getNewMissionTask();
  bool isCurrentStepCompleted();
  void ManipulationStep();
  void CheckINTofStep();
  void ServiceINTofStep();
  // publisher of CurrentWaypointPtr topic.
  std::optional<bool> isSOCINT;
  waypointPtr CurrentWaypointPtr;
  Task CurrentTask;
  Step CurrentStep;
  std::queue<Interrupts>
      Current_Interrupts; // Review the priority queue < opreator between two
                          // elements with void pointers.
  // Need to resolve the Time Elapsed and Counting.

  // callback ROS2 functions
  
  rclcpp::Publisher<std_msgs::msg::Float32Array>::SharedPtr WaypointPublisher;
  rclcpp::Subscriber<std_msgs::msg::String>::SharedPtr VisionSub;
  rclcpp::Subscriber<std_msgs::msg::Bool>::SharedPtr SOCINTSub;
  
  bool StopWorking{false};
  void StartTimer();
  void StopTimer();
  void CalcTimer();
  bool isTimerOn{false};
  std::chrono::steady_clock::time_point timeInital;
  MissionAnalyser MissionQueue;
  bool MetPositionandTimeReq();
};
