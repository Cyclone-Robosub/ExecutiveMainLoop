#ifndef TASK_HPP
#define TASK_HPP


#include <memory>
#include <optional>
#include <string>
#include <chrono>
#include <utility>
#include <queue>
#include <string>
#include "lib/crs_common/position/position.hpp"
#include "JSON_Parser/MissionAnalyser.hpp"

typedef std::shared_ptr<Position> waypointPtr;

struct Step {
  waypointPtr WaypointPointer;
  std::optional<std::string> VisionCommand;
  bool isInterruptable{false}; //Think About Hard vs Soft INT
  bool doBarrelRoll{false}; //Will also need to be more robust
  bool stopWorking{false};
  std::optional<std::pair<int,bool>> ManipulationCodeandStatus;

  //Make sure that the second of the pair when initalized is set to 0.
  std::optional<std::pair<double, double>> HoldWaypTime_TimeElapsed;

  // const static Waypoints pre-determined of vector?

  
  void StartTimer();
  void StopTimer();
  void CalcTimer();
  bool isTimerOn{false};
  std::chrono::steady_clock::time_point timeInital;
  
};

struct Task {
  std::string name;
  std::queue<Step> steps_queue;
};

#endif