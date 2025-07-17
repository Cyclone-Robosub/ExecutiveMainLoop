#include "WaypointMission.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <memory>

class WaypointExecutive {
  WaypointExecutive() {}
  void SetCurrentWaypoint();
  WaypointMission getMission();
  // publisher of Waypoint topic.
  private:
  bool isSOCINT{false};
  WaypointMission CurrentWaypointMission;
};