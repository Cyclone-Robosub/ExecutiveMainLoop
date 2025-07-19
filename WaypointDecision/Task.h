#include <array>
#include <memory>
#include <optional>
#include <string>
#include <utility>

typedef std::shared_ptr<std::array<float, 6>> waypointPtr;
struct Task {
  waypointPtr WaypointPointer;
  bool NeedsVision{false}; //Will become more robust.
  bool isInterruptable{false}; //Think About Hard vs Soft INT
  std::optional<std::pair<int,bool>> ManipulationCodeandStatus;

  //Make sure that the second of the pair when initalized is set to 0.
  std::optional<std::pair<double, double>> HoldWaypTime_TimeElapsed;

  // const static Waypoints pre-determined of vector?
};