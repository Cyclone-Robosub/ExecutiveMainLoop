#include <array>
#include <memory>
#include <optional>
#include <string>
#include <utility>

typedef std::shared_ptr<std::array<float, 6>> waypointPtr;
struct Task {
  waypointPtr WaypointPointer;
  bool NeedsVision{false};
  bool isInterruptable{false};
  std::optional<int> ManipulationCode;

  //Make sure that the second of the pair when initalized is set to 0.
  std::optional<std::pair<double, double>> HoldWaypTime_TimeElapsed;

  // const static Waypoints pre-determined of vector?
};