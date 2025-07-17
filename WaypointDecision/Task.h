#include <array>
#include <memory>
#include <optional>

typedef std::shared_ptr<std::array<double, 6>> waypointPtr;
struct Task {
  waypointPtr WaypointPointer;
  bool NeedsVision;
  bool isInterruptable = false;
  std::optional<int> ManipulationCode;
  std::optional<int> timeHold;

  // const static Waypoints pre-determined of vector?
};