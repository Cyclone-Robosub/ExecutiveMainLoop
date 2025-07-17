#include <array>
#include <memory>

typedef std::shared_ptr<std::array<float, 6>> waypoint;
struct WaypointMission{
	waypoint WaypointPointer;
	bool NeedsVision;
	bool NeedsManipulation;
	int timeSuspended = -1;
};