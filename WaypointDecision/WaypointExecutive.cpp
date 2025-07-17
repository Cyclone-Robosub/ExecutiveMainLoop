#include "WaypointExecutive.h"



///@brief : O(1) Algo and no conditional waiting.
void WaypointExecutive::SetCurrentWaypoint() {
	WaypointMission MissionWaypoint = getMission(); 
	//check position
	MissionWaypoint.WaypointPointer;

	//check vision if needed
	//listen to the vision topic;

	//check heading
	//check battery
	if(isSOCINT){
		//Pre-determined Waypoint; //Change the Mission Waypoint
	}
	//check real-time requirement 
	CurrentWaypointMission = MissionWaypoint;
}

///@brief O(1) Algo and no conditional waiting.
WaypointMission WaypointExecutive::getMission(){
	//fetch or predetermined waypoints.

	//Fetch
	//std::getline
	//make new WaypointMission Object
	//Fill in Data
	//Return

	//Predetermined -> Waypoint Objects?
}

void WaypointExecutive::SOCIntCallback(const std_msgs::Bool::SharedPtr msg){
	isSOCINT = msg->data;
}