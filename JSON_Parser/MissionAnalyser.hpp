#ifndef MISSION_ANALYSER_HPP
#define MISSION_ANALYSER_HPP

#include <string>
#include <queue>
#include "Task.h"
#include "lib/Json/json.hpp"
#include "lib/crs_common/position/position.hpp"

class MissionAnalyser {
    public:
        MissionAnalyser(std::string filePath);
        void parseJSONForMission();
        Task popNextTask();
        bool allTasksComplete();
    private:
        std::string filePath;
        std::queue<Task> mission;
        Position makePositionFromJSON(nlohmann::json::reference jsonData);

};
#endif