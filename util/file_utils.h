#pragma once

#include "hybrid_a_star/hybrid_astar.h"
#include "hybrid_a_star/motion_planning.h"
#include "qp/post_process.h"


void dumpPlanResults(
    std::string fname_output, 
    const std::vector<libMultiRobotPlanning::PlanResult<State, Action, double>>& solution,
    double runtime
);

bool loadScenario(
    std::string inputFile, size_t& dimx, size_t& dimy,   std::unordered_set<Location>& obstacles,
    std::vector<State>& startStates, std::vector<State>& goalStates,  bool cbs_scenario
);
