#pragma once
#include "hybrid_a_star/environment.h"
#include "hybrid_a_star/planresult.h"
#include "hybrid_a_star/hybrid_astar.h"
#include "common/motion_planning.h"

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::Environment;

// Instantiate all template classes

using Environment_t =  libMultiRobotPlanning::Environment<Location, State, Action, double>;
using Path = libMultiRobotPlanning::PlanResult<State, Action, double>;
using HybridAStarPlanner =  libMultiRobotPlanning::HybridAStar<State, Action, double, 
        Environment_t> ;