#pragma once

#include "hybrid_a_star/types.h"
#include "pbs/SingleAgentSolver.h"

// lowlevel planner for the PBS. need to replan.
class WinHybriAStarInterface
{
public:
    WinHybriAStarInterface(
        const Instance& instance, int agent_id, int T_plan
    ) ;

    Path findOptimalPath(
        const State& start_state,   const std::set<int>& higher_agents, 
        const std::vector<Path*>& paths, int agent);
    std::string getName() const  { return "hybrid_a_star_win";}; 

	const Instance& instance;

private:

    Environment_t env;
    HybridAStarPlanner hybridAStar;

    int T_plan;

};




