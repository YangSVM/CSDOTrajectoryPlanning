#pragma once

#include "hybrid_a_star/types.h"
#include "pbs/SingleAgentSolver.h"

// lowlevel planner for the PBS. need to replan.
class HybriAStarInterface: public SingleAgentSolver
{
public:
    HybriAStarInterface(
        const Instance& instance, int agent_id
    ) ;

    Path findOptimalPath(const std::set<int>& higher_agents, const std::vector<Path*>& paths, int agent) override;
    std::string getName() const override { return "hybrid_a_star";}; 


private:
    std::multimap<int, State> dynamic_obstacles;

    Environment_t env;
    HybridAStarPlanner hybridAStar;

};




