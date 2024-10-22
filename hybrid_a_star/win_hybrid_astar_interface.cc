#include "hybrid_a_star/win_hybrid_astar_interface.h"

WinHybriAStarInterface::WinHybriAStarInterface(
    const Instance& instance, int agent_id, int T_plan
) : 
    env(instance.dimx, instance.dimy, instance.obstacles, 
    std::multimap<int, State>(), instance.goal_states, agent_id),
    hybridAStar(env) , instance(instance), T_plan(T_plan)
{

};

Path WinHybriAStarInterface::findOptimalPath(
    const State& start_state,
    const std::set<int>& higher_agents, const std::vector<Path*>& paths, int agent){
    Path m_solution;

    // hybridAStar.resetDynamicObstacles( higher_agents, paths);
    int t_plan_end = start_state.time + T_plan;
    hybridAStar.resetPartTimeDynamicObstacles( higher_agents, paths, T_plan);
    hybridAStar.search(start_state, m_solution);

    return m_solution;
}


bool WinHybriAStarInterface::detectRunover( const State& start, const State& s){
    return hybridAStar.detectRunover(start, s);
    
}
