#include "hybrid_a_star/hybrid_astar_interface.h"

HybriAStarInterface::HybriAStarInterface(
    const Instance& instance, int agent_id
) : SingleAgentSolver(instance),
    env(instance.dimx, instance.dimy, instance.obstacles, dynamic_obstacles, instance.goal_states, agent_id),
    hybridAStar(env)
{

};

Path HybriAStarInterface::findOptimalPath(const std::set<int>& higher_agents, const std::vector<Path*>& paths, int agent){
    Path m_solution;

    hybridAStar.resetDynamicObstacles( higher_agents, paths);
    hybridAStar.search(instance.start_states[agent], m_solution);

    return m_solution;
}
