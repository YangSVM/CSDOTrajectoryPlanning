#include "hybrid_a_star/hybrid_astar_interface.h"

HybriAStarInterface::HybriAStarInterface(
    const Instance& instance, int agent_id
) : SingleAgentSolver(instance, agent_id),
    env(instance.dimx, instance.dimy, instance.obstacles, dynamic_obstacles, instance.goal_states, agent_id),
    hybridAStar(env)
{

};

Path HybriAStarInterface::findOptimalPath(const set<int>& higher_agents, const vector<Path*>& paths, int agent){
    Path m_solution;
    dynamic_obstacles.clear();
    // TODO 两次插入，改为一次
    // insert the higher agents' path as dynamic obstacles
    for ( const int & a : higher_agents){
        for ( auto state : paths[a]->states){
            dynamic_obstacles.insert(
                std::pair<int, State>(
                    state.first.time,
                    State(state.first.x, state.first.y, state.first.yaw, state.first.time))
            );
        }
        State last_state = paths[a]->states.back().first;
        dynamic_obstacles.insert(
            std::pair<int, State>(
                    -last_state.time,
                    State(last_state.x, last_state.y, last_state.yaw, last_state.time))
        );

    }

    hybridAStar.resetDynamicObstacles( higher_agents, paths);
    bool success = hybridAStar.search(instance.start_states[agent], m_solution);

    return m_solution;
}
