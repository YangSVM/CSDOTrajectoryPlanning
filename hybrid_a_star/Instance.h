#pragma once
#include"pbs/common.h"
#include "hybrid_a_star/motion_planning.h"
#include "hybrid_a_star/types.h"

// Currently only works for undirected unweighted 4-nighbor grids
class Instance 
{
public:
	Instance(){}
	Instance(
		const string& instance_fname, int num_of_agents = -1,
		int num_of_obstacles = -1, int warehouse_width = 0, 
		bool cbs_scenario= true);

	void printAgents() const;

	int getDefaultNumberOfAgents() const { return num_of_agents; }
    bool startAndGoalValid(
        const std::vector<State> &starts, const std::vector<State>& goals) ;


    string instance_fname;

    int num_of_agents;
    int num_of_obstacles;

    bool cbs_scenario;

    size_t dimx;
    size_t dimy;
    std::unordered_set<Location> obstacles;
    std::vector<State> start_states; 
    std::vector<State> goal_states; 

private:

    bool loadMap();
    // Class  SingleAgentSolver can access private members of Node 
    friend class SingleAgentSolver;
};

