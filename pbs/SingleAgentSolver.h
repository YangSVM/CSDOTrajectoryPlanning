#pragma once
#include "hybrid_a_star/Instance.h"
#include "pbs/common.h"


class SingleAgentSolver
{
public:
	uint64_t num_expanded = 0;
	uint64_t num_generated = 0;

	double runtime_build_CT = 0; // runtimr of building constraint table
	double runtime_build_CAT = 0; // runtime of building conflict avoidance table
	


	const Instance& instance;

	virtual Path findOptimalPath(const set<int>& higher_agents, const vector<Path*>& paths, int agent) = 0;
	virtual string getName() const = 0;


	SingleAgentSolver(const Instance& instance) :
		instance(instance){	};

  virtual ~SingleAgentSolver(){} ;

protected:
	int min_f_val; // minimal f value in OPEN
	double w = 1; // suboptimal bound

};

