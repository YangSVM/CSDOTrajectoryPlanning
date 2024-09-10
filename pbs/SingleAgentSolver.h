#pragma once
#include "hybrid_a_star/Instance.h"
#include "pbs/common.h"

class LLNode // low-level node
{
public:
	int location;
	int g_val;
	int h_val = 0;
	LLNode* parent;
	int timestep = 0;
	int num_of_conflicts = 0;
	bool in_openlist = false;
	bool wait_at_goal; // the action is to wait at the goal vertex or not. This is used for >lenghth constraints
    bool is_goal = false;
	// the following is used to comapre nodes in the OPEN list
	struct compare_node
	{
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		bool operator()(const LLNode* n1, const LLNode* n2) const
		{
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
            {
                if (n1->num_of_conflicts == n2->num_of_conflicts)
                {
                    if (n1->h_val == n2->h_val)
                    {
                        return rand() % 2 == 0;   // break ties randomly
                    }
                    return n1->h_val >= n2->h_val;  // break ties towards smaller h_vals (closer to goal location)
                }
                return n1->num_of_conflicts >= n2->num_of_conflicts;  // n1 > n2 if it has more conflicts
            }
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

		// the following is used to compare nodes in the FOCAL list
	struct secondary_compare_node
	{
		bool operator()(const LLNode* n1, const LLNode* n2) const // returns true if n1 > n2
		{
			if (n1->num_of_conflicts == n2->num_of_conflicts)
			{
                if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                {
                    if (n1->h_val == n2->h_val)
                    {
                        return rand() % 2 == 0;   // break ties randomly
                    }
                    return n1->h_val >= n2->h_val;  // break ties towards smaller h_vals (closer to goal location)
                }
                return n1->g_val+n1->h_val >= n2->g_val+n2->h_val;  // break ties towards smaller f_vals (prefer shorter solutions)
			}
			return n1->num_of_conflicts >= n2->num_of_conflicts;  // n1 > n2 if it has more conflicts
		}
	};  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)


	LLNode() : location(0), g_val(0), h_val(0), parent(nullptr), timestep(0), num_of_conflicts(0), in_openlist(false), wait_at_goal(false) {}

	LLNode(int location, int g_val, int h_val, LLNode* parent, int timestep, int num_of_conflicts = 0, bool in_openlist = false) :
		location(location), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
		num_of_conflicts(num_of_conflicts), in_openlist(in_openlist), wait_at_goal(false) {}

	inline int getFVal() const { return g_val + h_val; }
	void copy(const LLNode& other)
	{
		location = other.location;
		g_val = other.g_val;
		h_val = other.h_val;
		parent = other.parent;
		timestep = other.timestep;
		num_of_conflicts = other.num_of_conflicts;
		wait_at_goal = other.wait_at_goal;
        is_goal = other.is_goal;
	}
};


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

