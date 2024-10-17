#pragma once
#include "pbs/PBSNode.h"
#include "pbs/SingleAgentSolver.h"
// #include "hybrid_a_star/hybrid_astar_interface.h"
#include "hybrid_a_star/win_hybrid_astar_interface.h"

// windowed PBS
class WinPBS
{
public:
	/////////////////////////////////////////////////////////////////////////////////////
	// stats
	double runtime = 0;
	double runtime_generate_child = 0; // runtimr of generating child nodes
	double runtime_build_CT = 0; // runtimr of building constraint table
	double runtime_build_CAT = 0; // runtime of building conflict avoidance table
	double runtime_path_finding = 0; // runtime of finding paths for single agents
	double runtime_detect_conflicts = 0;
	double runtime_preprocessing = 0; // runtime of building heuristic table for the low level

	uint64_t num_HL_expanded = 0;
	uint64_t num_HL_generated = 0;
	uint64_t num_LL_expanded = 0;
	uint64_t num_LL_generated = 0;

	PBSNode* dummy_start = nullptr;
    PBSNode* goal_node = nullptr;



	bool solution_found = false;
	int solution_cost = -2;

	/////////////////////////////////////////////////////////////////////////////////////////
	// set params
	void setConflictSelectionRule(conflict_selection c) { conflict_seletion_rule = c;}
	void setNodeLimit(int n) { node_limit = n; }

	////////////////////////////////////////////////////////////////////////////////////////////
	// Runs the algorithm until the problem is solved or time is exhausted 
	bool solve(const vector<State>& starts, double time_limit);

	WinPBS(const Instance& instance,  int screen, int T_plan=10);
	void clearSearchEngines();
	~WinPBS();

	// Save results
	void saveResults(const string &fileName, const string &instanceName) const;
	void saveCT(const string &fileName) const; // write the CT to a file
    void savePaths(const string &fileName) const; // write the paths to a file
	void clear(); // used for rapid random  restart

	// output results
	bool getPaths( vector<Path>& solutions);

private:
	conflict_selection conflict_seletion_rule;

    stack<PBSNode*> open_list;
	list<PBSNode*> allNodes_table;


    list<int> ordered_agents;
    vector<vector<bool>> priority_graph; // [i][j] = true indicates that i is lower than j

    string getSolverName() const;

	int screen;
	
	double time_limit;
	int node_limit = MAX_NODES;

	clock_t start;

	int num_of_agents;

	int T_plan; // ! planning horizon.
	int t_start;


	vector<Path*> paths;
	// update to hybrid a star planner. can use member function directly.
	vector < WinHybriAStarInterface* > search_engines;  // used to find (single) agents' paths and mdd


    bool generateChild(int child_id, PBSNode* parent, int low, int high,
		const vector<State>& starts);

	bool hasConflicts(int a1, int a2) const;
    bool hasConflicts(int a1, const set<int>& agents) const;
	shared_ptr<Conflict> chooseConflict(const PBSNode &node) const;
    int getSumOfCosts() const;
	inline void releaseNodes();

	// print and save
	void printResults() const;
	static void printConflicts(const PBSNode &curr);
    void printPriorityGraph() const;

	bool validateSolution() const;


	bool terminate(PBSNode* curr); // check the stop condition and return true if it meets

    void getHigherPriorityAgents(const list<int>::reverse_iterator & p1, set<int>& agents);
    void getLowerPriorityAgents(const list<int>::iterator & p1, set<int>& agents);
    bool hasHigherPriority(int low, int high) const; // return true if agent low is lower than agent high

	// node operators
	void pushNode(PBSNode* node);
    void pushNodes(PBSNode* n1, PBSNode* n2);
	PBSNode* selectNode();

		 // high level search
	bool generateRoot(const vector<State>& starts);
    bool findPathForSingleAgent(const State& start, PBSNode& node, const set<int>& higher_agents, int a, Path& new_path);
	void update(PBSNode* node);

    void topologicalSort(list<int>& stack);
    void topologicalSortUtil(int v, vector<bool> & visited, list<int> & stack);

	bool SwapConflict(State& n1, State& n2) const;

	bool TargetConflict(State& n1, State& n2) const;
};
