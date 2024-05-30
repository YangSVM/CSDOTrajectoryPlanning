#pragma once
#include <vector>
#include "hybrid_a_star/planresult.h"
#include "hybrid_a_star/motion_planning.h"
#include "qp/corridor.h"

namespace libMultiRobotPlanning {

struct OptimizeResult{
    double x;
    double y;
    double yaw;
    double v;
    double a;
    double steer;
    double d_steer;
};


void calcActionD(const int& action, const double& deltat_abs,
    double& dx, double& dy, double& dyaw, double r);

// 注意要重新计算角度值。变量连续能够喂进优化器
void action_sample(const int& action,
    PlanResultShort<State, Action, double>& solution_refine,
    const int& index, const State& s0, const State& s1, const int& n);

/**
 * @param n sample number

 */
bool sample_path( const std::vector<PlanResult<State, Action, double>>& solution, 
    std::vector<PlanResultShort<State, Action, double>>& solution_refine,
    const int& n);

bool calcInitGuess(const std::vector<PlanResultShort<State, Action, double>>& solutions,
std::vector<std::vector<OptimizeResult>>& guesses, size_t Nt, size_t Na,
double dt
);

struct SolutionStatistics{
  double cost = -1;
  double makespan = -1; // makespan record the value before optimization.
  double flowtime = -1;
  double runtime = -1;  // total runtime. rt_search + rt_optimization
  double rt_search = -1;  // runtime of front end searching
  double rt_preprocess = -1;  // runtime of preprocessing
  double rt_optimization = -1; // runtime of total optimization
  double rt_max_optimization = -1;  // runtime while fully decentralized time
  int search_status = 2;  // 0: failed; 1: minor collision; 2: success.
  int solver_status = 0;   // 0: failed; 1: success; 
};

void dumpSolutions(
  std::string file_name,
  const std::vector<std::vector<OptimizeResult>>& solutions,
  const SolutionStatistics& stat
  );

void dumpCorridors(
  std::string file_name,
  const std::vector<std::vector<Corridor>>& corridors,
  const std::vector<PlanResultShort<State, Action, double>>& solution_refine
  );
void findCollisionPair(
    const std::vector<std::vector<OptimizeResult>>& guessess,
    std::vector< std::array<int, 3>>& relative_pair
);

// void readSolverConfig(libMultiRobotPlanning::NloptParam& param);


}


