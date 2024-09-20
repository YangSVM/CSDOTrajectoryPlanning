#pragma once
#include <vector>
#include "hybrid_a_star/planresult.h"
#include "common/motion_planning.h"
#include "sqp/common.h"

namespace libMultiRobotPlanning {



void InterpolateInitalGuess(const std::vector<PlanResult<State, Action, double>>& solution, 
  std::vector<std::vector<OptimizeResult>>& x0_bar, const QpParm& qp_parm);

void calcActionD(const int& action, const double& deltat_abs,
    double& dx, double& dy, double& dyaw, double r);

// 注意要重新计算角度值。变量连续能够喂进优化器
void action_sample(const int& action,
    PlanResultShort<State, Action, double>& solution_refine,
    const int& index, const State& s0, const State& s1, const int& n);


bool interpolateXYYaw( const std::vector<PlanResult<State, Action, double>>& solution, 
    std::vector<PlanResultShort<State, Action, double>>& solution_refine,
    const int& n);

bool calcVSteerW(const std::vector<PlanResultShort<State, Action, double>>& solutions,
std::vector<std::vector<OptimizeResult>>& guesses, double dt);


void dumpSolutions(
  std::string file_name,
  const std::vector<std::vector<OptimizeResult>>& solutions,
  const SolutionStatistics& stat
  );


// ------------- neighbor pair division -----------//
bool  findNeighborPairsByTrustRegion(
    const std::vector<std::vector<OptimizeResult>>& solution,
    const double& r, // trust region radius
    const double& rv,
    std::vector< std::array<int, 3>>& neighbor_pairs
);

struct InterPlane{
  int t;
  double a_f2f,b_f2f, c_f2f; // front circle to other's front circle
  double a_f2r,b_f2r, c_f2r; // front circle to other's rear circle
  double a_r2f,b_r2f, c_r2f; // rear circle to other's front circle
  double a_r2r,b_r2r, c_r2r; // rear circle to other's rear circle


  InterPlane(
    int t, 
    double a_f2f, double b_f2f, double c_f2f, double a_f2r, double b_f2r, double c_f2r,
    double a_r2f, double b_r2f, double c_r2f, double a_r2r, double b_r2r, double c_r2r
  )
    : t(t), a_f2f(a_f2f), b_f2f(b_f2f), c_f2f(c_f2f), a_f2r(a_f2r), b_f2r(b_f2r), c_f2r(c_f2r),
    a_r2f(a_r2f), b_r2f(b_r2f), c_r2f(c_r2f), a_r2r(a_r2r), b_r2r(b_r2r), c_r2r(c_r2r)
    {}
};


void calcPerpendicular(double x1, double y1, double x2, double y2,
  double& a, double&b, double& c1, double& c2);

void calcEqualInterPlanes(
  const std::vector<std::vector<OptimizeResult>>& x0_bar, 
  const std::vector<std::array<int, 3>>& neighbor_pairs,
  vector<vector<InterPlane>>& inter_planes
);

}


