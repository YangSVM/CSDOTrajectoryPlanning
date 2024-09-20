#pragma once

#include <vector>
#include <iostream>     // std::cout, std::fixed

using std::vector;
using std::cout;
using std::endl;



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

// parmameters of DSQP
struct QpParm{
  double r_trust;
  double max_omega;
  double max_v;
  double max_iter;
  double delta_solution_threshold;
  double max_violation;

  int osqp_max_iter;

  double dt;
  int num_interpolation;
  bool fixed_corridor;
};

}