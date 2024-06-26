#pragma once
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Core>
#include "osqp/osqp.h"
#include "hybrid_a_star/motion_planning.h"
#include "qp/corridor.h"
#include "hybrid_a_star/timer.h"
#include "qp/post_process.h"
#include "qp/utils.h"

using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;
using Eigen::ArrayXd;
using std::vector;
typedef Eigen::SparseMatrix<double> SpMat;
// triplet. 3 element tuple. (row_id, col_id, value)
typedef Eigen::Triplet<double> Triplet;  
using libMultiRobotPlanning::Corridor;

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

class SolverDQP{
public:
  SolverDQP(
    size_t num_agent, size_t max_time, 
    vector<vector<OptimizeResult>>& solutions,
    const std::vector<std::vector<OptimizeResult>>& guesses,
    const std::vector<std::array<int, 3>>& relative_pair,
    const std::vector<std::vector<Corridor>>& corridors,
    double dimx, double dimy,
    const std::unordered_set<Location>& obstacles,
    const QpParm& param,
    int logger_level = 2
  );
  int solveOSQP(
    int a,
    VectorXd&  solution_vec,
    SpMat& M, const VectorXd& ub, const VectorXd& lb,
    SpMat& P, const VectorXd& q
  );
  inline int getSolverStatus(){ return solve_status;};
  // inline int getSearchStatus(){ return search_status;};
  inline double  getMaxOfRuntimes(){ return max_individual_opt_runtime;};
  
  std::vector<int> num_iterations;

private:
  bool calcIndividualQP(
    int a, 
    vector<OptimizeResult>& solution,
    const std::vector<std::vector<Corridor>>& corridors,
    const std::vector<std::vector<InterPlane>>& inter_planes,
    const   vector<double>& cfg,
    const QpParm& param
  );

  void calcEqualInterPlanes(
    const ArrayXd& xfs0, const ArrayXd& yfs0, 
    const ArrayXd& xrs0, const ArrayXd& yrs0,
    const std::vector<std::array<int, 3>>& relative_pair,
    vector<vector<InterPlane>>& inter_planes
  );

  void calcKineConstraint(
    int a, SpMat& M, VectorXd& ub, VectorXd& lb, int si,
    const ArrayXd& x0, const ArrayXd& y0, const ArrayXd& yaw0, 
    const ArrayXd& steer0, const ArrayXd& v0_, const ArrayXd& w0_,
    const ArrayXd& x0_, const ArrayXd& y0_, const ArrayXd& yaw0_, 
    const ArrayXd& steer0_
  );
  void calcCfgConstraint(
    int a, SpMat& M, VectorXd& ub, VectorXd& lb, int si,
    const vector<double>& cfg
  );
  void calcCorridorConstraint(
    int a, SpMat& M, VectorXd& ub, VectorXd& lb, int si,
    const std::vector<std::vector<Corridor>>& corridors,
    const ArrayXd& x0, const ArrayXd& y0, const ArrayXd& yaw0
  );

  void calcTrustRegionConstraint(
    SpMat& M, VectorXd& ub, VectorXd& lb, int si, double r_trust,  
    const ArrayXd& x0, const ArrayXd& y0 );
  void calcMaxCtrlAndSteerConstraint(
    SpMat& M, VectorXd& ub, VectorXd& lb, int si, const  QpParm& param );
  void calcInterVehicleConstraint(
    int a, SpMat& M, VectorXd& ub, VectorXd& lb, int si, 
    // const ArrayXd& sc0,
    const std::vector< InterPlane >& inter_planes,
    int max_inter_times,
    const ArrayXd& x0, const ArrayXd& y0,  const ArrayXd& yaw0
  );
  
  void getAgentInitGuess(
    int a, ArrayXd& x0, ArrayXd& y0, ArrayXd& yaw0, ArrayXd& steer0, 
    ArrayXd& v0_, ArrayXd& w0_);

  void getAgentShortState(
    const ArrayXd& x0, const ArrayXd& y0, 
    const ArrayXd& yaw0, const ArrayXd& steer0,
    ArrayXd& x0_, ArrayXd& y0_, ArrayXd& yaw0_, ArrayXd& steer0_);


  void extractSingleSolutionVec2OptRes(
    vector<OptimizeResult>& solution, const VectorXd& x_solution);

  bool ExtractAndSimplify(
    const VectorXd& solution_vec, 
    ArrayXd& x0, ArrayXd& y0, ArrayXd& yaw0, ArrayXd& steer0, 
    ArrayXd& v0_, ArrayXd& w0_,
    ArrayXd& x0_, ArrayXd& y0_, ArrayXd& yaw0_, ArrayXd& steer0_
  );
  void updateCorridor(
    const VectorXd& solution_vec ,const  size_t& Nt, int a,
    double dimx, double dimy, const std::unordered_set<Location>& obstacles,
    vector<double>& lb,
    vector<double>& ub
  );

  bool isFeasible();

  double dimx, dimy;
  const std::unordered_set<Location>& m_obstacles;
  const QpParm& m_param;

  Timer timer;
  int max_iter;   // SQP maximum iterations.

  double dt; // time interval
  double WheelBase; // wheelbase
  double steer_max; // maximum front wheel angle (rad).
  size_t Na, Nt; // number fo agents, number of timesteps.
  int n_vars; // number of variables for one agent.

  // contraints numbers
  int n_kine;   // kinetic
  int n_config; // configuration
  int n_2circle ; // double circle corridor
  int n_trust ; // trust region update
  int n_ctrls ;   // ctrl constraint
  int n_inter ;   // inter-vehicle constraints


  
  VectorXd solution0;  // initial guess for one agent

  SpMat D;
  VectorXd E;

  // save all the agents' initial states.
  // shape (Na, Nt) for state variables(x,y,yaw,steer) ; 
  // shape (Na, Nt-1) for control var (v,w).
  MatrixXd x, y, yaw, steer, v, w;
  // corridors. error lower bound and error upper bound. for all agents.
  vector<double> elbs, eubs;


  int solve_status;
  // int search_status = 2;  // 0: failed; 1: minor collision; 2: success.
  double max_individual_opt_runtime = -1; // max optimization time.
  std::vector<std::pair<int, int>> unsuccess_status;
  int logger_level;
  Eigen::IOFormat CleanFmt;
};