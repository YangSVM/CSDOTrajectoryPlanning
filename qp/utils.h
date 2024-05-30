#pragma once
#define EIGEN_USE_MKL_ALL
#include "osqp/osqp.h"
#include "hybrid_a_star/motion_planning.h"
#include "qp/corridor.h"
#include "hybrid_a_star/timer.h"
#include "qp/post_process.h"

using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;
using Eigen::ArrayXd;
using std::vector;
typedef Eigen::SparseMatrix<double> SpMat;
// triplet. 3 element tuple. (row_id, col_id, value)
typedef Eigen::Triplet<double> Triplet;  
using libMultiRobotPlanning::OptimizeResult;
using libMultiRobotPlanning::Corridor;

struct QpParm{
  double trust_radius;
  double max_omega;
  double max_v;
  double max_iter;
  double delta_solution_threshold;
  double max_violation;

  int osqp_max_iter;

  double dt;
  int num_interpolation;
};
void readQpSolverConfig(std::string fname_config, QpParm& param);

void extractResult(
  size_t num_agent, size_t max_time, 
  const vector<vector<OptimizeResult>>& guesses,
  MatrixXd& x, MatrixXd& y, MatrixXd& yaw, MatrixXd& steer, 
  MatrixXd& v, MatrixXd& w, vector<double>& cfg
);


void spMat2CSC(
  const SpMat& sm, vector<c_float>& values, vector<c_int>& row_ids,
  vector<c_int>& col_ptrs);

void insertSparseMatrix(SpMat& sm, SpMat& add, int row_start_id, int col_start_id);

void assembleDiagSpMat(
  SpMat& sm, const ArrayXd& data);

void iterateSpMat(SpMat& mat, int row_start_id, int col_start_id,
 int row_len, int col_len, vector<Triplet>& tpl
);

void printTriplets( const vector<Triplet>& tpl, int si, int sj);
