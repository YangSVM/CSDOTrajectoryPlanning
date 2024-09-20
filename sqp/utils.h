#pragma once
// #define EIGEN_USE_MKL_ALL
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Core>
#include "osqp/osqp.h"
#include "sqp/common.h"
#include "common/motion_planning.h"
#include "sqp/corridor.h"
#include "hybrid_a_star/timer.h"
#include "hybrid_a_star/planresult.h"
#include "sqp/inter_agent_cons.h"

using Eigen::VectorXd;
using Eigen::VectorXi;
using Eigen::MatrixXd;
using Eigen::ArrayXd;
typedef Eigen::SparseMatrix<double> SpMat;
// triplet. 3 element tuple. (row_id, col_id, value)
typedef Eigen::Triplet<double> Triplet;  
using libMultiRobotPlanning::OptimizeResult;
using libMultiRobotPlanning::Corridor;
using  libMultiRobotPlanning::QpParm;
using libMultiRobotPlanning::PlanResultShort;

void readQpSolverConfig(std::string fname_config, QpParm& param);

void dumpCorridors(
  std::string file_name,
  const vector<vector<Corridor>>& corridors,
  const vector<vector<OptimizeResult>>& x0_bar
);

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
