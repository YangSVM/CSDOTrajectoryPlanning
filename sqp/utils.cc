
// #define EIGEN_USE_MKL_ALL
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <yaml-cpp/yaml.h>
#include <boost/algorithm/string/replace.hpp>

#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>

#include "hybrid_a_star/types.h"
#include "common/motion_planning.h"

#include "pbs/PBS.h"

#include "sqp/inter_agent_cons.h"
#include "sqp/corridor.h"
#include "sqp/utils.h"
#include "util/file_utils.h"


using namespace libMultiRobotPlanning;
using std::cout;


void readQpSolverConfig(std::string fname_config, QpParm& param){
  YAML::Node car_config;

  try {
    car_config = YAML::LoadFile(fname_config.c_str());
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[33mWARNING: Failed to load agent config file: "
              << fname_config << "\033[0m , Using default params. \n";
  }

  param.max_omega = car_config["max_omega"].as<double>();
  param.max_v = car_config["max_v"].as<double>();
  param.max_iter = car_config["max_iter"].as<double>();
  param.delta_solution_threshold = car_config["delta_solution_threshold"].as<double>();
  param.max_violation = car_config["max_violation"].as<double>();
  param.osqp_max_iter = car_config["osqp_max_iter"].as<int>();

  param.r_trust = car_config["r_trust"].as<double>();
  param.num_interpolation = car_config["num_interpolation"].as<int>();
  
  double decelerate_factor = car_config["decelerate_factor"].as<double>();
  param.dt = Constants::r * Constants::deltat / param.max_v 
      / (param.num_interpolation + 1 ) / decelerate_factor;
  
  param.fixed_corridor = car_config["fixed_corridor"].as<bool>();
}


void dumpCorridors(
  std::string file_name,
  const std::vector<std::vector<Corridor>>& corridors,
  const std::vector<vector<OptimizeResult>>& x0_bar
  ){
  size_t Na = corridors.size();
  size_t Nt = corridors[0].size();
  std::ofstream out;
  out = std::ofstream(file_name);
  for (size_t a = 0; a < Na; a++){
    out << "agent" << a<<":" <<std::endl;
    
    // size_t max_ti = solution_refine[a].states.size();
    for ( size_t t = 0; t< Nt; t++){
      double xf, yf, xr, yr;
      State state(x0_bar[a][t].x, x0_bar[a][t].y, x0_bar[a][t].yaw);
      state.GetDiscCenter(xf,yf,xr,yr);
      out << "  - ["<<xf<<", "<<yf<<", "
        <<corridors[a][t].xf_min<<", " <<corridors[a][t].xf_max<<
        ", "<<corridors[a][t].yf_min<<", "<<corridors[a][t].yf_max<<"]\n";
      out << "  - ["<<xr<<", "<<yr<<", "
        <<corridors[a][t].xr_min<<", " <<corridors[a][t].xr_max<<
        ", "<<corridors[a][t].yr_min<<", "<<corridors[a][t].yr_max<<"]\n";
    }

  }
  out.close();
}


//  cfg. (Na, 6). the start and end state of the problem (configuration).
void extractResult(
  size_t num_agent, size_t max_time, 
  const vector<vector<OptimizeResult>>& guesses,
  MatrixXd& x, MatrixXd& y, MatrixXd& yaw, MatrixXd& steer, 
  MatrixXd& v, MatrixXd& w, vector<double>& cfg
){
  cfg.clear();
  for(size_t i = 0; i < 6*num_agent; i++){
    cfg.push_back(0);
  }
  for ( size_t i = 0; i < num_agent; i ++){
    for(size_t t = 0 ; t < max_time; t++){
      x(i,t) = guesses[i][t].x;
      y(i,t) = guesses[i][t].y;
      yaw(i,t) = guesses[i][t].yaw;
      steer(i,t) = guesses[i][t].steer;

      if (t == max_time-1) continue; 
      // v, w just have (max_time-1) elements per agent
      v(i,t) = guesses[i][t].v;
      w(i,t) = guesses[i][t].d_steer;
    }
    cfg[6*i ] = guesses[i].front().x;  // x [i, 0]
    cfg[6*i +1] = guesses[i].back().x;  // x [i, end]
    cfg[6*i +2] = guesses[i].front().y;  // y [i, 0]
    cfg[6*i +3] = guesses[i].back().y;  // y [i, end]
    cfg[6*i +4] = guesses[i].front().yaw;  // yaw [i, 0]
    cfg[6*i +5] = guesses[i].back().yaw;  // yaw [i, end]

  }
}


void spMat2CSC(
  const SpMat& sm, vector<c_float>& values, vector<c_int>& row_ids,
  vector<c_int>& col_ptrs)
{  
  int nnz = sm.nonZeros();
  int n_col = sm.cols();
  values.reserve(nnz);
  row_ids.reserve(nnz);
  col_ptrs.reserve( n_col + 1 );

  col_ptrs.push_back(0);
  
  c_int index = 0;
  for (int k=0; k< sm.outerSize(); ++k){
    for (    SpMat::InnerIterator it(sm, k); it; ++it)
    {
      // sm.insert(it.row() + row_start_id, it.col() + col_start_id) =  it.value();
      row_ids.push_back( it.row() );
      values.push_back( it.value() );
      index++;
    }
    col_ptrs.push_back(index);
  }
  assert(col_ptrs.back() == nnz && "col last value should be nnz");
  assert(col_ptrs.size() == n_col+1 && "col size should be n_col +1");
  assert(row_ids.size() == nnz && "row size should be nnz");
  assert(values.size() == nnz && "row size should be nnz");

}


void insertSparseMatrix(SpMat& sm, SpMat& add, int row_start_id, int col_start_id){
  for (int k=0; k< add.outerSize(); ++k){
    for (SpMat::InnerIterator it(add, k); it; ++it)
    {
      sm.insert(it.row() + row_start_id, it.col() + col_start_id) =  it.value();
    }
  }

}



void assembleDiagSpMat(
  SpMat& sm, const ArrayXd& data){
  size_t N = data.rows()*data.cols();

  sm.reserve(VectorXi::Constant(sm.cols(), 1));
  for ( size_t a = 0 ;  a < N; a++){
      sm.insert(a,a) = data(a);
  }


}


// 用法和block一样
void iterateSpMat(SpMat& mat, int row_start_id, int col_start_id,
 int row_len, int col_len, vector<Triplet>& tpl
){
  for (int k=0; k< mat.outerSize(); ++k){ // 第几列
    if ( k < col_start_id || k >= col_start_id+ col_len) {
      continue;
    }
    for (SpMat::InnerIterator it(mat, k); it; ++it)
    {
      if ( it.row()< row_start_id || it.row() >= row_start_id+row_len ){
        continue;
      }
      // i, j, data
      tpl.push_back({it.row(), it.col(), it.value()} );

    }
  }
}

void printTriplets( const vector<Triplet>& tpl, int si, int sj){
  cout << "tpls: " << std::endl;
  for ( auto t : tpl ){
    cout << "(" << t.row()-si << "," << t.col() -sj<<"," << t.value() <<").";
  }
  cout << std::endl;
}
