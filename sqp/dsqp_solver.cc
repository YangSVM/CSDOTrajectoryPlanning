#include "sqp/dsqp_solver.h"
#include "hybrid_a_star/types.h"
#include "common/motion_planning.h"

#include "pbs/PBS.h"

#include "sqp/inter_agent_cons.h"
#include "sqp/corridor.h"
#include "sqp/utils.h"
#include "util/file_utils.h"

using namespace libMultiRobotPlanning;
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
using std::vector;


// class T must have attribution t.
// calculate the maximum constraint for one col
template <class T>
int calcMaxTime(const vector<T>&  planes, int Nt){
  vector<int> times(Nt, 0);
  int max_t = 0;
  for ( auto & p : planes){
    assert( p.t < Nt && "Nt must be the maximum value for time");
    times[p.t] += 1;
  }

  for ( auto & t: times){
    if ( t > max_t){
      max_t = t;
    }
  }
  return max_t;
}

bool SolverDSQP::calcIndividualSQP(
  int a, 
  vector<OptimizeResult>& solution,
  const std::vector<std::vector<Corridor>>& corridors,
  const std::vector<std::vector<InterPlane>>& inter_planes,
  const vector<double>& cfg,
  const QpParm& param
){

  Timer timer_prepos;
  // bool success = true;
  double th_solution = param.delta_solution_threshold;
  double delta_solution = th_solution +1;  // initial value. change in solution
  
  // suffix 0 means length is Nt.
  // suffix _ means length is Nt-1 other than Nt.
  ArrayXd x0, y0, yaw0, steer0, v0_, w0_;
  ArrayXd x0_, y0_, yaw0_, steer0_;

  // set all the variable will be used in the following equations.
  getAgentInitGuess(a,  x0, y0, yaw0, steer0, v0_, w0_);
  getAgentShortState(x0, y0, yaw0, steer0, x0_, y0_, yaw0_, steer0_);

  ArrayXd x_trust = x0;
  ArrayXd y_trust = y0;


  solution0 << x0, y0, yaw0, steer0, v0_, w0_;
  // should be changed among different agents.
  n_inter  = 4 * inter_planes[a].size(); // inter-vehicle collision avoidance.

  // the maximum times that a timestep has potential collsion with other agents.
  int max_inter_times = calcMaxTime<InterPlane>(inter_planes[a], Nt);

  int n_constraints =0; // number of constraints.
  // sparse matrix need this to initialize the memory.
  int max_col_num =0; // max number elements in one col


  n_constraints += n_kine;
  max_col_num += 4;

  n_constraints +=  n_config;
  max_col_num += 1;

  n_constraints +=  n_2circle;
  max_col_num += 4;

  n_constraints += n_trust ;
  max_col_num += 1;

  n_constraints += n_ctrls + Nt;  // n_ctrl and steer_max constraints.
  max_col_num += 1;

  n_constraints +=  n_inter; 
  max_col_num += max_inter_times*2 * 4; // maximum col * 2( 2 disc) * n_2circle


  VectorXd solution_vec(4*Nt+2*(Nt-1));  // solved solution.

  // SQP main Loop. wait until the solution is stable.
  int iter_count = 0;

  while ( delta_solution > th_solution && iter_count < max_iter){
    if ( logger_level >= 2) {
      cout << "iter count:\t" << iter_count << "\n";
    }
    int si = 0;  // start index of constraints.
    
    SpMat M(n_constraints, n_vars);
    M.reserve(VectorXi::Constant(n_vars, max_inter_times));
    VectorXd ub(n_constraints);
    VectorXd lb(n_constraints);
    lb.setZero();
    ub.setZero();

    calcKineConstraint(a, M, ub, lb, si, x0, y0, yaw0, steer0, v0_, w0_, 
      x0_, y0_, yaw0_, steer0_);
    si += n_kine;  

    if (logger_level >= 2){
      timer.stop(); 
      std::cout << " insert ABC and cfg time: " << timer.elapsedSeconds() << std::endl;
      timer.reset();
    }



    calcCfgConstraint(a, M, ub, lb, si, cfg);
    si += n_config;


    calcCorridorConstraint(a, M, ub, lb, si, corridors, x0, y0, yaw0);
    si += n_2circle;

    if (logger_level >= 2){
      timer.stop();
      std::cout << " insert D time: " << timer.elapsedSeconds() << std::endl;
      timer.reset();
    }


    //  trust region
    calcTrustRegionConstraint(M, ub, lb, si, param.r_trust, x_trust, y_trust);
    si += n_trust;

    // maximum control and steer constraints
    calcMaxCtrlAndSteerConstraint(M, ub, lb, si, param);
    si += n_ctrls + Nt;

    if (logger_level >= 2){
      timer.stop();
      std::cout << " cal and insert  trust region and umax  time: " << timer.elapsedSeconds() << std::endl;
      timer.reset();
    }

    // inter vehicle 
    calcInterVehicleConstraint(a, M, ub, lb, si, inter_planes.at(a), max_inter_times, x0, y0, yaw0);
    si+= n_inter;
    if (logger_level >= 2){
      timer.stop();
      std::cout << " cal and insert  multi-agent  time: " << timer.elapsedSeconds() << std::endl;
      timer.reset();
    }
    


    SpMat P_u(n_ctrls, n_ctrls);
    P_u.reserve(VectorXd::Constant(n_vars, 3));

    // minimize delta_v ^2 and w^2
    for ( size_t t = 0 ; t < Nt-1; t++){
      // just minimize the delta_v^2
      size_t i_v =  t;
      if ( t != 0 && t!=Nt-2){
        P_u.insert(i_v, i_v-1) = -1;
        P_u.insert(i_v, i_v) = 2;
        P_u.insert(i_v, i_v+1) = -1;
      }
      else if (t==0)
      {
        P_u.insert(i_v, i_v) = 1;
        P_u.insert(i_v, i_v+1) = -1;
      }
      else if (t==Nt-2)
      {
        P_u.insert(i_v, i_v-1) = -1;
        P_u.insert(i_v, i_v) = 1;
      }

      // minimize the w^2
      size_t i_w = (Nt-1)  + t;
      P_u.insert(i_w, i_w) = 1;
    }

    SpMat P(n_vars, n_vars);
    P.reserve(VectorXd::Constant(n_vars, 3));
    size_t n_states = n_vars - n_ctrls;

    insertSparseMatrix(P, P_u, n_states, n_states);
    VectorXd q(n_vars);
    q << VectorXd::Zero(n_vars);

    if ( logger_level >= 2 ){
      timer.stop();
      std::cout << " cal and insert  P q time: " << timer.elapsedSeconds() << std::endl;
    }  
    
    M.makeCompressed();
    P.makeCompressed();

    // print the max constraints 
    si = n_kine + n_config + n_2circle + n_trust;

    // // 遍历输出
    // vector<Triplet> ctrl_mat;
    // iterateSpMat(M, si, 0, n_ctrls, n_vars, ctrl_mat);
    // printTriplets(ctrl_mat, si, 0);
    
    if ( logger_level >= 2 ){
      timer_prepos.stop();
      cout << " individual pre process time: " << timer_prepos.elapsedSeconds() << std::endl;
    
    }


    c_float* solution_array;
    this->solve_status = solveOSQP(a, solution_vec, M, ub, lb, P, q);

    Timer tpost;

    // do not need to converage to local optimum.
    delta_solution = (solution_vec - solution0).dot(solution_vec - solution0);
    if ( logger_level >= 2) {
      cout << "delta_solution: " << delta_solution << std::endl;
    }    

    if ( logger_level >= 2){
      tpost.stop();
      cout << "individual post proccess time: " << tpost.elapsedSeconds() << std::endl;      
    }

    timer_prepos.reset();
    

    // update initial guess.
    iter_count++;
    ExtractAndSimplify(solution_vec, x0, y0, yaw0, steer0, v0_, w0_,
      x0_, y0_, yaw0_, steer0_);

    // check the violation of kinematic constraints.
    if (iter_count > max_iter/2 && isFeasible(a, x0, y0, yaw0, steer0, v0_, w0_) ){
      break;
    }
    solution0 = solution_vec;
    if ( ! param.fixed_corridor){
      updateCorridor(solution_vec, Nt, a, dimx, dimy, m_obstacles, corridor_lbs, corridor_ubs);
    }
    timer.reset();

  }

  if (logger_level >= 3){
    cout << "iteration numbers: " << iter_count << std::endl;
  }
  num_iterations.push_back(iter_count);

  extractSingleSolutionVec2OptRes(solution, solution_vec);
  if ( abs(this-> solve_status )> 1){
    unsuccess_status.push_back({a, this->solve_status});
  }

  return this->solve_status;
}

bool check_less_than(
  const vector<double>& small, const vector<double>& big, 
  vector<int>& illegal_pos, double & err_max, double& err_sum
){
  assert( small.size() == big.size());
  bool illegal=false;
  illegal_pos.clear();
  size_t num = small.size();
  for ( size_t i = 0 ; i <num; i++){
    if( small[i] <= big[i] ){
      continue;
    }
    illegal = true;
    illegal_pos.push_back(i);
    double err = small[i] - big[i];
    err_sum += err;
    if ( err >err_max) err_max = err;    
  }
  return illegal;
}

bool SolverDSQP::isFeasible( int a, const  ArrayXd& x0, const ArrayXd& y0, 
  const ArrayXd& yaw0, const ArrayXd& steer0, 
  const   ArrayXd& v0_, const  ArrayXd& w0_, bool fully_check){
  
  double error_kine_th = 1e-2, error_corridor_max_th = 1e-1,\
    error_inter_max_th = 1e-1;

  // checking the kinematic violation.
  ArrayXd x1 = x0.segment(0, Nt-1);
  ArrayXd x2 = x0.segment(1, Nt-1);

  ArrayXd y1 = y0.segment(0, Nt-1);
  ArrayXd y2 = y0.segment(1, Nt-1);

  ArrayXd yaw1 = yaw0.segment(0, Nt-1);
  ArrayXd yaw2 = yaw0.segment(1, Nt-1);

  ArrayXd steer1 = steer0.segment(0, Nt-1);
  ArrayXd steer2 = steer0.segment(1, Nt-1);

  double err_kin=0;
  ArrayXd err_array = (x1 + v0_*( yaw1.cos())*dt - x2);
  double err1 = (err_array *err_array ).sum();

  err_array =  (y1 + v0_* (yaw1.sin())*dt - y2);
  double err2 = (err_array *err_array ).sum();

  err_array =  (yaw1 + v0_* (steer1.tan())/WheelBase*dt - yaw2);
  double err3 = (err_array *err_array ).sum();

  err_array =  (steer1 + w0_*dt - steer2);
  double err4 = (err_array *err_array ).sum();

  err_kin = (err1 + err2 + err3 + err4)/Nt;
  if (logger_level >= 2){
    cout<< " agent  kine error "<<  err_kin << " [" << err1 << ", " << err2 <<", " << err3 \
    <<", "<< err4<< " ]."<<endl;
  }

  if ( !fully_check && err_kin > error_kine_th){
    return false;
  }

  // calculate the exact position of the double-circle positions
  ArrayXd xf = x0 + Constants::f2x * yaw0.cos();
  ArrayXd xr = x0 + Constants::r2x * yaw0.cos();
  ArrayXd yf = y0 + Constants::f2x * yaw0.sin();
  ArrayXd yr = y0 + Constants::r2x * yaw0.sin();

  // checking the violation of static collisions. output all the failure cases.
  vector<double> corrior_lb (corridor_lbs.begin() + a*Nt*4, corridor_lbs.begin() + (a + 1)*Nt*4);
  vector<double> corridor_ub (corridor_ubs.begin() + a*Nt*4, corridor_ubs.begin() + (a + 1)*Nt*4);
  
  vector<double> xf_vec(xf.data(), xf.data()+xf.size());
  vector<double> xr_vec(xr.data(), xr.data()+xr.size());
  vector<double> yf_vec(yf.data(), yf.data()+xf.size());
  vector<double> yr_vec(yr.data(), yr.data()+yr.size());

  vector<double> Y(xf_vec.begin(), xf_vec.end());
  Y.insert(Y.end(), yf_vec.begin(), yf_vec.end());
  Y.insert(Y.end(), xr_vec.begin(), xr_vec.end());
  Y.insert(Y.end(), yr_vec.begin(), yr_vec.end());

  vector<string> infos{"xf", "yf", "xr" , "yr"};

  double err_cor_sum=0, err_cor_max=0, err_cor_cnt=0;
  vector<int> illegal_timestamp;
  if(check_less_than(corrior_lb, Y, illegal_timestamp, err_cor_max, err_cor_sum)){
    err_cor_cnt += illegal_timestamp.size();
    if ( logger_level >= 2){
      for ( int ts : illegal_timestamp){
        cout << "warning corridor lower bound error. agent " << a << " timestamp " \
          << ts%Nt << " with " << infos[ts/Nt] <<endl;
      }
    }

  }
  if ( !fully_check && err_cor_max > error_corridor_max_th){
    return false;
  }
  if(check_less_than( Y, corridor_ub,illegal_timestamp, err_cor_max, err_cor_sum)){
    err_cor_cnt += illegal_timestamp.size();
    if ( logger_level >= 2){
      for ( int ts : illegal_timestamp){
        cout << "warning. corridor upper bound error. agent " << a << " timestamp " \
          << ts%Nt << " with " << infos[ts/Nt] <<endl;
      }
    }
  }

  if (logger_level>=3 && err_cor_sum > 0){
    cout << "corridor error mean: " << err_cor_sum/err_cor_cnt <<\
      " max: " << err_cor_max << endl;
  }
  if ( !fully_check && err_cor_max > error_corridor_max_th){
    return false;
  }

  // checking the violation of inter-agent collisions.
  double err_inter = 0,  err_inter_max=0;
  int err_inter_cnt = 0;
  if ( n_inter > 0){ // n_inter==0, G, H will be the last agent's value.
    Eigen::VectorXd Y_arr = Eigen::ArrayXd::Map(
        Y.data(), static_cast<Eigen::Index>(Y.size()));
    auto res = G*Y_arr + H; // should less than zeros.
    
    for ( size_t i = 0; i < res.size(); i++){
      if ( res[i] > 0){
        err_inter += res[i];
        err_inter_cnt ++;
        if ( res[i] > err_inter_max){
          err_inter_max = res[i];
        }
      }
    }
    if (logger_level>=3 && err_inter>0){
      cout << "violated inter agent constraints. error max: " << err_inter_max 
      << ". error mean: "<< err_inter/(double)err_inter_cnt << ". times: " 
        << err_inter_cnt<< endl;
    }
  }


  if ( err_kin <error_kine_th && err_inter_max < error_inter_max_th\
   && err_cor_max< error_corridor_max_th){
    return true;
  }
  return false;
}

// change the dense to osqp csc format.
int SolverDSQP::solveOSQP(
  int a,
  VectorXd& solution_vec,
  SpMat& M, const VectorXd& ub, const VectorXd& lb,
  SpMat& P, const VectorXd& q
){
  int n_vars = M.cols();
  int n_constraints = M.rows();

  // M constraints.
  vector<c_float> A_data;
  vector<c_int> A_indices;
  vector<c_int> A_indptr;
  Timer timer;
  spMat2CSC(M, A_data, A_indices, A_indptr);

  if ( logger_level >= 2 ){
    timer.stop();
    std::cout << "translate A time: " << timer.elapsedSeconds() << std::endl;
    timer.reset();
  }  
  
  vector<c_float> P_data;
  vector<c_int> P_indices;
  vector<c_int> P_indptr;

  spMat2CSC(P.triangularView<Eigen::Upper>(), P_data, P_indices, P_indptr);
  
  if ( logger_level >= 2 ){
    timer.stop();
    std::cout << "translate P time: " << timer.elapsedSeconds() << std::endl;
    timer.reset();
  }

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  data->n = n_vars;
  data->m = n_constraints;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                      P_indices.data(), P_indptr.data());

  vector<c_float> q_data(q.data(), q.data()+q.rows()*q.cols());
  data->q = q_data.data();


  data->A = csc_matrix(data->m, data->n, M.nonZeros(), M.valuePtr(), 
                       A_indices.data(), A_indptr.data());
  
  vector<c_float> l_data (lb.data(), lb.data()+lb.rows()*lb.cols());
  data->l = l_data.data();
  vector<c_float> u_data (ub.data(), ub.data()+ub.rows()*ub.cols());
  data->u = u_data.data();

  // Problem settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));

  // Define Solver settings as default
  osqp_set_default_settings(settings);
  if ( logger_level >= 2){
    settings->verbose = true; // print the osqp info
  }
  else{
    settings->verbose = false;
  }
  settings->max_iter = m_param.osqp_max_iter ;
  // settings->alpha = osqp_config_.alpha();  // Change alpha parameter
  // settings->eps_abs = osqp_config_.eps_abs();
  // settings->eps_rel = osqp_config_.eps_rel();
  // settings->max_iter = osqp_config_.max_iter();
  // settings->polish = osqp_config_.polish();
  // settings->verbose = osqp_config_.osqp_debug_log();

  // Workspace
  OSQPWorkspace *work;
  // osqp_setup(&work, data, settings);
  osqp_setup(&work, data, settings);

  osqp_warm_start_x(work, solution0.data());

  osqp_solve(work);

  if ( logger_level >= 2 ){
    timer.stop();
    std::cout << "osqp_solve() time: " << timer.elapsedSeconds() << std::endl;
    timer.reset();
  }
  bool succ = true;
  // 1: solved; 2: inaccurate; others: failed.
  int osqp_status = work->info->status_val ;
  // check state. status_val is defined in osqp/constants.h

  // thershold status
  int status_th = 2;

  c_float* solution_array;
  if (abs( osqp_status )> status_th ) {
    succ = false;
    solution_array = solution0.data();
  }
  else{
    solution_array = work->solution->x;
  }
  if (abs(osqp_status) > 1 && logger_level >= 3)
  {
    std::cout << "OSQP dual warm up unsuccess, "
          << "return status: " << work->info->status << std::endl;
    std::cout << "2: inaccurate; -2 : max iter reached; other: failed. \n";
    // please refer to the osqp constants.h 
    // unsuccess_status.push_back({a, work->info->status_val});  
  }
  
  // extract the solution before the osqp_cleanup.
  for ( int i = 0 ; i < 4*Nt + (Nt-1)*2; i++ ){
    solution_vec(i) = solution_array[i];
  }

  if ( logger_level >= 2 ){
    timer.stop();
    std::cout << "osqp other time: " << timer.elapsedSeconds() << std::endl;
    timer.reset();
  }

  // extract primal results
  int variable_index = 0;

  // Cleanup
  osqp_cleanup(work);   // clear all the data
  // c_free(data->A);
  // c_free(data->P);
  // c_free(data);
  // c_free(settings);
  return osqp_status;
}

bool SolverDSQP::ExtractAndSimplify(
  const VectorXd& solution_vec, 
  ArrayXd& x0, ArrayXd& y0, ArrayXd& yaw0, ArrayXd& steer0, 
  ArrayXd& v0_, ArrayXd& w0_,
  ArrayXd& x0_, ArrayXd& y0_, ArrayXd& yaw0_, ArrayXd& steer0_
){

  x0 = solution_vec.segment(0, Nt);
  y0 = solution_vec.segment(Nt, Nt);
  yaw0 = solution_vec.segment( 2*Nt, Nt);
  steer0 = solution_vec.segment( 3*Nt, Nt);

  v0_ = solution_vec.segment( 4*Nt, Nt-1);
  w0_ = solution_vec.segment(4*Nt + Nt-1, Nt-1);

  getAgentShortState(x0, y0, yaw0, steer0, x0_, y0_, yaw0_, steer0_);

  return true;
}

void SolverDSQP::extractSingleSolutionVec2OptRes(
  vector<OptimizeResult>& solution,
  const  VectorXd& solution_vec
  ){
int x_start = 0;
  int y_start = Nt;
  int yaw_start =2*Nt;
  int steer_start = 3*Nt;
  int v_start = 4*Nt;
  int w_start = 4*Nt + (Nt-1);
  double v_max=0;
  double w_max=0;

  for ( int t = 0; t < Nt; t++){
    int id_s =  t;
    int id_u = t;

    OptimizeResult ores;
    ores.x = solution_vec( x_start +  id_s);
    ores.y = solution_vec( y_start +  id_s);
    ores.yaw = solution_vec( yaw_start +  id_s);
    ores.steer = solution_vec( steer_start +  id_s);
    if ( t < Nt-1){
      ores.v = solution_vec( v_start +  id_u);
      ores.d_steer = solution_vec( w_start +  id_u);
      if(fabs(ores.v) > v_max){
        v_max = fabs(ores.v);
      }
      if(fabs(ores.d_steer) > w_max){
        w_max = fabs(ores.d_steer);
      }
    }

    solution.push_back(ores);
  }

  if ( logger_level >= 2){
    cout<< "max v: " << v_max <<". max w: "<<w_max<<".\n";
  }

}

// extract the initial variable of agent a.
void SolverDSQP::getAgentInitGuess(
  int a, ArrayXd& x0, ArrayXd& y0, ArrayXd& yaw0, ArrayXd& steer0, 
  ArrayXd& v0_, ArrayXd& w0_){
  
  x0 = x.row(a).array();
  y0 = y.row(a).array();
  yaw0 = yaw.row(a).array();
  steer0 = steer.row(a).array();

  v0_ = v.row(a).array();
  w0_ = w.row(a).array();
}

void SolverDSQP::getAgentShortState(
  const ArrayXd& x0, const ArrayXd& y0, 
  const ArrayXd& yaw0, const ArrayXd& steer0,
  ArrayXd& x0_, ArrayXd& y0_, ArrayXd& yaw0_, ArrayXd& steer0_)
{

  x0_ = x0.segment(0, Nt-1);
  y0_ = y0.segment(0, Nt-1);
  yaw0_ = yaw0.segment(0, Nt-1);
  steer0_ = steer0.segment(0, Nt-1);

}

void SolverDSQP::calcKineConstraint(
  int a, SpMat& M, VectorXd& ub, VectorXd& lb, int si,
  const ArrayXd& x0, const ArrayXd& y0, const ArrayXd& yaw0, 
  const ArrayXd& steer0, const ArrayXd& v0_, const ArrayXd& w0_,
  const ArrayXd& x0_, const ArrayXd& y0_, const ArrayXd& yaw0_,
  const ArrayXd& steer0_
){
  int m = Na*(Nt-1);


  SpMat a_eye(Nt-1, Nt);
  vector<Triplet> a_eye_tpl;
  // for ( size_t a = 0 ;  a < Na; a++){
  for ( size_t t = 0 ; t < Nt-1; t++){
    int i =  t;
    int j = t;
    a_eye_tpl.push_back(Triplet(i, j, 1));
    a_eye_tpl.push_back(Triplet(i, j+1, -1));
  }
  // }
  a_eye.setFromTriplets(a_eye_tpl.begin(), a_eye_tpl.end());


  // cout << MatrixXd(a_eye).format(CleanFmt)<< std::endl;
  ArrayXd a_yaw1_pre = -dt*(v0_*(yaw0_.sin()));
  SpMat a_yaw1((Nt-1), Nt);
  assembleDiagSpMat(a_yaw1, a_yaw1_pre);

  // cout << "a yaw 1: \n"<< MatrixXd(a_yaw1).format(CleanFmt)<< std::endl;

  ArrayXd a_yaw2_pre = dt*(v0_*(yaw0_.cos()));
  SpMat a_yaw2((Nt-1), Nt);
  assembleDiagSpMat(a_yaw2, a_yaw2_pre);


  ArrayXd a_steer_pre =  (dt/WheelBase*v0_)/( (steer0_.cos()).pow(2) ) ;
  SpMat a_steer((Nt-1), Nt);
  assembleDiagSpMat(a_steer, a_steer_pre);

  SpMat A(4*(Nt - 1), 4*Nt);
  A.reserve(VectorXi::Constant(4*Nt, 4));
  insertSparseMatrix(A, a_eye, 0, 0);
  insertSparseMatrix(A, a_yaw1, 0, 2*Nt);

  insertSparseMatrix(A, a_eye, (Nt-1), Nt);
  insertSparseMatrix(A, a_yaw2, (Nt-1), 2*Nt);

  insertSparseMatrix(A, a_eye, 2*(Nt-1), 2*Nt);
  insertSparseMatrix(A, a_steer, 2*(Nt-1), 3*Nt);

  insertSparseMatrix(A, a_eye, 3*(Nt-1), 3*Nt);

  // cout << "A : \n"<< MatrixXd(A).format(CleanFmt)<< std::endl;
  
  m = Nt - 1;
  SpMat B(4*m, 2*m);
  B.reserve(VectorXi::Constant(2*m, 3));  // max 3 elements in one column
  SpMat coeff_cos(m,m), coeff_sin(m,m), coeff_tan(m,m), eye_dt(m,m);
  assembleDiagSpMat( coeff_cos, dt*(yaw0_.cos()) );
  assembleDiagSpMat( coeff_sin, dt*(yaw0_.sin()) );
  assembleDiagSpMat( coeff_tan, dt/WheelBase*steer0_.array().tan() );
  assembleDiagSpMat( eye_dt, dt*ArrayXd::Ones(m,1) );

  insertSparseMatrix(B, coeff_cos, 0, 0);
  insertSparseMatrix(B, coeff_sin, m, 0);
  insertSparseMatrix(B, coeff_tan, 2*m, 0);
  insertSparseMatrix(B, eye_dt, 3*m, m);


  // C is  dense.
  MatrixXd C(4*m, 1);
  C<< dt*yaw0_*v0_*(yaw0_.sin()), -dt*yaw0_*v0_*(yaw0_.cos()), 
      -dt*(steer0_*v0_/WheelBase/( (steer0_.cos()).pow(2))).matrix().reshaped(m,1), MatrixXd::Zero(m, 1);

  MatrixXd state0(4*Nt, 1);
  state0 << x0, y0, yaw0, steer0;

  MatrixXd u0(2*(Nt-1), 1);
  u0 << v0_, w0_;

  MatrixXd res= A*state0+B*u0 + C ;
  if ( logger_level >= 2){
    cout << "max value (should == 0): "<< res.maxCoeff() <<std::endl;
    cout << "min value (shoulde == 0): "<< res.minCoeff() <<std::endl;
  }
  if ( logger_level >= 3){
    cout << "kinematic res (should == 0) : \n"<< res.transpose().format(CleanFmt)<< std::endl;
    cout << "u0 : \n"<< u0.transpose().format(CleanFmt)<< std::endl;
  }



  insertSparseMatrix(M, A, 0, 0);
  insertSparseMatrix(M, B, 0, 4*Nt);

  lb.block(0,0, 4*m ,1) = -C;
  ub.block(0,0, 4*m ,1) = -C;

}

void SolverDSQP::calcCfgConstraint(
  int a, SpMat& M, VectorXd& ub, VectorXd& lb, int si, 
  const vector<double>& cfg
){
  int m = (Nt-1);


  // start and goal constraints. 
  SpMat id_start_end(6, n_vars);
  id_start_end.reserve(VectorXi::Constant(n_vars, 1));

  // x0 x_{Nt-1}
  id_start_end.insert(0, 0)  = 1;
  id_start_end.insert( 1,  Nt-1)  = 1;
  // y0 y_{Nt-1}
  id_start_end.insert( 2, Nt  )  = 1;
  id_start_end.insert( 3, Nt + Nt-1)  = 1;
  // yaw0 yaw_{Nt-1}
  id_start_end.insert( 4, 2*Nt)  = 1;
  id_start_end.insert( 5, 2*Nt+ Nt-1)  = 1;

  insertSparseMatrix(M, id_start_end, si, 0);
  // M.block(si, 0, 6*Na, n_vars) = id_start_end;
  
  std::vector<double> cfg_a(cfg.begin() + 6*a, cfg.begin() + 6 + 6*a);
  Eigen::Map<VectorXd,0,Eigen::InnerStride<1>> cfg_eigen(cfg_a.data(), cfg_a.size());
  lb.block(si, 0, 6, 1) = cfg_eigen;
  ub.block(si, 0, 6, 1) = cfg_eigen;

  MatrixXd cfg_res = id_start_end*solution0 - cfg_eigen;
  // solution should be zero.
  if (logger_level >= 3){
    cout << "cfg_res max value: "<< cfg_res.maxCoeff() <<std::endl;
    cout << "cfg_res min value: "<< cfg_res.minCoeff() <<std::endl;
    cout << "cfg res: " << cfg_res << std::endl;
  }
  assert( (id_start_end*solution0 - cfg_eigen).cwiseAbs().maxCoeff() < 1e-2);


  if ( logger_level >= 2){
    cout << "cfg_res (should be exactly 0): \n"<< cfg_res.transpose().format(CleanFmt)<< std::endl;
  }
}

// sorted by agent. dimension (Na, 4, Nt). 4: xf yf xr yr. 
void extractAgentsCorridor(
  size_t Na, size_t Nt,
  const vector<vector<Corridor>>& Corridor, 

  vector<double>& lb,
  vector<double>& ub
){
  // lb.resize( Na*Nt* 4 );
  // ub.resize( Na*Nt * 4 );
  int m = 4*Nt ;
  for ( size_t a= 0; a < Na; a++){
    for ( size_t t = 0 ; t < Nt; t++){
      lb[a*m + t] =          Corridor[a][t].xf_min;
      lb[a*m +  Nt + t] = Corridor[a][t].yf_min;
      lb[a*m +2*Nt + t] = Corridor[a][t].xr_min;
      lb[a*m +3*Nt + t] = Corridor[a][t].yr_min;

      ub[a*m + t] =         Corridor[a][t].xf_max;
      ub[a*m +  Nt + t] = Corridor[a][t].yf_max;
      ub[a*m + 2*Nt + t] = Corridor[a][t].xr_max;
      ub[a*m +  3*Nt + t] = Corridor[a][t].yr_max;
    }
  }

}


void  SolverDSQP::updateCorridor(
  const VectorXd& solution_vec ,const  size_t& Nt, int a,
  double dimx, double dimy, const std::unordered_set<Location>& obstacles,
  vector<double>& lb,
  vector<double>& ub
){
  ArrayXd x = solution_vec.segment( 0, Nt); // segment(start_index, !length) 
  ArrayXd y = solution_vec.segment( Nt, Nt);
  ArrayXd yaw = solution_vec.segment( 2* Nt, Nt);

  ArrayXd xf = x + Constants::f2x * yaw.cos();
  ArrayXd xr = x + Constants::r2x * yaw.cos();
  ArrayXd yf = y + Constants::f2x * yaw.sin();
  ArrayXd yr = y + Constants::r2x * yaw.sin();

  int m = 4*Nt ;

  for ( int t = 0 ; t < Nt ;  t++){
    double xft = xf(t);
    double yft = yf(t);
    double xrt = xr(t);
    double yrt = yr(t);

    Box boxf(0,0,0,0), boxr(0,0,0,0);
    BoxStatus status_boxf = generateBox(dimx, dimy, xft, yft, obstacles, boxf);
    BoxStatus status_boxr  = generateBox(dimx, dimy, xrt, yrt, obstacles, boxr);

    // if (status_boxf.success && status_boxr.success){
      lb[a*m + t] =          boxf.x_min;
      lb[a*m +  Nt + t] = boxf.y_min;

      ub[a*m + t] =         boxf.x_max;
      ub[a*m +  Nt + t] = boxf.y_max;

      lb[a*m +2*Nt + t] =boxr.x_min;
      lb[a*m +3*Nt + t] = boxr.y_min;

      ub[a*m + 2*Nt + t] = boxr.x_max;
      ub[a*m +  3*Nt + t] = boxr.y_max;
    // }

    corridors[a][t].xf_min = boxf.x_min;
    corridors[a][t].xf_max = boxf.x_max;

    corridors[a][t].yf_min = boxf.y_min;
    corridors[a][t].yf_max = boxf.y_max;

    corridors[a][t].xr_min = boxr.x_min;
    corridors[a][t].xr_max = boxr.x_max;

    corridors[a][t].yr_min = boxr.y_min;
    corridors[a][t].yr_max = boxr.y_max;
  }

}

void SolverDSQP::calcCorridorConstraint(
  int a, SpMat& M, VectorXd& ub, VectorXd& lb, int si,
  const std::vector<std::vector<Corridor>>& corridors,
  const ArrayXd& x0, const ArrayXd& y0, 
  const ArrayXd& yaw0
){
  // dual circle approximation Na*(Nt-2) * 4 
  
  int z = Nt;
  double f2x = Constants::f2x;
  double r2x = Constants::r2x;  // negative

  // SpMat D( n_2circle, 3*Na*Nt );
  D.setZero();  // cannot insert to memory already exist
  D.data().squeeze();
  D.reserve(VectorXi::Constant(3*Nt, 4));


  SpMat diag1(z,z), diag2(z,z), diag3(z,z), diag4(z,z), eye_nant(z,z);
  assembleDiagSpMat(diag1, -f2x*yaw0.sin() );
  assembleDiagSpMat(diag2, f2x*yaw0.cos() );
  assembleDiagSpMat(diag3, -r2x*yaw0.sin() );
  assembleDiagSpMat(diag4, r2x*yaw0.cos() );
  assembleDiagSpMat(eye_nant, ArrayXd::Ones(z, 1) );

  if ( logger_level >= 2){
    timer.stop();
    std::cout << " calc and insert D part  time: " << timer.elapsedSeconds() << std::endl;
    timer.reset();
  }

  insertSparseMatrix(D, eye_nant, 0, 0);
  insertSparseMatrix(D, diag1, 0, 2*z);

  insertSparseMatrix(D, eye_nant, z, z);
  insertSparseMatrix(D, diag2, z, 2*z);

  insertSparseMatrix(D, eye_nant, 2*z, 0);
  insertSparseMatrix(D, diag3, 2*z, 2*z);

  insertSparseMatrix(D, eye_nant, 3*z, z);
  insertSparseMatrix(D, diag4, 3*z, 2*z);
  if ( logger_level >= 2){
    timer.stop();
    std::cout << " calc and insert D  time: " << timer.elapsedSeconds() << std::endl;
    timer.reset();
  }
  
  E << f2x * (yaw0.cos().matrix() + (yaw0*yaw0.sin()).matrix()),
            f2x * (yaw0.sin().matrix() - (yaw0*yaw0.cos()).matrix()),
            r2x * (yaw0.cos().matrix() + (yaw0*yaw0.sin()).matrix()),
            r2x * (yaw0.sin().matrix() - (yaw0*yaw0.cos()).matrix());

  if ( logger_level >= 2){
    timer.stop();
    std::cout << " calc and insert E  time: " << timer.elapsedSeconds() << std::endl;
    timer.reset();
  }

  // vector<double> corridor_lbs( Na*Nt * 4 ), corridor_ubs( Na*Nt * 4 );

  vector<double> elb (corridor_lbs.begin() + a*Nt*4, corridor_lbs.begin() + (a + 1)*Nt*4);
  vector<double> eub (corridor_ubs.begin() + a*Nt*4, corridor_ubs.begin() + (a + 1)*Nt*4);

  if ( logger_level >= 2){
    timer.stop();
    std::cout << " extract corridor time: " << timer.elapsedSeconds() << std::endl;
    timer.reset();
  }

  Eigen::Map<VectorXd,0,Eigen::InnerStride<1>> elb_eigen(elb.data(), elb.size());
  Eigen::Map<VectorXd,0,Eigen::InnerStride<1>> eub_eigen(eub.data(), eub.size());

  lb.block(si, 0, n_2circle, 1) = elb_eigen-E;
  ub.block(si, 0, n_2circle, 1) = eub_eigen-E;

  insertSparseMatrix(M, D, si, 0);

  MatrixXd xyyaw0(3*Nt, 1);
  xyyaw0 << x0, y0, yaw0;
  MatrixXd dxelb = D*xyyaw0  - ( elb_eigen-E );
  MatrixXd dxeub = ( eub_eigen-E ) - D*xyyaw0 ;
  if ( logger_level >= 3){
    cout << "double disc corridor lb (should >=0 ): \n"<< dxelb.transpose().format(CleanFmt)<< std::endl;
    cout << "double disc corridor ub (should >=0 ) : \n"<< dxeub.transpose().format(CleanFmt)<< std::endl;
  }
  if ( logger_level >= 2){
    cout << "double disc corridor  min value(should >=0 ): "<< dxelb.minCoeff() <<std::endl;
    cout << "double disc corridor  min value(should>=0): "<< dxeub.minCoeff() <<std::endl;
  }
  // assert( dxelb.minCoeff()  > -1e-5 && "initial should be satisfied");
  // assert( dxeub.minCoeff()  > -1e-5 && "initial should be satisfied");
  // todo change to warning.
  
}

void SolverDSQP::calcTrustRegionConstraint(
  SpMat& M, VectorXd& ub, VectorXd& lb, int si , double r_trust,
  const ArrayXd& x0, const ArrayXd& y0
  ){
  double r = r_trust;
  SpMat eye_n_trust(n_trust, n_trust);
  assembleDiagSpMat(eye_n_trust, ArrayXd::Ones(n_trust, 1) );
  insertSparseMatrix(M, eye_n_trust, si, 0);

  lb.block(si, 0, n_trust, 1) = (-r * ArrayXd::Ones(n_trust)).matrix();
  lb.block(si, 0, n_trust/2, 1) += x0.matrix();
  lb.block(si+n_trust/2, 0, n_trust/2, 1) += y0.matrix();

  ub.block(si, 0, n_trust, 1) =  (r * ArrayXd::Ones(n_trust)).matrix();
  ub.block(si, 0, n_trust/2, 1) += x0.matrix();
  ub.block(si+n_trust/2, 0, n_trust/2, 1) += y0.matrix();

  VectorXd lb_trust = lb.block(si, 0, n_trust, 1);
  VectorXd ub_trust = ub.block(si, 0, n_trust, 1);
  
  if ( logger_level >= 3){
    cout << "trust lb: "<< (solution0.block(0,0, 2*Nt, 1) - lb_trust).minCoeff() << std::endl;
    cout << "trust ub: "<< (ub_trust - solution0.block(0,0, 2*Nt, 1)).minCoeff() << std::endl;
  }
}

void SolverDSQP::calcMaxCtrlAndSteerConstraint(
  SpMat& M, VectorXd& ub, VectorXd& lb, int si , const  QpParm& param)
{
  double v_max = param.max_v;
  double w_max = param.max_omega;
  // steer_max

  SpMat eye_n_ctrl(n_ctrls, n_ctrls);
  assembleDiagSpMat(eye_n_ctrl, ArrayXd::Ones(n_ctrls, 1) );
  insertSparseMatrix(M, eye_n_ctrl, si, n_vars-n_ctrls);

  SpMat eye_n_steer(Nt, Nt);
  assembleDiagSpMat(eye_n_steer, ArrayXd::Ones(Nt, 1) );
  insertSparseMatrix(M, eye_n_steer, si+ n_ctrls,  3*Nt);

  lb.block(si, 0, n_ctrls/2, 1) = (-v_max * ArrayXd::Ones(n_ctrls/2)).matrix();
  lb.block(si+n_ctrls/2, 0, n_ctrls/2, 1) = (-w_max * ArrayXd::Ones(n_ctrls/2)).matrix();

  ub.block(si, 0, n_ctrls/2, 1) = (v_max * ArrayXd::Ones(n_ctrls/2)).matrix();
  ub.block(si+n_ctrls/2, 0, n_ctrls/2, 1) = (w_max * ArrayXd::Ones(n_ctrls/2)).matrix();

  lb.block( si + n_ctrls, 0, Nt, 1) =  (- steer_max * ArrayXd::Ones(Nt)).matrix();
  ub.block( si + n_ctrls, 0, Nt, 1) =  ( steer_max * ArrayXd::Ones(Nt)).matrix();


  // for warning and debug.
  VectorXd lb_umax = lb.block(si, 0, n_ctrls, 1);
  VectorXd ub_umax = ub.block(si, 0, n_ctrls, 1);
  VectorXd u0 = M.block(si, 0, n_ctrls, n_vars)*solution0;
  // VectorXd u0 = solution0.block(4*Na*Nt, 0, 2*Na*(Nt-1), 1);

  if ( logger_level >= 2){
    cout << "umax lb (should >=0): "<< (u0 - lb_umax).minCoeff() << std::endl;
    cout << "umax ub (should >=0): "<< (ub_umax - u0).minCoeff() << std::endl;
  }
  if ( logger_level >= 3){
    cout << "lb umax: \n" << (u0- lb_umax).format(CleanFmt)<< std::endl;
    
    cout << "ub umax: \n"  << (ub_umax - u0).format(CleanFmt)<< std::endl;
    
    cout << "u0:\n" << u0.format(CleanFmt) << std::endl;
  }

}

void calcInterIndex( const int& t, 
  const int & Nt,
  int& xf, int& yf, int& xr, int& yr){

  xf =  t;
  yf = Nt +  t;
  xr = 2*Nt +   t;
  yr = 3*Nt + t;
}

void SolverDSQP::calcInterInterPlaneSpMat( int a,
  const std::vector< InterPlane >& inter_planes
){
  if ( G_curr_id == a ){
    return;
  }
  G_curr_id = a;
  G.resize(n_inter, 4*Nt);
  H.resize(n_inter);
  G.setZero();
  H.setZero();

  for (size_t k=0; k < inter_planes.size(); k++){
    int xf_i, yf_i, xr_i, yr_i;
    int xf_j, yf_j, xr_j, yr_j;
    const InterPlane& plane = inter_planes[k];
    int t = plane.t;
    // int ai = inter_planes[k];
    // int aj = inter_planes[k];
    calcInterIndex(t, Nt, xf_i, yf_i, xr_i, yr_i);

    // calcInterIndex(aj, t, Na, Nt, xf_j, yf_j, xr_j, yr_j);
    G.insert(4 * k, xf_i) = plane.a_f2f;
    G.insert(4 * k, yf_i) = plane.b_f2f;  

    H(4*k) = plane.c_f2f;

    G.insert(4 * k+1, xf_i) =  plane.a_f2r;
    G.insert(4 * k+1, yf_i) = plane.b_f2r ;

    H(4*k+1) = plane.c_f2r;

    G.insert(4 * k+2, xr_i) = plane.a_r2f;
    G.insert(4 * k+2, yr_i) = plane.b_r2f;

    H(4*k+2) = plane.c_r2f;

    G.insert(4 * k+3, xr_i) = plane.a_r2r;
    G.insert(4 * k+3, yr_i) = plane.b_r2r;

    H(4*k+3) = plane.c_r2r;
  }

}


void SolverDSQP::calcInterVehicleConstraint(
  int a, SpMat& M, VectorXd& ub, VectorXd& lb, int si,
  const std::vector< InterPlane >& inter_planes,
  int max_inter_times,
  const ArrayXd& x0, const ArrayXd& y0,  const ArrayXd& yaw0
)
{
  if ( n_inter == 0 ){
    if ( logger_level >= 3){
      cout << "No inter vehicle constraints" << std::endl;
    }
    return;
  }

 
  double rv = Constants::rv;

  // extract the inter plane to sparse matrix and put it in G, H
  calcInterInterPlaneSpMat(a, inter_planes);
  SpMat M_GD = G*D; //mutual of G and D

  insertSparseMatrix(M, M_GD, si, 0);

  // lb.block(si, 0, n_inter, 1) =;
  double inf = std::numeric_limits<double>::infinity();
  ub.block(si, 0, n_inter, 1) = -(H + G*E);
  lb.block(si, 0, n_inter, 1) = - inf* ArrayXd::Ones(n_inter, 1).matrix();

  MatrixXd mat_inter = ub.block(si, 0, n_inter, 1) - M_GD*solution0.topLeftCorner(3*Nt, 1) ;
  if ( logger_level >= 3){
    cout << "mat_inter (should be >= 0):\n" << mat_inter.format(CleanFmt) << std::endl;
  }
}



SolverDSQP::SolverDSQP(
  vector<vector<OptimizeResult>>& solutions,
  const vector<vector<OptimizeResult>>& x0_bar,
  const vector<vector<InterPlane>>& inter_planes,
  double dimx, double dimy,
  const std::unordered_set<Location>& obstacles,
  const QpParm& param,
  int logger_level
): dimx(dimx), dimy(dimy), m_obstacles(obstacles), m_param(param),
Na( x0_bar.size()), Nt( x0_bar[0].size()),
n_vars(4*Nt + 2*(Nt-1)),
  solution0(n_vars, 1), D(Nt * 4, 3*Nt ), E( Nt * 4),
  x(Na, Nt), y(Na, Nt), yaw(Na, Nt), steer(Na, Nt), v(Na, Nt-1), w(Na, Nt-1),  
  corridor_lbs( Na*Nt * 4 ), corridor_ubs( Na*Nt * 4 ),
  solve_status(true),
  logger_level(logger_level),CleanFmt(4, 0, ", ", "\n", "[", "]")
{
 
  double time_max_corridor;

  // initial_static_legal: the initial guess is not colliding.
  initial_static_legal = calcCorridors(
    x0_bar, obstacles, dimx, dimy, 
    corridors, Na, Nt, time_max_corridor, logger_level
  );


  Timer timer_stat;
  double time_other = 0;
  solutions.resize(Na);

 
  // all the timestep need to be the same. just check the last element for simplicity.
  assert(x0_bar.back().size() == Nt);

  // s_k = (x,y,yaw,phi). phi: front wheel angle.
  n_kine =  4*(Nt-1) ;  // s_{k+1} =  A s_k + B u_k * dt. k=0~Nt-1
  n_config = 6;  // start and end config (x,y,yaw) for each agent
  n_2circle = Nt * 4;  // double circle approximation. corridor constraints.
  n_trust =  Nt * 2; // x, y trust region update
  n_ctrls =  (Nt - 1) * 2; // control variable boundaries.


  dt = param.dt;
  WheelBase = Constants::WB;
  steer_max = atan(WheelBase / Constants::r);
  max_iter = param.max_iter;

  vector<double> cfg;

  extractResult(Na, Nt, x0_bar, x, y, yaw, steer, v, w, cfg);
  extractAgentsCorridor(Na, Nt, corridors, corridor_lbs, corridor_ubs);


  if (logger_level >= 2){
    timer.stop();
    std::cout << " extractResult time: " << timer.elapsedSeconds() << std::endl;
    timer.reset();
  }

  // bool success;
  timer_stat.stop();
  time_other = timer_stat.elapsedSeconds();
  
  max_individual_opt_runtime = 0;
  for ( int a = 0 ; a < Na; a++){
    if ( logger_level >= 2) {
      cout << "\n\n---------------process agent " << a << "------------\n";
    }
    timer_stat.reset();

    vector<OptimizeResult> solution; 
    solve_status = calcIndividualSQP(a, solution, corridors, inter_planes, cfg, param);
    if ( !solve_status ){
      // break;
      continue;
    }

    solutions[a] = solution;
    timer_stat.stop();
    if ( timer_stat.elapsedSeconds() > max_individual_opt_runtime){
      max_individual_opt_runtime = timer_stat.elapsedSeconds();
    }

    if ( logger_level >= 2){
      cout << " agent "<< a << " calc individual time: " << timer_stat.elapsedSeconds() << std::endl;
    }
  }

  timer_stat.reset();

  if ( !unsuccess_status.empty() ){
    int worst_status = 2;
    std::cout << "solver inaccurate. " << std::endl;
    std::cout << "unsuccess agents: " << std::endl;
    for ( auto& p : unsuccess_status){
      std::cout << "agent "<< p.first <<", solver status: "<< p.second<<std::endl;
      if ( abs(p.second) > worst_status){
        worst_status = p.second ;
      }
    }

    // define in  usr/local/include/osqp/constants.h
    // status: 2. solved inaccurate; -3: primal infeasible; 4: primal infeasible inaccurate
    this->solve_status = worst_status;
  }
  else{
    std::cout << "solver solved all the qp successfully!" << std::endl;
    this->solve_status = 1; 

  }

  timer_stat.stop();
  time_other += timer_stat.elapsedSeconds();
  max_individual_opt_runtime += time_other;
  max_individual_opt_runtime += time_max_corridor;
}