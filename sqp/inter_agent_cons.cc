#include <fstream>
#include <iomanip>
#include "sqp/inter_agent_cons.h"
#include "common/motion_planning.h"
#include "sqp/corridor.h"

namespace libMultiRobotPlanning
{

// ------------- Neighbor Pair Division ---------//

bool  findNeighborPairsByTrustRegion(
    const std::vector<std::vector<OptimizeResult>>& solution,
    const double& r, // trust region radius
    const double& rv,
    std::vector< std::array<int, 3>>& neighbor_pairs
){
    bool initial_inter_success = true;
    int Na = solution.size();
    // get the max time
    int Nt = solution[0].size();

    neighbor_pairs.reserve(Na*(Na-1)/2 * Nt);
    // double rv = Constants::rv;

    for ( int t = 0; t < Nt; t++){
        for ( int i = 0; i < Na - 1; i++){
            State si(solution[i][t].x, solution[i][t].y, solution[i][t].yaw);


            for (int j = i + 1; j < Na; j++){
                State sj(solution[j][t].x, solution[j][t].y, solution[j][t].yaw);
                double d = si.agentDistance(sj);

                if (d < 2*sqrt(2)*r)  
                {                
                    // is relative
                    neighbor_pairs.emplace_back(std::array<int, 3>{t, i, j});
                    if (si.agentCollision(sj)){
                        initial_inter_success = false;
                    }
                }

            }
        }
    }
    return initial_inter_success;

}


// 计算垂直平分线
// return a x1+b y1 + c <= 0
void calcPerpendicular(double x1, double y1, double x2, double y2,
  double& a, double&b, double& c1, double& c2){
    double rv = Constants::rv;
    a = x2 - x1;
    b = y2 - y1;
    double c = (x1*x1 + y1*y1 - x2*x2 - y2*y2)/2;
    // TODO 不知道是否正确
    double d = sqrt( pow(x1-x2, 2) + pow(y1-y2, 2) );
    c1 = c + rv * d;
    c2 = c - rv *d;
    double x0 = (x1+x2)/2;
    double y0 = (y1+y2)/2;
    assert( fabs( a*x0 + b*y0 + c ) < 1e-3); // the plane pass the point
    assert ( a * x1 + b *y1 +c < 0 ); // 
    assert ( a * x2 + b *y2 +c > 0 ); // 
}

void calcEqualInterPlanes(
    const std::vector<std::vector<OptimizeResult>>& x0_bar, 
    const std::vector<std::array<int, 3>>& neighbor_pairs,
    vector<vector<InterPlane>>& inter_planes
){
    size_t Na = x0_bar.size();
    size_t Nt = x0_bar[0].size();

    inter_planes.resize(Na);

    for ( auto & p :neighbor_pairs){
    int t = p[0];
    int ai = p[1];
    int aj = p[2];

    int ai_index = ai*Nt + t;
    int aj_index = aj*Nt + t;
    OptimizeResult ri = x0_bar[ai][t];
    OptimizeResult rj = x0_bar[aj][t];

    State sit (ri.x, ri.y, ri.yaw);
    State sjt (rj.x, rj.y, rj.yaw);
    double xfi, yfi, xri, yri;
    sit.GetDiscCenter( xfi, yfi,  xri, yri);
    double xfj, yfj, xrj, yrj;
    sjt.GetDiscCenter( xfj, yfj,  xrj, yrj);


    double a_f2f, b_f2f, c_f2f, c_f2f_;
    double a_f2r, b_f2r, c_f2r, c_f2r_;
    double a_r2f, b_r2f, c_r2f, c_r2f_;
    double a_r2r, b_r2r, c_r2r, c_r2r_;

    calcPerpendicular(
        xfi, yfi, 
        xfj, yfj, 
        a_f2f, b_f2f, c_f2f, c_f2f_
    );
    calcPerpendicular(
        xfi, yfi, 
        xrj, yrj, 
        a_f2r, b_f2r, c_f2r, c_f2r_
    );
    calcPerpendicular(
        xri, yri, 
        xfj, yfj, 
        a_r2f, b_r2f, c_r2f, c_r2f_
    );
    calcPerpendicular(
        xri, yri, 
        xrj, yrj, 
        a_r2r, b_r2r, c_r2r, c_r2r_ 
    );

    InterPlane plane_i(
        t, 
        a_f2f, b_f2f, c_f2f, a_f2r, b_f2r, c_f2r, 
        a_r2f, b_r2f, c_r2f, a_r2r, b_r2r, c_r2r
    );

    InterPlane plane_j(
        t,
        -a_f2f, -b_f2f, -c_f2f_, -a_r2f, -b_r2f, -c_r2f_, 
        -a_f2r, -b_f2r, -c_f2r_, -a_r2r, -b_r2r, -c_r2r_
    );
    inter_planes[ai].push_back(plane_i);
    inter_planes[aj].push_back(plane_j);
    }

}

// ------------- interpolation -------------------//
void InterpolateInitalGuess(
    const std::vector<PlanResult<State, Action, double>>& solution, 
    std::vector<std::vector<OptimizeResult>>& x0_bar, const QpParm& qp_parm){


    std::vector<PlanResultShort<State, Action, double>> solution_refine;
    interpolateXYYaw(solution, solution_refine, qp_parm.num_interpolation);
    
    calcVSteerW(solution_refine, x0_bar,  qp_parm.dt);
}

/**
 * @brief 根据动作生成值
 * 
 * @param action 动作编号，和Constants中定义相同
 * @param deltat_abs 转过的圆弧角度的绝对值
 * @param dx 车辆坐标系下的纵向位移
 * @param dy 横向
 * @param dyaw 右手系坐标
 * @param r 转弯半径
 */
void calcActionD(const int& action, const double& deltat_abs,
    double& dx, double& dy, double& dyaw, double r){
    std::vector<double> dx_ = {r * deltat_abs,
                   r * sin(deltat_abs),
                   r * sin(deltat_abs),
                   -r * deltat_abs,
                   -r * sin(deltat_abs),
                    -r * sin(deltat_abs)};
    std::vector<double> dy_ = {0,
                    -r * (1 - cos(deltat_abs)),
                    r * (1 - cos(deltat_abs)),
                    0,
                    -r * (1 - cos(deltat_abs)),
                    r * (1 - cos(deltat_abs))};
    std::vector<double> dyaw_ = {0, -deltat_abs,  deltat_abs,
                        0, deltat_abs, -deltat_abs};

    dx = dx_[action];
    dy = dy_[action];
    dyaw = dyaw_[action];
    
}

// 注意要重新计算角度值。变量连续能够喂进优化器
// n： 里面插多少个点
void action_sample(const int& action,
    PlanResultShort<State, Action, double>& solution_refine,
    const int& index, const State& s0, const State& s1, const int& n){
    
    for (int i = 0 ; i < n + 1 ; i++){
        solution_refine.actions.emplace_back(action);
    }

    if (action == 6){
        // wait action. copy
        for (int i = 1 ; i < n + 2; i++){
            solution_refine.states.emplace_back(
                s0.x, s0.y, s0.yaw, (n+1)*index + i);
        }
        return;
    }

    State s  = s0;
    double deltat;
    double r=Constants::r;
    if (action == 0 || action == 3){
        deltat = sqrt(pow(s1.x - s0.x, 2) + pow(s1.y - s0.y, 2))/Constants::r;
    }
    else{
        deltat = normalizeAngleAbsInPi(s1.yaw - s0.yaw);
        // adjust the radius.
        double d = sqrt(pow(s1.x - s0.x, 2) + pow(s1.y - s0.y, 2));
        r = d/(2.0*sin(fabs(deltat)/2.0));
    }
    double deltat_abs = fabs(deltat); 
    double xSucc, ySucc, yawSucc;

    // solution 1.
    double dx, dy, dyaw;
    
    calcActionD(action, deltat_abs/(double)(n+1), dx, dy, dyaw, r);

    for (int i = 1 ; i < n + 1; i++){

        xSucc = s.x + dx * cos(s.yaw) - dy * sin(s.yaw);
        ySucc = s.y + dx * sin(s.yaw) +  dy * cos(s.yaw);
        yawSucc = s.yaw + dyaw;

        solution_refine.states.emplace_back(xSucc, ySucc, yawSucc, (n+1)*index + i);
        s.x = xSucc;
        s.y = ySucc;
        s.yaw = yawSucc;
    }

    // solution2. 不要累积误差

    // double dx, dy, dyaw;
    // for (int i = 1 ; i < n + 1; i++){

    //     calcActionD(action, deltat_abs/(double)(n+1)*i, dx, dy, dyaw, r);
    //     xSucc = s0.x + dx * cos(s0.yaw) - dy * sin(s0.yaw);
    //     ySucc = s0.y + dx * sin(s0.yaw) + dy * cos(s0.yaw);
    //     yawSucc = s0.yaw + dyaw;

    //     solution_refine.states.emplace_back(xSucc, ySucc, yawSucc, (n+1)*index + i);
    // }

    // emplace the head of next seg.
    s  = s1;
    if (action == 0 || action == 3){
        dyaw = 0;
    }
    else{
        dyaw = deltat;
    }
    
   yawSucc = dyaw + s0.yaw;

    solution_refine.states.emplace_back(s1.x, s1.y, yawSucc, (n+1)*(index + 1));
 
    
    return;
}

/**
 * @param n sample number

 */
bool interpolateXYYaw( const std::vector<PlanResult<State, Action, double>>& solutions, 
    std::vector<PlanResultShort<State, Action, double>>& solutions_refine,
    const int& n){
    
    int num_agent = solutions.size();
    for (int a = 0 ; a < num_agent; a++){

        int num_states = solutions[a].states.size();
        int num_actions = solutions[a].actions.size();
        assert( num_states == num_actions + 1);

        int num_states_after = num_actions*n + num_states;
        PlanResultShort<State, Action, double> solution_refine;

        solution_refine.states.reserve(num_states_after);
        solution_refine.actions.reserve(num_states_after - 1);
        
        State s = solutions[a].states[0].first;
        
        solution_refine.states.emplace_back(s);
        for (int i = 0 ; i < num_states - 1; i++){
            action_sample(solutions[a].actions[i].first, solution_refine, i, s,
                solutions[a].states[i+1].first, n);
            
            s = solution_refine.states.back();

        }

        solutions_refine.emplace_back(solution_refine);
    }
    

    return true;
}


// Na: number of agents, Nt: number of maximum sample times.
// The solution is varied time length. The guesses is fixed time length.
bool calcVSteerW(const std::vector<PlanResultShort<State, Action, double>>& solutions,
std::vector<std::vector<OptimizeResult>>& guesses, double delta_t_
){
    size_t Na = solutions.size();
    
    size_t Nt = 0;
    for ( size_t a = 0 ; a <Na ; a++){
        if (solutions[a].states.size() >Nt) {
            Nt = solutions[a].states.size() ;
        }
    }

    guesses.resize(Na);
    for ( size_t a = 0 ; a < Na; a++){
        guesses[a].resize(Nt);
    }

    for (size_t a = 0; a < Na; a ++){
        size_t x_size = solutions[a].states.size();

        // load the poses
        for (size_t i = 0; i  < x_size; ++i) {
            guesses[a][i].x = solutions[a].states[i].x;
            guesses[a][i].y = solutions[a].states[i].y;
            guesses[a][i].yaw = solutions[a].states[i].yaw;
        }
        for (size_t i = x_size; i  < Nt; ++i){
            guesses[a][i].x = solutions[a].states.back().x;
            guesses[a][i].y= solutions[a].states.back().y;
            guesses[a][i].yaw = solutions[a].states.back().yaw;
        }

        // load the phi: steering angle
        guesses[a][0].steer = 0;
        for (size_t i = 1; i  < x_size; ++i) {
            int action = solutions[a].actions[i-1];
            double phi;
            double phi_action = 
                    std::atan((Constants::LF-Constants::LB)/Constants::r);
            if ( action == 0 || action == 3 || action == 6){
                phi = 0;
            }
            else if ( action ==1 || action == 4){
                phi = -phi_action;
            }
            else if ( action == 2 || action == 5){
                phi = phi_action;
            }
            else{
                assert(false && "unknow action.");
            }
            guesses[a][i].steer = phi;
        }
        for (size_t i = x_size; i  < Nt; ++i){
            guesses[a][i].steer = 0;
        }

        // load velocity from position
        // initial and end speed are set to be zeros
        // load omega from yaw.
        // guesses[a][0].v = 0;
        // guesses[a][0].d_steer = 0;
        for (size_t i = 0; i + 1 < x_size; ++i) {

            // double discrete_v = (((guesses[a][i + 1].x - guesses[a][i].x) / delta_t_) *
            //                     std::cos(guesses[a][i][2]) +
            //                     ((guesses[a][i].x - guesses[a][i-1].x) / delta_t_) *
            //                     std::cos(guesses[a][i][2])) /
            //                     2.0 +
            //                     (((guesses[a][i+1].y- guesses[a][i].y) / delta_t_) *
            //                     std::sin(guesses[a][i][2]) +
            //                     ((guesses[a][i].y- guesses[a][i-1].y) / delta_t_) *
            //                     std::sin(guesses[a][i][2])) /
            //                     2.0;
            double discrete_v =((guesses[a][i + 1].x - guesses[a][i].x) / delta_t_) *
                                std::cos(guesses[a][i].yaw) +
                                ((guesses[a][i+1].y- guesses[a][i].y) / delta_t_) *
                                std::sin(guesses[a][i].yaw) ;
            guesses[a][i].v = discrete_v;

            double discrete_omega = (guesses[a][i + 1].yaw - guesses[a][i].yaw) / delta_t_;
            double d_steer = (guesses[a][i + 1].steer - guesses[a][i].steer) / delta_t_;
                                
            guesses[a][i].d_steer = d_steer;
        }
        for (size_t i = x_size; i  < Nt; ++i){
            guesses[a][i].v = 0;
            guesses[a][i].d_steer = 0;
        }



    }
    
    return true;

}

void dumpSolutions(
  std::string file_name,
  const std::vector<std::vector<OptimizeResult>>& solutions ,
  const SolutionStatistics& stat
)
{
  std::ofstream out;
  out = std::ofstream(file_name);
  size_t Na = solutions.size();
  size_t Nt = solutions[0].size();
  
  out <<  std::fixed << std::setprecision(3);

  out << "statistics:" << std::endl;
  out << "  cost: " << stat.cost << std::endl;  // un supported
  out << "  makespan: " << stat.makespan << std::endl;
  out << "  flowtime: " << stat.flowtime << std::endl;
  out << "  runtime: " << stat.runtime << std::endl;
  out << "  runtime_search: " << stat.rt_search << std::endl;
  out << "  runtime_preprocess: " << stat.rt_preprocess << std::endl;
  out << "  runtime_optimization: " << stat.rt_optimization << std::endl;
  out << "  runtime_decentralized_optimization: " << stat.rt_max_optimization << std::endl;
  out << "  search_status: " << stat.search_status << std::endl;
  out << "  solver_status: " << stat.solver_status << std::endl;
  out << "schedule:\n";

  for (size_t a = 0; a < Na; a++){
    out << "  agent" << a<<":" <<std::endl;

    // save as degree not radian for easy reading.
    for (size_t t = 0 ; t< solutions[a].size(); t++) {
      out << "    - x: " << solutions[a][t].x << std::endl
          << "      y: " << solutions[a][t].y << std::endl
          << "      yaw: " << solutions[a][t].yaw  << std::endl
          << "      steer: " << solutions[a][t].steer * 180/3.14 << std::endl
          << "      t: " << t << std::endl;
      if (t == Nt - 1) {continue;}
      out << "      v: " << solutions[a][t].v << std::endl
          << "      omega: " << solutions[a][t].d_steer * 180/3.14  << std::endl;
    }
  }
  out.close();
}




// -------------------- neighbor pairs division ----------------------




} // namespace libMultiRobotPlanning
