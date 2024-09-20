#include <vector>
#include <math.h>
#include <assert.h>
#include "sqp/corridor.h"
#include "hybrid_a_star/planresult.h"
#include "common/motion_planning.h"
#include "hybrid_a_star/timer.h"
#include "sqp/inter_agent_cons.h"

namespace libMultiRobotPlanning{

State getState(size_t agentIdx,
                const std::vector<PlanResultShort<State, Action, double>>& solution,
                size_t t) {
  assert(agentIdx < solution.size());
  if (t < solution[agentIdx].states.size()) {
    return solution[agentIdx].states[t];
  }
  assert(!solution[agentIdx].states.empty());
  return solution[agentIdx].states.back();
}



bool isPointOutOfMap(double x, double y, double dimx, double dimy){
    double rv = Constants::rv;
    if ( x < rv || x>dimx-rv || y < rv || y > dimy-rv )
        return true;
    return false;
}

bool isPointCollision(
    double x, double y, const std::unordered_set<Location>& obstacles,
    Location& obs_collision
){
    // use box. 
    double rv = Constants::rv  ;
    Box box(x,y,x,y);
    for (const auto o : obstacles){
        Box box_d = box;
        for ( int i = 0 ; i < 4; i++){
            box_d = box_d.ExpandBox(i, o.r + rv);
        }
        if ( box_d.x_min < o.x && o.x < box_d.x_max &&
            box_d.y_min < o.y && o.y < box_d.y_max )
        {
            obs_collision = o;
            return true;
        }
    }
    return false;
}

void projectNearBorder(
    double dimx, double dimy,
    double& x_new, double&y_new,
    const double& x, const  double&y    
)
{
    double rv = Constants::rv;

    x_new = x;
    y_new = y;

    double eps = 1e-3;
    if (x < rv){
        x_new = rv + eps;
    }
    else if (x > dimx - rv)
    {
        x_new = dimx - rv - eps;
    }
    if ( y < rv){
        y_new = rv + eps;
    }
    else if ( y > dimy - rv)
    {
        y_new = dimy - rv -eps;
    }

}

// TODO 
bool generateLegalPoint(
    const Location& obstacle_collision,
    const std::unordered_set<Location>& obstacles,
    double dimx, double dimy, 
    double& x, double& y,
    Box& res
){
    int n_cadidates = 20; // need to be divided by 4.
    double x0 = x;
    double y0 = y;
    double theta0 = atan2( y0 - obstacle_collision.y, x0 - obstacle_collision.x);
    double d_safe = 0.2;
    double d = Constants::rv + obstacle_collision.r + d_safe;
    for ( int i = 0 ; i < n_cadidates; i++){
        // j = 0,-0,1,-1,2,-2,3,-3...
        int j = i/2;
        if ( i % 2 == 1){
            j = -j;
        }
        double theta = theta0 + j * 2* M_PI/n_cadidates;
        x = obstacle_collision.x + d * cos(theta);
        y = obstacle_collision.y + d * sin(theta);

        if ( x > Constants::rv && x < dimx - Constants::rv &&
            y > Constants::rv && y < dimy - Constants::rv ){                
                Box box(0,0,0,0);
                bool success_ = generateLocalBox(x, y, obstacles, dimx, dimy, box);
                if ( isBoxValid(box, obstacles, dimx, dimy)){
                    res = box;
                    return true;
                }
            }
    }
    // return an invalid box. It's area is 0.
    std::cout << "warning. no valid box found for it." << std::endl;
    Box box(x,y,x,y);
    res = box;
    return false;
}

BoxStatus generateBox(
    double dimx, double dimy, double x, double y, 
    const std::unordered_set<Location>& obstacles,
    Box& res
){
    // 2 bit status. lowest bit: 0 failed. 1 success.
    // highest bit. 0: legal. 1 out of map. 2: collsion
    BoxStatus status{0,0};


    Box box(x, y, x, y);
    Location obs_collision(0,0,0);

    //  if it is out of box, should generate 
    if ( isPointOutOfMap(x, y, dimx, dimy) ) {
        status.intial_status = 1;
        // project the point to the border.
        projectNearBorder(dimx, dimy, x, y , x, y);
    }

    if ( isPointCollision(x, y, obstacles, obs_collision))
    {
        status.intial_status = 2;
        // std::cout << "warning! state collision." << std::endl;
        // change the xf, yf to a legal position
        status.success = generateLegalPoint(obs_collision, obstacles, dimx, dimy, x, y, box);
    }
    else{
        status.success = generateLocalBox(x, y, obstacles, dimx, dimy, box);
    }

    // the box should be valid.
    assert(isBoxValid(box, obstacles, dimx, dimy));
    res = box;
    return status;
}

// output:
// front: xmin xmax ymin ymax, rear: xmin xmax ymin ymax
// initial guess status.
bool calcCorridors(
  const vector<vector<OptimizeResult>>& guesses,
    const std::unordered_set<Location>& obstacles,
    double dimx, double dimy, 
    std::vector<std::vector<Corridor>>& corridors,
    size_t Na, size_t Nt, 
    double& time_max_corridor,
    int screen
){
    Timer timer;
    bool initial_success = true;
    corridors.resize(Na);
    for ( size_t a = 0 ; a < Na; a++){
        corridors[a].resize(Nt);
    }
     
    for (size_t a = 0; a < Na; a ++){
        timer.reset();


        for (size_t i = 0; i < Nt; i++){            
            double xf, yf, xr, yr;            
            State si(guesses[a][i].x, guesses[a][i].y, guesses[a][i].yaw);
            si.GetDiscCenter(xf,yf,xr,yr);

            Box boxf(0,0,0,0), boxr(0,0,0,0);
            BoxStatus status_boxf = generateBox(dimx, dimy, xf, yf, obstacles, boxf);
            BoxStatus status_boxr  = generateBox(dimx, dimy, xr, yr, obstacles, boxr);

            if ( screen >= 1){ // warning info
                if ( !status_boxf.success ){
                    std::cout << "Warning! Corridor. Agent "<< a << " at time: "<< i 
                        << ". cannot generate valid box front. \n";
                }
                if ( !status_boxr.success ){
                    std::cout << "Warning! Corridor. Agent "<< a << " at time: "<< i  
                        << ". cannot generate valid box rear. \n";
                }
            }
            if ( screen >= 3){ // debug info
                if ( status_boxf.intial_status > 0 ){
                    std::cout << "Debug. Corridor. Agent "<< a<< " at time: "<< i  
                         << ". intial box front illegal. ";
                    if (status_boxf.intial_status == 1){
                        std::cout << "out of map.\n";
                    }
                    else if (status_boxf.intial_status == 2)
                    {
                        std::cout << "collision with obstacle.\n";
                    }                    
                }
                if ( status_boxr.intial_status > 0 ){
                    std::cout << "Debug. Corridor. Agent "<< a<< " at time: "<< i  
                         << ". intial box rear illegal. ";
                    if (status_boxr.intial_status == 1){
                        std::cout << "out of map.\n";
                    }
                    else if (status_boxr.intial_status == 2)
                    {
                        std::cout << "collision with obstacle.\n";
                    }                    
                }
            }

            if ( status_boxf.intial_status >0 || status_boxr.intial_status > 0 ){
                initial_success = false;
            }
            
            corridors[a][i].xf_min = boxf.x_min;
            corridors[a][i].xf_max = boxf.x_max;
            corridors[a][i].yf_min = boxf.y_min;
            corridors[a][i].yf_max = boxf.y_max;
            
            corridors[a][i].xr_min = boxr.x_min;
            corridors[a][i].xr_max = boxr.x_max;
            corridors[a][i].yr_min = boxr.y_min;
            corridors[a][i].yr_max = boxr.y_max;
        }
        timer.stop();
        if ( time_max_corridor < timer.elapsedSeconds()){
            time_max_corridor = timer.elapsedSeconds();
        }
    }
    return initial_success;
}


// 先写一个近似的碰撞检测， 原理是将矩形进行膨胀操作，再看点是否在矩形内
bool isBoxValid(const Box& box,
    const std::unordered_set<Location>& obstacles,
    double dimx, double dimy
){
    double rv = Constants::rv;
    // cannot go out of the map.
    if (box.x_min < rv || box.x_max>dimx-rv || box.y_min < rv || box.y_max > dimy-rv)
        return false;
    for (const auto o : obstacles){
        Box box_d = box;
        for ( int i = 0 ; i < 4; i++){
            box_d = box_d.ExpandBox(i, o.r + rv);
        }
        if ( box_d.x_min < o.x && o.x < box_d.x_max &&
            box_d.y_min < o.y && o.y < box_d.y_max )
        {
            return false;
        }
    }
    return true;
};

Box initRegion(double xc, double yc, double ds){
    return Box(xc-ds, yc-ds, xc+ds, yc+ds);
}

bool generateLocalBox(double xc, double yc, 
      const std::unordered_set<Location>& obstacles,
      double dimx, double dimy,
      Box& res,
      double ds, double l_limit
      ){
    std::vector<int> id = {0, 1, 2, 3};
    std::vector<double> directions = {0,1,2,3}; // +y, -x, -y, +x
    std::vector<double> lens = {0, 0, 0, 0};

    Box box(xc, yc, xc, yc);
    // assert(isBoxValid(box, obstacles, dimx, dimy));
    int num_expand = 0;
    // r_approved
    int n_id_valid = 4;
    while (n_id_valid > 0)
    {
        for ( auto i : id){
            if ( i == -1){
                continue;
            }
            Box b_trail = box.ExpandBox(directions[i], ds);

            if (isBoxValid(b_trail, obstacles, dimx, dimy)){
                num_expand++;
                lens[i] += ds;
                box = b_trail;
                if (lens[i] >= l_limit){
                    n_id_valid -= 1;
                    id[i] = -1;
                }
            }
            else{
                n_id_valid -= 1;
                id[i] = -1;
            }
        }
    }
    
    res = box;
    bool success = false;
    if (num_expand > 0){
        success = true;
    }
    return success;

}

} // namespace libMultiRobotPlanning