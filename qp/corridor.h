#pragma once
#include "hybrid_a_star/motion_planning.h"
#include "hybrid_a_star/planresult.h"

namespace libMultiRobotPlanning{
struct Corridor{
    double xf_min, xf_max, yf_min, yf_max;
    double xr_min, xr_max, yr_min, yr_max;
};


struct BoxStatus{
    bool success;  // generate box successfully  or not.
    int intial_status; // 0 legal. 1 initial position out of map. 2 initial position collision.
};
struct Box
{
    double x_min, y_min, x_max, y_max;
    // left down corner point (xmin, ymin), right up corner point (xmax, ymax)
    Box(double x_min, double y_min, double x_max, double y_max)
        : x_min(x_min), y_min(y_min), x_max(x_max), y_max(y_max)
    {};

    Box ExpandBox(int direction, double ds){
        Box other(this->x_min, this->y_min, this->x_max, this->y_max);
        if (direction == 0){ 
            // up
            other.y_max += ds;
        }
        else if (direction == 1)
        {
            // left
            other.x_min -= ds;
        }
        else if (direction == 2)
        {
            // down
            other.y_min -= ds;
        }
        else if(direction ==3){
            other.x_max += ds;
        }
        else{
            assert(0);
        }
        return other;
    }

    bool isOverlap(const Box& other, double d=0){
        if (other.x_min - this->x_max > d ||
            this->x_min - other.x_max > d ||
            other.y_min - this->y_max > d ||
            this->y_min - other.y_max > d 
            )
            return false;
        return true;
    }
};

bool isBoxValid(const Box& box,
    const std::unordered_set<Location>& obstacles,
    double dimx, double dimy
);

Box initRegion(double xc, double yc, double ds);
bool generateLocalBox(double xc, double yc, 
      const std::unordered_set<Location>& obstacles,
      double dimx, double dimy,
      Box& res,
      double ds = 0.1, double l_limit = 10.0);

bool calcCorridors(
    const std::vector<PlanResultShort<State, Action, double>>& solutions,
    const std::unordered_set<Location>& obstacles,
    double dimx, double dimy, 
    std::vector<std::vector<Corridor>>& corridors,
    size_t Na, size_t Nt, 
    double& time_max_corridor,
    int screen = 3
);

void findRelativePair(
    const std::vector<std::vector<Corridor>>& corridors,
    std::vector< std::array<int, 3>>& relative_pair
);

bool  findRelativeByTrustRegion(
    const std::vector<PlanResultShort<State, Action, double>>& solution,
    const double& r, // trust region radius
    const double& rv, // rv radius
    std::vector< std::array<int, 3>>& relative_pair
);

BoxStatus generateBox(
    double dimx, double dimy, double x, double y, 
    const std::unordered_set<Location>& obstacles,
    Box& res
);
} // namespace libMultiRobotPlanning