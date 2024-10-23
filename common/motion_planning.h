#pragma once
#include <vector>
#include <math.h>
#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <unordered_set>


class Constants {
  public: 
// [m] --- The minimum turning radius of the vehicle
static float r ;
static  float deltat ;
// [#] --- A movement cost penalty for turning (choosing non straight motion
// primitives)
static  float penaltyTurning ;
// [#] --- A movement cost penalty for reversing (choosing motion primitives >
// 2)
static  float penaltyReversing ;
// [#] --- A movement cost penalty for change of direction (changing from
// primitives < 3 to primitives > 2)
static  float penaltyCOD ;
// map resolution
static  float mapResolution ;
static  float xyResolution ;
// static  float xyResolution = 0.05;
static  float yawResolution ;
// 1e5 
static float maxClosedSetSize;

// width of car
static  float carWidth ;
// distance from rear to vehicle front end
static  float LF ;
// distance from rear to vehicle back end
static  float LB ;
static float WB ;  // wheel base

// for double circle collision detection with agent and obstacle.
static float f2x ;
static float r2x ;
static float rv;

// obstacle default radius
static  float obsRadius ;
static  float constraintWaitTime ;

static float speed ;
static float t_inc;

static int T_plan;

// R = 3, 6.75 DEG。6 sample actions. forward(straight right left), back(straight right left)
static std::vector<double> dx;
static std::vector<double> dy ;
static std::vector<double> dyaw ;

static  double dubinsShotDistanceSquare ;

// normalize to [0, 2*pi)
static inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}
};  // Constants

// return theta in (-pi, pi ].
inline float normalizeAngleAbsInPi(double x){
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

// calculate agent collision more precisely BUT need LONGER time
#define PRCISE_COLLISION

struct Location {
  Location(double x, double y, double r) : x(x), y(y), r(r) {}
  double x;
  double y;
  double r;

  bool operator<(const Location& other) const {
    return std::tie(x, y, r) < std::tie(other.x, other.y, other.r);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y, r) == std::tie(other.x, other.y, other.r);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << "). "<< "r: "<< c.r;
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct State {
  State(double x, double y, double yaw, int time = 0)
      : time(time), x(x), y(y), yaw(yaw) {

        this->xf = x + Constants::f2x * cos(yaw);
        this->xr = x + Constants::r2x * cos(yaw);
        this->yf = y + Constants::f2x * sin(yaw);
        this->yr = y + Constants::r2x * sin(yaw);

        // rotation matrix
        rot.resize(2, 2);
        rot(0, 0) = cos(this->yaw);
        rot(0, 1) = -sin(this->yaw);
        rot(1, 0) = sin(this->yaw);
        rot(1, 1) = cos(this->yaw);

#ifdef PRCISE_COLLISION
    float d_center2real = (Constants::LF + Constants::LB)/2 - Constants::LB;
    this->xc = x + d_center2real * cos(yaw);
    this->yc = y + d_center2real * sin(yaw);
#endif
  }

  State() = default;

  bool operator==(const State& s) const {
    return std::tie(time, x, y, yaw) == std::tie(s.time, s.x, s.y, s.yaw);
  }

  bool agentCollision(const State& other) const {
#ifndef PRCISE_COLLISION
    if (pow(this->x - other.x, 2) + pow(this->y - other.y, 2) <
        pow(2 * Constants::LF, 2) + pow(Constants::carWidth, 2))
      return true;
    return false;
#else // use seperating axis theorem can be faster
 
    float length = Constants::LF + Constants::LB;
    float width = Constants::carWidth;
    float shift_x = other.xc - this->xc;
    float shift_y = other.yc - this->yc;

 
    float cos_v = cos(this->yaw);
    float sin_v = sin(this->yaw);
    float cos_o = cos(other.yaw);
    float sin_o = sin(other.yaw);
    float half_l_v = length/2;
    float half_w_v = width/2;
    float half_l_o = length/2;
    float half_w_o = width/2;

    float dx1 = cos_v * length/2;
    float dy1 = sin_v * length/2;
    float dx2 = sin_v * width/2;
    float dy2 = -cos_v * width/2;

    float dx3 = cos_o * length/2;
    float dy3 = sin_o * length/2;
    float dx4 = sin_o * width/2;
    float dy4 = -cos_o * width/2;

    // use seperating axis therom
    return ((fabs(shift_x * cos_v + shift_y * sin_v) <=
             fabs(dx3 * cos_v + dy3 * sin_v) + fabs(dx4 * cos_v + dy4 * sin_v) + half_l_v)
            && (fabs(shift_x * sin_v - shift_y * cos_v) <=
                 fabs(dx3 * sin_v - dy3 * cos_v) + fabs(dx4 * sin_v - dy4 * cos_v) + half_w_v)
            && (fabs(shift_x * cos_o + shift_y * sin_o) <=
                 fabs(dx1 * cos_o + dy1 * sin_o) + fabs(dx2 * cos_o + dy2 * sin_o) + half_l_o)
            && (fabs(shift_x * sin_o - shift_y * cos_o) <=
                 fabs(dx1 * sin_o - dy1 * cos_o) + fabs(dx2 * sin_o - dy2 * cos_o) + half_w_o));
#endif
  }

  //  to exactly collision detection. https://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection
    bool obsCollision(const Location& obstacle) const {
    boost::numeric::ublas::matrix<double> obs(1, 2);
    obs(0, 0) = obstacle.x - this->x;
    obs(0, 1) = obstacle.y - this->y;

    auto rotated_obs = boost::numeric::ublas::prod(obs, rot);
    if (rotated_obs(0, 0) > -Constants::LB - obstacle.r  &&
        rotated_obs(0, 0) < Constants::LF + obstacle.r &&
        rotated_obs(0, 1) > -Constants::carWidth / 2.0 - obstacle.r &&
        rotated_obs(0, 1) < Constants::carWidth / 2.0 + obstacle.r )
      return true;
    return false;
    
  }

  void GetDiscCenter(double & xf, double & yf, double & xr, double & yr) const{
    xf = this->xf;
    yf = this->yf;
    xr = this->xr;
    yr = this->yr;
  }

  double agentDistance(const State&sj){
    // Get the minimum distance for all the disc centers.
    double distance = pow(xf - sj.xf, 2) + pow(yf - sj.yf, 2);
    // 
    distance = std::min(distance,  pow(xf - sj.xr, 2) + pow(yf - sj.yr, 2));
    distance = std::min(distance,  pow(xr - sj.xf, 2) + pow(yr - sj.yf, 2));
    distance = std::min(distance,  pow(xr - sj.xr, 2) + pow(yr - sj.yr, 2));

    return std::sqrt(distance);
  }
  
  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ":" << s.yaw << ")@" << s.time;
  }

  int time;
  double x;
  double y;
  double yaw;

 private:
  float xc, yc;  // center position
  float xf, xr, yf, yr; // for double circle collision detection.
  boost::numeric::ublas::matrix<double> rot;  // exact collision detection

};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.yaw);
    return seed;
  }
};
}  // namespace std

double calcStatesDistance(const State& s1, const State& s2);
bool areStatesClose(const State& s1, const State& s2);

using Action = int;  // int<7 int ==6 wait
using Cost = double; 

void readAgentConfig(std::string fname_config) ;
