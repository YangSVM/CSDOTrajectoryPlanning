#include "hybrid_a_star/motion_planning.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <boost/algorithm/string/replace.hpp>


// [m] --- The minimum turning radius of the vehicle
float Constants::r = 5.2;
float Constants::deltat = 6.75 / 180.0 * M_PI;
// [#] --- A movement cost penalty for turning (choosing non straight motion
// primitives)
float Constants::penaltyTurning = 1.3;
// [#] --- A movement cost penalty for reversing (choosing motion primitives >
// 2)
float Constants::penaltyReversing = 2.0;
// [#] --- A movement cost penalty for change of direction (changing from
// primitives < 3 to primitives > 2)
float Constants::penaltyCOD = 2.0;
// map resolution
float Constants::mapResolution = 2.0;
float Constants::xyResolution = r * deltat;
// float Constants::xyResolution = 0.05;
float Constants::yawResolution = deltat;

// width of car
float Constants::carWidth = 2.0 ;
// distance from rear to vehicle front end
float Constants::LF = 3.8;
// distance from rear to vehicle back end
float Constants::LB = 1.0 ;
float Constants::WB = 1.0 ;  // wheelbase

// for double circle collision detection with agent and obstacle.
float Constants::f2x = 1/4.0 * (3.0*LF - LB);
float Constants::r2x = 1/4.0 * (LF - 3.0 * LB);
float Constants::rv = 1.0/2.0 * pow(pow(LF + LB, 2)/4 +  carWidth* carWidth, 0.5);

// obstacle default radius
float Constants::obsRadius = 0.8;
float Constants::constraintWaitTime = 2;

float Constants::speed = 1;   // 正常行驶速度
float Constants::t_inc = deltat * r /speed;  // 每个采样点时间的时间间隔

double Constants::dubinsShotDistance = 100;
// R = 3, 6.75 DEG。6个采样动作. 前(中右左)，后(中右左)
std::vector<double> Constants::dx = {r * deltat, r* sin(deltat),  r* sin(deltat),
                     -r* deltat, -r* sin(deltat), -r* sin(deltat)};
std::vector<double> Constants::dy = {0, -r*(1 - cos(deltat)), r*(1 - cos(deltat)),
                     0, -r*(1 - cos(deltat)), r*(1 - cos(deltat))};
std::vector<double> Constants::dyaw = {0, -deltat, deltat, 0, deltat, -deltat};


void readAgentConfig(std::string fname_config) {
  YAML::Node car_config;

  try {
    car_config = YAML::LoadFile(fname_config.c_str());
  } catch (std::exception& e) {
    std::cerr << "\033[1m\033[33mWARNING: Failed to load agent config file: "
              << fname_config << "\033[0m , Using default params. \n";
  }
  // int car_r = car_config["r"].as<int>();
  Constants::r = car_config["r"].as<double>();
  Constants::deltat = car_config["deltat"].as<double>();
  Constants::penaltyTurning = car_config["penaltyTurning"].as<double>();
  Constants::penaltyReversing = car_config["penaltyReversing"].as<double>();
  Constants::penaltyCOD = car_config["penaltyCOD"].as<double>();
  // map resolution
  Constants::mapResolution = car_config["mapResolution"].as<double>();
  // change to set calcIndex resolution
  Constants::xyResolution = Constants::r * Constants::deltat;
  Constants::yawResolution = Constants::deltat;

  Constants::carWidth = car_config["carWidth"].as<double>();
  Constants::LF = car_config["LF"].as<double>();
  Constants::LB = car_config["LB"].as<double>();
  Constants::WB = car_config["WB"].as<double>();

  // for double circle collision detection with agent and obstacle.
  Constants::f2x = 1/4.0 * (3.0*Constants::LF - Constants::LB);
  Constants::r2x = 1/4.0 * (Constants::LF - 3.0 * Constants::LB);
  Constants::rv = 1.0/2.0 * pow(pow(Constants::LF + Constants::LB, 2)/4+ 
      Constants::carWidth * Constants::carWidth, 0.5);

  // obstacle default radius
  Constants::obsRadius = car_config["obsRadius"].as<double>();
  // least time to wait for constraint
  Constants::constraintWaitTime = car_config["constraintWaitTime"].as<double>();

  Constants::speed = 1.0;   // 正常行驶速度
  Constants::t_inc = Constants::deltat * Constants::r /Constants::speed;  // 每个采样点时间的时间间隔

  Constants::dx = {Constants::r * Constants::deltat,
                   Constants::r * sin(Constants::deltat),
                   Constants::r * sin(Constants::deltat),
                   -Constants::r * Constants::deltat,
                   -Constants::r * sin(Constants::deltat),
                   -Constants::r * sin(Constants::deltat)};
  Constants::dy = {0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat)),
                   0,
                   -Constants::r * (1 - cos(Constants::deltat)),
                   Constants::r * (1 - cos(Constants::deltat))};
  Constants::dyaw = {0, -Constants::deltat,  Constants::deltat,
                     0, Constants::deltat, -Constants::deltat};
}

