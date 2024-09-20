#include "util/file_utils.h"
#include "common/motion_planning.h"
#include <yaml-cpp/yaml.h>
#include <boost/algorithm/string/replace.hpp>
#include <iomanip>      // std::setprecision
#include <fstream>


void dumpPlanResults(
    std::string fname_output, 
    const std::vector<libMultiRobotPlanning::PlanResult<State, Action, double>>& solution,
    double runtime
){
    std::ofstream out;
    out = std::ofstream(fname_output);

    double makespan = 0, flowtime = 0, cost = 0;
    for (const auto& s : solution) cost += s.cost;

    for (size_t a = 0; a < solution.size(); ++a) {
      // calculate makespan
      double current_makespan = 0;
      for (size_t i = 0; i < solution[a].actions.size(); ++i) {
        // some action cost have penalty coefficient

        if (solution[a].actions[i].second < Constants::dx[0])
          current_makespan += solution[a].actions[i].second;
        else if (solution[a].actions[i].first % 3 == 0)
          current_makespan += Constants::dx[0];
        else
          current_makespan += Constants::r * Constants::deltat;
      }
      flowtime += current_makespan;
      if (current_makespan > makespan) makespan = current_makespan;
    }
    std::cout << " Runtime: " << runtime << std::endl
              << " Makespan:" << makespan << std::endl
              << " Flowtime:" << flowtime << std::endl
              << " cost:" << cost << std::endl;
    // output to file
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  flowtime: " << flowtime << std::endl;
    out << "  runtime: " << runtime << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      yaw: " << state.first.yaw << std::endl
            << "      t: " << state.first.time << std::endl;
      }
    }
}


bool loadScenario(
    std::string input_file, size_t& dimx, size_t& dimy,   std::unordered_set<Location>& obstacles,
    std::vector<State>& start_states, std::vector<State>& goal_states, bool cbs_scenario
){
    YAML::Node map_config;

    try {
        map_config = YAML::LoadFile(input_file);
    } catch (std::exception& e) {
        std::cerr << "\033[1m\033[31mERROR: Failed to load map file: " << input_file
                << "\033[0m \n";
        return false;
    }
    const auto& dim = map_config["map"]["dimensions"];
    dimx = dim[0].as<int>();
    dimy = dim[1].as<int>();
    for (const auto& node : map_config["map"]["obstacles"]) {
    double r_obs = Constants::obsRadius;
    if ( !cbs_scenario){
        r_obs =  node[2].as<double>();
    }
    obstacles.insert(Location(node[0].as<double>(), node[1].as<double>(), r_obs));
  }
    for (const auto& node : map_config["agents"]) {
        const auto& start = node["start"];
        const auto& goal = node["goal"];
        double yaw_start, yaw_goal;
    if ( !cbs_scenario){
        yaw_start = -start[2].as<double>();
        yaw_goal = -goal[2].as<double>();
    }
    else{
        yaw_start = start[2].as<double>();
        yaw_goal = goal[2].as<double>();
    }
    start_states.emplace_back(State(start[0].as<double>(), start[1].as<double>(),
                                    yaw_start));
    // std::cout << "s: " << startStates.back() << std::endl;
    goal_states.emplace_back(State(goal[0].as<double>(), goal[1].as<double>(),
                            yaw_goal));
    }

    return true;

}