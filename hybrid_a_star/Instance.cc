#include <yaml-cpp/yaml.h> // load instance yaml file

#include "hybrid_a_star/Instance.h"
#include <assert.h>

Instance::Instance(const std::string& instance_fname, 
	int num_of_agents,  int num_of_obstacles):
	instance_fname(instance_fname),  num_of_agents(num_of_agents),
  num_of_obstacles(num_of_obstacles)
{
	bool succ = loadMap();
	if (!succ)
	{
		std::cerr << "Map file " << instance_fname << " not found." << std::endl;
		exit(-1);
	}
  succ = startAndGoalValid(start_states, goal_states);
	if (!succ)
	{
		std::cerr << "Scenario illegal." << std::endl;
		exit(-1);
	}
}

bool Instance::loadMap()
{
	YAML::Node map_config;

    try {
        map_config = YAML::LoadFile(instance_fname);
    } catch (std::exception& e) {
        std::cerr << "\033[1m\033[31mERROR: Failed to load map file: " << instance_fname
                << "\033[0m \n";
        return false;
    }
    const auto& dim = map_config["map"]["dimensions"];
    dimx = dim[0].as<int>();
    dimy = dim[1].as<int>();
    for (const auto& node : map_config["map"]["obstacles"]) {
    int sz_obs = node.size();
    double r_obs = Constants::obsRadius;
    if ( sz_obs > 2){
        r_obs =  node[2].as<double>();
    }
    obstacles.insert(Location(node[0].as<double>(), node[1].as<double>(), r_obs));
  }
    for (const auto& node : map_config["agents"]) {
        const auto& start = node["start"];
        const auto& goal = node["goal"];
        double yaw_start, yaw_goal;

        yaw_start = start[2].as<double>();
        yaw_goal = goal[2].as<double>();
    start_states.emplace_back(State(start[0].as<double>(), start[1].as<double>(),
                                    yaw_start));
    // std::cout << "s: " << startStates.back() << std::endl;
    goal_states.emplace_back(State(goal[0].as<double>(), goal[1].as<double>(),
                            yaw_goal));
    }
    num_of_agents = start_states.size();
    num_of_obstacles = obstacles.size();
    return true;
}



void Instance::printAgents() const
{
  for (int i = 0; i < num_of_agents; i++) 
  {
    std::cout << "Agent" << i 
    << " : S=(" << start_states[i].x << "," << start_states[i].y << ". " << start_states[i].yaw 
    << ") ; G=(" <<  goal_states[i].x << "," << goal_states[i].y << ". " << goal_states[i].yaw  << ")" << std::endl;
  }
}

bool Instance::startAndGoalValid(
  const std::vector<State> &starts, const std::vector<State>& goals) {
  assert(goals.size() == starts.size());
  size_t Na = goals.size();
  for (size_t i = 0; i < Na; i++){
    for (size_t j = i + 1; j < Na; j++) {
      if (goals[i].agentCollision(goals[j])) {
        std::cout << "ERROR: Goal point of " << i  << " & "
                  << j  << " collide!\n";
        return false;
      }
      if (starts[i].agentCollision(starts[j])) {
        std::cout << "ERROR: Start point of " << i  << " & "
                  << j  << " collide!\n";
        return false;
      }
    }
  }
  // checking the violation of staic obstacles.
  double rv = Constants::rv;
  for ( size_t a = 0 ; a < Na; a++){
    for ( auto obs : obstacles){
      double xf, xr, yf, yr;
      starts[a].GetDiscCenter(xf,yf,xr,yr);
      if (
        pow( obs.x - xf , 2) + pow( obs.y - yf, 2)  - pow( rv + obs.r, 2) <0 ||
        pow( obs.x - xr , 2) + pow( obs.y - yr, 2)  - pow( rv + obs.r, 2) <0 
      ){
        std::cout <<"warning. Maybe inaccurate becasue the doule circle.\n ";
        std::cout << "agent " << a << " start and obstacle " << obs <<std::endl;
        if ( starts[a].obsCollision(obs)){
          return false;
        }
      }
      goals[a].GetDiscCenter(xf, yf, xr, yr);
      if (
        pow( obs.x - xf , 2) + pow( obs.y - yf, 2)  - pow( rv + obs.r, 2) <0 ||
        pow( obs.x - xr , 2) + pow( obs.y - yr, 2)  - pow( rv + obs.r, 2) <0 
        ){
        std::cout <<"warning. Maybe inaccurate becasue the doule circle.\n ";
        std::cout << "agent " << a << " goal and obstacle " << obs <<std::endl;
        if ( goals[a].obsCollision(obs)){
          return false;
        }
      }
    }
  }

  return true;
}