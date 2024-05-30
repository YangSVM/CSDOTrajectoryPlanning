/*sequantial_planner.cpp
* Solve a MVTP instance by a given order.
*/
#include <boost/program_options.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <yaml-cpp/yaml.h>

#include "hybrid_a_star/types.h"
#include "hybrid_a_star/hybrid_astar.h"
#include "util/file_utils.h"
#include "hybrid_a_star/motion_planning.h"
#include "hybrid_a_star/environment.h"
#include "pbs/common.h"
#include "hybrid_a_star/timer.h"
#include "hybrid_a_star/Instance.h"
#include "hybrid_a_star/hybrid_astar_interface.h"

// using libMultiRobotPlanning::HybridAStar;
// using libMultiRobotPlanning::Neighbor;
// using libMultiRobotPlanning::PlanResult;
// using libMultiRobotPlanning::Environment;
// using namespace libMultiRobotPlanning;


// typedef  libMultiRobotPlanning::Environment<Location, State, Action, double, Conflict, Constraint, Constraints> Env;
// typedef libMultiRobotPlanning::PlanResult<State, Action, double> Path;
// typedef libMultiRobotPlanning::HybridAStar<State, Action, double, 
//         Env> HybridAStarPlanner;

/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;


	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("input,i", po::value<string>()->required(), "input file for map")
		("output,o", po::value<string>(), "output file for statistics")
		("outputPaths", po::value<string>(), "output file for paths")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")
    ("timeLimit,t", po::value<double>()->default_value(60),"time limit for the process(seconds)")
    ("batchsize,b", po::value<int>()->default_value(10),"batch size for iter")
    ("scenario,c", po::value<bool>()->default_value(true), "cl-cbs benchmark or mvtp benchmark")

		("sipp", po::value<bool>()->default_value(1), "using SIPP as the low-level solver")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);

	srand((int)time(0));

	///////////////////////////////////////////////////////////////////////////
	srand(0);
    
  int batchSize=vm["batchsize"].as<int>();
  double time_limit=vm["timeLimit"].as<double>();
  // cbs_scenario use a different coordination.
  bool cbs_scenario = vm["scenario"].as<bool>();



  std::string fname_config(__FILE__);
  boost::replace_all(fname_config, "sequantial_planner.cc", "config.yaml");
  readAgentConfig(fname_config);

  std::string inputFile = vm["input"].as<string>();
  Instance instance(inputFile);

  std::cout << "Calculating Solution...\n";

  std::multimap<int, State> dynamic_obstacles;

  std::list<Path> solution;

  int num_car = instance.num_of_agents;
  double timer = 0;
  bool success=false;
  set<int> higher_agents;
  vector<Path*> paths;

  for ( int i = 0; i < num_car; i++){
    Timer iterTimer;

    HybriAStarInterface planner(instance, i);
    Path  m_solution = planner.findOptimalPath(higher_agents, paths, i);

    success = ! m_solution.states.empty();
    iterTimer.stop();

    if (m_solution.states.empty() ) {
      std::cout << "\033[1m\033[31m No." << i
                << "agent fail to find a solution \033[0m\n";
      break;
    } else {
      solution.push_back(m_solution);
      paths.emplace_back(&solution.back());
      higher_agents.insert(i);

      timer += iterTimer.elapsedSeconds();
      std::cout << "Complete " << i
                << " iter. Runtime:" << iterTimer.elapsedSeconds()
                << std::endl;
    }
  }


  if (success){
    std::vector<Path> solution_vec;
    for (auto p : solution){
      solution_vec.push_back(p);
    }
    std::cout << "\033[1m\033[32m Successfully find solution! \033[0m\n";
    
   dumpPlanResults(vm["output"].as<string>(), solution_vec, timer);
  }
  else{
    std::cout << "\033[1m\033[31m Fail to find paths \033[0m\n";
  }

 
	return 0;

}