// #define EIGEN_USE_MKL_ALL
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <yaml-cpp/yaml.h>
#include <boost/algorithm/string/replace.hpp>

#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>


#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>

#include "hybrid_a_star/types.h"
#include "common/motion_planning.h"

#include "pbs/PBS.h"

#include "sqp/common.h"
#include "sqp/inter_agent_cons.h"
#include "sqp/corridor.h"
#include "sqp/dsqp_solver.h"
#include "util/file_utils.h"


using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using namespace libMultiRobotPlanning;
using std::cout;


int main(int argc, char* argv[]){
	namespace po = boost::program_options;


	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("input,i", po::value<string>()->required(), "input file for map")
		("output,o", po::value<string>(), "output file for trajectories. must end with .yaml .")

		("timeLimit,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")

		("initial_guess", po::bool_switch()->default_value(false), "dump the initial guess.")
		("corridor", po::bool_switch()->default_value(false), "dump the corridors.")
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
  
  double time_limit=vm["timeLimit"].as<double>();
  bool dump_corridor = vm["corridor"].as<bool>();
  bool dump_initial_guess = vm["initial_guess"].as<bool>();

  int screen = vm["screen"].as<int>();
  std::string output_file = vm["output"].as<string>();
  size_t sz_output = output_file.size();
  std::string output_prefix = output_file.substr(0, sz_output - 5); // must end with ".yaml"

  SolutionStatistics solution_stat;


  std::string fname_config(__FILE__);
  boost::replace_all(fname_config, "csdo.cc", "config.yaml");
  readAgentConfig(fname_config);

  std::string inputFile = vm["input"].as<string>();
  Instance instance(inputFile);

  std::cout << "Read config Checking. constants::deltat "<< Constants::deltat << " " 
    << "Constants::maxClosedSetSize "<< Constants::maxClosedSetSize << std::endl;

	// ---------- 1.  search the intial guess ------------------------//
  std::cout << "Searching starts...\n";
 	srand(0);
	clock_t start = clock();
  PBS pbs(instance,  vm["screen"].as<int>());
  // run
  bool success = pbs.solve(time_limit);
	double runtime_search = (double)(clock() - start)/CLOCKS_PER_SEC  ;

  solution_stat.rt_search = runtime_search;

	std::cout << "initial guess search time: " << runtime_search<< std::endl;
	if (! success ){
    exit(-1);   // cannot search a initial guess.
  }

  std::vector<Path> solution;
  pbs.getPaths(solution); 

	// ---------- 2.1. deal with inter-vehicle constraints ------------------------//
  Timer ttimer;

  QpParm param;
  readQpSolverConfig(fname_config, param);
  
  vector<vector<OptimizeResult>> x0_bar; // interpolated intial guess. $x_0\bar$
  InterpolateInitalGuess(solution, x0_bar, param);

  std::vector< std::array<int, 3> > neighbor_pairs;  // NPairs. list of tuple <ai, aj, t>
  bool initial_inter_legal = findNeighborPairsByTrustRegion(
      x0_bar, param.r_trust, Constants::rv, neighbor_pairs);
  
  // record the initial guess as illegal if colliding.
  if ( !initial_inter_legal){
    solution_stat.search_status = 1; // recorded as minor collision.
  }

  vector<vector<InterPlane>> inter_planes; // neighbor pair division planes.
  calcEqualInterPlanes(x0_bar, neighbor_pairs, inter_planes);

  ttimer.stop();
  solution_stat.rt_preprocess = ttimer.elapsedSeconds();

  // dump should not include in the time.
  if ( dump_initial_guess){
    dumpSolutions(output_prefix+"_guesses.yaml", x0_bar, solution_stat);
  }

	// ---------- 2.2.  run the DSQP  ------------------------//
  ttimer.reset();
  std::vector<std::vector<OptimizeResult>> optimize_res;
  int logger_level = vm["screen"].as<int>();  // 0 error . 1 warning; 2 info; 3 debug.
  SolverDSQP solver(optimize_res, x0_bar,  inter_planes, 
    instance.dimx, instance.dimy, instance.obstacles, param, logger_level);
  ttimer.stop();

  // record the time and solution.
  if ( ! solver.get_initial_static_legal() ){
    solution_stat.search_status = 1;  // minor collision
  }
  solution_stat.rt_optimization = ttimer.elapsedSeconds();
  solution_stat.solver_status = solver.getSolverStatus();
  solution_stat.rt_max_optimization = solver.getMaxOfRuntimes() ;
  solution_stat.runtime = solution_stat.rt_search +\
  solution_stat.rt_preprocess + solution_stat.rt_max_optimization;

  if (dump_corridor){
    dumpCorridors(output_prefix+"_guesses.yaml", solver.corridors, x0_bar);
  }
  
  dumpSolutions(output_file, optimize_res, solution_stat);
  
  
  return 1;
}