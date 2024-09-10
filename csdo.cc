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
#include "hybrid_a_star/motion_planning.h"

#include "pbs/PBS.h"

#include "qp/post_process.h"
#include "qp/corridor.h"
#include "qp/dqp_solver.h"
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
		("output,o", po::value<string>(), "output file for paths")
		// ("outputPaths", po::value<string>(), "output file for paths")
		("agentNum,k", po::value<int>()->default_value(-1), "number of agents")
		("obstacleNum,n", po::value<int>()->default_value(-1), "number of obstacles")
		("timeLimit,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")

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
  
  double time_limit=vm["timeLimit"].as<double>();


  int screen = vm["screen"].as<int>();
  std::string output_file = vm["output"].as<string>();
  size_t sz_output = output_file.size();
  std::string output_prefix = output_file.substr(0, sz_output - 5);

  SolutionStatistics stat;


  std::string fname_config(__FILE__);
  boost::replace_all(fname_config, "csdo.cc", "config.yaml");
  readAgentConfig(fname_config);

  std::string inputFile = vm["input"].as<string>();
  Instance instance(inputFile);

  std::cout << "after read config. constants::deltat "<< Constants::deltat << std::endl;


  std::cout << "Searching starts...\n";
 	srand(0);
	clock_t start = clock();
  PBS pbs(instance, vm["sipp"].as<bool>(), vm["screen"].as<int>());
  // run
  bool success = pbs.solve(time_limit);
	double runtime_search = (double)(clock() - start)/CLOCKS_PER_SEC  ;

  stat.rt_search = runtime_search;

	std::cout << "initial guess search time: " << runtime_search<< std::endl;
	if ( success ){
		pbs.savePaths(vm["output"].as<string>());
	}
  else{
    exit(-1);   // cannot search a initial guess.
  }

  std::vector<Path> solution;
  success = pbs.getPaths(solution); 
  double makespan = -1;
  double flowtime = 0;
  for ( size_t a = 0 ; a < solution.size(); a++){
    int ta  = solution[a].states.size() ;
    if ( ta > makespan){
      makespan = ta;
    }
    flowtime += ta;
  }
  double step_sz = Constants::r * Constants::deltat;
  stat.makespan = makespan * step_sz;
  stat.flowtime = flowtime * step_sz;

  // std::ofstream out;
  // out = std::ofstream(output_file);
  size_t Na = instance.getDefaultNumberOfAgents();
  Timer pre_all;
  if (success) {
    std::vector<PlanResultShort<State, Action, double>> solution_refine;
    using namespace libMultiRobotPlanning;
    QpParm param;
    readQpSolverConfig(fname_config, param);
    double preprocess_time=0;
    Timer ttimer;

    sample_path(solution, solution_refine, param.num_interpolation);
    size_t Nt = 0; 
    for ( size_t ia = 0; ia < Na; ia++){
      size_t nt_ia = solution_refine[ia].states.size();
      if (Nt < nt_ia){
        Nt = nt_ia;
      }
    }

    ttimer.stop();
    preprocess_time += ttimer.elapsedSeconds();
    cout << "sample time : " << ttimer.elapsedSeconds() << std::endl;
    ttimer.reset();

    std::vector<std::vector<OptimizeResult>> guesses;
    std::vector< std::array<int, 3> > relative_pair;

    // 补齐Nt时间的点
    calcInitGuess(solution_refine, guesses, Nt, Na, param.dt);

    ttimer.stop();
    preprocess_time += ttimer.elapsedSeconds();
    cout << "calculate initial v and w time : " << ttimer.elapsedSeconds() << std::endl;

    // dump should not include in the time.
    dumpSolutions(output_prefix+"_guesses.yaml", guesses, stat);

    ttimer.reset();


    double r_trust_region = param.trust_radius;
    if ( screen >= 3){
      cout << "trust region radius: " << r_trust_region << std::endl;
    }
    bool initial_inter_legal = findRelativeByTrustRegion(
      solution_refine, r_trust_region, Constants::rv, relative_pair);
    if ( !initial_inter_legal){
      stat.search_status = 1; // minor collision.
    }

    ttimer.stop();
    preprocess_time += ttimer.elapsedSeconds();
    cout << "findRelative By Trust Region time : " << ttimer.elapsedSeconds() << std::endl;
    ttimer.reset();

    cout<< "total preprocess time: " << preprocess_time << std::endl;
    pre_all.stop();
    cout<< "total preprocess time v2: " << pre_all.elapsedSeconds() << std::endl;
    stat.rt_preprocess = preprocess_time;


    cout << "param: dt "<<param.dt << ". max v: " << param.max_v
      << ". max w: "<< param.max_omega << ". trust r: " << param.trust_radius
      << std::endl;

    Nt = 0; 
    for (size_t a= 0 ; a< Na; a++){
        if (guesses[a].size() > Nt){
            Nt = guesses[a].size();
        }
    }


    Timer timer_optimize;
    double time_opt = 0;
    double time_max_corridor = 0;
    std::vector<std::vector<Corridor>> corridors;
    bool  initial_static_legal = calcCorridors(
        solution_refine, instance.obstacles, instance.dimx, instance.dimy, 
        corridors, Na, Nt, time_max_corridor
    );
    if ( ! initial_static_legal ){
      stat.search_status = 1;  // minor collision
    }

    timer_optimize.stop();
    time_opt += timer_optimize.elapsedSeconds();
    cout << "calculate total static corridor time : " << timer_optimize.elapsedSeconds() << std::endl;
    cout << "calculate decentralized static corridor time : " << time_max_corridor << std::endl;
    // test
    dumpCorridors(output_prefix+"_corridor.yaml" , corridors, solution_refine);
    timer_optimize.reset();

    std::vector<std::vector<OptimizeResult>> optimize_res;
    int logger_level = vm["screen"].as<int>();  // 0 error . 1 warning; 2 info; 3 debug.
    SolverDQP solver(Na, Nt, optimize_res, guesses, relative_pair, corridors, 
      instance.dimx, instance.dimy, instance.obstacles, param, logger_level);
    timer_optimize.stop();
    time_opt += timer_optimize.elapsedSeconds();
    // time_opt += solver.getMaxOfRuntimes();
    
    stat.solver_status = solver.getSolverStatus();
    // stat.search_status = solver.getSearchStatus();
    stat.rt_max_optimization = solver.getMaxOfRuntimes() + time_max_corridor;

    stat.rt_optimization = time_opt;
    stat.runtime = stat.rt_search + stat.rt_preprocess + stat.rt_max_optimization;
    // maybe change to the pbs path make span number.
    // stat.makespan = optimize_res.front().size() - 1;

    cout << "qp solver time: " << stat.rt_optimization << std::endl;
    cout << "qp fully decentralized solver time: " << stat.rt_max_optimization << std::endl;
    cout << "search time: " << stat.rt_search << std::endl;
    
    cout << "number of iterations: ";
    for ( auto& num_iter : solver.num_iterations){
      cout << num_iter << " ";
    }
    cout << std::endl;

    stat.makespan = Nt* param.dt;
    stat.flowtime = Nt* param.dt * Na;

    dumpSolutions(output_file, optimize_res, stat);
  
    return 1;
}
}