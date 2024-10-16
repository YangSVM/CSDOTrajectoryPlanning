/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2020
*/

/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/algorithm/string/replace.hpp>

// #include "pbs/PBS.h"
#include "pbs/WinPBS.h"
#include "util/file_utils.h"
#include "hybrid_a_star/timer.h"
#include "util/Logger.h"

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
		
		("timeLimit,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")

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
    
	// int batchSize=vm["batchsize"].as<int>();
	double time_limit=vm["timeLimit"].as<double>();
	// cbs_scenario use a different coordination.



	std::string fname_config(__FILE__);
	boost::replace_all(fname_config, "win_pbs_search.cc", "config.yaml");
	readAgentConfig(fname_config);

	std::string inputFile = vm["input"].as<string>();
	std::string outputFile = vm["output"].as<string>();
	Instance instance(inputFile);
	size_t na = instance.getDefaultNumberOfAgents(); // number of agents.

	std::cout << "Calculating Solution...\n";


	srand(0);
	Timer timer;

    WinPBS wpbs(instance, vm["screen"].as<int>(), Constants::T_plan);

	double runtime = 0, iter_time = 0;
	bool success =false;
	vector<State> states(instance.start_states.begin(), instance.start_states.end());
	vector<Path> solutions_part;
	Logger logger;
	size_t timestep = 0;
	bool isStuck = true;
	while (runtime < time_limit){
		// run
		wpbs.clear();  // clear after use.
		bool find_local_traj = wpbs.solve(states ,time_limit);

		timer.stop();
		iter_time = timer.elapsedSeconds();
		runtime += timer.elapsedSeconds();

		// update. set the start states as T_plan later.
		wpbs.getPaths(solutions_part);
		timestep += Constants::T_plan;

		for ( int a = 0 ; a < na; a++){
			State s_next;
			if (solutions_part[a].states.size() < Constants::T_plan + 1){
				s_next = solutions_part[a].states.back().first;
				s_next.time = timestep;
			}
			else{
				s_next = solutions_part[a].states[Constants::T_plan].first;
			}

			{
				double dx = states[a].x - s_next.x;
				double dy = states[a].y - s_next.y;
				double dyaw = normalizeAngleAbsInPi(states[a].yaw - s_next.yaw);
				if ( fabs(dx) + fabs(dy) + fabs(dyaw) > 1e-3 ){
					isStuck = false;
				}
			}

			states[a] = s_next;
		}
		

		if ( isStuck){
			cout << "stuck now. time: " << timestep <<endl;
			break;
		}
		isStuck = true;


		logger.log(iter_time, solutions_part);
		

		// ----------- 3. judge it is success or not. --------------- //
		// method 1. all the paths end with goal but only check the T_plan inter conflict.
		// int max_time = 0;
		// for ( int a = 0 ; a < na; a++){
		// 	if ( max_time < solutions_part[a].states.size()){
		// 		max_time = solutions_part[a].states.size();
		// 	}
		// }
		// if ( max_time < Constants::T_plan){
		// 	success = true;
		// }

		// method 2. at goal.
		{
			success = true;
			for ( size_t a = 0 ; a < na; a++){
				double dx = instance.goal_states[a].x - states[a].x;
				double dy = instance.goal_states[a].y - states[a].y;
				double dyaw = normalizeAngleAbsInPi( instance.goal_states[a].yaw - states[a].yaw);
				if ( fabs(dx) + fabs(dy) + fabs(dyaw) > 1e-3){
					success = false;
					break;
				}
			}
		}

		// write a logger to log the trajectories and time.

		if (success){ // all agents arrived at their goals.
			break;
		}
	}
	
	std::cout << "total time: " << runtime<< std::endl;
	if ( success ){
		wpbs.savePaths(outputFile);
		
	}
	// dumpPlanResults(vm["output"].as<string>(), solution_vec, runtime);
	logger.dump_to_file(outputFile);


	return 0;

}