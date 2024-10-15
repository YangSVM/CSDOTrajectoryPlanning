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

#include "pbs/PBS.h"
#include "util/file_utils.h"


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



	std::string fname_config(__FILE__);
	boost::replace_all(fname_config, "pbs_search.cc", "config.yaml");
	readAgentConfig(fname_config);

	std::string inputFile = vm["input"].as<string>();
	Instance instance(inputFile);

	std::cout << "Calculating Solution...\n";


	srand(0);
	clock_t start = clock();
    PBS pbs(instance, vm["screen"].as<int>());
    // run
    bool success = pbs.solve(time_limit);
	double runtime = (double)(clock() - start)/CLOCKS_PER_SEC  ;

	std::cout << "total time: " << runtime<< std::endl;
	if ( success ){
		pbs.savePaths(vm["output"].as<string>());
	}
	// dumpPlanResults(vm["output"].as<string>(), solution_vec, runtime);

	return 0;

}