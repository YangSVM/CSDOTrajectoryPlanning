#include "util/Logger.h"
#include "util/file_utils.h"

// Logger::Logger(){
//     // solutions.resize(na);
// }


void Logger::log(double iter_time, const vector<Path>& solutions_part, size_t T){
    iter_times.push_back(iter_time);

    T= Constants::T_plan;

    size_t na = solutions_part.size();
    vector<Path> solutions_trunc;
    solutions_trunc.reserve(na);

    // complete the states to T +1 if it is not.
    for ( size_t a = 0 ; a < na ; a++ ){
        Path path = solutions_part[a];
        int t_end = path.states.back().first.time;
        auto state_end =  path.states.back();
        while ( path.states.size() < T+1){
            state_end.first.time++;
            path.states.push_back(state_end );
            path.actions.push_back(std::make_pair<int, double>(6, 0.0) );
        }
        solutions_trunc.push_back(path);
    }


    // truncate the states to T + 1.

    for ( size_t a = 0 ; a < na ; a++ ){
        Path path = solutions_trunc[a];

        path.states.resize(T+  1);
        path.actions.resize(T);
        path.times.resize(T);
        solutions_trunc[a] = path;
    }

    if ( iter_times.size() == 1){
        solutions = solutions_trunc;
        return;
    }
    
    // if not the first time. append the part solutions.


    // concate the paths.
    for ( size_t a = 0 ; a <na; a++){
        solutions[a] += solutions_trunc[a];
    }


}

void Logger::dump_to_file(const string& outputFile){
    double runtime_avg = 0 ;
    double runtime_max = 0;
    cout << "\n iter time :";
    for ( auto rt : iter_times){
        runtime_avg += rt;
        cout << rt << " ";
        if (rt > runtime_max){
            runtime_max = rt;
        }
    }
    cout << endl;
    runtime_avg = runtime_avg/iter_times.size();

    dumpPlanResults(outputFile, solutions, runtime_max);
}