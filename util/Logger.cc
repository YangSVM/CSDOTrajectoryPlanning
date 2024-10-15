#include "util/Logger.h"
#include "util/file_utils.h"

// Logger::Logger(){
//     // solutions.resize(na);
// }


void Logger::log(double iter_time, const vector<Path>& solutions_part, size_t T){
    iter_times.push_back(iter_time);

    if ( T == -1){
        T = solutions_part.size();
    }

    size_t na = solutions_part.size();
    vector<Path> solutions_trunc;
    solutions_trunc.reserve(na);

    // complete the states to T +1 if it is not.
    for ( size_t a = 0 ; a < na ; a++ ){
        Path path = solutions_part[a];
        while ( path.states.size() < T+1){
            path.states.push_back(path.states.back());
            path.actions.push_back(path.actions.back());
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
    for ( auto rt : iter_times){
        runtime_avg += rt;
    }
    runtime_avg = runtime_avg/iter_times.size();

    dumpPlanResults(outputFile, solutions, runtime_avg);
}