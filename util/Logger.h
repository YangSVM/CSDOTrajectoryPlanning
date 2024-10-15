#pragma once
#include "common/motion_planning.h"
#include "pbs/common.h"
#include "hybrid_a_star/types.h"

class Logger
{
public:
Logger(){};
~Logger(){};

void log(double iter_time, const vector<Path>& solutions_part , size_t T = -1);
void dump_to_file(const string& output_file);

private:

vector<double> iter_times;
vector<Path> solutions;

};