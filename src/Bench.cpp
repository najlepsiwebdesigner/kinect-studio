#include "Bench.h"
std::map<std::string, std::vector<double> > Bench::labels2measurements;
std::map<std::string, std::chrono::high_resolution_clock::time_point> Bench::labelStarts;
std::map<std::string, std::vector<double> > Bench::labels2counts;
