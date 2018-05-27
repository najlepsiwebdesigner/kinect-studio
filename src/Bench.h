#ifndef BENCH_H
#define BENCH_H

#include <map>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>
#include <numeric>
#include <fstream>

class Bench {
public:
	static void start(std::string label) {
		Bench::labelStarts[label] = std::chrono::high_resolution_clock::now();
	}

	static void stop(std::string label) {
		auto start = Bench::labelStarts[label];
		auto end = std::chrono::high_resolution_clock::now();
		auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		Bench::labels2measurements[label].push_back(millis);
	}

	static void count(std::string label, double number) {
		Bench::labels2counts[label].push_back(number);
	}

	static void printLabel(std::string label) {
		std::cout << "Collected measurements size: " << Bench::labels2measurements[label].size() << std::endl;
	}

	static void printSummary() {

		std::cout << "Bench summary" << std::endl
				  << "-------------" << std::endl;

		for (auto const& row : Bench::labels2measurements) {
			std::cout << row.first << ":" << std::endl
					   << "  number of samples: " << row.second.size() << std::endl
					   << "  average time: " << accumulate( row.second.begin(), row.second.end(), 0.0)/row.second.size() 
					   << std::endl << std::endl; 

		    std::string output_filename = "/Volumes/rdisk/" + row.first + "-times.txt";
		    std::ofstream output_file(output_filename);
		    for (const auto &e : row.second) output_file << e << "\n";
		    // std::ostream_iterator<double> output_iterator(output_file, "\n");
    		// std::copy(row.second.begin(), row.second.end(), output_iterator);

		}

		for (auto const& row : Bench::labels2counts) {
			std::cout << row.first << ":" << std::endl
					  << "  number of samples: " << row.second.size() << std::endl
					  << "  average value: " << accumulate( row.second.begin(), row.second.end(), 0.0)/row.second.size()
					  << std::endl << std::endl; 
			
			std::string output_filename = "/Volumes/rdisk/" + row.first + "-values.txt";
		    std::ofstream output_file(output_filename);
		    for (const auto &e : row.second) output_file << e << "\n";
		    // std::ostream_iterator<double> output_iterator(output_file, "\n");
    		// std::copy(row.second.begin(), row.second.end(), output_iterator);
		}

	}

private:
	// this is map which ties labels to all collected measurements
	static std::map<std::string, std::vector<double> > labels2measurements;
	static std::map<std::string, std::chrono::high_resolution_clock::time_point> labelStarts;
	static std::map<std::string, std::vector<double> > labels2counts;
};

#endif // BENCH_H
