/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <memory>
#include <string>
#include "instance.h"
#include "pbs.h"
#include "common.h"


/* Main function */
int main(int argc, char **argv) {
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "produce help message")

            // params for the input instance and experiment settings
            ("map,m", po::value<string>()->default_value("random-32-32-20.map"), "input file for map")
            ("agents,a", po::value<string>()->default_value("random-32-32-20-random-1.scen"), "input file for agents")

            ("heuristic,h",
             po::value<string>()->default_value("./data/sortation_large/heuristic/sortation_large-random-1.heuristic"),
             "heuristic for agents")
            ("output,o", po::value<string>()->default_value("./output/output.txt"), "output file for schedule")
            ("statistic,c", po::value<string>()->default_value("./output/output.csv"),
             "output file for statistic result")
            ("agentNum,k", po::value<int>()->default_value(1), "number of agents")
            ("agentIdx", po::value<string>()->default_value(""), "customize the indices of the agents (e.g., \"0,1\")")
            ("seed,d", po::value<int>()->default_value(0), "random seed")
            ("solver,s", po::value<int>()->default_value(0),
             "single agent solver for PBS, choose from sipp-ip and bezier")
            ("outputPaths", po::value<string>(), "output file for paths")
            ("partialExpansion,p", po::value<bool>()->default_value(1), "enable partial expansion")
            ("cutoffTime,t", po::value<double>()->default_value(300), "cutoff time (seconds)")

            // params for instance generators
            ("rows", po::value<int>()->default_value(0), "number of rows")
            ("cols", po::value<int>()->default_value(0), "number of columns")
            ("obs", po::value<int>()->default_value(0), "number of obstacles")
            ("warehouseWidth", po::value<int>()->default_value(0),
             "width of working stations on both sides, for generating instances")

            // params for CBS
            ("sipp", po::value<bool>()->default_value(false), "using sipp as the single agent solver");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        cout << desc << endl;
        return 1;
    }

    po::notify(vm);
    /////////////////////////////////////////////////////////////////////////
    /// check the correctness and consistence of params
    //////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////////////
    /// load the instance
    //////////////////////////////////////////////////////////////////////
    std::shared_ptr<Instance> instance_ptr = std::make_shared<Instance>(vm["map"].as<string>(),
                                                                        vm["agents"].as<string>(),
                                                                        vm["agentNum"].as<int>(),
                                                                        vm["agentIdx"].as<string>(),
                                                                        vm["rows"].as<int>(), vm["cols"].as<int>(),
                                                                        vm["obs"].as<int>(),
                                                                        vm["warehouseWidth"].as<int>(),
                                                                        vm["partialExpansion"].as<bool>(),
                                                                        vm["solver"].as<int>());

    srand(vm["seed"].as<int>());

    //////////////////////////////////////////////////////////////////////
    /// initialize the solver
    //////////////////////////////////////////////////////////////////////
    PBS pbs(instance_ptr, vm["solver"].as<int>(), vm["cutoffTime"].as<double>());
    auto init_start_time = Time::now();
    pbs.sipp_ptr->getHeuristic(vm["heuristic"].as<std::string>());
    auto init_end_time = Time::now();
    time_s debug_init_d = init_end_time - init_start_time;
    double debug_init_time = debug_init_d.count();
    printf("Finish initialization the heuristic in %f seconds\n", debug_init_time);
    auto global_start_time = Time::now();
    bool pbs_success = pbs.solve(vm["output"].as<string>());
    auto global_end_time = Time::now();
    std::chrono::duration<float> global_run_time = global_end_time - global_start_time;
    printf("Runtime for MASS is: %f\n", global_run_time.count());
    string traj_name = vm["agents"].as<string>();
    if (pbs_success) {
        printf("Solution found!\n");
        pbs.updateCost();
        if (vm.count("outputPaths"))
            pbs.saveTimedPath(vm["outputPaths"].as<std::string>());
        pbs.savePath("durationPath.txt");
    } else {
        printf("No solution found!\n");
    }
    pbs.saveResults(pbs_success, vm["statistic"].as<string>(), vm["map"].as<string>());
}