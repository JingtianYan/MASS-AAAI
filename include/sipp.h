#pragma once

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include <ilcplex/ilocplex.h>

#include <chrono>
#include <vector>

#include "instance.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "berstein.h"
#include "milp_cache.h"
#include "common.h"
#include "tpg_solver.h"
#include "crise_solver.h"
//#include "sipp_ip.h"

using std::vector;


typedef vector<Node> Successors;

struct NodeEqual{
    bool operator()(const std::shared_ptr<Node>& lhs, const std::shared_ptr<Node>& rhs) const {
        return lhs->current_point == rhs->current_point and lhs->curr_o == rhs->curr_o
               and lhs->interval_index == rhs->interval_index;
    }
};

struct NodeHash {
    size_t operator()(const std::shared_ptr<Node>& key) const {
        size_t hashA = std::hash<int>()(key->current_point);
        size_t hashB = std::hash<int>()(key->curr_o);
        size_t hashC = std::hash<int>()(key->interval_index);

        size_t combined = (hashA << 8) + (hashC << 2) + hashB;
        return combined;
    }
};


class SIPP {

public:
    explicit SIPP(const std::shared_ptr<Instance>&, double cutoff_time);
    bool run(int agent, ReservationTable& rt, MotionInfo& solution, Path& path, double& solution_cost, TimedPath& timed_path);
    bool getInitNode(ReservationTable& rt, std::shared_ptr<Node>& init_node);
    void getHeuristic(const std::string& heuristic_file);
    void showSolution(std::shared_ptr<Node>& s);

private:
    void PrintNonzeroRT(ReservationTable& rt) const;
    void Reset();
    inline int DistHeuristic(int curr_loc)
    {
        int h_val = instance_ptr->getManhattanDistance(curr_loc, curr_agent.goal_location);
        return h_val;
    }
    bool Dijkstra(size_t curr_id);
    void updateResultNodes(std::shared_ptr<Node> end_node, Path& time_interval_path, MotionInfo& solution, TimedPath& timed_path);

    void getSuccessors(const std::shared_ptr<Node>& s, std::vector<int>& to_locs, ReservationTable& rt);
    void GetNextInterval(const std::shared_ptr<IntervalEntry>& prev_interval,
                         IntervalQueue& new_interval_list,
                         std::vector<int>& next_locs,
                         ReservationTable& rt,
                         const std::shared_ptr<Node>& s);
    static void findFreeIntervals(std::vector<TimeInterval>& reservations, std::vector<TimeInterval>& freeIntervals);
    void nodeExpansion(const std::shared_ptr<Node>& n, ReservationTable& rt);
    inline void pushToOpen(const std::shared_ptr<Node>& new_node)
    {
        // dominance check here
        if (dominanceCheck(new_node)){
            open.push(new_node);
            allNodes_table[new_node].push_back(new_node);
            count_node_generated++;
        }
    }
    bool dominanceCheck(const std::shared_ptr<Node>& new_node);

private:
    // Instance& instance;
    std::shared_ptr<Instance> instance_ptr;
    vector<std::shared_ptr<Node>> result_nodes;
    Agent curr_agent;
    std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeCompare> open;
    std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual> closed_set;
    std::unordered_set<std::shared_ptr<Node>, NodeHash, NodeEqual> useless_nodes;
    // define typedef for hash_map
    typedef boost::unordered_map<std::shared_ptr<Node>, list<std::shared_ptr<Node>>,
            NodeHash, NodeEqual> hashtable_t;
    hashtable_t allNodes_table;

    std::vector<std::vector<std::vector<double>>> heuristic_vec;
    std::shared_ptr<FailureCache> failure_cache_ptr;
    std::shared_ptr<SuccessCache> success_cache_ptr;
    double cutoff_time = 20.0;

public:
    // statistic
    double total_runtime_ = 0.0;
    double init_runtime = 0.0;
    double retrieve_runtime = 0.0;
    double motion_solver_runtime = 0.0;
    double expand_runtime = 0.0;

    size_t count_called = 0;
    size_t count_node_expanded = 0;
    size_t count_node_re_expand = 0;
    size_t count_node_generated = 0;
    size_t count_node_closed = 0;
    size_t count_potential_interval = 0;
    size_t count_motion_solver = 0;
    int hit_count_ = 0;
    int debug_rt_location_ = 0;

private:
    std::mutex mutex;
};