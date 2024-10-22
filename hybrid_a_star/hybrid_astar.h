/**
 * @file hybrid_astar_ig.hpp
 * @author yibin (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

#define USE_FIBONACCI_HEAP
#include <boost/heap/d_ary_heap.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/program_options.hpp>

#include "hybrid_a_star/neighbor.h"
#include "hybrid_a_star/planresult.h"
#include "common/motion_planning.h"

#include <yaml-cpp/yaml.h>
#include <fstream>

namespace libMultiRobotPlanning {
/*!
\example sh_astar.cpp Implementation of Spatiotemporal Hybrid-State A*
algorithm.
*/

/*! \brief Spatiotemporal Hybrid A* Algorithm.

This class implements the Spatiotemporal Hybrid-State A* algorithm. **Noted
that it could also used as traditional Hybrid-State Astar using correct
implementation**

This class can either use a fibonacci heap, or a d-ary heap. The former is the
default. Comment "USE_FIBONACCI_HEAP" to use the d-ary heap instead.

\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Environment This class needs to provide the custom hybrid A* logic. In
    particular, it needs to support the following functions:
  - `Cost admissibleHeuristic(const State& s)`\n
    This function can return 0 if no suitable heuristic is available.

  - `bool isSolution(const State& s, Cost gscore,
      std::unordered_map<State, std::tuple<State, Action, Cost, Cost>`,
                        StateHasher>& cameFrom)\n
     Return true if the given state is close to goal state and it has a
    collision-free path to goal. It is also needed to insert the generate
    path-to-goal into the current cameFrom map.

  - `State getGoal()`\n
    Return goal state of current agent.

  - `void getNeighbors(const State& s, std::vector<Neighbor<State, Action,
   Cost> >& neighbors)`\n
    Fill the list of neighboring state for the given state s.

  - `int calcIndex(const State& s)`\n
    This function calculate a index(int) for a given state s, the index uses as
    a key in closed list.

  - `void onExpandNode(const State& s, Cost fScore, Cost gScore)`\n
    This function is called on every expansion and can be used for statistical
    purposes.

  - `void onDiscover(const State& s, Cost fScore, Cost gScore)`\n
    This function is called on every node discovery and can be used for
   statistical purposes.

\tparam StateHasher A class to convert a state to a hash value. Default:
   std::hash<State>
*/

// there is no post-process as on the original paper.
template <typename State, typename Action, typename Cost, typename Environment,
          typename StateHasher = std::hash<State>>
class HybridAStar {
 public:
  HybridAStar(Environment& environment) : m_env(environment) {}
  ~HybridAStar() {}

  bool search(const State& startState,
              PlanResult<State, Action, Cost>& solution, Cost initialCost = 0) {
    solution.states.clear();
    solution.actions.clear();
    solution.cost = 0;
    int t_start = startState.time;

    openSet_t openSet;  // heap. priority queue of Node
    // map node index to open_set handle.
    std::unordered_map<uint64_t, heapHandle_t, std::hash<uint64_t>> stateToHeap;
    std::unordered_set<uint64_t, std::hash<uint64_t>> closedSet; // node index.
    std::unordered_map<State, std::tuple<State, Action, Cost, Cost>,
                       StateHasher>
        cameFrom;         // store the parent node in the unorderd  map

    auto handle =
        openSet.push(Node(startState, Action(),
                          m_env.admissibleHeuristic(startState), initialCost));
    stateToHeap.insert(std::make_pair<>(m_env.calcIndex(startState), handle));
    (*handle).handle = handle;

    std::vector<Neighbor<State, Action, Cost>> neighbors;
    neighbors.reserve(10);

    while (!openSet.empty()) {
          // std::cout << "low debug 1.1.1 enter the search loop.\n";
      if (closedSet.size()>Constants::maxClosedSetSize){
        std::cout<<"\033[1m\033[31mClosed set max size reached. \
          Hybrid A* ig failed to find a solution for single agent\033[0m\n";
        std::cout << "If you want to use it in large map. \
          Please larger the closed set maximum size.\n";
        return false;
      }


      Node current = openSet.top();
      if ( current.state.time - startState.time > 1e3 ){
        std::cout << "\033[1m\033[31m Hybrid A* max time reached. "
          <<"Maybe there is static invalid. \033[0m \n" ;
        return false;
      }
      // m_env.onExpandNode(current.state, current.fScore, current.gScore);
      State path_end(0,0,0);

      // ReedsShepp forward to test if it is a solution. 
      // path_end is near the goal but not the same.
      if (m_env.isSolution(current.state, current.gScore, path_end, cameFrom, t_start, T_plan)) {
        solution.states.clear();
        solution.actions.clear();

        auto iter = cameFrom.find(path_end);
        solution.cost = std::get<3>(iter->second);
        solution.fmin =
            std::get<3>(iter->second) +
            m_env.admissibleHeuristic(iter->first);  // current.fScore;
        while (iter != cameFrom.end()) {
          solution.states.push_back(
              std::make_pair<>(iter->first, std::get<3>(iter->second)));
          solution.actions.push_back(std::make_pair<>(
              std::get<1>(iter->second), std::get<2>(iter->second)));
          iter = cameFrom.find(std::get<0>(iter->second));
        }
        solution.states.push_back(std::make_pair<>(startState, initialCost));
        std::reverse(solution.states.begin(), solution.states.end());
        std::reverse(solution.actions.begin(), solution.actions.end());

        openSet.clear();
        return true;
      }

      openSet.pop();
      stateToHeap.erase(m_env.calcIndex(current.state));
      closedSet.insert(m_env.calcIndex(current.state));
      // traverse neighbors
      neighbors.clear();
      m_env.getNeighbors(current.state, current.action, neighbors);
      for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {
        // 1. do nothing if it is closed.
        if (closedSet.find(m_env.calcIndex(neighbor.state)) == closedSet.end()) { 
          Cost tentative_gScore = current.gScore + neighbor.cost;
          auto iter = stateToHeap.find(m_env.calcIndex(neighbor.state));
          if (iter == stateToHeap.end()) {  // 2. Discover a new node(x,y,yaw,t).
            Cost fScore =
                tentative_gScore + m_env.admissibleHeuristic(neighbor.state);
            auto handle = openSet.push(Node(neighbor.state, neighbor.action,
                                            fScore, tentative_gScore));
            (*handle).handle = handle;
            stateToHeap.insert(
                std::make_pair<>(m_env.calcIndex(neighbor.state), handle));
            // m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
            // std::cout << "  this is a new node " << fScore << "," <<
            // tentative_gScore << std::endl;
          } 
          else { // find an exsiting node in open_set.
            auto handle = iter->second;

            // 3. We found this node before with a better path
            if (tentative_gScore >= (*handle).gScore) {
              continue;
            }

            // update f and gScore
            Cost delta = (*handle).gScore - tentative_gScore;
            (*handle).gScore = tentative_gScore;
            (*handle).fScore -= delta;
            (*handle).state = neighbor.state;
            openSet.increase(handle);
            // m_env.onDiscover(neighbor.state, (*handle).fScore,
            //                  (*handle).gScore);
          }

          // Best path for this node so far
          cameFrom.erase(neighbor.state);  // erase first or it won't update.
          cameFrom.insert(std::make_pair<>(
              neighbor.state,
              std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                tentative_gScore)));
        }
      }
    }
    openSet.clear();
    return false;
  }

  bool resetDynamicObstacles(
    const std::set<int>& higher_agents, 
    const std::vector<PlanResult<State, Action, Cost>*>& paths){
    m_env.resetDynamicObstacles(higher_agents, paths);
    return true;
  }

  bool resetPartTimeDynamicObstacles(
    const std::set<int>& higher_agents, 
    const std::vector<PlanResult<State, Action, Cost>*>& paths, int T_plan){
    m_env.resetPartTimeDynamicObstacles(higher_agents, paths, T_plan);
    this->T_plan = T_plan;
    return true;
  }

  bool detectRunover( const State& start, const State& s){
    return m_env.detectRunover(start, s);
  }

 private:
 int T_plan;
  struct Node {
    Node(const State& state, Action action, Cost fScore, Cost gScore)
        : state(state), action(action), fScore(fScore), gScore(gScore) {}

    bool operator<(const Node& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if (fScore != other.fScore) {
        return fScore > other.fScore;
      } else {
        return gScore < other.gScore;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state << " fScore: " << node.fScore
         << " gScore: " << node.gScore;
      return os;
    }

    State state;
    Action action;
    Cost fScore;
    Cost gScore;

#ifdef USE_FIBONACCI_HEAP
    typename boost::heap::fibonacci_heap<Node>::handle_type handle;
#else
    typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
        handle;
#endif
  };

#ifdef USE_FIBONACCI_HEAP
  typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
  typedef typename openSet_t::handle_type heapHandle_t;
#else
  typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                           boost::heap::mutable_<true>>
      openSet_t;
  typedef typename openSet_t::handle_type heapHandle_t;
#endif

 private:
  Environment& m_env;
};

}  // namespace libMultiRobotPlanning