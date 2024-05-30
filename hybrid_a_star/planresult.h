/**
 * @file planresult.hpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief PlanResult header
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */
#pragma once

#include <vector>
#include "stdint.h"
#include <stddef.h>

namespace libMultiRobotPlanning {

/*! \brief Represents the path for an agent

    This class is used to store the result of a planner for a single agent.
    It has both the ordered list of states that need to be traversed as well as
   the ordered
    list of actions together with their respective costs

    \tparam State Custom state for the search. Needs to be copy'able
    \tparam Action Custom action for the search. Needs to be copy'able
    \tparam Cost Custom Cost type (integer or floating point types)
*/
template <typename State, typename Action, typename Cost>
struct PlanResult {
  //! states and their gScore
  std::vector<std::pair<State, Cost> > states;
  //! actions and their cost
  std::vector<std::pair<Action, Cost> > actions;
  //! actual cost of the result
  Cost cost;
  //! lower bound of the cost (for suboptimal solvers)
  Cost fmin;
  std::vector<uint64_t> times;

  inline size_t size() { return states.size();  }
  inline bool empty() {return states.empty(); }
};

template <typename State, typename Action, typename Cost>
struct PlanResultShort {
  //! states and their gScore
  std::vector<State> states;
  //! actions and their cost
  std::vector<Action>  actions;
  //! actual cost of the result
  Cost cost;
  //! lower bound of the cost (for suboptimal solvers)
  Cost fmin;
  std::vector<uint64_t> times;
};

}  // namespace libMultiRobotPlanning
