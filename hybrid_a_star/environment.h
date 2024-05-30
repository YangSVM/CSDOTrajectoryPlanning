/**
 * @file environment.h
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief Environment class header
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once
#include <math.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <boost/functional/hash.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "hybrid_a_star/neighbor.h"
#include "hybrid_a_star/planresult.h"
#include "hybrid_a_star/motion_planning.h"
#include "hybrid_a_star/Instance.h"

namespace libMultiRobotPlanning {

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using namespace libMultiRobotPlanning;
typedef ompl::base::SE2StateSpace::StateType OmplState;

/**
 * @brief  Environment class
 *
 * @tparam Location
 * @tparam State
 * @tparam Action
 * @tparam Cost
 */
template <typename Location, typename State, typename Action, typename Cost >
class Environment {
 public:
  Environment(size_t maxx, size_t maxy, const std::unordered_set<Location>& obstacles,
              const std::multimap<int, State>& dynamic_obstacles,
              const std::vector<State>& goals_, int agent_id)
      : m_maxx(maxx), m_maxy(maxy), 
      m_obstacles(obstacles),
        // m_dynamic_obstacles(std::move(dynamic_obstacles)),
        m_agentIdx(0),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
        all_goals(goals_),
        agent_id(agent_id)
         {
    m_dimx = (int)maxx / Constants::mapResolution;
    m_dimy = (int)maxy / Constants::mapResolution;
    // std::cout << "env build " << m_dimx << " " << m_dimy << " "
    //           << m_obstacles.size() << std::endl;
     std::vector<State> goals(goals_.begin()+ agent_id, goals_.begin()+ agent_id+1);
    holonomic_cost_maps = std::vector<std::vector<std::vector<double>>>(
        goals.size(), std::vector<std::vector<double>>(
                          m_dimx, std::vector<double>(m_dimy, 0)));
    m_goals.clear();

    for (const auto &g : goals) {
      if (g.x < 0 || g.x > maxx || g.y < 0 || g.y > maxy) {
        std::cout << "\033[1m\033[31m Goal out of boundary, Fail to build "
                     "environment \033[0m\n";
        return;
      }
      m_goals.emplace_back(
          State(g.x, g.y, Constants::normalizeHeadingRad(g.yaw)));
    }
    updateCostmap();
  }

  Environment(const Environment &) = delete;
  Environment &operator=(const Environment &) = delete;

  bool resetDynamicObstacles(
    const std::set<int>& higher_agents, 
    const std::vector<PlanResult<State, Action, Cost>*>& paths){
    m_dynamic_obstacles.clear();
    for ( const int & a : higher_agents){
      for ( auto state : paths[a]->states){
          m_dynamic_obstacles.insert(
              std::pair<int, State>(
                  state.first.time,
                  State(state.first.x, state.first.y, state.first.yaw, state.first.time))
          );
      }
      State last_state = paths[a]->states.back().first;
      m_dynamic_obstacles.insert(
          std::pair<int, State>(
                  -last_state.time,
                  State(last_state.x, last_state.y, last_state.yaw))
      );
    }

    // do not go through the lower rank's goal
    int num_agents = all_goals.size();
    for ( int a = 0 ; a <num_agents; a++){
      if ( higher_agents.find(a) != higher_agents.end() || a == agent_id ) {
        continue;
      }

      m_dynamic_obstacles.insert(
        std::pair<int, State>(
                -1,
                State(all_goals[a].x, all_goals[a].y, all_goals[a].yaw))
      );
    }

    
    return true;
  }


  void onExpandHighLevelNode(int /*cost*/) {
    m_highLevelExpanded++;
    if (m_highLevelExpanded % 50 == 0)
      std::cout << "Now expand " << m_highLevelExpanded
                << " high level nodes.\n";
  }

  int highLevelExpanded() { return m_highLevelExpanded; }


  int admissibleHeuristic(const State &s) {
    // non-holonomic-without-obstacles heuristic: use a Reeds-Shepp
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    OmplState *rsStart = (OmplState *)reedsSheppPath.allocState();
    OmplState *rsEnd = (OmplState *)reedsSheppPath.allocState();
    rsStart->setXY(s.x, s.y);
    rsStart->setYaw(s.yaw);
    rsEnd->setXY(m_goals[m_agentIdx].x, m_goals[m_agentIdx].y);
    rsEnd->setYaw(m_goals[m_agentIdx].yaw);
    double reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    // std::cout << "ReedsShepps cost:" << reedsSheppCost << std::endl;
    // Euclidean distance
    double euclideanCost = sqrt(pow(m_goals[m_agentIdx].x - s.x, 2) +
                                pow(m_goals[m_agentIdx].y - s.y, 2));
    // std::cout << "Euclidean cost:" << euclideanCost << std::endl;
    // holonomic-with-obstacles heuristic
    double twoDoffset =
        sqrt(pow((s.x - (int)s.x) -
                     (m_goals[m_agentIdx].x - (int)m_goals[m_agentIdx].x),
                 2) +
             pow((s.y - (int)s.y) -
                     (m_goals[m_agentIdx].y - (int)m_goals[m_agentIdx].y),
                 2));
    double twoDCost =
        holonomic_cost_maps[m_agentIdx][(int)s.x / Constants::mapResolution]
                           [(int)s.y / Constants::mapResolution] -
        twoDoffset;
    // std::cout << "holonomic cost:" << twoDCost << std::endl;

    return std::max({reedsSheppCost, euclideanCost, twoDCost});
    return 0;
  }

  bool isInRange( const State& state){
    // 距离终点越近，概率越高
    int random = rand() % 10 + 1;   // random int [1,10]
    float dx = std::abs(state.x - getGoal().x) / random;
    float dy = std::abs(state.y - getGoal().y) / random;
    return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;
  }

  bool isSolution(
      const State& state, double gscore, State& path_end, 
      // unordered_map <key, value, hash_function>
      std::unordered_map<State, std::tuple<State, Action, double, double>,
                         std::hash<State>>& _camefrom) {
    double goal_distance =
        sqrt(pow(state.x - getGoal().x, 2) + pow(state.y - getGoal().y, 2));

    // std::cout << "  goal position: "<< getGoal()<<std::endl;
    // std::cout << "  now position: "<< state <<std::endl;

    // if (goal_distance > 2 * (Constants::LB + Constants::LF)) return false;
    if (!isInRange(state)) return false;
    ompl::base::ReedsSheppStateSpace reedsSheppSpace(Constants::r);
    OmplState* rsStart = (OmplState*)reedsSheppSpace.allocState();
    OmplState* rsEnd = (OmplState*)reedsSheppSpace.allocState();
    rsStart->setXY(state.x, state.y);
    rsStart->setYaw(state.yaw);
    rsEnd->setXY(getGoal().x, getGoal().y);
    rsEnd->setYaw(getGoal().yaw);
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath reedsShepppath =
        reedsSheppSpace.reedsShepp(rsStart, rsEnd);

    std::vector<State> path;
    std::unordered_map<State, std::tuple<State, Action, double, double>,
                       std::hash<State>>
        cameFrom;
    cameFrom.clear();
    path.emplace_back(state);
    for (auto pathidx = 0; pathidx < 5; pathidx++) {
      if (fabs(reedsShepppath.length_[pathidx]) < 1e-6) continue;
      double n_dyaw_act, deltaLength, act, cost;
      switch (reedsShepppath.type_[pathidx]) {
        case 0:  // RS_NOP
          continue;
          break;
        case 1:  // RS_LEFT
          n_dyaw_act = reedsShepppath.length_[pathidx]; //
          deltaLength = Constants::r * reedsShepppath.length_[pathidx];
          act = 2;
          cost = reedsShepppath.length_[pathidx] * Constants::r *
                 Constants::penaltyTurning;
          break;
        case 2:  // RS_STRAIGHT
          n_dyaw_act = 0;
          deltaLength = reedsShepppath.length_[pathidx] * Constants::r;
          act = 0;
          cost = deltaLength;
          break;
        case 3:  // RS_RIGHT
          n_dyaw_act = -reedsShepppath.length_[pathidx];
          deltaLength = Constants::r * reedsShepppath.length_[pathidx];
          act = 1;
          cost = reedsShepppath.length_[pathidx] * Constants::r *
                 Constants::penaltyTurning;
          break;
        default:
          std::cout << "\033[1m\033[31m"
                    << "Warning: Receive unknown ReedsSheppPath type"
                    << "\033[0m\n";
          break;
      }
      if (cost < 0) {
        cost = -cost * Constants::penaltyReversing;
        act = act + 3;
      }

      State s = path.back();
      std::vector<std::pair<State, double>> next_path;
      if (generatePath(s, act, n_dyaw_act, deltaLength, next_path)){
        // std::cout << "  generate path param: "<< s <<" "<<act<<" "<< deltaYaw <<std::endl;
        for (auto iter = next_path.begin(); iter != next_path.end(); iter++) {
          State next_s = iter->first;
          if (!stateValid(next_s))
            return false;   // collision. cannot fast-forward
          else {
            gscore += iter->second;
            if (!(next_s == path.back())) {
              cameFrom.insert(std::make_pair<>(
                  next_s,
                  std::make_tuple<>(path.back(), act, iter->second, gscore)));
            }
            path.emplace_back(next_s);
          }
        }
      }
      else{
        return false;
      }

    }
    // m_goals[m_agentIdx] = path.back();
    path_end = path.back();

    _camefrom.insert(cameFrom.begin(), cameFrom.end());

    std::cout<< "path: "<<path.back()<<std::endl;

    return true;
  }

  void getNeighbors(const State &s, Action action,
                    std::vector<Neighbor<State, Action, double>> &neighbors) {
    neighbors.clear();
    double g = Constants::dx[0];
    for (Action act = 0; act < 6; act++) {  // has 6 directions for Reeds-Shepp
      double xSucc, ySucc, yawSucc;
      g = Constants::dx[0];
      xSucc = s.x + Constants::dx[act] * cos(s.yaw) -
              Constants::dy[act] * sin(s.yaw);
      ySucc = s.y + Constants::dx[act] * sin(s.yaw) +
              Constants::dy[act] * cos(s.yaw);
      yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
      // if (act != action) {  // penalize turning
      //   g = g * Constants::penaltyTurning;
      //   if (act >= 3)  // penalize change of direction
      //     g = g * Constants::penaltyCOD;
      // }
      // if (act > 3) {  // backwards
      //   g = g * Constants::penaltyReversing;
      // }
      if (act % 3 != 0) {  // penalize turning
        g = g * Constants::penaltyTurning;
      }
      if ((act < 3 && action >= 3) || (action < 3 && act >= 3)) {
        // penalize change of direction
        g = g * Constants::penaltyCOD;
      }
      if (act >= 3) {  // backwards
        g = g * Constants::penaltyReversing;
      }
      State tempState(xSucc, ySucc, yawSucc, s.time + 1);
      
      if (stateValid(tempState)) {
        neighbors.emplace_back(
            Neighbor<State, Action, double>(tempState, act, g));
      }
    }
    // wait
    g = Constants::dx[0];
    State tempState(s.x, s.y, s.yaw, s.time + 1);
    if (stateValid(tempState)) {
      neighbors.emplace_back(Neighbor<State, Action, double>(tempState, 6, g));
    }

  }

  State getGoal() { return m_goals[m_agentIdx]; }
  State getPathEnd() { return m_goals[m_agentIdx]; }
  
  uint64_t calcIndex(const State &s) {
    return (uint64_t)s.time * (2 * M_PI / Constants::deltat) *
          (m_dimx / Constants::xyResolution) *
          (m_dimy / Constants::xyResolution) +
      (uint64_t)(Constants::normalizeHeadingRad(s.yaw) /
                Constants::yawResolution) *
          (m_dimx / Constants::xyResolution) *
          (m_dimy / Constants::xyResolution) +
      (uint64_t)(s.y / Constants::xyResolution) *
          (m_dimx / Constants::xyResolution) +
      (uint64_t)(s.x / Constants::xyResolution);
  }

  void onExpandLowLevelNode(const State & /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

  bool startAndGoalValid(const std::vector<State> &m_starts, const size_t iter,
                         const int batchsize) {
    assert(m_goals.size() == m_starts.size());
    for (size_t i = 0; i < m_goals.size(); i++)
      for (size_t j = i + 1; j < m_goals.size(); j++) {
        if (m_goals[i].agentCollision(m_goals[j])) {
          std::cout << "ERROR: Goal point of " << i + iter * batchsize << " & "
                    << j + iter * batchsize << " collide!\n";
          return false;
        }
        if (m_starts[i].agentCollision(m_starts[j])) {
          std::cout << "ERROR: Start point of " << i + iter * batchsize << " & "
                    << j + iter * batchsize << " collide!\n";
          return false;
        }
      }
    return true;
  }

 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, double>> &solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State &s) {
    double x_ind = s.x / Constants::mapResolution;
    double y_ind = s.y / Constants::mapResolution;
    if (x_ind < 0 || x_ind >= m_dimx || y_ind < 0 || y_ind >= m_dimy)
      return false;
    
    // TODO more precision out of map detection.
    // will loss the success rate greatly.
    double rv = Constants::rv;
    double xf, yf, xr,yr;
    s.GetDiscCenter(xf,yf,xr,yr);
    if ( 
     xf < rv || xr < rv || xf > m_maxx - rv || xr > m_maxx - rv ||
     yf < rv || yr < rv || yf > m_maxy - rv || yr > m_maxy - rv
    )
     return false;

    for (const auto& obs: m_obstacles) {
      if (s.obsCollision(obs)) return false;
    }

    auto it = m_dynamic_obstacles.equal_range(s.time);
    for (auto itr = it.first; itr != it.second; ++itr) {
      if (s.agentCollision(itr->second)) return false;
    }

    it = m_dynamic_obstacles.equal_range(s.time+1);
    for (auto itr = it.first; itr != it.second; ++itr) {
      if (s.agentCollision(itr->second)) return false;
    }
    it = m_dynamic_obstacles.equal_range(s.time-1);
    for (auto itr = it.first; itr != it.second; ++itr) {
      if (s.agentCollision(itr->second)) return false;
    }

    auto itlow = m_dynamic_obstacles.lower_bound(-s.time);
    auto itup = m_dynamic_obstacles.upper_bound(-1);
    for (auto it = itlow; it != itup; ++it)
      if (s.agentCollision(it->second)) return false;


    return true;
  }

 private:
  struct compare_node {
    bool operator()(const std::pair<State, double> &n1,
                    const std::pair<State, double> &n2) const {
      return (n1.second > n2.second);
    }
  };
  void updateCostmap() {
    boost::heap::fibonacci_heap<std::pair<State, double>,
                                boost::heap::compare<compare_node>>
        heap;

    std::set<std::pair<int, int>> temp_obs_set;
    for (const auto & obs : m_obstacles) {
      temp_obs_set.insert(
          std::make_pair((int)obs.x / Constants::mapResolution,
                         (int)obs.y / Constants::mapResolution));
    }

    for (size_t idx = 0; idx < m_goals.size(); idx++) {
      heap.clear();
      int goal_x = (int)m_goals[idx].x / Constants::mapResolution;
      int goal_y = (int)m_goals[idx].y / Constants::mapResolution;
      heap.push(std::make_pair(State(goal_x, goal_y, 0), 0));

      while (!heap.empty()) {
        std::pair<State, double> node = heap.top();
        heap.pop();

        int x = node.first.x;
        int y = node.first.y;
        for (int dx = -1; dx <= 1; dx++)
          for (int dy = -1; dy <= 1; dy++) {
            if (dx == 0 && dy == 0) continue;
            int new_x = x + dx;
            int new_y = y + dy;
            if (new_x == goal_x && new_y == goal_y) continue;
            if (new_x >= 0 && new_x < m_dimx && new_y >= 0 && new_y < m_dimy &&
                holonomic_cost_maps[idx][new_x][new_y] == 0 &&
                temp_obs_set.find(std::make_pair(new_x, new_y)) ==
                    temp_obs_set.end()) {
              holonomic_cost_maps[idx][new_x][new_y] =
                  holonomic_cost_maps[idx][x][y] +
                  sqrt(pow(dx * Constants::mapResolution, 2) +
                       pow(dy * Constants::mapResolution, 2));
              heap.push(std::make_pair(State(new_x, new_y, 0),
                                       holonomic_cost_maps[idx][new_x][new_y]));
            }
          }
      }
    }

    // for (size_t idx = 0; idx < m_goals.size(); idx++) {
    //   std::cout << "---------Cost Map -------Agent: " << idx
    //             << "------------\n";
    //   for (size_t i = 0; i < m_dimx; i++) {
    //     for (size_t j = 0; j < m_dimy; j++)
    //       std::cout << holonomic_cost_maps[idx][i][j] << "\t";
    //     std::cout << std::endl;
    //   }
    // }
  }

  bool generatePath(State startState, int act, double n_dyaw_act,
                    double deltaLength,
                    std::vector<std::pair<State, double>> &result) {
    // std::cout << "enter generate path:\n";
    // std::cout << "  start state: "<< startState << std::endl;
    // std::cout << "  act: "<< act << std::endl;
    // std::cout << "  deltaYaw: "<< deltaYaw << std::endl;
    // std::cout << "  deltaLength: "<< deltaLength << std::endl;

    double xSucc, ySucc, yawSucc, dx, dy, dyaw, ratio;
    result.emplace_back(std::make_pair<>(startState, 0));
    if (act == 0 || act == 3) {
      for (size_t i = 0; i < (size_t)(deltaLength / Constants::dx[act]); i++) {
        State s = result.back().first;
        xSucc = s.x + Constants::dx[act] * cos(s.yaw) -
                Constants::dy[act] * sin(s.yaw);
        ySucc = s.y + Constants::dx[act] * sin(s.yaw) +
                Constants::dy[act] * cos(s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
        State nextState(xSucc, ySucc, yawSucc, result.back().first.time + 1);
        if (!stateValid(nextState)) return false;
        result.emplace_back(std::make_pair<>(nextState, Constants::dx[0]));
      }
      ratio = (deltaLength -
               (int)(deltaLength / Constants::dx[act]) * Constants::dx[act]) /
              Constants::dx[act];
      dyaw = 0;
      dx = ratio * Constants::dx[act];
      dy = 0;
    } else {
      for (size_t i = 0; i < (size_t)(n_dyaw_act / Constants::dyaw[act]); i++) {
        State s = result.back().first;
        xSucc = s.x + Constants::dx[act] * cos(s.yaw) -
                Constants::dy[act] * sin(s.yaw);
        ySucc = s.y + Constants::dx[act] * sin(s.yaw) +
                Constants::dy[act] * cos(s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
        State nextState(xSucc, ySucc, yawSucc, result.back().first.time + 1);
        if (!stateValid(nextState)) return false;
        result.emplace_back(std::make_pair<>(
            nextState, Constants::dx[0] * Constants::penaltyTurning));
      }
      ratio =
          (n_dyaw_act -
           (int)(n_dyaw_act / Constants::dyaw[act]) * Constants::dyaw[act]) /
          Constants::dyaw[act];
      dyaw = ratio * Constants::dyaw[act];
      dx = ratio * Constants::dx[act];
      dy = ratio*Constants::dy[act];

    }
    State s = result.back().first;
    xSucc = s.x + dx * cos(s.yaw) - dy * sin(s.yaw);
    ySucc = s.y + dx * sin(s.yaw) + dy * cos(s.yaw);
    yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw);
    // std::cout << m_agentIdx << " ratio::" << ratio << std::endl;
    State nextState(xSucc, ySucc, yawSucc, result.back().first.time + 1);
    if (!stateValid(nextState)) return false;
    result.emplace_back(std::make_pair<>(nextState, ratio * Constants::dx[0]));

    return true;
  }

 private:
  int m_dimx;
  int m_dimy;
  double m_maxx,  m_maxy;
  std::vector<std::vector<std::vector<double>>> holonomic_cost_maps;
  const std::unordered_set<Location>& m_obstacles;
  std::multimap<int, State> m_dynamic_obstacles;
  std::vector<State> m_goals;
  // std::vector< std::vector<int> > m_heuristic;
  std::vector<double> m_vel_limit;
  size_t m_agentIdx;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;

  const std::vector<State>& all_goals;
  int agent_id;
};

}  // namespace libMultiRobotPlanning
