#ifndef HIERARCHICALMCTS_H
#define HIERARCHICALMCTS_H

#include "mcts.h"
#include "history.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <iostream>
#include <algorithm>
#include <stack>

typedef std::pair<int, int> option_t; // from -> to

namespace std {

template<typename a, typename b>
struct hash< std::pair<a, b> > {
private:
  const hash<a> ah;
  const hash<b> bh;
public:
  hash() : ah(), bh() {}
  size_t operator()(const std::pair<a, b> &p) const {
    size_t seed = ah(p.first);
    return bh(p.second) + 0x9e3779b9 + (seed<<6) + (seed>>2);
  }
};

} // namespaces

/**
 * @brief The HierarchicalMCTS class
 *
 * MCTS algorithm for MDP with state abstraction
 */
class HierarchicalMCTS : public MCTS {
public:
  struct input_t {
    input_t(std::size_t b, int o) : belief_hash(b), ending_observation(o) { }

    std::size_t belief_hash;
    int ending_observation;
  };

  struct result_t {
    result_t(double r, int s, bool t, std::size_t b, int o)
        : reward(r), steps(s), global_terminal(t), belief_hash(b),
          ending_observation(o) { }

    double reward;
    int steps;
    bool global_terminal; // global terminal state
    std::size_t belief_hash;
    int ending_observation;

    friend std::ostream &operator<<(std::ostream &os, const result_t &o) {
      return os << "{"
             << "reward=" << o.reward << ", "
             << "steps=" << o.steps << ", "
             << "terminal=" << o.global_terminal << ", "
             << "belief_hash=" << o.belief_hash << ", "
             << "last_observation=" << o.ending_observation << "}";
    }
  };

  struct data_t {
    struct {
      STATISTIC value;
      std::unordered_map<option_t, STATISTIC> qvalues;
    } V[2];
  };


public:
  HierarchicalMCTS(const SIMULATOR &simulator, const PARAMS &params,
                   bool action_abstraction, int first_observation, STATE *ground_state);

  virtual ~HierarchicalMCTS();

  virtual int SelectAction();

  virtual void SearchImp();

  virtual bool Update(int action, int observation, STATE &state);

  result_t SearchTree(option_t option, const input_t &input,
                      STATE *&state, int depth);

  result_t Simulate(option_t action, const input_t &input, STATE *&state,
                    int depth);

  result_t Rollout(option_t option, const input_t &input, STATE *&state,
                   int depth);

  result_t PollingRollout(option_t option, const input_t &input, STATE *&state,
                          int depth);

  result_t HierarchicalRollout(option_t option, const input_t &input, STATE *&state,
                               int depth);

  option_t GreedyUCB(option_t option, int last_observation,
                           data_t &data, bool ucb);

  option_t GreedyPrimitiveAction(option_t option, const input_t &input, STATE &state);
  option_t RandomPrimitiveAction(option_t option, const input_t &input);
  option_t SmartPrimitiveAction(option_t option, const input_t &input, STATE &state);
  option_t GetPrimitiveAction(option_t option, const input_t &input, STATE &state);

  option_t RandomSubtask(option_t option, const input_t &input);
  option_t SmartSubtask(option_t option, const input_t &input, STATE &state);
  option_t GetSubtask(option_t option, const input_t &input, STATE &state);

  option_t RandomOption(const input_t &input);

  bool IsTerminated(option_t option, int last_observation);
  bool IsGoal(option_t option, int last_observation);
  bool IsPrimitive(option_t option);

  double LocalReward(option_t option, int last_observation, int depth);

  option_t Option(int from, int to);
  option_t PrimitiveAction(int action);

  double GetExplorationConstant(option_t option);
  void AddOption(int from, int to, STATE *state);
  void ExploreOptions(int iterations, int max_depth);
  bool Applicable(int last_observation, option_t action);
  data_t *Query(option_t option, size_t belief_hash);
  data_t *Insert(option_t option, size_t belief_hash);
  void Clear();

  static void UnitTest();

public:
  enum {
    ROOT = -1,
    PRIMITIVE = -2,
    ABSTRACT_GOAL = '0'
  };

private:
  std::unordered_map<option_t, std::unordered_set<option_t>> mTaskGraph;
  std::unordered_map<option_t, STATE*> mExits;
  std::unordered_map<int, std::unordered_set<option_t>> mAvailableOptions;
  const option_t mRootTask; // root task
  std::stack<option_t> mCallStack;
  std::unordered_map<option_t, std::unordered_map<size_t, data_t *>> mTree;
  BELIEF_STATE mBelief;
  bool mActionAbstraction;
};

#endif // HIERARCHICALMCTS_H
