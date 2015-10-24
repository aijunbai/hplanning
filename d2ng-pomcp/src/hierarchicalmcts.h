#ifndef HIERARCHICALMCTS_H
#define HIERARCHICALMCTS_H

#include "mcts.h"
#include "history.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <iostream>
#include <algorithm>

typedef int macro_action_t;

inline std::size_t hash_value(macro_action_t Action, const HISTORY &history) {
  using boost::hash_combine;
  std::size_t seed = 0;

  hash_combine(seed, boost::hash_value(Action));
  hash_combine(seed, history.BeliefHash());
  return seed;
}

/**
 * @brief The HierarchicalMCTS class
 *
 * MCTS algorithm for MDP with state abstraction
 */
class HierarchicalMCTS : public MCTS {
public:
  struct data_t {
    struct {
      STATISTIC value_;
      std::unordered_map<macro_action_t, STATISTIC> qvalues_;
    } UCB;

    data_t() {}
    data_t(const STATISTIC &value, const std::unordered_map<macro_action_t, STATISTIC> &qvalues)
    {
      UCB.value_ = value;
      UCB.qvalues_ = qvalues;
    }
    data_t(const data_t &data)
    {
      UCB.value_ = data.UCB.value_;
      UCB.qvalues_ = data.UCB.qvalues_;
    }

    const data_t &operator=(const data_t &o) {
      if (this != &o) {
        UCB.value_ = o.UCB.value_;
        UCB.qvalues_ = o.UCB.qvalues_;
      }
      return *this;
    }
  };

  struct result_t {
    result_t(double r, int s, bool t): reward(r), steps(s), terminal(t) {}

    double reward;
    int steps;
    bool terminal;  // global terminal state
  };

public:
  HierarchicalMCTS(const SIMULATOR &simulator, const PARAMS &params);
  virtual ~HierarchicalMCTS();

  virtual int SelectAction();
  virtual void SearchImp();
  virtual bool Update(int action, int observation, STATE &state);

  result_t SearchTree(macro_action_t Action, HISTORY &history, STATE &state, int depth);
  result_t Rollout(macro_action_t Action, HISTORY &history, STATE &state, int depth);
  macro_action_t GreedyUCB(macro_action_t Action, HISTORY &history, data_t &data, bool ucb);
  int SelectPrimitiveAction(macro_action_t Action, HISTORY &history);
  bool Terminate(macro_action_t Action, HISTORY &history);
  bool Primitive(macro_action_t Action);
  macro_action_t MacroAction(int o);

private:
  std::unordered_map<macro_action_t, std::vector<macro_action_t>> mSubTasks;
  std::unordered_map<macro_action_t, std::unordered_set<int>> mGoals;  // target observation for subtasks
  macro_action_t mRootTask;  // current root task
  std::unordered_map<size_t, data_t> mTable;
  BELIEF_STATE mRootBelief;
};

#endif // HIERARCHICALMCTS_H
