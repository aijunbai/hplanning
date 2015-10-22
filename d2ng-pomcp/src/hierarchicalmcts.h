#ifndef HIERARCHICALMCTS_H
#define HIERARCHICALMCTS_H

#include "metamcts.h"
#include "history.h"
#include <unordered_map>
#include <vector>
#include <iostream>
#include <algorithm>

typedef int MacroAction;

struct data_t {
  STATISTIC value_;
  std::unordered_map<MacroAction, STATISTIC> qvalues_;

  data_t() {}
  data_t(const STATISTIC &value, const std::unordered_map<MacroAction, STATISTIC> &qvalues)
      : value_(value), qvalues_(qvalues) {}
  data_t(const data_t &data) : value_(data.value_), qvalues_(data.qvalues_) {}

  const data_t &operator=(const data_t &o) {
    if (this != &o) {
      value_ = o.value_;
      qvalues_ = o.qvalues_;
    }
    return *this;
  }
};

inline std::size_t hash_value(const std::vector<MacroAction> &stack,
                              HISTORY &h) {
  using boost::hash_combine;
  std::size_t seed = 0;

  for (auto it = stack.begin(); it != stack.end(); ++it) {
    hash_combine(seed, std::hash<MacroAction>()(*it));
  }

  hash_combine(seed, h.BeliefHash());
  return seed;
}

/**
 * @brief The HierarchicalMCTS class
 *
 * MCTS algorithm for MDP with state abstraction
 */
class HierarchicalMCTS : public MetaMCTS {
public:
  HierarchicalMCTS(const SIMULATOR &simulator, const PARAMS &params);
  virtual ~HierarchicalMCTS();

  virtual int SelectAction();
  virtual void SearchImp();
  virtual bool Update(int action, int observation, STATE &state);

  double SearchTree(std::vector<MacroAction> stack, HISTORY &history,
                    STATE &state, int depth);
  bool Terminate(MacroAction A, HISTORY &h);
  MacroAction GreedyUCB(MacroAction Action, data_t &data, HISTORY &history, bool ucb);
  double Rollout(std::vector<MacroAction> stack, HISTORY &history, STATE &state,
                 int depth);
  int SelectPrimitiveAction(std::vector<MacroAction> stack, HISTORY &history);

private:
  std::unordered_map<MacroAction, std::vector<MacroAction>> mSubTasks;
  std::unordered_map<size_t, data_t> mTable;
  BELIEF_STATE mRootBelief;
};

#endif // HIERARCHICALMCTS_H
