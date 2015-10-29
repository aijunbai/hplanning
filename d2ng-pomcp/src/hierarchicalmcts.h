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

/**
 * @brief The HierarchicalMCTS class
 *
 * MCTS algorithm for MDP with state abstraction
 */
class HierarchicalMCTS : public MCTS {
public:
  struct input_t {
    input_t(std::size_t b, int o):
      belief_hash(b), last_observation(o) {}
    std::size_t belief_hash;
    int last_observation;
  };

  struct result_t {
    result_t(double r, int s, bool t, std::size_t b, int o):
      reward(r), steps(s), terminal(t),
      belief_hash(b), last_observation(o) {}

    double reward;
    int steps;
    bool terminal;  // global terminal state
    std::size_t belief_hash;
    int last_observation;

    friend std::ostream &operator <<(std::ostream &os, const result_t &o) {
      return os << "{"
                << "reward=" << o.reward << ", "
                << "steps=" << o.steps << ", "
                << "terminal=" << o.terminal << ", "
                << "belief_hash=" << o.belief_hash << ", "
                << "last_observation=" << o.last_observation << "}";
    }
  };

  struct bound_t {
    bound_t(double l, double u): lower(l), upper(u) {}

    double lower;
    double upper;

    double range() const {
      return upper - lower;
    }

    friend std::ostream &operator <<(std::ostream &os, const bound_t &o) {
      return os << "{"
                << "lower=" << o.lower
                << ", upper=" << o.upper
                << ", range=" << o.range()
                << "}";
    }
  };

  struct belief_t {
    int size = 0;
    std::unordered_map<std::size_t, std::pair<STATE*, int>> samples;

    void add_sample(const STATE &state, const SIMULATOR &simulator) {
      size += 1;
      std::size_t hash = state.hash();
      if (samples.count(state.hash())) {
        samples[hash].second += 1;
      }
      else {
        STATE *sample = simulator.Copy(state);
        samples[hash] = std::make_pair(sample, 1);
      }
    }

    void clear(const SIMULATOR &simulator) {
      for (auto &e: samples) {
        simulator.FreeState(e.second.first);
      }
    }

    STATE *sample(const SIMULATOR &simulator) {
      int i = SimpleRNG::ins().Random(size);

      for (auto &e: samples) {
        if (i < e.second.second) {
          STATE *sample = simulator.Copy(*e.second.first);
          return sample;
        }
        else {
          i -= e.second.second;
        }
      }

      assert(0);
      return 0;
    }
  };

  struct data_t {
    STATISTIC value;
    std::unordered_map<macro_action_t, STATISTIC> qvalues;
    std::vector<result_t> cache;

    bound_t bound(macro_action_t a, const MCTS *mcts);
    static void clear(const SIMULATOR &simulator);
    static std::unordered_map<std::size_t, belief_t> beliefpool;
  };

public:
  HierarchicalMCTS(const SIMULATOR &simulator, const PARAMS &params);
  virtual ~HierarchicalMCTS();

  virtual int SelectAction();
  virtual void SearchImp();
  virtual bool Update(int action, int observation, STATE &state);

  result_t SearchTree(macro_action_t Action, const input_t &input, STATE *&state, int depth);
  result_t Rollout(macro_action_t Action, const input_t &input, STATE *&state, int depth);
  macro_action_t GreedyUCB(macro_action_t Action, int last_observation, data_t &data, bool ucb);
  int SelectPrimitiveAction(macro_action_t Action, const HISTORY &history);
  bool Terminate(macro_action_t Action, int last_observation);
  bool Primitive(macro_action_t Action);
  macro_action_t MacroAction(int o);
  void UpdateConnection(int last_observation, int observation);
  bool Applicable(int last_observation, macro_action_t action);
  data_t *Query(macro_action_t Action, size_t belief_hash);
  void Clear();

private:
  std::unordered_map<macro_action_t, std::vector<macro_action_t>> mSubTasks;
  std::unordered_map<macro_action_t, std::unordered_set<int>> mGoals;  // target observation for subtasks
  std::unordered_map<int, std::unordered_map<macro_action_t, bool>> mApplicable;
  const macro_action_t mRootTask;  // root task
  std::unordered_map<macro_action_t, std::unordered_map<size_t, data_t*>> mTree;
  BELIEF_STATE mRootSampling;
  double mConvergedBound;

  static STATISTIC mCacheRate;
};

#endif // HIERARCHICALMCTS_H
