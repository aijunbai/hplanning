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

typedef std::pair<int, int> macro_action_t;

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
    input_t(std::size_t b, int o) : belief_hash(b), last_observation(o) { }

    std::size_t belief_hash;
    int last_observation;
  };

  struct result_t {
    result_t(double r, int s, bool t, std::size_t b, int o)
        : reward(r), steps(s), global_terminal(t), belief_hash(b),
          last_observation(o) { }

    double reward;
    int steps;
    bool global_terminal; // global terminal state
    std::size_t belief_hash;
    int last_observation;

    friend std::ostream &operator<<(std::ostream &os, const result_t &o) {
      return os << "{"
             << "reward=" << o.reward << ", "
             << "steps=" << o.steps << ", "
             << "terminal=" << o.global_terminal << ", "
             << "belief_hash=" << o.belief_hash << ", "
             << "last_observation=" << o.last_observation << "}";
    }
  };

  struct bound_t {
    bound_t() : lower(-Infinity), upper(Infinity) { }

    bound_t(double l, double u) : lower(l), upper(u) { }

    double lower;
    double upper;

    void set(double l, double u) {
      lower = l;
      upper = u;
    }

    double width() const { return upper - lower; }

    double sample() const { return SimpleRNG::ins().GetUniform(lower, upper); }

    friend std::ostream &operator<<(std::ostream &os, const bound_t &o) {
      return os << "{"
             << "lower=" << o.lower << ", upper=" << o.upper
             << ", width=" << o.width() << "}";
    }
  };

  struct belief_t {
    belief_t() : size(0) { }

    int size = 0;
    std::unordered_map<std::size_t, std::pair<STATE *, int>> samples;

    void add_sample(const STATE &state, const SIMULATOR &simulator) {
      size += 1;
      std::size_t hash = state.hash();
      if (samples.count(state.hash())) {
        samples[hash].second += 1;
      } else {
        STATE *sample = simulator.Copy(state);
        samples[hash] = std::make_pair(sample, 1);
      }
    }

    void clear(const SIMULATOR &simulator) {
      for (auto &e : samples) {
        simulator.FreeState(e.second.first);
      }
    }

    STATE *sample(const SIMULATOR &simulator) {
      int i = SimpleRNG::ins().Random(size);

      for (auto &e : samples) {
        if (i < e.second.second) {
          STATE *sample = simulator.Copy(*e.second.first);
          return sample;
        } else {
          i -= e.second.second;
        }
      }

      assert(0);
      return 0;
    }
  };

  struct data_t {
    struct {
      STATISTIC value;
      std::unordered_map<macro_action_t, STATISTIC> qvalues;
    } V[2];
//    std::vector<result_t> cache;

//    bound_t ucb_bound(macro_action_t a, const MCTS *mcts);
//
//    bool optimal_prob_at_least(macro_action_t a, const MCTS *mcts, int N, double threshold);
//
//    static double greater_prob(double x1, double x2, double y1, double y2);

//    static void clear(const SIMULATOR &simulator);

//    static std::unordered_map<std::size_t, belief_t> beliefpool;
  };



public:
  HierarchicalMCTS(const SIMULATOR &simulator, const PARAMS &params);

  virtual ~HierarchicalMCTS();

  virtual int SelectAction();

  virtual void SearchImp();

  virtual bool Update(int action, int observation, STATE &state);

  result_t SearchTree(macro_action_t Action, const input_t &input,
                      STATE *&state, int depth);

  result_t Simulate(macro_action_t action, const input_t &input, STATE *&state,
                    int depth);

  result_t Rollout(macro_action_t Action, const input_t &input, STATE *&state,
                   int depth);

  result_t PollingRollout(macro_action_t Action, const input_t &input, STATE *&state,
                          int depth);

  result_t HierarchicalRollout(macro_action_t Action, const input_t &input, STATE *&state,
                               int depth);

  macro_action_t GreedyUCB(macro_action_t Action, int last_observation,
                           data_t &data, bool ucb);

  macro_action_t GreedyPrimitiveAction(macro_action_t Action, const input_t &input);

  macro_action_t RandomPrimitiveAction(macro_action_t Action, const input_t &input);

  bool IsTerminated(macro_action_t Action, int last_observation);

  bool IsGoal(macro_action_t Action, int last_observation);

  bool IsPrimitive(macro_action_t Action);

  double LocalReward(macro_action_t Action, int last_observation, int depth);

  macro_action_t MacroAction(int from, int to);

  macro_action_t PrimitiveAction(int action);

  double GetExplorationConstant(macro_action_t Action);

  void UpdateConnection(int from, int to);

  bool Applicable(int last_observation, macro_action_t action);

  data_t *Query(macro_action_t Action, size_t belief_hash);

  data_t *Insert(macro_action_t Action, size_t belief_hash);

  void Clear();

  static void UnitTest();

private:
  enum {
    ROOT = -1,
    PRIMITIVE = -2
  };

  std::unordered_map<macro_action_t, std::vector<macro_action_t>> mSubTasks;
  std::unordered_map<int, std::unordered_set<macro_action_t>> mAvailableActions;
  const macro_action_t mRootTask; // root task
  std::stack<macro_action_t> mCallStack;
  std::unordered_map<macro_action_t, std::unordered_map<size_t, data_t *>> mTree;
  BELIEF_STATE mRootSampling;

//  static STATISTIC mCacheRate;
//  static STATISTIC mCacheDepth;
//  static STATISTIC mCacheStep;
};

#endif // HIERARCHICALMCTS_H
