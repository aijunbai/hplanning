#include "hierarchicalmcts.h"
#include "utils.h"
#include "coord.h"
#include <sstream>

using namespace std;

HierarchicalMCTS::HierarchicalMCTS(const SIMULATOR &simulator,
                                   const PARAMS &params)
    : MCTS(simulator, params), mRootTask(-1)
{
  mSubTasks[mRootTask] = vector<macro_action_t>();  // root action

  for (int a = 0; a < Simulator.GetNumActions(); ++a) {
    mSubTasks[a] = vector<macro_action_t>();  // primitive actions
  }

  if (Simulator.mActionAbstraction) {
    assert(Simulator.GetNumObservations() > 0);
    mGoals[mRootTask].insert(0);  // ground target state assumed to be in macro state 0 for rooms domain

    for (int o = 0; o < Simulator.GetNumObservations(); ++o) {
      mSubTasks[MacroAction(o)] = vector<macro_action_t>();
      mGoals[MacroAction(o)].insert(o);

      for (int a = 0; a < Simulator.GetNumActions(); ++a) {
        mSubTasks[MacroAction(o)].push_back(a);
      }
    }

    for (int o = 0; o < Simulator.GetNumObservations(); ++o) {
      mSubTasks[mRootTask].push_back(MacroAction(o));
    }
  } else {
    for (int a = 0; a < Simulator.GetNumActions(); ++a) {
      mSubTasks[mRootTask].push_back(a);
    }
  }

  for (int i = 0; i < Params.NumStartStates; i++) {
    mRootSampling.AddSample(Simulator.CreateStartState());
  }

  if (Simulator.mActionAbstraction) {
    int iterations = 1000, max_depth = 1000;

    for (int i = 0; i < iterations; ++i) {
      HISTORY history;
      STATE *state = mRootSampling.CreateSample(Simulator);
      Simulator.Validate(*state);
      bool terminal = false;
      int step = 0;

      for (; !terminal && step < max_depth; ++step) {
        int observation;
        double reward;
        int action = SimpleRNG::ins().Random(Simulator.GetNumActions());
        terminal = Simulator.Step(*state, action, observation, reward);
        UpdateConnection(history.LastObservation(), observation);
        history.Add(action, observation, reward, Params.MemorySize);
      }

      Simulator.FreeState(state);
    }
    if (Params.Verbose >= 2) {
      PRINT_VALUE(mApplicable);
    }
  }
}

HierarchicalMCTS::~HierarchicalMCTS() {  Clear(); }

bool HierarchicalMCTS::Applicable(int last_observation, macro_action_t action) {
  if (last_observation >= 0 && !Primitive(action) && action != mRootTask) {
    if (!mApplicable[last_observation][action]) {
      return false;
    }
  }

  return true;
}

HierarchicalMCTS::data_t *HierarchicalMCTS::Query(macro_action_t Action,
                                                  size_t belief_hash) {
  if (mTree.count(Action)) {
    if (mTree[Action].count(belief_hash)) {
      return mTree[Action][belief_hash];
    }
  }

  return 0;
}

void HierarchicalMCTS::Clear() {
  for (auto &n : mTree) {
    for (auto &e : n.second) {
      delete e.second;
    }
  }
  mTree.clear();
  mRootSampling.Free(Simulator);
}

bool HierarchicalMCTS::Update(int action, int observation, double reward, STATE &state) {
  UpdateConnection(History.LastObservation(), observation);
  History.Add(action, observation, reward, Params.MemorySize);

  // Delete old tree and create new root
  Clear();
  STATE *sample = Simulator.Copy(state);
  mRootSampling.AddSample(sample);

  return true;
}

int HierarchicalMCTS::SelectAction() {
  return SelectPrimitiveAction(mRootTask, History);
}

int HierarchicalMCTS::SelectPrimitiveAction(macro_action_t Action,
                                            const HISTORY &history) {
  if (mSubTasks[Action].empty()) {
    return Action;
  }

  data_t *data = Query(Action, history.BeliefHash());
  macro_action_t action;

  if (data) {
    if (Params.Verbose >= 1) {
      if (Params.Verbose >= 3) {
        cerr << "history=[";
        history.Display(cerr);
        cerr << "]" << endl;
      }
      stringstream ss;
      ss << "V(" << Action << ", ";
      ss << "history)";
      data->value.Print(ss.str(), cerr);
      for (auto ii = data->qvalues.begin(); ii != data->qvalues.end(); ++ii) {
        stringstream ss;
        ss << "Q(" << Action << ", ";
        ss << "history, " << ii->first << ")";
        ii->second.Print(ss.str(), cerr);
      }
    }

    action = GreedyUCB(Action, history.LastObservation(), *data, false);
    return SelectPrimitiveAction(action, history);
  } else {
    if (Params.Verbose >= 1) {
      if (Params.Verbose >= 3) {
        cerr << "history=[";
        history.Display(cerr);
        cerr << "]" << endl;
      }
      cerr << "Random Selecting V(" << Action << ", ";
      cerr << "history)" << endl;
      }

    do {
      action = SimpleRNG::ins().Sample(mSubTasks[Action]);
    } while (Terminate(action, history.LastObservation()) ||
             !Applicable(history.LastObservation(), action));
  }

  return SelectPrimitiveAction(action, history);
}

void HierarchicalMCTS::SearchImp() {
  int historyDepth = History.Size();

  STATE *state = mRootSampling.CreateSample(Simulator);
  Simulator.Validate(*state);

  if (Terminate(mRootTask, History.LastObservation())) {
    if (Params.Verbose >= 2) {
      cerr << "Removing observation " << History.Back().Observation
           << " from task graph" << endl;
    }
    for (auto it = mGoals.begin(); it != mGoals.end(); ++it) {
      it->second.erase(History.Back().Observation);
    }
  }

  SearchTree(mRootTask, History, *state, 0);

  Simulator.FreeState(state);
  History.Truncate(historyDepth);
}


HierarchicalMCTS::result_t HierarchicalMCTS::SearchTree(macro_action_t Action, HISTORY &history,
                                    STATE &state, int depth)
{
  TreeDepth = max(TreeDepth, depth);

  if (Params.Verbose >= 3) {
    cerr << "SearchTree" << endl;
    PRINT_VALUE(Action);
    PRINT_VALUE(depth);
    cerr << "history={";
    history.Display(cerr);
    cerr << "}" << endl;
    cerr << "state={\n";
    Simulator.DisplayState(state, cerr);
    cerr << "}" << endl;
    PRINT_VALUE(Terminate(Action, history.LastObservation()));
  }

  if (Primitive(Action)) {
    return Rollout(Action, history, state, depth);
  }
  else {
    if (depth >= Params.MaxDepth || Terminate(Action, history.LastObservation())) {
      return result_t(0.0, 0, false);
    }

    const size_t belief_hash = history.BeliefHash();
    data_t *data = Query(Action, belief_hash);

    if (!data) {
      TreeSize += 1;
      mTree[Action][history.BeliefHash()] = new data_t();
      return Rollout(Action, history, state, depth);
    }
    else {
      const int first_step = history.Size() - 1;

      const int action = GreedyUCB(Action, history.LastObservation(), *data, true);
      auto subtask = SearchTree(action, history, state, depth);  // history and state will be updated
      int steps = subtask.steps;
      result_t completion(0.0, 0, false);
      if (!subtask.terminal) {
        completion = SearchTree(Action, history, state, depth + steps);
      }
      const double totalReward = subtask.reward + pow(Simulator.GetDiscount(), steps) * completion.reward;

      if (Params.AllStateUpdating) {
        for (int k = 0; k < steps; ++k) {
          const size_t belief_hash2 = first_step < 0? 0: history[first_step + k].BeliefHash;
          assert(k != 0 || belief_hash == belief_hash2);

          data_t *data = Query(Action, belief_hash2);
          assert(k != 0 || data);

          if (data) {
            double reward = 0.0;
            for (int l = k + 1; l <= steps; ++l) {
              reward += pow(Simulator.GetDiscount(), l - k - 1) * history[first_step + l].Reward;
            }
            const double totalReward2 = reward + pow(Simulator.GetDiscount(), steps - k) * completion.reward;
            assert(k != 0 || fabs(totalReward2 - totalReward) < 1.0e-6);
            data->value.Add(totalReward2);
            data->qvalues[action].Add(totalReward2);
          }
        }
      }
      else {
        data->value.Add(totalReward);
        data->qvalues[action].Add(totalReward);
      }

      steps += completion.steps;
      return result_t(totalReward, steps, subtask.terminal || completion.terminal);
    }
  }
}

void HierarchicalMCTS::UpdateConnection(int last_observation, int observation) {
  if (Simulator.mActionAbstraction) {
    if (last_observation >= 0) {
      mApplicable[last_observation][MacroAction(observation)] = true;
      mApplicable[observation][MacroAction(last_observation)] = true;
    }
  }
}


HierarchicalMCTS::result_t HierarchicalMCTS::Rollout(macro_action_t Action, HISTORY &history, STATE &state, int depth)
{
  if (Params.Verbose >= 3) {
    cerr << "Rollout" << endl;
    PRINT_VALUE(Action);
    PRINT_VALUE(depth);
    cerr << "history={";
    history.Display(cerr);
    cerr << "}" << endl;
    cerr << "state={\n";
    Simulator.DisplayState(state, cerr);
    cerr << "}" << endl;
    PRINT_VALUE(Terminate(Action, history.LastObservation()));
  }

  if (Primitive(Action)) {
    int observation;
    double immediateReward;
    bool terminal = Simulator.Step(state, Action, observation, immediateReward);
    UpdateConnection(History.LastObservation(), observation);
    history.Add(Action, observation, immediateReward, Params.MemorySize);
    return result_t(immediateReward, 1, terminal);
  }
  else {
    if (depth >= Params.MaxDepth || Terminate(Action, history.LastObservation())) {
      return result_t(0.0, 0, false);
    }

    int action;
    do {
      action = SimpleRNG::ins().Sample(mSubTasks[Action]);
    } while (Terminate(action, history.LastObservation()) ||
             !Applicable(history.LastObservation(), action));

    auto subtask = Rollout(action, history, state, depth);  // history and state will be updated
    int steps = subtask.steps;
    result_t completion(0.0, 0, false);
    if (!subtask.terminal) {
      completion = Rollout(Action, history, state, depth + steps);
    }

    const double totalReward = subtask.reward + pow(Simulator.GetDiscount(), steps) * completion.reward;
    steps += completion.steps;
    return result_t(totalReward, steps, subtask.terminal || completion.terminal);
  }
}

macro_action_t HierarchicalMCTS::GreedyUCB(macro_action_t Action,
                                           int last_observation, data_t &data,
                                           bool ucb) {
  static std::vector<int> besta;
  besta.clear();
  double bestq = -Infinity;
  int N = data.value.GetCount();

  for (uint i = 0; i < mSubTasks[Action].size(); ++i) {
    int action = mSubTasks[Action][i];
    if (Terminate(action, last_observation) ||
        !Applicable(last_observation, action)) {
      continue;
    }

    int n = data.qvalues[action].GetCount();
    double q = data.qvalues[action].GetValue();

      if (n == 0) {
        return action;
      }

    if (ucb) {
      q += FastUCB(N, n);
    }

    if (q >= bestq) {
      if (q > bestq)
        besta.clear();
      bestq = q;
      besta.push_back(action);
    }
  }

  assert(!besta.empty());
  return SimpleRNG::ins().Sample(besta);
}

/**
 * @brief HierarchicalMCTS::Terminate
 * @param Action is the target macro state
 * @param history
 * @param state
 * @return
 */
bool HierarchicalMCTS::Terminate(macro_action_t Action, int last_observation) {
  return !Primitive(Action) && last_observation >= 0 &&
         mGoals[Action].count(last_observation);
}

bool HierarchicalMCTS::Primitive(macro_action_t Action) {
  return mSubTasks[Action].empty();
}

/**
 * @brief HierarchicalMCTS::MacroAction
 * @param o
 * @return the macro action with o as the targeting observation
 */
macro_action_t HierarchicalMCTS::MacroAction(int o) {
  return Simulator.GetNumActions() + o;
}
