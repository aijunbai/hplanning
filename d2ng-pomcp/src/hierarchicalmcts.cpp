#include "hierarchicalmcts.h"
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
    mRootBelief.AddSample(Simulator.CreateStartState());
  }

  if (Simulator.mActionAbstraction) {
    int iterations = 1000, max_depth = 1000;

    for (int i = 0; i < iterations; ++i) {
      HISTORY history;
      STATE *state = mRootBelief.CreateSample(Simulator);
      Simulator.Validate(*state);
      bool terminal = false;
      int step = 0;

      for (; !terminal && step < max_depth; ++step) {
        int observation;
        double reward;
        int action = SimpleRNG::ins().Random(Simulator.GetNumActions());
        terminal = Simulator.Step(*state, action, observation, reward);
        UpdateHistory(history, action, observation, reward);
      }

      Simulator.FreeState(state);
    }
    if (Params.Verbose >= 2) {
      PRINT_VALUE(mApplicable);
    }
  }
}

HierarchicalMCTS::~HierarchicalMCTS() { mRootBelief.Free(Simulator); }

bool HierarchicalMCTS::Applicable(HISTORY &history, macro_action_t action)
{
  if (history.Size() && !Primitive(action) && action != mRootTask) {
    if (!mApplicable[history.Back().Observation][action]) {
      return false;
    }
  }

  return true;
}

bool HierarchicalMCTS::Update(int action, int observation, double reward, STATE &state) {
  UpdateHistory(History, action, observation, reward);

  // Delete old tree and create new root
  mTable.clear();
  mRootBelief.Free(Simulator);
  STATE *sample = Simulator.Copy(state);
  mRootBelief.AddSample(sample);

  return true;
}

int HierarchicalMCTS::SelectAction() {
  return SelectPrimitiveAction(mRootTask, History);
}

int HierarchicalMCTS::SelectPrimitiveAction(macro_action_t Action, HISTORY &history) {
  if (mSubTasks[Action].empty()) {
    return Action;
  }

  size_t hash = hash_value(Action, history.BeliefHash());
  auto it = mTable.find(hash);

  macro_action_t action;
  if (it != mTable.end()) {
    if (Params.Verbose >= 1) {
      if (Params.Verbose >= 3) {
        cerr << "history=[";
        history.Display(cerr);
        cerr << "]" << endl;
      }
      stringstream ss;
      ss << "V(" << Action << ", ";
      ss << "history)";
      it->second.UCB.value_.Print(ss.str(), cerr);
      for (auto ii = it->second.UCB.qvalues_.begin(); ii != it->second.UCB.qvalues_.end(); ++ii) {
        stringstream ss;
        ss << "Q(" << Action << ", ";
        ss << "history, " << ii->first << ")";
        ii->second.Print(ss.str(), cerr);
      }
    }

    action = GreedyUCB(Action, history, it->second, false);
    return SelectPrimitiveAction(action, history);
  }
  else {
    if (Params.Verbose >= 1) {
      if (Params.Verbose >= 3) {
        cerr << "history=[";
        history.Display(cerr);
        cerr << "]" << endl;
      }
      cerr << "Random Selecting V(" << Action << ", ";
      cerr << "history)";
      }

    do {
      action = SimpleRNG::ins().Sample(mSubTasks[Action]);
    } while (Terminate(action, history) || !Applicable(history, action));
  }

  return SelectPrimitiveAction(action, history);
}

void HierarchicalMCTS::SearchImp() {
  int historyDepth = History.Size();

  STATE *state = mRootBelief.CreateSample(Simulator);
  Simulator.Validate(*state);

  if (Terminate(mRootTask, History)) {
    if (Params.Verbose >= 2) {
      cerr << "Removing observation " << History.Back().Observation << " from task graph" << endl;
    }
    for (auto it = mGoals.begin(); it != mGoals.end(); ++it) {
      it->second.erase(History.Back().Observation);
    }
  }

  SearchTree(mRootTask, History, *state, 0);

  Simulator.FreeState(state);
  History.Truncate(historyDepth);
}

void HierarchicalMCTS::InsertNode(size_t hash)
{
  TreeSize += 1;
  mTable.insert(make_pair(hash, data_t()));
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
    PRINT_VALUE(Terminate(Action, history));
  }

  if (Primitive(Action)) {
    return Rollout(Action, history, state, depth);
  }
  else {
    if (depth >= Params.MaxDepth || Terminate(Action, history)) {
      return result_t(0.0, 0, depth >= Params.MaxDepth);
    }

    size_t belief_hash = history.BeliefHash();
    size_t hash = hash_value(Action, belief_hash);
    auto it = mTable.find(hash);

    if (it == mTable.end()) {
      InsertNode(hash);
      return Rollout(Action, history, state, depth);
    }
    else {
      int first_step = history.Size() - 1;

      int action = GreedyUCB(Action, history, it->second, true);
      auto reward_term = SearchTree(action, history, state, depth);  // history and state will be updated
      int steps = reward_term.steps;
      result_t completion_term(0.0, 0, false);
      if (!reward_term.terminal) {
        completion_term = SearchTree(Action, history, state, depth + steps);
      }
      double totalReward = reward_term.reward + pow(Simulator.GetDiscount(), steps) * completion_term.reward;

      if (Params.AllStateUpdating) {
        for (int k = 0; k < steps; ++k) {
          size_t belief_hash2 = first_step < 0? 0: history[first_step + k].BeliefHash;
          size_t hash2 = hash_value(Action, belief_hash2);
          assert(k != 0 || belief_hash == belief_hash2);
          assert(k != 0 || hash == hash2);
          auto it = mTable.find(hash2);
          assert(k != 0 || it != mTable.end());

          if (it != mTable.end()) {
            double reward = 0.0;
            for (int l = k + 1; l <= steps; ++l) {
              reward += pow(Simulator.GetDiscount(), l - k - 1) * history[first_step + l].Reward;
            }
            double totalReward2 = reward + pow(Simulator.GetDiscount(), steps - k) * completion_term.reward;
            assert(k != 0 || fabs(totalReward2 - totalReward) < 1.0e-6);
            it->second.UCB.value_.Add(totalReward2);
            it->second.UCB.qvalues_[action].Add(totalReward2);
          }
        }
      }
      else {
        it->second.UCB.value_.Add(totalReward);
        it->second.UCB.qvalues_[action].Add(totalReward);
      }

      steps += completion_term.steps;
      return result_t(totalReward, steps, reward_term.terminal || completion_term.terminal);
    }
  }
}

void HierarchicalMCTS::UpdateHistory(HISTORY &history, int action, int observation, double reward)
{
  if (Simulator.mActionAbstraction) {
    if (history.Size()) {
      mApplicable[history.Back().Observation][MacroAction(observation)] = true;
      mApplicable[observation][MacroAction(history.Back().Observation)] = true;
    }
  }
  history.Add(action, observation, reward, Params.MemorySize);
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
    PRINT_VALUE(Terminate(Action, history));
  }

  if (Primitive(Action)) {
    int observation;
    double immediateReward;
    bool terminal = Simulator.Step(state, Action, observation, immediateReward);
    UpdateHistory(History, Action, observation, immediateReward);
    return result_t(immediateReward, 1, terminal);
  }
  else {
    if (depth >= Params.MaxDepth || Terminate(Action, history)) {
      return result_t(0.0, 0, depth >= Params.MaxDepth);
    }

    int action;
    do {
      action = SimpleRNG::ins().Sample(mSubTasks[Action]);
    } while (Terminate(action, history) || !Applicable(history, action));

    auto reward_term = Rollout(action, history, state, depth);  // history and state will be updated
    int steps = reward_term.steps;
    result_t completion_term(0.0, 0, false);
    if (!reward_term.terminal) {
      completion_term = Rollout(Action, history, state, depth + steps);
    }

    double totalReward = reward_term.reward + pow(Simulator.GetDiscount(), steps) * completion_term.reward;
    steps += completion_term.steps;
    return result_t(totalReward, steps, reward_term.terminal || completion_term.terminal);
  }
}

macro_action_t HierarchicalMCTS::GreedyUCB(macro_action_t Action, HISTORY &history, data_t &data, bool ucb) {
  static std::vector<int> besta;
  besta.clear();
  double bestq = -Infinity;
  int N = data.UCB.value_.GetCount();

  for (uint i = 0; i < mSubTasks[Action].size(); ++i) {
    int action = mSubTasks[Action][i];
    if (Terminate(action, history) || !Applicable(history, action)) {
      continue;
    }

    int n = data.UCB.qvalues_[action].GetCount();
    double q = data.UCB.qvalues_[action].GetValue();

    if (ucb) {
      if (n == 0) {
        return action;
      }

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
bool HierarchicalMCTS::Terminate(macro_action_t Action, HISTORY &history) {
  return !Primitive(Action) && history.Size() && mGoals[Action].count(history.Back().Observation);
}

bool HierarchicalMCTS::Primitive(macro_action_t Action)
{
  return mSubTasks[Action].empty();
}

/**
 * @brief HierarchicalMCTS::MacroAction
 * @param o
 * @return the macro action with o as the targeting observation
 */
macro_action_t HierarchicalMCTS::MacroAction(int o)
{
  return Simulator.GetNumActions() + o;
}
