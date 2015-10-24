#include "hierarchicalmcts.h"
#include <sstream>

using namespace std;

HierarchicalMCTS::HierarchicalMCTS(const SIMULATOR &simulator,
                                   const PARAMS &params)
    : MCTS(simulator, params), mRootTask(-1)
{
  mSubTasks[mRootTask] = vector<macro_action_t>();  // root action
  mGoals[mRootTask].insert(0);  // ground target state assumed to be in macro state 0

  for (int a = 0; a < Simulator.GetNumActions(); ++a) {
    mSubTasks[a] = vector<macro_action_t>();  // primitive actions
  }

  if (Simulator.mStateAbstraction) {
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

  STATE *state = Simulator.CreateStartState(); //可能的开始状态
  mRootBelief.AddSample(state);
  for (int i = 1; i < Params.NumStartStates; i++) {
    mRootBelief.AddSample(Simulator.CreateStartState());
  }
}

HierarchicalMCTS::~HierarchicalMCTS() { mRootBelief.Free(Simulator); }

bool HierarchicalMCTS::Update(int action, int observation, STATE &state) {
  History.Add(action, observation, Params.MemorySize); //更新历史

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

  size_t hash = hash_value(Action, history);
  auto it = mTable.find(hash);
  assert(it != mTable.end());

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

  macro_action_t action = GreedyUCB(Action, history, it->second, false);
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

    size_t hash = hash_value(Action, history);
    auto it = mTable.find(hash);

    if (it == mTable.end()) {
      TreeSize += 1;
      mTable.insert(make_pair(hash, data_t()));
      auto it = mTable.find(hash);
      assert(it != mTable.end());
      return Rollout(Action, history, state, depth);
    }
    else {
      int action = GreedyUCB(Action, history, it->second, true);
      auto reward_term = SearchTree(action, history, state, depth);  // history and state will be updated
      if (reward_term.terminal) {
        it->second.UCB.value_.Add(reward_term.reward);
        it->second.UCB.qvalues_[action].Add(reward_term.reward);
        return reward_term;
      }

      int steps = reward_term.steps;
      auto completion_term = SearchTree(Action, history, state, depth + steps);
      double totalReward = reward_term.reward + pow(Simulator.GetDiscount(), steps) * completion_term.reward;
      it->second.UCB.value_.Add(totalReward);
      it->second.UCB.qvalues_[action].Add(totalReward);
      steps += completion_term.steps;
      return result_t(totalReward, steps, reward_term.terminal || completion_term.terminal);
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
    PRINT_VALUE(Terminate(Action, history));
  }

  if (Primitive(Action)) {
    int observation;
    double immediateReward;
    bool terminal = Simulator.Step(state, Action, observation, immediateReward);
    history.Add(Action, observation, Params.MemorySize);
    return result_t(immediateReward, 1, terminal);
  }
  else {
    if (depth >= Params.MaxDepth || Terminate(Action, history)) {
      return result_t(0.0, 0, depth >= Params.MaxDepth);
    }

    int action;
    do {
      action = SimpleRNG::ins().Sample(mSubTasks[Action]);
    } while (Terminate(action, history));

    auto reward_term = Rollout(action, history, state, depth);  // history and state will be updated
    if (reward_term.terminal) {
      return reward_term;
    }

    int steps = reward_term.steps;
    auto completion_term = Rollout(Action, history, state, depth + steps);
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
    if (Terminate(action, history)) {
      continue;
    }

    int n = data.UCB.qvalues_[action].GetCount();
    double q = data.UCB.qvalues_[action].GetValue();

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
bool HierarchicalMCTS::Terminate(macro_action_t Action, HISTORY &history) {
  return history.Size() && mGoals[Action].count(history.Back().Observation);
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
