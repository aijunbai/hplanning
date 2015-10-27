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
        history.Add(action, observation);
      }

      Simulator.FreeState(state);
    }
    if (Params.Verbose >= 2) {
      PRINT_VALUE(mApplicable);
    }
  }
}

HierarchicalMCTS::~HierarchicalMCTS() { 
  Clear();
}

bool HierarchicalMCTS::Applicable(int last_observation, macro_action_t action)
{
  if (last_observation >= 0 && !Primitive(action) && action != mRootTask) {
    if (!mApplicable[last_observation][action]) {
      return false;
    }
  }

  return true;
}

HierarchicalMCTS::data_t *HierarchicalMCTS::Query(macro_action_t Action, size_t belief_hash) {
  if (mTable.count(belief_hash)) {
    if (mTable[belief_hash].count(Action)) {
      return mTable[belief_hash][Action];
    }
  }

  return 0;
}

void HierarchicalMCTS::Clear() {
  for (auto it = mTable.begin(); it != mTable.end(); ++it) {
    for (auto ii = it->second.begin(); ii != it->second.end(); ++ii) {
      delete ii->second;
    }
  }
  mTable.clear();

  for (auto it = mBelief.begin(); it != mBelief.end(); ++it) {
    it->second.Free(Simulator);
  }
  mBelief.clear();
  mRootSampling.Free(Simulator); 
}

bool HierarchicalMCTS::Update(int action, int observation, STATE &state) {
  UpdateConnection(History.LastObservation(), observation);
  History.Add(action, observation);

  // Delete old tree and create new root
  Clear();
  STATE *sample = Simulator.Copy(state);
  mRootSampling.AddSample(sample);

  return true;
}

int HierarchicalMCTS::SelectAction() {
  return SelectPrimitiveAction(mRootTask, History);
}

int HierarchicalMCTS::SelectPrimitiveAction(macro_action_t Action, const HISTORY &history) {
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
      data->UCB.value_.Print(ss.str(), cerr);
      for (auto ii = data->UCB.qvalues_.begin(); ii != data->UCB.qvalues_.end(); ++ii) {
        stringstream ss;
        ss << "Q(" << Action << ", ";
        ss << "history, " << ii->first << ")";
        ii->second.Print(ss.str(), cerr);
      }
    }

    action = GreedyUCB(Action, history.LastObservation(), *data, false);
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
      cerr << "Removing observation " << History.Back().Observation << " from task graph" << endl;
    }
    for (auto it = mGoals.begin(); it != mGoals.end(); ++it) {
      it->second.erase(History.Back().Observation);
    }
  }

  input_t input(History.BeliefHash(), History.LastObservation());
  SearchTree(mRootTask, input, *state, 0);

  Simulator.FreeState(state);
  assert(History.Size() == historyDepth);
  History.Truncate(historyDepth);
}

HierarchicalMCTS::result_t HierarchicalMCTS::SearchTree(
    macro_action_t Action, HierarchicalMCTS::input_t &input, STATE &state, int depth)
{
  TreeDepth = max(TreeDepth, depth);

  if (Params.Verbose >= 3) {
    cerr << "SearchTree" << endl;
    PRINT_VALUE(Action);
    PRINT_VALUE(depth);
    cerr << "state={\n";
    Simulator.DisplayState(state, cerr);
    cerr << "}" << endl;
    PRINT_VALUE(Terminate(Action, input.last_observation));
  }

  if (Primitive(Action)) {
    return Rollout(Action, input, state, depth);
  }
  else {
    if (depth >= Params.MaxDepth || Terminate(Action, input.last_observation)) {
      return result_t(
            0.0, 0, depth >= Params.MaxDepth,
            input.belief_hash, input.last_observation);
    }

    STATE *sample = Simulator.Copy(state);
    mBelief[input.belief_hash].AddSample(sample);
    data_t *data = Query(Action, input.belief_hash);

    if (!data) {
      TreeSize += 1;
      mTable[input.belief_hash][Action] = new data_t();
      return Rollout(Action, input, state, depth);
    }
    else {
      // return cached result here.
      int action = GreedyUCB(Action, input.last_observation, *data, true);
      result_t reward_term = SearchTree(action, input, state, depth);  // history and state will be updated
      int steps = reward_term.steps;
      result_t completion_term(
            0.0, 0, false,
            reward_term.belief_hash, reward_term.last_observation);
      if (!reward_term.terminal) {
        input_t input(reward_term.belief_hash, reward_term.last_observation);
        completion_term = SearchTree(Action, input, state, depth + steps);
      }
      double totalReward = reward_term.reward + pow(Simulator.GetDiscount(), steps) * completion_term.reward;
      data->UCB.value_.Add(totalReward);
      data->UCB.qvalues_[action].Add(totalReward);

      steps += completion_term.steps;
      return result_t(
            totalReward, steps, reward_term.terminal || completion_term.terminal,
            completion_term.belief_hash, completion_term.last_observation);
    }
  }
}

void HierarchicalMCTS::UpdateConnection(int last_observation, int observation)
{
  if (Simulator.mActionAbstraction) {
    if (last_observation >= 0) {
      mApplicable[last_observation][MacroAction(observation)] = true;
      mApplicable[observation][MacroAction(last_observation)] = true;
    }
  }}

HierarchicalMCTS::result_t HierarchicalMCTS::Rollout(
    macro_action_t Action, HierarchicalMCTS::input_t &input, STATE &state, int depth)
{
  if (Params.Verbose >= 3) {
    cerr << "Rollout" << endl;
    PRINT_VALUE(Action);
    PRINT_VALUE(depth);
    cerr << "state={\n";
    Simulator.DisplayState(state, cerr);
    cerr << "}" << endl;
    PRINT_VALUE(Terminate(Action, input.last_observation));
  }

  if (Primitive(Action)) {
    int observation;
    double immediateReward;
    bool terminal = Simulator.Step(state, Action, observation, immediateReward);
    UpdateConnection(input.last_observation, observation);

    size_t belief_hash = 0;
    if (Simulator.mStateAbstraction) {  // whole history
      belief_hash = input.belief_hash;
      boost::hash_combine(belief_hash, Action);
      boost::hash_combine(belief_hash, observation);
    }
    else {  // memory size = 1
      boost::hash_combine(belief_hash, observation);  // observation is the ground state
      boost::hash_combine(belief_hash, depth);
    }

    return result_t(immediateReward, 1, terminal, belief_hash, observation);
  }
  else {
    if (depth >= Params.MaxDepth || Terminate(Action, input.last_observation)) {
      return result_t(
            0.0, 0, depth >= Params.MaxDepth,
            input.belief_hash, input.last_observation);
    }

    int action;
    do {
      action = SimpleRNG::ins().Sample(mSubTasks[Action]);
    } while (Terminate(action, input.last_observation) || !Applicable(input.last_observation, action));

    auto reward_term = Rollout(action, input, state, depth);  // history and state will be updated
    int steps = reward_term.steps;
    result_t completion_term(
          0.0, 0, false,
          reward_term.belief_hash, reward_term.last_observation);
    if (!reward_term.terminal) {
      input_t input(reward_term.belief_hash, reward_term.last_observation);
      completion_term = Rollout(Action, input, state, depth + steps);
    }

    double totalReward = reward_term.reward + pow(Simulator.GetDiscount(), steps) * completion_term.reward;
    steps += completion_term.steps;
    return result_t(
          totalReward, steps, reward_term.terminal || completion_term.terminal,
          completion_term.belief_hash, completion_term.last_observation);
  }
}

macro_action_t HierarchicalMCTS::GreedyUCB(macro_action_t Action, int last_observation, data_t &data, bool ucb) {
  static std::vector<int> besta;
  besta.clear();
  double bestq = -Infinity;
  int N = data.UCB.value_.GetCount();

  for (uint i = 0; i < mSubTasks[Action].size(); ++i) {
    int action = mSubTasks[Action][i];
    if (Terminate(action, last_observation) || !Applicable(last_observation, action)) {
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
bool HierarchicalMCTS::Terminate(macro_action_t Action, int last_observation) {
  return !Primitive(Action) && last_observation >= 0 && mGoals[Action].count(last_observation);
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
