#include "hierarchicalmcts.h"
#include <sstream>

using namespace std;

std::unordered_map<std::size_t, BELIEF_STATE> HierarchicalMCTS::data_t::beliefpool;

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
  if (mTable.count(Action)) {
    if (mTable[Action].count(belief_hash)) {
      return mTable[Action][belief_hash];
    }
  }

  return 0;
}

void HierarchicalMCTS::Clear() {
  for (auto &d: mTable) {
    for (auto &e: d.second) {
      delete e.second;
    }
  }
  mTable.clear();
  for (auto &d: data_t::beliefpool) {
    d.second.Free(Simulator);
  }
  data_t::beliefpool.clear();
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
      data->UCB.value.Print(ss.str(), cerr);
      for (auto ii = data->UCB.qvalues.begin(); ii != data->UCB.qvalues.end(); ++ii) {
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
  SearchTree(mRootTask, input, state, 0);

  Simulator.FreeState(state);
  assert(History.Size() == historyDepth);
  History.Truncate(historyDepth);
}

HierarchicalMCTS::result_t HierarchicalMCTS::SearchTree(
    macro_action_t Action, const HierarchicalMCTS::input_t &input, STATE *&state, int depth)
{
  TreeDepth = max(TreeDepth, depth);

  if (Params.Verbose >= 3) {
    cerr << "SearchTree" << endl;
    PRINT_VALUE(Action);
    PRINT_VALUE(depth);
    cerr << "state={\n";
    Simulator.DisplayState(*state, cerr);
    cerr << "}" << endl;
    PRINT_VALUE(Terminate(Action, input.last_observation));
  }

  if (Primitive(Action)) {
    return Rollout(Action, input, state, depth);
  }
  else {
    if (depth >= Params.MaxDepth || Terminate(Action, input.last_observation)) {
      return result_t(0.0, 0, false, input.belief_hash, input.last_observation);
    }

    data_t *data = Query(Action, input.belief_hash);

    if (!data) {
      TreeSize += 1;
      mTable[Action][input.belief_hash] = new data_t();
      return Rollout(Action, input, state, depth);
    }
    else {
      int action = GreedyUCB(Action, input.last_observation, *data, true);

      if (Simulator.mActionAbstraction && Params.UseCache && Action != mRootTask) {
        if (data->cache[action].size() >= uint(Params.UseCache)) {
          result_t cache = SimpleRNG::ins().Sample(data->cache[action]);  // cached result
          assert(data_t::beliefpool.count(cache.belief_hash));
          Simulator.FreeState(state);  // drop current state
          state = Simulator.Copy(*data_t::beliefpool[cache.belief_hash].GetSample());  // sample an exit state
          data->UCB.value.Add(cache.reward);
          data->UCB.qvalues[action].Add(cache.reward);
          return cache;
        }
      }

      result_t subtask = SearchTree(action, input, state, depth);  // history and state will be updated
      int steps = subtask.steps;
      result_t completion(
            0.0, 0, false,
            subtask.belief_hash, subtask.last_observation);
      if (!subtask.terminal) {
        input_t input(subtask.belief_hash, subtask.last_observation);
        completion = SearchTree(Action, input, state, depth + steps);
      }
      double totalReward = subtask.reward + pow(Simulator.GetDiscount(), steps) * completion.reward;
      data->UCB.value.Add(totalReward);
      data->UCB.qvalues[action].Add(totalReward);

      steps += completion.steps;
      result_t ret(
            totalReward, steps, subtask.terminal || completion.terminal,
            completion.belief_hash, completion.last_observation);

      if (Simulator.mActionAbstraction && Params.UseCache && Action != mRootTask) {  // cache the result if converged
        double m = data->UCB.qvalues[action].GetCount() - 1;
        if (m >= Params.UseCache) {
          if (ret.terminal || Terminate(Action, ret.last_observation)) {  // truly an exit
            data->cache[action].push_back(ret);
            STATE *sample = Simulator.Copy(*state);
            data_t::beliefpool[completion.belief_hash].AddSample(sample);  // terminal state
          }
        }
      }
      return ret;
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
  }
}

HierarchicalMCTS::result_t HierarchicalMCTS::Rollout(
    macro_action_t Action, const HierarchicalMCTS::input_t &input, STATE *&state, int depth)
{
  if (Params.Verbose >= 3) {
    cerr << "Rollout" << endl;
    PRINT_VALUE(Action);
    PRINT_VALUE(depth);
    cerr << "state={\n";
    Simulator.DisplayState(*state, cerr);
    cerr << "}" << endl;
    PRINT_VALUE(Terminate(Action, input.last_observation));
  }

  if (Primitive(Action)) {
    int observation;
    double immediateReward;
    bool terminal = Simulator.Step(*state, Action, observation, immediateReward);
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
      return result_t(0.0, 0, false, input.belief_hash, input.last_observation);
    }

    int action;
    do {
      action = SimpleRNG::ins().Sample(mSubTasks[Action]);
    } while (Terminate(action, input.last_observation) || !Applicable(input.last_observation, action));

    auto subtask = Rollout(action, input, state, depth);  // history and state will be updated
    int steps = subtask.steps;
    result_t completion(
          0.0, 0, false,
          subtask.belief_hash, subtask.last_observation);
    if (!subtask.terminal) {
      input_t input(subtask.belief_hash, subtask.last_observation);
      completion = Rollout(Action, input, state, depth + steps);
    }

    double totalReward = subtask.reward + pow(Simulator.GetDiscount(), steps) * completion.reward;
    steps += completion.steps;
    return result_t(
          totalReward, steps, subtask.terminal || completion.terminal,
          completion.belief_hash, completion.last_observation);
  }
}

macro_action_t HierarchicalMCTS::GreedyUCB(macro_action_t Action, int last_observation, data_t &data, bool ucb) {
  static std::vector<int> besta;
  besta.clear();
  double bestq = -Infinity;
  int N = data.UCB.value.GetCount();

  for (uint i = 0; i < mSubTasks[Action].size(); ++i) {
    int action = mSubTasks[Action][i];
    if (Terminate(action, last_observation) || !Applicable(last_observation, action)) {
      continue;
    }

    int n = data.UCB.qvalues[action].GetCount();
    double q = data.UCB.qvalues[action].GetValue();

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
