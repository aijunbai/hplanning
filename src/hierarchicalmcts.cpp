#include "hierarchicalmcts.h"
#include "coord.h"

using namespace std;

HierarchicalMCTS::HierarchicalMCTS(const SIMULATOR &simulator,
                                   const PARAMS &params, bool action_abstraction,
                                   int first_observation, STATE *ground_state)
    : MCTS(simulator, params, first_observation),
      mRootTask(-1, 0),
      mActionAbstraction(action_abstraction) {
  if (!mActionAbstraction) {
    for (int a = 0; a < Simulator.GetNumActions(); ++a) {
      mSubTasks[mRootTask].insert(PrimitiveAction(a));
    }
  }

  mRootSampling.AddSample(Simulator.Copy(*ground_state));

  if (mActionAbstraction) {
    int iterations = 1000, max_depth = 1000;

    for (int i = 0; i < iterations; ++i) {
      HISTORY history(first_observation);
      STATE *state = mRootSampling.CreateSample(Simulator);
      Simulator.Validate(*state);
      bool terminal = false;

      for (int step = 0; !terminal && step < max_depth; ++step) {
        int observation;
        double reward;
        int action = SimpleRNG::ins().Random(Simulator.GetNumActions());
        terminal = Simulator.Step(*state, action, observation, reward);
        AddAbstractAction(history.LastObservation(), observation, state);
        history.Add(action, observation);
      }

      Simulator.FreeState(state);
    }
    if (Params.Verbose >= 2) {
      PRINT_VALUE(mAvailableActions);
    }
  }

  mCallStack.push(mRootTask);
}

HierarchicalMCTS::~HierarchicalMCTS() {
  Clear();

  for (auto &e : mExits) {
    Simulator.FreeState(e.second);
  }
}

void HierarchicalMCTS::UnitTest() {

}

bool HierarchicalMCTS::Applicable(int last_observation, macro_action_t action) {
  if (last_observation >= 0 && !IsPrimitive(action) && action != mRootTask) {
    return action.first == last_observation &&
           mAvailableActions[last_observation].count(action);
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

HierarchicalMCTS::data_t *HierarchicalMCTS::Insert(macro_action_t Action,
                                                   size_t belief_hash) {
  assert(!Query(Action, belief_hash));
  mTree[Action][belief_hash] = new data_t();
  TreeSize += 1;
  return mTree[Action][belief_hash];
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

bool HierarchicalMCTS::Update(int action, int observation, STATE &state) {
  AddAbstractAction(History.LastObservation(), observation, &state);
  History.Add(action, observation);

  // Delete old tree and create new root
  Clear();
  STATE *sample = Simulator.Copy(state);
  mRootSampling.AddSample(sample);

  return true;
}

int HierarchicalMCTS::SelectAction() {
  input_t input(0, History.LastObservation());

  if (mActionAbstraction) {
    if (input.last_observation == '0') {  // enter target macro state XXX
      if (Params.Verbose >= 2) {
        cerr << "Cancelling action abstraction" << endl;
      }

      mSubTasks[mRootTask].clear();
      for (int a = 0; a < Simulator.GetNumActions(); ++a) {
        mSubTasks[mRootTask].insert(PrimitiveAction(a));
      }

      while (mCallStack.size()) {
        mCallStack.pop();
      }
      mCallStack.push(mRootTask);
      mActionAbstraction = false;
    }
    else {
      if (Params.Stack) {
        while (mCallStack.size() &&
               (IsPrimitive(mCallStack.top()) ||
                IsTerminated(mCallStack.top(), input.last_observation))) {
          mCallStack.pop();
        }

        if (mCallStack.empty()) {
          mCallStack.push(mRootTask);
        }
      }
    }
  }

  if (Params.Verbose >= 2) {
    cerr << "Searching for task " << mCallStack.top() << endl;
  }

  Search();

  if (mActionAbstraction) {
    if (Params.Stack) {
      if (input.last_observation != -1) {
        while (!IsPrimitive(mCallStack.top())) {
          data_t *data = Query(mCallStack.top(), input.belief_hash);

          if (data) {
            if (Params.Verbose >= 1) {
              stringstream ss;
              ss << "V(" << mCallStack.top() << ", ";
              ss << "history)[" << Params.LocalReward << "]";
              data->V[Params.LocalReward].value.Print(ss.str(), cerr);
              for (auto ii = data->V[Params.LocalReward].qvalues.begin();
                   ii != data->V[Params.LocalReward].qvalues.end(); ++ii) {
                stringstream ss;
                ss << "Q(" << mCallStack.top() << ", ";
                ss << "history, " << ii->first << ")[" << Params.LocalReward << "]";
                ii->second.Print(ss.str(), cerr);
              }
            }

            macro_action_t subtask = GreedyUCB(mCallStack.top(), input.last_observation, *data, false);
            mCallStack.push(subtask);
          }
          else {
            assert(0);
            break;
          }
        }
      }
    }
  }

  STATE *state = mRootSampling.CreateSample(Simulator);
  macro_action_t action = GreedyPrimitiveAction(mCallStack.top(), input, *state);
  Simulator.FreeState(state);
  assert(IsPrimitive(action));

  return action.second;
}

macro_action_t HierarchicalMCTS::InformativePrimitiveAction(
    macro_action_t Action,
    const input_t &input, STATE &state)
{
  if (IsPrimitive(Action)) {
    return Action;
  }

  if (Action != mRootTask) {
    STATE *exit = mExits[Action];
    assert(exit);

    if (exit) {
      return PrimitiveAction(Simulator.SuggestAction(state, *exit));
    }
  }

  macro_action_t action = RandomSubtask(Action, input);
  return InformativePrimitiveAction(action, input, state);
}

macro_action_t HierarchicalMCTS::GetPrimitiveAction(
    macro_action_t Action,
    const input_t &input, STATE &state)
{
  if (Simulator.Knowledge.RolloutLevel >= SIMULATOR::KNOWLEDGE::SMART) {
    return InformativePrimitiveAction(Action, input, state);
  }
  else {
    return RandomPrimitiveAction(Action, input);
  }
}

macro_action_t HierarchicalMCTS::RandomSubtask(macro_action_t Action, const input_t &input)
{
  assert(!IsPrimitive(Action));

  if (IsPrimitive(Action)) {
    return Action;
  }

  macro_action_t action;
  do {
    if (Action != mRootTask || !mActionAbstraction) {
      action = *next(begin(mSubTasks[Action]),
                     SimpleRNG::ins().Random(mSubTasks[Action].size()));
    }
    else {
      action = *next(begin(mAvailableActions[input.last_observation]),
                     SimpleRNG::ins().Random(mAvailableActions[input.last_observation].size()));
    }
  } while (IsTerminated(action, input.last_observation) ||
           !Applicable(input.last_observation, action));

  return action;
}

macro_action_t HierarchicalMCTS::RandomPrimitiveAction(
    macro_action_t Action, const input_t &input)
{
  if (IsPrimitive(Action)) {
    return Action;
  }

  macro_action_t action = RandomSubtask(Action, input);
  return RandomPrimitiveAction(action, input);
}

macro_action_t HierarchicalMCTS::GreedyPrimitiveAction(
    macro_action_t Action,
    const input_t &input, STATE &state) {
  if (IsPrimitive(Action)) {
    return Action;
  }

  data_t *data = Query(Action, input.belief_hash);
  macro_action_t action;

  if (data) {
    if (Params.Verbose >= 1) {
      stringstream ss;
      ss << "V(" << Action << ", ";
      ss << "history)[" << Params.LocalReward << "]";
      data->V[Params.LocalReward].value.Print(ss.str(), cerr);
      for (auto ii = data->V[Params.LocalReward].qvalues.begin(); ii != data->V[Params.LocalReward].qvalues.end(); ++ii) {
        stringstream ss;
        ss << "Q(" << Action << ", ";
        ss << "history, " << ii->first << ")[" << Params.LocalReward << "]";
        ii->second.Print(ss.str(), cerr);
      }
    }

    action = GreedyUCB(Action, input.last_observation, *data, false);
    return GreedyPrimitiveAction(action, input, state);
  } else {
    if (Params.Verbose >= 1) {
      cerr << "Random Selecting V(" << Action << ", ";
      cerr << "history)" << endl;
    }

    return GetPrimitiveAction(Action, input, state);
  }
}

void HierarchicalMCTS::SearchImp() {
  assert(mRootSampling.GetNumSamples() == 1);
  int historyDepth = History.Size();

  STATE *state = mRootSampling.CreateSample(Simulator);
  Simulator.Validate(*state);

  input_t input(0, History.LastObservation());
  SearchTree(mCallStack.top(), input, state, 0);

  Simulator.FreeState(state);
  assert(History.Size() == historyDepth);
  History.Truncate(historyDepth);
}


HierarchicalMCTS::result_t
HierarchicalMCTS::SearchTree(macro_action_t Action,
                             const HierarchicalMCTS::input_t &input,
                             STATE *&state, int depth) {
  //cerr << "Action: " << Action << endl;
  //cerr << "depth: " << depth << endl;

  TreeDepth = max(TreeDepth, depth);

  if (Params.Verbose >= 3) {
    cerr << "SearchTree" << endl;
    PRINT_VALUE(Action);
    PRINT_VALUE(depth);
    cerr << "state={\n";
    Simulator.DisplayState(*state, cerr);
    cerr << "}" << endl;
  }

  if (IsPrimitive(Action)) {
    return Simulate(Action, input, state, depth); // simulate primitive Action
  } else {
    if (depth >= Params.MaxDepth || IsTerminated(Action, input.last_observation)) {
      return result_t(0.0, 0, false, input.belief_hash, input.last_observation);
    }

    data_t *data = Query(Action, input.belief_hash);

    if (!data) {
      Insert(Action, input.belief_hash);
      return Rollout(Action, input, state, depth);
    } else {
      const macro_action_t action = GreedyUCB(Action, input.last_observation, *data, true);
      const result_t subtask = SearchTree(action, input, state,
                                          depth); // history and state will be updated
      int steps = subtask.steps;
      result_t completion(0.0, 0, false, subtask.belief_hash,
                          subtask.last_observation);
      if (!subtask.global_terminal) {
        input_t input(subtask.belief_hash, subtask.last_observation);
        completion = SearchTree(Action, input, state, depth + steps);
      }
      double totalReward =
          subtask.reward +
          pow(Simulator.GetDiscount(), steps) * completion.reward;

      data->V[0].value.Add(totalReward);
      data->V[0].qvalues[action].Add(totalReward);

      steps += completion.steps;

      if (Params.LocalReward) {
        double localReward = 0.0;
        if (mActionAbstraction) {
          localReward = pow(Simulator.GetDiscount(), steps) *
                        LocalReward(Action, completion.last_observation, steps);
        }

        data->V[1].value.Add(totalReward + localReward);
        data->V[1].qvalues[action].Add(totalReward + localReward);
      }

      const result_t ret(totalReward, steps, subtask.global_terminal || completion.global_terminal,
                         completion.belief_hash, completion.last_observation);
      return ret;
    }
  }
}

void HierarchicalMCTS::AddAbstractAction(int from, int to, STATE *state) {
  if (mActionAbstraction) {
    if (from >= 0 && to >= 0 && from != to) {
      macro_action_t Actions[2] = {MacroAction(from, to), MacroAction(to, from)};

      for (auto Action: Actions) {
        if (mSubTasks[mRootTask].count(Action) == 0) {
          mSubTasks[mRootTask].insert(Action);
          for (int a = 0; a < Simulator.GetNumActions(); ++a) {
            mSubTasks[Action].insert(PrimitiveAction(a));
          }
          mAvailableActions[Action.first].insert(Action);
          mExits[Action] = Simulator.Copy(*state);
        }
      }
    }
  }
}

HierarchicalMCTS::result_t
HierarchicalMCTS::PollingRollout(macro_action_t Action,
                                 const HierarchicalMCTS::input_t &input, STATE *&state,
                                 int depth) {
  //cerr << "Rollout Action: " << Action << endl;
  //cerr << "Rollout depth: " << depth << endl;
  assert(!IsPrimitive(Action));

  if (depth >= Params.MaxDepth || IsTerminated(Action, input.last_observation)) {
    return result_t(0.0, 0, false, input.belief_hash, input.last_observation);
  }

  macro_action_t action = GetPrimitiveAction(Action, input, *state);
  result_t atomic = Simulate(action, input, state, depth);
  assert(atomic.steps == 1);

  if (!atomic.global_terminal) {
    input_t input(atomic.belief_hash, atomic.last_observation);
    result_t completion = Rollout(Action, input, state, depth + 1);
    double totalReward = atomic.reward + Simulator.GetDiscount() * completion.reward;
    return result_t(totalReward,
                    atomic.steps + completion.steps,
                    completion.global_terminal,
                    completion.belief_hash,
                    completion.last_observation);
  }

  return atomic;
}

HierarchicalMCTS::result_t
HierarchicalMCTS::Simulate(macro_action_t action, const input_t &input, STATE *&state,
                           int depth) {
  assert(IsPrimitive(action));

  int observation;
  double immediateReward;
  bool terminal =
      Simulator.Step(*state, action.second, observation, immediateReward);
  AddAbstractAction(input.last_observation, observation, state);

  size_t belief_hash = 0;
  if (Simulator.mStateAbstraction) { // whole history
    belief_hash = input.belief_hash;
    boost::hash_combine(belief_hash, action);
    boost::hash_combine(belief_hash, observation);
    boost::hash_combine(belief_hash, depth);
  } else { // memory size = 1
    boost::hash_combine(belief_hash, observation); // observation is the ground state
    boost::hash_combine(belief_hash, depth);
  }
  return result_t(immediateReward, 1, terminal, belief_hash, observation);
}

HierarchicalMCTS::result_t
HierarchicalMCTS::Rollout(macro_action_t Action,
                          const HierarchicalMCTS::input_t &input, STATE *&state,
                          int depth) {
  if (Params.Verbose >= 3) {
    cerr << "Rollout" << endl;
    PRINT_VALUE(Action);
    PRINT_VALUE(depth);
    cerr << "state={\n";
    Simulator.DisplayState(*state, cerr);
    cerr << "}" << endl;
  }

  if (IsPrimitive(Action)) {
    return Simulate(Action, input, state, depth);
  }
  else {
    if (Params.Polling) {
      return PollingRollout(Action, input, state, depth);
    }
    else {
      return HierarchicalRollout(Action, input, state, depth);
    }
  }
}

HierarchicalMCTS::result_t
HierarchicalMCTS::HierarchicalRollout(macro_action_t Action,
                                      const HierarchicalMCTS::input_t &input, STATE *&state,
                                      int depth) {
  assert(!IsPrimitive(Action));

  if (depth >= Params.MaxDepth || IsTerminated(Action, input.last_observation)) {
    return result_t(0.0, 0, false, input.belief_hash, input.last_observation);
  }

  macro_action_t action = RandomSubtask(Action, input);
  auto subtask = Rollout(action, input, state, depth); // history and state will be updated
  int steps = subtask.steps;
  result_t completion(0.0, 0, false, subtask.belief_hash,
                      subtask.last_observation);
  if (!subtask.global_terminal) {
    input_t input(subtask.belief_hash, subtask.last_observation);
    completion = Rollout(Action, input, state, depth + steps);
  }

  double totalReward =
      subtask.reward +
      pow(Simulator.GetDiscount(), steps) * completion.reward;
  steps += completion.steps;

  return result_t(totalReward, steps, subtask.global_terminal || completion.global_terminal,
                  completion.belief_hash, completion.last_observation);
}

macro_action_t HierarchicalMCTS::GreedyUCB(macro_action_t Action,
                                           int last_observation, data_t &data,
                                           bool ucb) {
  std::vector<macro_action_t> besta;
  double bestq = -Infinity;
  int N = data.V[Params.LocalReward].value.GetCount();

  for (macro_action_t action : mSubTasks[Action]) {
    if (IsTerminated(action, last_observation) ||
        !Applicable(last_observation, action) ||
        action.first == action.second) {
      continue;
    }

    int n = data.V[Params.LocalReward].qvalues[action].GetCount();
    double q = data.V[Params.LocalReward].qvalues[action].GetValue();

    if (ucb) {
      double exploration = GetExplorationConstant(Action);
      q += FastUCB(N, n, exploration);
      assert(n != 0 || q == Infinity);
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
 * @brief HierarchicalMCTS::IsTerminated
 * @param Action is the target macro state
 * @param history
 * @param state
 * @return
 */
bool HierarchicalMCTS::IsTerminated(macro_action_t Action, int last_observation) {
  if (mActionAbstraction) {
    if (Params.LocalReward) {
      if (Action == mRootTask) {
        return !IsPrimitive(Action) &&
               last_observation >= 0 &&
               Action.second == last_observation;
      }
      else {
        return !IsPrimitive(Action) &&
               last_observation >= 0 &&
               Action.first != last_observation;
      }
    }
    else {
      return !IsPrimitive(Action) &&
             last_observation >= 0 &&
             Action.second == last_observation;
    }
  }

  return false;
}

bool HierarchicalMCTS::IsGoal(macro_action_t Action, int last_observation) {
  if (mActionAbstraction) {
    return !IsPrimitive(Action) &&
           last_observation >= 0 &&
           Action.second == last_observation;
  }

  return false;
}

double HierarchicalMCTS::LocalReward(macro_action_t Action, int last_observation, int depth) {
  if (mActionAbstraction && Params.LocalReward) {
    if (depth >= Params.MaxDepth ||
      (IsTerminated(Action, last_observation) && !IsGoal(Action, last_observation))) {
      return -100.0;
    }
  }

  return 0.0;
}

bool HierarchicalMCTS::IsPrimitive(macro_action_t Action) {
  return Action.first == PRIMITIVE;
}

/**
 * @brief HierarchicalMCTS::MacroAction
 * @param o
 * @return the macro action with o as the targeting observation
 */
macro_action_t HierarchicalMCTS::MacroAction(int from, int to) {
  return make_pair(from, to);
}

macro_action_t HierarchicalMCTS::PrimitiveAction(int action) {
  return make_pair(PRIMITIVE, action);
}

double HierarchicalMCTS::GetExplorationConstant(macro_action_t Action)
{
  if (mActionAbstraction && Params.LocalReward) {
    if (Action == mRootTask || IsPrimitive(Action)) {
      return Simulator.GetRewardRange();
    }

    return Simulator.GetRewardRange() * 1.5;
  }

  return Simulator.GetRewardRange();
}
