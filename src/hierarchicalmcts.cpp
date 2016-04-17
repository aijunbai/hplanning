#include "hierarchicalmcts.h"
#include "coord.h"

using namespace std;

HierarchicalMCTS::HierarchicalMCTS(const SIMULATOR &simulator,
                                   const PARAMS &params, bool action_abstraction,
                                   int first_observation, STATE *ground_state)
  : MCTS(simulator, params, first_observation),
    mRootTask(ROOT, '0'),
    mActionAbstraction(action_abstraction) {
  mBelief.AddSample(Simulator.Copy(*ground_state));

  if (!mActionAbstraction) {
    for (int a = 0; a < Simulator.GetNumActions(); ++a) {
      mTaskGraph[mRootTask].insert(PrimitiveAction(a));
    }
  }
  else {
    ExploreOptions(1000, 5000);

    if (Params.Verbose >= 2) {
      PRINT_VALUE(mAvailableOptions);
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

bool HierarchicalMCTS::Applicable(int last_observation, option_t action) {
  if (last_observation >= 0 && !IsPrimitive(action) && action != mRootTask) {
    assert(mActionAbstraction);
    return action.first == last_observation &&
        mAvailableOptions[last_observation].count(action);
  }

  return true;
}

HierarchicalMCTS::data_t *HierarchicalMCTS::Query(option_t option,
                                                  size_t belief_hash) {
  if (mTree.count(option)) {
    if (mTree[option].count(belief_hash)) {
      return mTree[option][belief_hash];
    }
  }

  return 0;
}

HierarchicalMCTS::data_t *HierarchicalMCTS::Insert(option_t option,
                                                   size_t belief_hash) {
  assert(!Query(option, belief_hash));
  mTree[option][belief_hash] = new data_t();
  TreeSize += 1;
  return mTree[option][belief_hash];
}

void HierarchicalMCTS::Clear() {
  for (auto &n : mTree) {
    for (auto &e : n.second) {
      delete e.second;
    }
  }
  mTree.clear();
  mBelief.Free(Simulator);
}

bool HierarchicalMCTS::Update(int action, int observation, STATE &state) {
  AddOption(History.EndingObservation(), observation, &state);
  History.Add(action, observation);

  // Delete old tree and create new root
  Clear();
  STATE *sample = Simulator.Copy(state);
  mBelief.AddSample(sample);

  return true;
}

int HierarchicalMCTS::SelectAction() {
  input_t input(0, History.EndingObservation());

  if (mActionAbstraction) {
    ExploreOptions(100, 500);

    if (input.ending_observation == mRootTask.second) {  // enter target macro state XXX
      if (Params.Verbose >= 2) {
        cerr << "Cancelling action abstraction" << endl;
      }

      mTaskGraph.clear();
      for (int a = 0; a < Simulator.GetNumActions(); ++a) {
        mTaskGraph[mRootTask].insert(PrimitiveAction(a));
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
                IsTerminated(mCallStack.top(), input.ending_observation))) {
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
      if (input.ending_observation != -1) {
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

            option_t subtask = GreedyUCB(mCallStack.top(), input.ending_observation, *data, false);
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

  STATE *state = mBelief.CreateSample(Simulator);
  option_t action = GreedyPrimitiveAction(mCallStack.top(), input, *state);
  Simulator.FreeState(state);
  assert(IsPrimitive(action));

  return action.second;
}

option_t HierarchicalMCTS::SmartPrimitiveAction(
    option_t option, const input_t &input, STATE &state)
{
  if (IsPrimitive(option)) {
    return option;
  }

  option_t action = SmartSubtask(option, input, state);
  return SmartPrimitiveAction(action, input, state);
}

option_t HierarchicalMCTS::GetPrimitiveAction(
    option_t option, const input_t &input, STATE &state)
{
  if (Simulator.Knowledge.RolloutLevel >= SIMULATOR::KNOWLEDGE::SMART) {
    return SmartPrimitiveAction(option, input, state);
  }
  else {
    return RandomPrimitiveAction(option, input);
  }
}

option_t HierarchicalMCTS::GetSubtask(option_t option, const input_t &input, STATE &state)
{
  if (Simulator.Knowledge.RolloutLevel >= SIMULATOR::KNOWLEDGE::SMART) {
    return SmartSubtask(option, input, state);
  }
  else {
    return RandomSubtask(option, input);
  }
}

option_t HierarchicalMCTS::RandomOption(const input_t &input)
{
  assert(mActionAbstraction);

  option_t action = *next(begin(mAvailableOptions[input.ending_observation]),
      SimpleRNG::ins().Random(mAvailableOptions[input.ending_observation].size()));
  assert(!IsTerminated(action, input.ending_observation) &&
           Applicable(input.ending_observation, action));

  return action;
}

option_t HierarchicalMCTS::SmartSubtask(option_t option, const input_t &input, STATE &state)
{
  assert(!IsPrimitive(option));

  if (mActionAbstraction) {
    if (option == mRootTask) {
      return RandomOption(input);
    }
    else { // option
      STATE *exit = mExits[option]; assert(exit);
      return PrimitiveAction(Simulator.SuggestAction(state, *exit));
    }
  }

  return *next(begin(mTaskGraph[option]),
               SimpleRNG::ins().Random(mTaskGraph[option].size()));
}

option_t HierarchicalMCTS::RandomSubtask(option_t option, const input_t &input)
{
  assert(!IsPrimitive(option));

  if (mActionAbstraction) {
    if (option == mRootTask) {
      return RandomOption(input);
    }
  }

  return *next(begin(mTaskGraph[option]),
               SimpleRNG::ins().Random(mTaskGraph[option].size()));
}

option_t HierarchicalMCTS::RandomPrimitiveAction(
    option_t option, const input_t &input)
{
  if (IsPrimitive(option)) {
    return option;
  }

  option_t action = RandomSubtask(option, input);
  return RandomPrimitiveAction(action, input);
}

option_t HierarchicalMCTS::GreedyPrimitiveAction(
    option_t option, const input_t &input, STATE &state) {
  if (IsPrimitive(option)) {
    return option;
  }

  data_t *data = Query(option, input.belief_hash);

  if (data) {
    if (Params.Verbose >= 1) {
      stringstream ss;
      ss << "V(" << option << ", ";
      ss << "history)[" << Params.LocalReward << "]";
      data->V[Params.LocalReward].value.Print(ss.str(), cerr);
      for (auto ii = data->V[Params.LocalReward].qvalues.begin(); ii != data->V[Params.LocalReward].qvalues.end(); ++ii) {
        stringstream ss;
        ss << "Q(" << option << ", ";
        ss << "history, " << ii->first << ")[" << Params.LocalReward << "]";
        ii->second.Print(ss.str(), cerr);
      }
    }

    option_t action = GreedyUCB(option, input.ending_observation, *data, false);
    return GreedyPrimitiveAction(action, input, state);
  }
  else {
    if (Params.Verbose >= 1) {
      cerr << "Random Selecting V(" << option << ", ";
      cerr << "history)" << endl;
    }

    return GetPrimitiveAction(option, input, state);
  }
}

void HierarchicalMCTS::SearchImp() {
  assert(mBelief.GetNumSamples() == 1);
  int historyDepth = History.Size();

  STATE *state = mBelief.CreateSample(Simulator);
  Simulator.Validate(*state);

  input_t input(0, History.EndingObservation());
  SearchTree(mCallStack.top(), input, state, 0);

  Simulator.FreeState(state);
  assert(History.Size() == historyDepth);
  History.Truncate(historyDepth);
}


HierarchicalMCTS::result_t
HierarchicalMCTS::SearchTree(
    option_t option,
    const HierarchicalMCTS::input_t &input,
    STATE *&state, int depth)
{
  TreeDepth = max(TreeDepth, depth);

  if (Params.Verbose >= 3) {
    cerr << "SearchTree" << endl;
    PRINT_VALUE(option);
    PRINT_VALUE(depth);
    cerr << "state={\n";
    Simulator.DisplayState(*state, cerr);
    cerr << "}" << endl;
  }

  if (IsPrimitive(option)) {
    return Simulate(option, input, state, depth); // simulate primitive option
  } else {
    if (depth >= Params.MaxDepth || IsTerminated(option, input.ending_observation)) {
      return result_t(0.0, 0, false, input.belief_hash, input.ending_observation);
    }

    data_t *data = Query(option, input.belief_hash);

    if (!data) {
      Insert(option, input.belief_hash);
      return Rollout(option, input, state, depth);
    } else {
      const option_t action = GreedyUCB(option, input.ending_observation, *data, true);
      const result_t subtask = SearchTree(action, input, state, depth);
      int steps = subtask.steps;
      result_t completion(0.0, 0, false, subtask.belief_hash, subtask.ending_observation);
      if (!subtask.global_terminal) {
        input_t input(subtask.belief_hash, subtask.ending_observation);
        completion = SearchTree(option, input, state, depth + steps);
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
              LocalReward(option, completion.ending_observation, steps);
        }

        data->V[1].value.Add(totalReward + localReward);
        data->V[1].qvalues[action].Add(totalReward + localReward);
      }

      return result_t(totalReward, steps, subtask.global_terminal || completion.global_terminal,
                      completion.belief_hash, completion.ending_observation);
    }
  }
}

void HierarchicalMCTS::ExploreOptions(int iterations, int max_depth)
{
  if (mActionAbstraction) {
    for (int i = 0; i < iterations; ++i) {
      int ending_observation = History.EndingObservation();
      STATE *state = mBelief.CreateSample(Simulator);
      Simulator.Validate(*state);
      bool terminal = false;

      for (int step = 0; !terminal && step < max_depth; ++step) {
        int observation;
        double reward;
        int action = SimpleRNG::ins().Random(Simulator.GetNumActions());
        terminal = Simulator.Step(*state, action, observation, reward);
        AddOption(ending_observation, observation, state);
        ending_observation = observation;
      }

      Simulator.FreeState(state);
    }
  }
}

void HierarchicalMCTS::AddOption(int from, int to, STATE *state) {
  if (mActionAbstraction) {
    assert(from >= 0 && to >= 0);
    if (from >= 0 && to >= 0 && from != to) {
      option_t options[2] = {Option(from, to), Option(to, from)};

      for (auto option: options) {
        if (!mTaskGraph[mRootTask].count(option)) {
          mTaskGraph[mRootTask].insert(option);
          for (int a = 0; a < Simulator.GetNumActions(); ++a) {
            mTaskGraph[option].insert(PrimitiveAction(a));
          }
          mAvailableOptions[option.first].insert(option);
          mExits[option] = Simulator.Copy(*state);
        }
      }
    }
  }
}

HierarchicalMCTS::result_t
HierarchicalMCTS::PollingRollout(option_t option,
                                 const HierarchicalMCTS::input_t &input, STATE *&state,
                                 int depth) {
  if (IsPrimitive(option)) {
    return Simulate(option, input, state, depth);
  }
  else {
    if (depth >= Params.MaxDepth || IsTerminated(option, input.ending_observation)) {
      return result_t(0.0, 0, false, input.belief_hash, input.ending_observation);
    }

    option_t action = GetPrimitiveAction(option, input, *state);
    result_t atomic = Simulate(action, input, state, depth);
    assert(atomic.steps == 1);

    if (!atomic.global_terminal) {
      input_t input(atomic.belief_hash, atomic.ending_observation);
      result_t completion = PollingRollout(option, input, state, depth + 1);
      double totalReward = atomic.reward + Simulator.GetDiscount() * completion.reward;

      return result_t(totalReward,
                      atomic.steps + completion.steps,
                      completion.global_terminal,
                      completion.belief_hash,
                      completion.ending_observation);
    }

    return atomic;
  }
}

HierarchicalMCTS::result_t
HierarchicalMCTS::Simulate(option_t action, const input_t &input, STATE *&state, int depth) {
  assert(IsPrimitive(action));

  int observation;
  double immediateReward;
  bool terminal =
      Simulator.Step(*state, action.second, observation, immediateReward);
  AddOption(input.ending_observation, observation, state);

  size_t belief_hash = 0;
  if (Simulator.mStateAbstraction) { // whole history
    if (observation != input.ending_observation) { // entering into new room
      boost::hash_combine(belief_hash, input.ending_observation);
      boost::hash_combine(belief_hash, observation);
    }
    else {
      boost::hash_combine(belief_hash, input.belief_hash);
      boost::hash_combine(belief_hash, action);
      boost::hash_combine(belief_hash, observation);
    }
  }
  else { // memory size = 1
    boost::hash_combine(belief_hash, observation); // observation is the ground state
  }

  return result_t(immediateReward, 1, terminal, belief_hash, observation);
}

HierarchicalMCTS::result_t
HierarchicalMCTS::Rollout(option_t option,
                          const HierarchicalMCTS::input_t &input, STATE *&state,
                          int depth) {
  if (Params.Verbose >= 3) {
    cerr << "Rollout" << endl;
    PRINT_VALUE(option);
    PRINT_VALUE(depth);
    cerr << "state={\n";
    Simulator.DisplayState(*state, cerr);
    cerr << "}" << endl;
  }

  if (Params.Polling) {
    return PollingRollout(option, input, state, depth);
  }
  else {
    return HierarchicalRollout(option, input, state, depth);
  }
}

HierarchicalMCTS::result_t
HierarchicalMCTS::HierarchicalRollout(option_t option,
                                      const HierarchicalMCTS::input_t &input, STATE *&state,
                                      int depth) {
  if (IsPrimitive(option)) {
    return Simulate(option, input, state, depth);
  }
  else {
    if (depth >= Params.MaxDepth || IsTerminated(option, input.ending_observation)) {
      return result_t(0.0, 0, false, input.belief_hash, input.ending_observation);
    }

    option_t action = GetSubtask(option, input, *state);
    auto subtask = HierarchicalRollout(action, input, state, depth); // history and state will be updated
    int steps = subtask.steps;
    result_t completion(0.0, 0, false, subtask.belief_hash,
                        subtask.ending_observation);
    if (!subtask.global_terminal) {
      input_t input(subtask.belief_hash, subtask.ending_observation);
      completion = HierarchicalRollout(option, input, state, depth + steps);
    }

    double totalReward =
        subtask.reward +
        pow(Simulator.GetDiscount(), steps) * completion.reward;
    steps += completion.steps;

    return result_t(totalReward, steps, subtask.global_terminal || completion.global_terminal,
                    completion.belief_hash, completion.ending_observation);
  }
}

option_t HierarchicalMCTS::GreedyUCB(option_t option,
                                     int last_observation, data_t &data,
                                     bool ucb) {
  static std::vector<option_t> besta;
  besta.clear();

  double bestq = -Infinity;
  int N = data.V[Params.LocalReward].value.GetCount();

  std::unordered_set<option_t> &actions = (mActionAbstraction && option == mRootTask)?
        mAvailableOptions[last_observation]: mTaskGraph[option];

  for (option_t action : actions) {
    assert(!IsTerminated(action, last_observation) &&
             Applicable(last_observation, action));
    int n = data.V[Params.LocalReward].qvalues[action].GetCount();
    double q = data.V[Params.LocalReward].qvalues[action].GetValue();

    if (ucb) {
      double exploration = GetExplorationConstant(option);
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
 * @param history
 * @param state
 * @return
 */
bool HierarchicalMCTS::IsTerminated(option_t option, int last_observation) {
  if (mActionAbstraction) {
    if (Params.LocalReward) {
      if (option == mRootTask) {
        return !IsPrimitive(option) &&
            last_observation >= 0 &&
            option.second == last_observation;
      }
      else {
        return !IsPrimitive(option) &&
            last_observation >= 0 &&
            option.first != last_observation;
      }
    }
    else {
      return !IsPrimitive(option) &&
          last_observation >= 0 &&
          option.second == last_observation;
    }
  }

  return false;
}

bool HierarchicalMCTS::IsGoal(option_t option, int last_observation) {
  if (mActionAbstraction) {
    return !IsPrimitive(option) &&
        last_observation >= 0 &&
        option.second == last_observation;
  }

  return false;
}

double HierarchicalMCTS::LocalReward(option_t option, int last_observation, int depth) {
  if (mActionAbstraction && Params.LocalReward) {
    if (depth >= Params.MaxDepth ||
        (IsTerminated(option, last_observation) && !IsGoal(option, last_observation))) {
      return -100.0;
    }
  }

  return 0.0;
}

bool HierarchicalMCTS::IsPrimitive(option_t option) {
  return option.first == PRIMITIVE;
}

/**
 * @brief HierarchicalMCTS::MacroAction
 * @param o
 * @return the macro action with o as the targeting observation
 */
option_t HierarchicalMCTS::Option(int from, int to) {
  return make_pair(from, to);
}

option_t HierarchicalMCTS::PrimitiveAction(int action) {
  return make_pair(PRIMITIVE, action);
}

double HierarchicalMCTS::GetExplorationConstant(option_t option)
{
  if (mActionAbstraction && Params.LocalReward) {
    if (option == mRootTask || IsPrimitive(option)) {
      return Simulator.GetRewardRange();
    }

    return Simulator.GetRewardRange() * 1.5;
  }

  return Simulator.GetRewardRange();
}
