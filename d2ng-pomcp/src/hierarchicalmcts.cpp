#include "hierarchicalmcts.h"
#include "coord.h"
#include "rooms.h"

using namespace std;

std::unordered_map<std::size_t, HierarchicalMCTS::belief_t>
    HierarchicalMCTS::data_t::beliefpool;
STATISTIC HierarchicalMCTS::mCacheRate;
STATISTIC HierarchicalMCTS::mCacheDepth;
STATISTIC HierarchicalMCTS::mCacheStep;

HierarchicalMCTS::HierarchicalMCTS(const SIMULATOR &simulator,
                                   const PARAMS &params)
    : MCTS(simulator, params), mRootTask(-1) {
  mSubTasks[mRootTask] = vector<macro_action_t>();  // root action

  for (int a = 0; a < Simulator.GetNumActions(); ++a) {
    mSubTasks[a] = vector<macro_action_t>();  // primitive actions
  }

  if (Simulator.mActionAbstraction) {
    assert(Simulator.GetNumObservations() > 0);
    mGoals[mRootTask].insert(0);  // goal for root task

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

  mCallStack.push(mRootTask);
}

HierarchicalMCTS::~HierarchicalMCTS() {
  if (Params.Verbose >= 2) {
    PRINT_VALUE(Params.ExplorationConstant);
    PRINT_VALUE(mCacheRate);
    PRINT_VALUE(mCacheDepth);
    PRINT_VALUE(mCacheStep);
  }
  Clear();
}

void HierarchicalMCTS::data_t::clear(const SIMULATOR &simulator) {

  for (auto &d : data_t::beliefpool) {
    d.second.clear(simulator);
  }
  data_t::beliefpool.clear();
}

HierarchicalMCTS::bound_t HierarchicalMCTS::data_t::ucb_bound(macro_action_t a,
                                                              const MCTS *mcts) {
  int N = value.GetCount();
  int n = qvalues[a].GetCount();
  double q = qvalues[a].GetValue();
  double bound = mcts->FastUCB(N, n);

  return bound_t(q - bound, q + bound);
}

/**
 * @brief HierarchicalMCTS::data_t::optimal_prob_at_least
 * @param a
 * @param mcts
 * @param N
 * @param threshold
 * @return probability of a being optimal at least threshold
 */
bool HierarchicalMCTS::data_t::optimal_prob_at_least(macro_action_t a,
                                                     const MCTS *mcts, int N,
                                                     double threshold) {
  if (threshold <= 0.0) {
    return true;
  }

  bound_t bounda = ucb_bound(a, mcts);
  vector<bound_t> bounds;
  for (auto &e : qvalues) {
    if (e.first != a) {
      bounds.push_back(ucb_bound(e.first, mcts));
    }
  }

  if (qvalues.size() == 2) {
    assert(bounds.size() == 1);
    return greater_prob(bounds[0].lower, bounds[0].upper, bounda.lower,
                        bounda.upper) >= threshold;
  }

  int n = 0;
  for (int i = 0; i < N; ++i) {
    if (double(n + N - i) / double(N) < threshold) {
      return false;
    }

    if (double(n) / double(N) >= threshold) {
      return true;
    }

    double q = bounda.sample();
    bool optimal = true;
    for (auto &e : bounds) {
      double sample = e.sample();
      if (sample > q) {
        optimal = false;
        break;
      }
    }
    if (optimal) {
      n += 1;
    }
  }

  return double(n) / double(N) >= threshold;
}

void HierarchicalMCTS::UnitTest() {
  assert(data_t::greater_prob(1, 2, 3, 4) == 1.0);
  assert(data_t::greater_prob(1, 4, 3, 4) == 2.5 / 3.0);
  assert(data_t::greater_prob(1, 5, 3, 4) == 2.5 / 4.0);
  assert(data_t::greater_prob(3, 5, 3, 4) == 0.5 / 2.0);
  assert(data_t::greater_prob(5, 6, 3, 4) == 0.0);
}

/**
 * @brief HierarchicalMCTS::data_t::greater_prob
 * @param x1: x \in [x1, x2]
 * @param x2: x \in [x1, x2]
 * @param y1: y \in [y1, y2]
 * @param y2: y \in [y1, y2]
 * @return probability that y >= x
 */
double HierarchicalMCTS::data_t::greater_prob(double x1, double x2, double y1,
                                              double y2) {
  assert(x1 <= x2 && y1 <= y2);

  if (y2 > x1) {
    double area = 0.5 * Sqr(y2 - x1);

    if (y2 > x2) {
      area -= 0.5 * Sqr(y2 - x2);
    }
    if (y1 > x1) {
      area -= 0.5 * Sqr(y1 - x1);
    }
    if (y1 > x2) {
      area += 0.5 * Sqr(y1 - x2);
    }

    double prob = area / (x2 - x1) / (y2 - y1);
    return MinMax(0.0, prob, 1.0);
  } else {
    return 0.0;
  }
}

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

HierarchicalMCTS::data_t *HierarchicalMCTS::Insert(macro_action_t Action,
                                                   size_t belief_hash) {
  data_t *data = Query(Action, belief_hash);
  if (!data) {
    data = new data_t();
    mTree[Action][belief_hash] = data;
    TreeSize += 1;
  }
  return data;
}

void HierarchicalMCTS::Clear() {
  for (auto &n : mTree) {
    for (auto &e : n.second) {
      delete e.second;
    }
  }
  mTree.clear();
  mRootSampling.Free(Simulator);
  data_t::clear(Simulator);
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
  input_t input(History.BeliefHash(), History.LastObservation());

  if (Simulator.mActionAbstraction) {
    if (input.last_observation == 0) {  // enter target macro state
      if (Params.Verbose >= 2) {
        cerr << "Cancelling action abstraction" << endl;
      }

      mSubTasks[mRootTask].clear();
      for (int a = 0; a < Simulator.GetNumActions(); ++a) {
        mSubTasks[mRootTask].push_back(a);
      }

      for (auto it = mGoals.begin(); it != mGoals.end(); ++it) {
        it->second.erase(0);
      }

      while (mCallStack.size()) {
        mCallStack.pop();
      }
      mCallStack.push(mRootTask);
    }
    else {
      if (Params.Stack) {
        while (mCallStack.size() &&
              (Terminate(mCallStack.top(), input.last_observation) ||
              Primitive(mCallStack.top()))) {
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

  if (Simulator.mActionAbstraction) {
    if (Params.Stack) {
      if (input.last_observation != -1) {
        while (!Primitive(mCallStack.top())) {
          data_t *data = Query(mCallStack.top(), input.belief_hash);
          if (data) {
            int subtask = GreedyUCB(mCallStack.top(), input.last_observation, *data, false);
            mCallStack.push(subtask);
          }
          else {
            break;
          }
        }
      }
    }
  }

  if (Params.Verbose >= 2) {
    cerr << "Executing task " << mCallStack.top() << endl;
  }
  int action = GreedyPrimitiveAction(mCallStack.top(), input);
  return action;
}

int HierarchicalMCTS::RandomPrimitiveAction(macro_action_t Action,
                                            const input_t &input)
{
  if (Primitive(Action)) {
    return Action;
  }

  macro_action_t action;
  do {
    action = SimpleRNG::ins().Sample(mSubTasks[Action]);
  } while (Terminate(action, input.last_observation) ||
           !Applicable(input.last_observation, action));

  return RandomPrimitiveAction(action, input);
}

int HierarchicalMCTS::GreedyPrimitiveAction(macro_action_t Action,
                                            const input_t &input) {
  if (Primitive(Action)) {
    return Action;
  }

  data_t *data = Query(Action, input.belief_hash);
  macro_action_t action;

  if (data) {
    if (Params.Verbose >= 1) {
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

    action = GreedyUCB(Action, input.last_observation, *data, false);
    return GreedyPrimitiveAction(action, input);
  } else {
    if (Params.Verbose >= 1) {
      cerr << "Random Selecting V(" << Action << ", ";
      cerr << "history)" << endl;
      }

    return RandomPrimitiveAction(Action, input);
  }
}

void HierarchicalMCTS::SearchImp() {
  int historyDepth = History.Size();

  STATE *state = mRootSampling.CreateSample(Simulator);
  Simulator.Validate(*state);

  input_t input(History.BeliefHash(), History.LastObservation());
  SearchTree(mCallStack.top(), input, state, 0);

  Simulator.FreeState(state);
  assert(History.Size() == historyDepth);
  History.Truncate(historyDepth);
}


HierarchicalMCTS::result_t
HierarchicalMCTS::SearchTree(macro_action_t Action,
                             const HierarchicalMCTS::input_t &input,
                             STATE *&state, int depth) {
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
    return Rollout(Action, input, state, depth); // simulate primitive Action
  } else {
    if (depth >= Params.MaxDepth || Terminate(Action, input.last_observation)) {
      return result_t(0.0, 0, false, input.belief_hash, input.last_observation);
    }

    data_t *data = Query(Action, input.belief_hash);

    if (!data) {
      Insert(Action, input.belief_hash);
      return Rollout(Action, input, state, depth);
    } else {
      bool converged = false;

      if (Simulator.mActionAbstraction &&
          data->value.GetCount() > int(data->qvalues.size()) &&
          Params.Converged < 1.0) {
        int greedy = GreedyUCB(Action, input.last_observation, *data, false);

        if (data->optimal_prob_at_least(greedy, this, 33, Params.Converged)) {
          converged = true;

          if (data->cache.size() &&
              SimpleRNG::ins().Bernoulli(Params.CacheRate)) {
            result_t cache =
                SimpleRNG::ins().Sample(data->cache); // cached result
            Simulator.FreeState(state);               // drop current state
            state = data_t::beliefpool[cache.belief_hash].sample(
                Simulator); // sample an exit state
            mCacheRate.Add(1.0);
            mCacheDepth.Add(depth);
            mCacheStep.Add(cache.steps);
            return cache;
          }
        }
      }
      mCacheRate.Add(0.0);

      const int action = GreedyUCB(Action, input.last_observation, *data, true);
      const result_t subtask = SearchTree(action, input, state,
                                          depth); // history and state will be updated
      int steps = subtask.steps;
      result_t completion(0.0, 0, false, subtask.belief_hash,
                          subtask.last_observation);
      if (!subtask.terminal) {
        input_t input(subtask.belief_hash, subtask.last_observation);
        completion = SearchTree(Action, input, state, depth + steps);
      }
      const double totalReward =
          subtask.reward +
          pow(Simulator.GetDiscount(), steps) * completion.reward;
        data->value.Add(totalReward);
        data->qvalues[action].Add(totalReward);
      steps += completion.steps;
      const result_t ret(totalReward, steps, subtask.terminal || completion.terminal,
                         completion.belief_hash, completion.last_observation);

      if (Simulator.mActionAbstraction && converged) {
        if (ret.terminal ||
            Terminate(Action, ret.last_observation)) { // truly an exit
          data->cache.push_back(ret);
          data_t::beliefpool[completion.belief_hash].add_sample(
              *state, Simulator); // terminal state
        }
      }

      return ret;
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


HierarchicalMCTS::result_t
HierarchicalMCTS::PollingRollout(macro_action_t Action,
                          const HierarchicalMCTS::input_t &input, STATE *&state,
                          int depth)
{
  assert(!Primitive(Action));

  if (depth >= Params.MaxDepth || Terminate(Action, input.last_observation)) {
    return result_t(0.0, 0, false, input.belief_hash, input.last_observation);
  }

  macro_action_t action = RandomPrimitiveAction(Action, input);
  int observation;
  double immediateReward;
  bool terminal =
      Simulator.Step(*state, action, observation, immediateReward);
  UpdateConnection(input.last_observation, observation);

  size_t belief_hash = 0;
  if (Simulator.mStateAbstraction) { // whole history
    belief_hash = input.belief_hash;
    boost::hash_combine(belief_hash, action);
    boost::hash_combine(belief_hash, observation);
  } else { // memory size = 1
    boost::hash_combine(belief_hash,
                        observation); // observation is the ground state
    boost::hash_combine(belief_hash, depth);
  }

  if (!terminal) {
    input_t input(belief_hash, observation);
    result_t completion = Rollout(Action, input, state, depth+1);
    double totalReward = immediateReward + Simulator.GetDiscount() * completion.reward;
    return result_t(totalReward, 1 + completion.steps, completion.terminal, completion.belief_hash, completion.last_observation);
  }

  return result_t(immediateReward, 1, terminal, belief_hash, observation);
}

HierarchicalMCTS::result_t
HierarchicalMCTS::Rollout(macro_action_t Action,
                                      const HierarchicalMCTS::input_t &input, STATE *&state,
                                      int depth)
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
    bool terminal =
        Simulator.Step(*state, Action, observation, immediateReward);
    UpdateConnection(input.last_observation, observation);

    size_t belief_hash = 0;
    if (Simulator.mStateAbstraction) { // whole history
      belief_hash = input.belief_hash;
      boost::hash_combine(belief_hash, Action);
      boost::hash_combine(belief_hash, observation);
    } else { // memory size = 1
      boost::hash_combine(belief_hash,
                          observation); // observation is the ground state
      boost::hash_combine(belief_hash, depth);
    }
    return result_t(immediateReward, 1, terminal, belief_hash, observation);
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
                          int depth)
{
  assert(!Primitive(Action));

  if (depth >= Params.MaxDepth || Terminate(Action, input.last_observation)) {
    return result_t(0.0, 0, false, input.belief_hash, input.last_observation);
  }

  int action;
  do {
    action = SimpleRNG::ins().Sample(mSubTasks[Action]);
  } while (Terminate(action, input.last_observation) ||
           !Applicable(input.last_observation, action));

  auto subtask = Rollout(action, input, state,
                                     depth); // history and state will be updated
  int steps = subtask.steps;
  result_t completion(0.0, 0, false, subtask.belief_hash,
                      subtask.last_observation);
  if (!subtask.terminal) {
    input_t input(subtask.belief_hash, subtask.last_observation);
    completion = Rollout(Action, input, state, depth + steps);
  }

  double totalReward =
      subtask.reward +
      pow(Simulator.GetDiscount(), steps) * completion.reward;
  steps += completion.steps;
  return result_t(totalReward, steps, subtask.terminal || completion.terminal,
                  completion.belief_hash, completion.last_observation);
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
