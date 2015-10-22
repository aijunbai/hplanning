#include "hierarchicalmcts.h"
#include <sstream>

using namespace std;

HierarchicalMCTS::HierarchicalMCTS(const SIMULATOR &simulator,
                                   const PARAMS &params)
    : MetaMCTS(simulator, params) {
  for (int a = 0; a < Simulator.GetNumActions(); ++a) {
    mSubTasks[a] = vector<MacroAction>();
  }

  if (Simulator.mStateAbstraction) {
    for (int o = 0; o < Simulator.GetNumObservations(); ++o) {
      mSubTasks[Simulator.GetNumActions() + o] = vector<MacroAction>();
      for (int a = 0; a < Simulator.GetNumActions(); ++a) {
        mSubTasks[Simulator.GetNumActions() + o].push_back(a);
      }
    }

    mSubTasks[-1] = vector<MacroAction>();
    for (int o = 0; o < Simulator.GetNumObservations(); ++o) {
      mSubTasks[-1].push_back(Simulator.GetNumActions() + o);
    }
  } else {
    mSubTasks[-1] = vector<MacroAction>();
    for (int a = 0; a < Simulator.GetNumActions(); ++a) {
      mSubTasks[-1].push_back(a);
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
  return SelectPrimitiveAction(vector<MacroAction>(1, -1), History);
}

int HierarchicalMCTS::SelectPrimitiveAction(std::vector<MacroAction> stack,
                                            HISTORY &history) {
  MacroAction Action = stack.back();
  if (mSubTasks[Action].empty()) {
    return Action;
  }

  size_t hash = hash_value(stack, history);
  auto it = mTable.find(hash);
  assert(it != mTable.end());

  if (Params.Verbose >= 1) {
//    cerr << "history=[";
//    history.Display(cerr);
//    cerr << "]" << endl;
    stringstream ss;
    ss << "V(";
    for (uint i = 0; i < stack.size(); ++i) {
      ss << stack[i] << ", ";
    }
    ss << "history)";
    it->second.value_.Print(ss.str(), cerr);
    for (auto ii = it->second.qvalues_.begin(); ii != it->second.qvalues_.end(); ++ii) {
      stringstream ss;
      ss << "Q(";
      for (uint i = 0; i < stack.size(); ++i) {
        ss << stack[i] << ", ";
      }
      ss << "history, " << ii->first << ")";
      ii->second.Print(ss.str(), cerr);
    }
  }

  MacroAction action = GreedyUCB(Action, it->second, history, false);
  stack.push_back(action);
  return SelectPrimitiveAction(stack, history);
}

void HierarchicalMCTS::SearchImp() {
  int historyDepth = History.Size();

  STATE *state = mRootBelief.CreateSample(Simulator);
  Simulator.Validate(*state);

  SearchTree(vector<MacroAction>(1, -1), History, *state, 0);

  Simulator.FreeState(state);
  History.Truncate(historyDepth);
}

double HierarchicalMCTS::SearchTree(vector<MacroAction> stack, HISTORY &history,
                                    STATE &state, int depth) {
  MacroAction Action = stack.back();

  cerr << "SearchTree" << endl;
  PRINT_VALUE(Action);
  PRINT_VALUE(depth);
  cerr << "history=[";
  history.Display(cerr);
  cerr << "]" << endl;

  if (depth >= Params.MaxDepth) { // search horizon reached
    return 0.0;
  }

  if (Terminate(Action, history)) {
    stack.pop_back();
    return SearchTree(stack, history, state, depth);
  }

  if (mSubTasks[Action].empty()) { // primitive
    int observation;
    double immediateReward;
    bool terminal =
        Simulator.Step(state, Action, observation, immediateReward); //一步模拟

    if (terminal) {
      return immediateReward;
    } else {
      stack.pop_back();
      history.Add(Action, observation, Params.MemorySize);
      double delayedReward = SearchTree(stack, history, state, depth + 1);
      return immediateReward + Simulator.GetDiscount() * delayedReward;
    }
  } else { // macro action
    size_t hash = hash_value(stack, history);
    auto it = mTable.find(hash);

    if (it == mTable.end()) {
      mTable.insert(make_pair(hash, data_t()));
      auto it = mTable.find(hash);
      assert(it != mTable.end());
      STATE *copy = Simulator.Copy(state);
      double totalReward = Rollout(stack, history, *copy, depth);
      it->second.value_.Add(totalReward);
      Simulator.FreeState(copy);
      return totalReward;
    } else {
      int action = GreedyUCB(Action, it->second, history, true);
      stack.push_back(action);
      double totalReward = SearchTree(stack, history, state, depth);
      it->second.value_.Add(totalReward);
      it->second.qvalues_[action].Add(totalReward);
      return totalReward;
    }
  }
}

double HierarchicalMCTS::Rollout(std::vector<MacroAction> stack,
                                 HISTORY &history, STATE &state, int depth) {
  MacroAction Action = stack.back();

  if (depth >= Params.MaxDepth) { // search horizon reached
    return 0.0;
  }

  if (Terminate(Action, history)) { // XXX
    stack.pop_back();
    return Rollout(stack, history, state, depth);
  }

  if (mSubTasks[Action].empty()) { // primitive
    int observation;
    double immediateReward;
    bool terminal =
        Simulator.Step(state, Action, observation, immediateReward); //一步模拟

    if (terminal) {
      return immediateReward;
    } else {
      stack.pop_back();
      history.Add(Action, observation, Params.MemorySize);
      double delayedReward = Rollout(stack, history, state, depth + 1);
      return immediateReward + Simulator.GetDiscount() * delayedReward;
    }
  } else { // macro action
    int action = -1;
    do {
      action = SimpleRNG::ins().Sample(mSubTasks[Action]);
    } while (Terminate(action, history));  // XXX

    stack.push_back(action);
    return Rollout(stack, history, state, depth);
  }
}

MacroAction HierarchicalMCTS::GreedyUCB(MacroAction Action, data_t &data,
                                        HISTORY &history, bool ucb) {
  static std::vector<int> besta;
  besta.clear();
  double bestq = -Infinity;
  int N = data.value_.GetCount();

  for (uint i = 0; i < mSubTasks[Action].size(); ++i) {
    int action = mSubTasks[Action][i];

    if (Terminate(action, history)) {
      assert(0);
      continue;
    }

    int n = data.qvalues_[action].GetCount();
    double q = data.qvalues_[action].GetValue();

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

bool HierarchicalMCTS::Terminate(MacroAction A, HISTORY &h) {
  return A >= Simulator.GetNumActions() && h.Size() &&
         h.Back().Observation == A - Simulator.GetNumActions(); // XXX
}
