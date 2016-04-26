#include "flatmcts.h"
#include "testsimulator.h"

using namespace std;
using namespace utils;

FlatMCTS::FlatMCTS(const SIMULATOR &simulator, const PARAMS &params, int first_observation)
    : MCTS(simulator, params, first_observation) {
  STATE *state = Simulator.CreateStartState();  

  Root = ExpandNode(state, History);  
  Root->Beliefs().AddSample(state);

  for (int i = 1; i < Params.NumStartStates; i++) {
    Root->Beliefs().AddSample(
        Simulator.CreateStartState());
  }
  if (Params.Verbose >= 1) Simulator.DisplayBeliefs(Root->Beliefs(), cout);

  assert(VNODE::GetNumAllocated() == 1);
}

FlatMCTS::~FlatMCTS() {
  VNODE::Free(Root, Simulator);
  VNODE::FreeAll();

  assert(VNODE::GetNumAllocated() == 0);
}

bool FlatMCTS::Update(int action, int observation, STATE &state) {
  History.Add(action, observation); 

  if (Simulator.mFullyObservable) {  // running an MDP in fact in cases of hplanning
    // Delete old tree and create new root
    VNODE::Free(Root, Simulator);
    Root = ExpandNode(&state, History);

    STATE *sample = Simulator.Copy(state);
    Root->Beliefs().AddSample(sample);
    if (Params.Verbose >= 1) Simulator.DisplayBeliefs(Root->Beliefs(), cout);

    return true;
  }
  else {
    BELIEF_STATE beliefs;

    // Find matching vnode from the rest of the tree
    QNODE &qnode = Root->Child(action);
    VNODE *vnode = qnode.Child(observation);

    if (vnode) {
      if (Params.Verbose >= 1)
        cout << "Matched " << vnode->Beliefs().GetNumSamples() << " states"
        << endl;
      beliefs.Copy(vnode->Beliefs(), Simulator); 
    } else {
      if (Params.Verbose >= 1) cout << "No matching node found" << endl;
    }

    if (Params.Verbose >= 1) Simulator.DisplayBeliefs(beliefs, cout);

    if (Params.UseParticleFilter) {  
      ParticleFilter(beliefs);

      if (Params.Verbose >= 1) Simulator.DisplayBeliefs(beliefs, cout);
    }

    // Generate transformed states to avoid particle deprivation
    if (Params.UseTransforms) {
      AddTransforms(beliefs);  

      if (Params.Verbose >= 1) Simulator.DisplayBeliefs(beliefs, cout);
    }

    // If we still have no particles, fail
    if (beliefs.Empty() && (!vnode || vnode->Beliefs().Empty()))
      return false;  

    // Find a state to initialise prior (only requires fully observed state)
    const STATE *sample = 0;
    if (vnode && !vnode->Beliefs().Empty())
      sample = vnode->Beliefs().GetSample();  
    else
      sample = beliefs.GetSample();

    if (vnode && Params.ReuseTree) {
      int size1 = VNODE::GetNumAllocated();
      VNODE::Free(Root, Simulator, vnode);
      int size2 = VNODE::GetNumAllocated();

      assert(size2 < size1);

      Root = vnode;
      Root->Beliefs().Free(Simulator);
    } else { // Delete old tree and create new root
      VNODE::Free(Root, Simulator);
      Root = ExpandNode(sample, History);
    }

    Root->Beliefs() = beliefs;  

    return true;
  }
}

int FlatMCTS::SelectAction() {
  Search();
  int action = Params.ThompsonSampling ? ThompsonSampling(Root, false, 0) : GreedyUCB(Root, false);

  if (Params.Verbose >= 1) {
    DisplayValue(1, cerr);
  }

  return action;
}

void FlatMCTS::SearchImp() {
  int historyDepth = History.Size();

  STATE *state = Root->Beliefs().CreateSample(Simulator);  
  Simulator.Validate(*state);

  SimulateV(*state, Root, 0);  
  if (Params.Verbose >= 3) DisplayValue(4, cout);

  Simulator.FreeState(state);
  History.Truncate(historyDepth);
}

int FlatMCTS::GreedyUCB(VNODE *vnode, bool ucb) const //argmax_a {Q[a]}
{
  static std::vector<int> besta;
  besta.clear();
  double bestq = -Infinity;
  int N = vnode->UCB.Value.GetCount();

  for (int action = 0; action < Simulator.GetNumActions(); action++) {
    double q;
    int n;

    QNODE &qnode = vnode->Child(action);

    if (!qnode.Applicable()) {  
      continue;
    }

    q = qnode.UCB.Value.GetValue();
    n = qnode.UCB.Value.GetCount();

    if (ucb) {
      q += FastUCB(N, n, Simulator.GetRewardRange());
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

double FlatMCTS::SimulateV(STATE &state, VNODE *vnode, int depth) {
  int action = Params.ThompsonSampling ? ThompsonSampling(vnode, true, depth) : GreedyUCB(vnode, true);

  TreeDepth = max(TreeDepth, depth);
  TreeSize += 1;

  if (depth >= Params.MaxDepth) {  // search horizon reached
    return 0.0;
  }

  if (depth >= 1) {
    AddSample(vnode, state);  
  }

  QNODE &qnode = vnode->Child(action);
  double totalReward = SimulateQ(state, qnode, action, depth);  

  if (Params.ThompsonSampling) {
    vnode->GetCumulativeReward(state).Add(totalReward);
  }
  else {
    vnode->UCB.Value.Add(totalReward);
  }

  return totalReward;  
}

double FlatMCTS::SimulateQ(STATE &state, QNODE &qnode, int action, int depth) {
  int observation;
  double immediateReward;
  double delayedReward = 0.0;

  bool terminal = Simulator.Step(state, action, observation, immediateReward);  
  if (Params.ThompsonSampling) {
    qnode.Update(observation, immediateReward, 1);  
  }

  History.Add(action, observation);

  if (Params.Verbose >= 3) {
    Simulator.DisplayAction(action, cout);
    Simulator.DisplayObservation(state, observation, cout);
    Simulator.DisplayReward(immediateReward, cout);
    Simulator.DisplayState(state, cout);
  }

  VNODE *&vnode = qnode.Child(observation);

  if (!terminal) {
    if (vnode) {  
      delayedReward = SimulateV(state, vnode, depth + 1);
    } else {                       
      vnode = ExpandNode(&state, History);  

      STATE *copy = Simulator.Copy(state);
      delayedReward = Rollout(*copy, depth + 1);
      Simulator.FreeState(copy);

      if (Params.ThompsonSampling) {
        vnode->GetCumulativeReward(state).Add(delayedReward);
      }
      else {
        vnode->UCB.Value.Add(delayedReward);
      }
    }
  } else {
    if (!vnode) {
      vnode = ExpandNode(&state, History);  
    }

    if (Params.ThompsonSampling) {
      vnode->GetCumulativeReward(state).Add(0.0);
    }
    else {
      vnode->UCB.Value.Add(0.0);
    }
  }

  double totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;
  if (!Params.ThompsonSampling) {
    qnode.UCB.Value.Add(totalReward);
  }

  return totalReward;
}

VNODE *FlatMCTS::ExpandNode(const STATE *state, HISTORY &history) {
  VNODE *vnode = VNODE::Create(/*history*/);
  vnode->UCB.Value.Set(0, 0);
  Simulator.Prior(state, history, vnode);  
  return vnode;
}

void FlatMCTS::AddSample(VNODE *node, const STATE &state) {
  STATE *sample = Simulator.Copy(state);
  node->Beliefs().AddSample(sample);
}

int FlatMCTS::ThompsonSampling(VNODE *vnode, bool sampling, int depth) const {
  vector<int> unexplored_actions;

  for (int action = 0; action < Simulator.GetNumActions(); action++) {
    QNODE &qnode = vnode->Child(action);

    if (!qnode.Applicable()) {  
      continue;
    }

    if (qnode.TS.UpdateCount <= 0) {
      unexplored_actions.push_back(action);
    }
  }

  if (!unexplored_actions.empty()) {
    return SimpleRNG::ins().Sample(unexplored_actions);
  }

  int besta = -1;
  double bestq = -Infinity;

  for (int action = 0; action < Simulator.GetNumActions(); action++) {
    QNODE &qnode = vnode->Child(action);

    if (!qnode.Applicable()) {  
      continue;
    }

    double q = QValue(qnode, sampling, depth);

    if (q > bestq)  // XXX
    {
      bestq = q;
      besta = action;
    }
  }

  assert(besta != -1);
  return besta;
}

double FlatMCTS::HValue(VNODE *vnode, bool sampling, int depth) const {
  if (vnode) {  
    return vnode->/*GetCumulativeReward().*/ ThompsonSampling(sampling);  // XXX
  } else if (depth + 1 >= Params.MaxDepth) {  // search horizon reached
    return 0.0;
  }

  return NormalGammaInfo().ThompsonSampling(sampling);  
}

double FlatMCTS::QValue(QNODE &qnode, bool sampling, int depth) const  
{
  double qvalue = 0;

  {
    const std::vector<std::pair<int, double>> &observations =
        qnode.TS.Observation.ThompsonSampling(sampling);  
    for (std::vector<std::pair<int, double>>::const_iterator it =
        observations.begin();
         it != observations.end(); ++it) {
      qvalue += it->second * HValue(qnode.Child(it->first), sampling, depth);
    }
  }

  qvalue *= Simulator.GetDiscount();

  {
    const std::vector<std::pair<double, double>> &rewards =
        qnode.TS.ImmediateReward.ThompsonSampling(sampling);  
    for (std::vector<std::pair<double, double>>::const_iterator it =
        rewards.begin();
         it != rewards.end(); ++it) {
      qvalue += it->second * it->first;
    }
  }

  return qvalue;
}

double FlatMCTS::Rollout(STATE &state, int depth)  
{
  if (Params.Verbose >= 3) cout << "Starting rollout" << endl;

  double totalReward = 0.0;
  double discount = 1.0;
  bool terminal = false;
  int numSteps;
  for (numSteps = 0; numSteps + depth < Params.MaxDepth && !terminal; ++numSteps) {
    int observation;
    double reward;

    int action = Simulator.SelectRandom(state, History);  
    terminal = Simulator.Step(state, action, observation, reward);  
    History.Add(action, observation);

    if (Params.Verbose >= 4) {
      Simulator.DisplayAction(action, cout);
      Simulator.DisplayObservation(state, observation, cout);
      Simulator.DisplayReward(reward, cout);
      Simulator.DisplayState(state, cout);
    }

    totalReward += reward * discount;
    discount *= Simulator.GetDiscount();
  }

  if (Params.Verbose >= 3)
    cout << "Ending rollout after " << numSteps << " steps, with total reward "
    << totalReward << endl;
  return totalReward;
}

void FlatMCTS::ParticleFilter(BELIEF_STATE &beliefs)  // unweighted particle filter
{
  int attempts = 0, added = 0;
  int max_attempts = (Params.NumStartStates - beliefs.GetNumSamples()) * 10;

  int realObs = History.Back().Observation;
  int stepObs;
  double stepReward;

  if (Params.Verbose >= 1) {
    cout << "MCTS::ParticleFilter: last step belief size "
    << Root->Beliefs().GetNumSamples() << ", current belief size "
    << beliefs.GetNumSamples() << endl;
  }

  while (beliefs.GetNumSamples() < Params.NumStartStates &&
         attempts < max_attempts) {
    STATE *state = Root->Beliefs().CreateSample(Simulator);
    Simulator.Step(*state, History.Back().Action, stepObs, stepReward);
    if (Params.ThompsonSampling) {
      Root->Child(History.Back().Action).Update(stepObs, stepReward);
    }

    if (stepObs == realObs) {
      beliefs.AddSample(state);
      added++;
    } else {
      Simulator.FreeState(state);
    }
    attempts++;
  }

  if (Params.Verbose >= 1) {
    cout << "MCTS::ParticleFilter: Created " << added
    << " local transformations out of " << attempts << " attempts" << endl;
  }
}

void FlatMCTS::AddTransforms(BELIEF_STATE &beliefs) {
  int attempts = 0, added = 0;

  if (Params.Verbose >= 1) {
    cout << "MCTS::AddTransforms: last step belief size "
    << Root->Beliefs().GetNumSamples() << ", current belief size "
    << beliefs.GetNumSamples() << endl;
  }

  // Local transformations of state that are consistent with history
  while (added < Params.NumTransforms && attempts < Params.MaxAttempts) {
    STATE *transform = CreateTransform();
    if (transform) {
      beliefs.AddSample(transform);
      added++;
    }
    attempts++;
  }

  if (Params.Verbose >= 1) {
    cout << "MCTS::AddTransforms: Created " << added
    << " local transformations out of " << attempts << " attempts" << endl;
  }
}

STATE *FlatMCTS::CreateTransform() const {
  int stepObs;
  double stepReward;

  STATE *state = Root->Beliefs().CreateSample(Simulator);
  Simulator.Step(*state, History.Back().Action, stepObs, stepReward);
  if (Params.ThompsonSampling) {
    Root->Child(History.Back().Action).Update(stepObs, stepReward);
  }

  if (Simulator.LocalMove(*state, History, stepObs)) return state;

  Simulator.FreeState(state);
  return 0;
}

void FlatMCTS::DisplayValue(int depth, ostream &ostr) const {
  HISTORY history(0);
  ostr << "MCTS Values:" << endl;

  std::vector<double> qvalues(Simulator.GetNumActions());
  for (int action = 0; action < Simulator.GetNumActions(); action++) {
    QNODE &qnode = Root->Child(action);

    if (qnode.Applicable()) {
      qvalues[action] = Params.ThompsonSampling ?
                        QValue(qnode, false, depth) :
                        qnode.UCB.Value.GetMean();
    }
  }

  Root->DisplayValue(history, depth, ostr, &qvalues);
}

void FlatMCTS::UnitTest() {
  UnitTestGreedy();
  UnitTestUCB();
  UnitTestRollout();
  for (int depth = 1; depth <= 3; ++depth) UnitTestSearch(depth);
}

void FlatMCTS::UnitTestGreedy() {
  TEST_SIMULATOR testSimulator(5, 5, 0);
  PARAMS params;
  FlatMCTS mcts(testSimulator, params, 0);
  int numAct = testSimulator.GetNumActions();

  HISTORY History(0);
  VNODE *vnode = mcts.ExpandNode(testSimulator.CreateStartState(), History);
  vnode->UCB.Value.Set(1, 0);
  vnode->Child(0).UCB.Value.Set(1, 1/*, 2, 1*/);
  for (int action = 1; action < numAct; action++)
    vnode->Child(action).UCB.Value.Set(0, 0/*, 1, 1*/);

  int besta = mcts.GreedyUCB(vnode, false);
  assert(besta == 0);
}

void FlatMCTS::UnitTestUCB() {
  TEST_SIMULATOR testSimulator(5, 5, 0);
  PARAMS params;
  FlatMCTS mcts(testSimulator, params, 0);
  int numAct = testSimulator.GetNumActions();
  HISTORY History(0);

  // With equal value, action with lowest count is selected
  VNODE *vnode1 = mcts.ExpandNode(testSimulator.CreateStartState(), History);
  vnode1->UCB.Value.Set(1, 0);
  for (int action = 0; action < numAct; action++)
    if (action == 3)
      vnode1->Child(action).UCB.Value.Set(99, 0/*, 1, 100*/);
    else
      vnode1->Child(action).UCB.Value.Set(100 + action, 0/*, 1, 101 +action*/);
  assert(mcts.GreedyUCB(vnode1, true) == 3);

  // With high counts, action with highest value is selected
  VNODE *vnode2 = mcts.ExpandNode(testSimulator.CreateStartState(), History);
  vnode2->UCB.Value.Set(1, 0);
  for (int action = 0; action < numAct; action++)
    if (action == 3)
      vnode2->Child(action).UCB.Value.Set(99, 1/*, 100 + numObs,1*/);
    else
      vnode2->Child(action).UCB.Value.Set(100 + numAct - action, 0/*, 1,101 + numAct - action*/);
  assert(mcts.GreedyUCB(vnode2, true) == 3);

  // Action with low value and low count beats actions with high counts
  VNODE *vnode3 = mcts.ExpandNode(testSimulator.CreateStartState(), History);
  vnode3->UCB.Value.Set(1, 0);
  for (int action = 0; action < numAct; action++)
    if (action == 3)
      vnode3->Child(action).UCB.Value.Set(1, 1/*, 2, 1*/);
    else
      vnode3->Child(action).UCB.Value.Set(100 + action, 1/*, 101 + action,1*/);
  assert(mcts.GreedyUCB(vnode3, true) == 3);

  // Actions with zero count is always selected
  VNODE *vnode4 = mcts.ExpandNode(testSimulator.CreateStartState(), History);
  vnode4->UCB.Value.Set(1, 0);
  for (int action = 0; action < numAct; action++)
    if (action == 3)
      vnode4->Child(action).UCB.Value.Set(0, 0/*, 1, 1*/);
    else
      vnode4->Child(action).UCB.Value.Set(1, 1/*, 2, 1*/);
  assert(mcts.GreedyUCB(vnode4, true) == 3);
}

void FlatMCTS::UnitTestRollout() {
  TEST_SIMULATOR testSimulator(2, 2, 0);
  PARAMS params;
  params.NumSimulations = 1000;
  params.MaxDepth = 10;
  FlatMCTS mcts(testSimulator, params, 0);
  double totalReward = 0.0;
  for (int n = 0; n < mcts.Params.NumSimulations; ++n) {
    STATE *state = testSimulator.CreateStartState();
    totalReward += mcts.Rollout(*state, 0);
  }
  double rootValue = totalReward / mcts.Params.NumSimulations;
  double meanValue = testSimulator.MeanValue();
  assert(fabs(meanValue - rootValue) < 0.1);
}

void FlatMCTS::UnitTestSearch(int depth) {
  TEST_SIMULATOR testSimulator(3, 2, depth);
  PARAMS params;
  params.MaxDepth = depth + 1;
  params.NumSimulations = pow(10, depth + 1);
  FlatMCTS mcts(testSimulator, params, 0);
  mcts.Search();
  double rootValue = mcts.Root->UCB.Value.GetValue();
  double optimalValue = testSimulator.OptimalValue();
  assert(fabs(optimalValue - rootValue) < 0.1);
}
