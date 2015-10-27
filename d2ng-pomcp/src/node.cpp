#include "node.h"
#include "history.h"
#include "utils.h"
#include "simulator.h"

using namespace std;

void QNODE::Initialise()
{
  mApplicable = false;
  Children.clear();
  TS.UpdateCount = 0;
  TS.Observation.Clear();
  TS.ImmediateReward.Clear();
  UCB.Value.Initialise();
}

void QNODE::DisplayValue(HISTORY &history, int maxDepth, ostream &ostr,
                         const double *qvalue) const {
  history.Display(ostr);

  if (qvalue) {
    ostr << "qvalue=" << *qvalue << ", " << endl;
  }

  if (history.Size() >= maxDepth) return;

  for (auto it = Children.begin(); it != Children.end(); ++it) {
    history.Back().Observation = it->first;
    it->second->DisplayValue(history, maxDepth, ostr);
  }
}

MEMORY_POOL<VNODE> VNODE::VNodePool;
unordered_map<size_t, VNODE*> VNODE::BeliefPool;

void VNODE::Initialise(size_t belief_hash) {
  assert(BeliefState.Empty());

  BeliefHash = belief_hash;
  Children.clear();
  TS.CumulativeRewards.clear();
  UCB.Value.Initialise();
}

VNODE *VNODE::Create(HISTORY &history, int memory_size) {
  size_t belief_hash = history.BeliefHash();

  if (memory_size >= 0 && history.Size() >= memory_size && BeliefPool.count(belief_hash)) {
    assert(0);  // will never create a replicate node according to MCTS algorithm
    return BeliefPool[belief_hash];
  }
  else {
    VNODE *vnode = VNODE::VNodePool.Allocate();
    BeliefPool[belief_hash] = vnode;
    vnode->Initialise(belief_hash);
    return vnode;
  }
}

void VNODE::Free(VNODE *root, const SIMULATOR &simulator, VNODE *ignore) {
  if (root->IsAllocated() && root != ignore) {
    root->BeliefState.Free(simulator);
    VNODE::BeliefPool.erase(root->BeliefHash);
    VNODE::VNodePool.Free(root);

    for (auto it = root->Children.begin(); it != root->Children.end(); ++it) {
      for (auto ii = it->second.Children.begin();  ii != it->second.Children.end(); ++ii) {
        Free(ii->second, simulator, ignore);
      }
    }
  }
}

void VNODE::FreeAll() { VNODE::VNodePool.DeleteAll(); VNODE::BeliefPool.clear(); }

void VNODE::SetPrior(int actions, int count, double value, bool applicable) {
  for (int action = 0; action < actions; action++) {
    QNODE &qnode = Children[action];
    qnode.SetPrior(count, value, applicable);
  }
}

void VNODE::DisplayValue(HISTORY &history, int maxDepth, ostream &ostr,
                         const std::vector<double> *qvalues) const {
  if (history.Size() >= maxDepth) return;

  for (auto it = Children.begin(); it != Children.end(); ++it) {
    int action = it->first;
    history.Add(action, -1, -1, 0.0);
    const QNODE &qnode = Children[action];

    if (qnode.Applicable()) {
      ostr << "count=" << qnode.GetCount() << ", ";
      if (qvalues) {
        qnode.DisplayValue(history, maxDepth, ostr, &(qvalues->at(action)));
      } else {
        qnode.DisplayValue(history, maxDepth, ostr);
      }
    }
    history.Pop();
  }
}

NormalGammaInfo &VNODE::GetCumulativeReward(const STATE &s) {
#ifdef NDEBUG
  return TS.CumulativeRewards[s.hash()];
#else
  std::size_t key = s.hash();
  auto it = TS.CumulativeRewards.find(key);

  if (it != TS.CumulativeRewards.end()) {
    return it->second;
  } else {
    return TS.CumulativeRewards[key];
  }
#endif
}
