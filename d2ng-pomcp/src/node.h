#ifndef NODE_H
#define NODE_H

#include "beliefstate.h"
#include "utils.h"
#include "statistic.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <map>
#include <unordered_map>

#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/math/distributions.hpp>
#include <boost/functional/hash.hpp>

class HISTORY;
class SIMULATOR;
class QNODE;
class VNODE;

class QNODE {
 public:
  QNODE() {
    Initialise();
  }

  void Initialise();

  void Update(int observation, double reward, int count = 0) {
    TS.Observation.Add(observation);
    TS.ImmediateReward.Add(reward);
    TS.UpdateCount += count;
  }

  bool Applicable() const { return mApplicable; }

  int GetCount() const {
    assert(TS.UpdateCount == 0 || UCB.Value.GetCount() == 0);
    return TS.UpdateCount + UCB.Value.GetCount();
  }

  void SetPrior(int count, double value, int applicable) {
    mApplicable = applicable;
    UCB.Value.Set(count, value);

    if (mApplicable) {
      TS.ImmediateReward.Set(count, value);
    } else {
      TS.ImmediateReward.Clear();
    }
  }

  VNODE *&Child(int c) {
    return Children[c];
  }

  VNODE *Child(int c) const {
    return Children[c];
  }

  void DisplayValue(HISTORY &history, int maxDepth, std::ostream &ostr,
                    const double *qvalue = 0) const;

public:
  mutable std::unordered_map<int, VNODE*> Children;

private:
  bool mApplicable;

public:
  struct {
    int UpdateCount;
    DirichletInfo_POMCP<int> Observation;
    DirichletInfo_POMCP<double> ImmediateReward;
  } TS;

  struct {
    STATISTIC Value;  // for uct
  } UCB;
};

class VNODE : public MEMORY_OBJECT {
 public:
  void Initialise(size_t belief_hash);

  static VNODE *Create(HISTORY &history);
  static void Free(VNODE *root, const SIMULATOR &simulator, VNODE *ignore = 0);
  static void FreeAll();

  QNODE &Child(int c) {
    return Children[c];
  }
  const QNODE &Child(int c) const {
    return Children[c];
  }
  BELIEF_STATE &Beliefs() { return BeliefState; }
  const BELIEF_STATE &Beliefs() const { return BeliefState; }

  void SetPrior(int actions, int count, double value, bool applicable);

  void DisplayValue(HISTORY &history, int maxDepth, std::ostream &ostr,
                    const std::vector<double> *qvalues = 0) const;

  NormalGammaInfo &GetCumulativeReward(const STATE &s);

  double ThompsonSampling(bool sampling) {
    double count = 0.0;
    double sum = 0.0;

    for (auto it = TS.CumulativeRewards.begin(); it != TS.CumulativeRewards.end(); ++it) {
      if (it->second.GetCount() > 0.0) {
        sum += it->second.GetCount() * it->second.ThompsonSampling(sampling);
        count += it->second.GetCount();
      }
    }

    assert(count > 0.0);
    return sum / count;
  }

  static MEMORY_POOL<VNODE> VNodePool;

  static int GetNumAllocated() {
    return VNodePool.GetNumAllocated();
  }

 private:
  mutable std::unordered_map<int, QNODE> Children;
  BELIEF_STATE BeliefState;
  size_t BeliefHash;

public:
  struct {
    NormalGammaInfo_POMCP CumulativeRewards;
  } TS;

  struct {
    STATISTIC Value;  // for uct
  } UCB;

  size_t GetBeliefHash() const {
    return BeliefHash;
  }
};

#endif  // NODE_H
