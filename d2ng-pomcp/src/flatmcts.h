#ifndef MCTS_H
#define MCTS_H

#include "simulator.h"
#include "node.h"
#include "statistic.h"
#include "dot_graph.h"
#include "metamcts.h"

class FlatMCTS: public MetaMCTS {
 public:
  FlatMCTS(const SIMULATOR &simulator, const PARAMS &params);
  virtual ~FlatMCTS();

  virtual int SelectAction();
  virtual bool Update(int action, int observation, STATE &state);

  virtual void SearchImp();
  double Rollout(STATE &state);

  const BELIEF_STATE &BeliefState() const { return Root->Beliefs(); }
  void DisplayValue(int depth, std::ostream &ostr) const;
  void DisplayPolicy(int depth, std::ostream &ostr) const;

  static void UnitTest();
  static void UnitTestGreedy();
  static void UnitTestUCB();
  static void UnitTestRollout();
  static void UnitTestSearch(int depth);

public:
  STATISTIC StatRedundantNodes;

 protected:
  VNODE *Root;
  int TreeDepth, PeakTreeDepth;

  int GreedyUCB(VNODE* vnode, bool ucb) const;
  int ThompsonSampling(VNODE *vnode, bool sampling) const;

  double SimulateV(STATE &state, VNODE *vnode);
  double SimulateQ(STATE &state, QNODE &qnode, int action);
  VNODE *ExpandNode(const STATE *state, HISTORY &history);
  void AddSample(VNODE *node, const STATE &state);
  void ParticleFilter(BELIEF_STATE &beliefs);
  void AddTransforms(BELIEF_STATE &beliefs);
  STATE *CreateTransform() const;
  void Resample(BELIEF_STATE &beliefs);

  double QValue(QNODE &qnode, bool sampling) const;
  double HValue(VNODE *vnode, bool sampling) const;
};

#endif  // MCTS_H
