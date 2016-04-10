#ifndef MCTS_H
#define MCTS_H

#include "simulator.h"
#include "node.h"
#include "statistic.h"
#include "dot_graph.h"
#include "mcts.h"

/**
 * POMCP and DNG-POMCP
 */
class FlatMCTS : public MCTS {
public:
  FlatMCTS(const SIMULATOR &simulator, const PARAMS &params);

  virtual ~FlatMCTS();

  virtual int SelectAction();

  virtual bool Update(int action, int observation, STATE &state);

  virtual void SearchImp();

  double Rollout(STATE &state, int depth);

  const BELIEF_STATE &BeliefState() const { return Root->Beliefs(); }

  void DisplayValue(int depth, std::ostream &ostr) const;

  static void UnitTest();

  static void UnitTestGreedy();

  static void UnitTestUCB();

  static void UnitTestRollout();

  static void UnitTestSearch(int depth);

protected:
  VNODE *Root;

  int GreedyUCB(VNODE *vnode, bool ucb) const;

  int ThompsonSampling(VNODE *vnode, bool sampling, int depth) const;

  double SimulateV(STATE &state, VNODE *vnode, int depth);

  double SimulateQ(STATE &state, QNODE &qnode, int action, int depth);

  VNODE *ExpandNode(const STATE *state, HISTORY &history);

  void AddSample(VNODE *node, const STATE &state);

  void ParticleFilter(BELIEF_STATE &beliefs);

  void AddTransforms(BELIEF_STATE &beliefs);

  STATE *CreateTransform() const;

  void Resample(BELIEF_STATE &beliefs);

  double QValue(QNODE &qnode, bool sampling, int depth) const;

  double HValue(VNODE *vnode, bool sampling, int depth) const;
};

#endif  // MCTS_H
