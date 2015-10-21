#ifndef MCTS_H
#define MCTS_H

#include "simulator.h"
#include "node.h"
#include "statistic.h"
#include "dot_graph.h"

class MCTS {
 public:
  struct PARAMS {
    PARAMS();

    int Verbose;
    int MaxDepth;
    int NumSimulations;
    int NumStartStates;
    bool UseTransforms;
    bool UseParticleFilter;
    int NumTransforms;
    int MaxAttempts;
    double ExplorationConstant;
    bool ReuseTree;
    bool ThompsonSampling;
    double TimeOutPerAction;
    int MemorySize;
  };

  MCTS(const SIMULATOR &simulator, const PARAMS &params);
  virtual ~MCTS();

  int SelectAction();
  virtual bool Update(int action, int observation, STATE &state);  // update history and ground state (if possible)

  void Search();
  void SearchImp();
  double Rollout(STATE &state);

  const BELIEF_STATE &BeliefState() const { return Root->Beliefs(); }
  const HISTORY &GetHistory() const { return History; }
  void DisplayValue(int depth, std::ostream &ostr) const;
  void DisplayPolicy(int depth, std::ostream &ostr) const;

  static void UnitTest();
  static void InitFastUCB(double exploration);

  // Fast lookup table for UCB
  static const int UCB_N = 1 << 14, UCB_n = 1 << 7;
  static double UCB[UCB_N][UCB_n];
  static bool InitialisedFastUCB;
  double FastUCB(int N, int n) const;

public:
  STATISTIC StatBeliefSize;     //统计每次 UCTSearch 开始时信念大小
  STATISTIC StatNumSimulation;  //统计 Any time 模式下每次 simulation 次数

  STATISTIC StatTreeSize;    //统计每次 UCTSearch 结束时树的大小
  STATISTIC StatPeakTreeDepth;
  STATISTIC StatRedundantNodes;

 protected:
  const SIMULATOR &Simulator;
  PARAMS Params;
  VNODE *Root;
  HISTORY History;
  int TreeDepth, PeakTreeDepth;

  int ActionSelection(VNODE* vnode, bool ucb) const;
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

  static void UnitTestGreedy();
  static void UnitTestUCB();
  static void UnitTestRollout();
  static void UnitTestSearch(int depth);
};

#endif  // MCTS_H
