#ifndef METAMCTS_H
#define METAMCTS_H

#include "simulator.h"
#include "statistic.h"
#include "dot_graph.h"

class MCTS
{
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
  };

  MCTS(const SIMULATOR &simulator, const PARAMS &params);
  virtual ~MCTS();

  virtual int SelectAction() = 0;
  virtual void SearchImp() = 0;
  virtual bool Update(int action, int observation, STATE &state) = 0;  // update history and ground state (if possible)

  void Search();
  const HISTORY &GetHistory() const { return History; }
  static void InitFastUCB(double exploration);

  // Fast lookup table for UCB
  static const int UCB_N = 1 << 14, UCB_n = 1 << 7;
  static double UCB[UCB_N][UCB_n];
  static bool InitialisedFastUCB;
  double FastUCB(int N, int n) const;

protected:
 const SIMULATOR &Simulator;
 PARAMS Params;
 HISTORY History;

public:
 int TreeDepth;
 int TreeSize;
};

#endif // METAMCTS_H
