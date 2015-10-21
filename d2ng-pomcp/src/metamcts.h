#ifndef METAMCTS_H
#define METAMCTS_H

#include "simulator.h"
#include "node.h"
#include "statistic.h"
#include "dot_graph.h"

class MetaMCTS
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
    int MemorySize;
  };

  MetaMCTS(const SIMULATOR &simulator, const PARAMS &params);
  virtual ~MetaMCTS();

  virtual int SelectAction() = 0;
  virtual void SearchImp() = 0;
  virtual bool Update(int action, int observation, STATE &state) = 0;  // update history and ground state (if possible)

  void Search();
  static void InitFastUCB(double exploration);

  // Fast lookup table for UCB
  static const int UCB_N = 1 << 14, UCB_n = 1 << 7;
  static double UCB[UCB_N][UCB_n];
  static bool InitialisedFastUCB;
  double FastUCB(int N, int n) const;

protected:
 const SIMULATOR &Simulator;
 PARAMS Params;

public:
 STATISTIC StatNumSimulation;  //统计 Any time 模式下每次 simulation 次数
};

#endif // METAMCTS_H
