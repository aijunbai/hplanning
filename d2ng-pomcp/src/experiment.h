#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include "mcts.h"
#include "simulator.h"
#include "statistic.h"
#include <fstream>


struct RESULTS {
  void Clear();

  STATISTIC Time;
  STATISTIC TimePerAction;
  STATISTIC Reward;
  STATISTIC DiscountedReturn;
  STATISTIC UndiscountedReturn;
  STATISTIC ExploredNodes;
  STATISTIC ExploredDepth;
};

inline void RESULTS::Clear() {
  Time.Initialise();
  TimePerAction.Initialise();
  Reward.Initialise();
  DiscountedReturn.Initialise();
  UndiscountedReturn.Initialise();
  ExploredNodes.Initialise();
  ExploredDepth.Initialise();
}


class EXPERIMENT {
public:
  struct PARAMS {
    PARAMS();

    int NumRuns;
    int NumSteps;
    double TimeOut;
    int MinDoubles, MaxDoubles;
    int TransformDoubles;
    int TransformAttempts;
    double Accuracy;
    int UndiscountedHorizon;
  };

  EXPERIMENT(const SIMULATOR &real, const SIMULATOR &simulator,
             const std::string &outputFile, EXPERIMENT::PARAMS &expParams,
             MCTS::PARAMS &searchParams);

  void Run();

  void MultiRun();

  void DiscountedReturn();

private:
  const SIMULATOR &Real;
  const SIMULATOR &Simulator;
  EXPERIMENT::PARAMS &ExpParams;
  MCTS::PARAMS &SearchParams;
  RESULTS Results;

  std::ofstream OutputFile;
};


#endif  // EXPERIMENT_H
