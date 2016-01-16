#include "mcts.h"
#include "boost/timer.hpp"

using namespace std;
using namespace UTILS;

double MCTS::UCB[UCB_N][UCB_n];
bool MCTS::InitialisedFastUCB = false;

MCTS::PARAMS::PARAMS()
    : Verbose(0),
      MaxDepth(100),
      NumSimulations(1000),
      NumStartStates(1000),
      UseTransforms(true),
      UseParticleFilter(false),
      NumTransforms(0),
      MaxAttempts(0),
      ReuseTree(false),
      ThompsonSampling(false),
      TimeOutPerAction(-1),
//      Converged(1.0),
//      CacheRate(0.0),
      Polling(true),
      Stack(false) { }


MCTS::MCTS(const SIMULATOR &simulator, const PARAMS &params)
    : Simulator(simulator), Params(params), TreeDepth(0), TreeSize(0) {
}

MCTS::~MCTS() {
}

void MCTS::InitFastUCB() {
  cout << "Initialising fast UCB table... ";

  for (int N = 0; N < UCB_N; ++N)
    for (int n = 0; n < UCB_n; ++n)
      if (n == 0)
        UCB[N][n] = Infinity;
      else
        UCB[N][n] = sqrt(log(N + 1) / n);

  cout << "done" << endl;
  InitialisedFastUCB = true;
}

double MCTS::FastUCB(int N, int n, double exploration) const {
  if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
    return n == 0? Infinity: exploration * UCB[N][n];

  return n == 0? Infinity: exploration * sqrt(log(N + 1) / n);
}


void MCTS::Search() {
  TreeDepth = 0;
  TreeSize = 0;

  if (Params.TimeOutPerAction > 0.0) {  // Anytime mode
    boost::timer timer;
    int i = 0;

    while (1) {
      i += 1;
      SearchImp();

      if (timer.elapsed() > Params.TimeOutPerAction) {
        break;
      }
    }
  } else {
    for (int i = 0; i < Params.NumSimulations; i++)  //总共仿真（迭代）次数
    {
      SearchImp();
    }
  }
}
