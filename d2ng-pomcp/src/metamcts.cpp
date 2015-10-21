#include "metamcts.h"
#include "testsimulator.h"
#include "statistic.h"
#include "boost/timer.hpp"
#include "distribution.h"

#include <math.h>
#include <algorithm>

using namespace std;
using namespace UTILS;

MetaMCTS::PARAMS::PARAMS()
  : Verbose(0),
    MaxDepth(100),
    NumSimulations(1000),
    NumStartStates(1000),
    UseTransforms(true),
    UseParticleFilter(false),
    NumTransforms(0),
    MaxAttempts(0),
    ExplorationConstant(1.0),
    ReuseTree(false),
    ThompsonSampling(false),
    TimeOutPerAction(-1),
    MemorySize(-1) {}


MetaMCTS::MetaMCTS(const SIMULATOR &simulator, const PARAMS &params)
  :Simulator(simulator), Params(params)
{

}

MetaMCTS::~MetaMCTS()
{
  if (Params.Verbose >= 1) {
    StatNumSimulation.Print("#Num simulations", cout);
  }

}


double MetaMCTS::UCB[UCB_N][UCB_n];
bool MetaMCTS::InitialisedFastUCB = false;

void MetaMCTS::InitFastUCB(double exploration)
{
  cout << "Initialising fast UCB table... ";

  for (int N = 0; N < UCB_N; ++N)
    for (int n = 0; n < UCB_n; ++n)
      if (n == 0)
        UCB[N][n] = Infinity;
      else
        UCB[N][n] = exploration * sqrt(log(N + 1) / n);

  cout << "done" << endl;
  InitialisedFastUCB = true;
}

double MetaMCTS::FastUCB(int N, int n) const
{
  if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
    return UCB[N][n];

  if (n == 0)
    return Infinity;
  else {
    return Params.ExplorationConstant * sqrt(log(N + 1) / n);
  }
}


void MetaMCTS::Search() {
  if (Params.TimeOutPerAction > 0.0) {  // Anytime mode
    boost::timer timer;
    int i = 0;

    while (1) {
      i += 1;
      SearchImp();

      if (timer.elapsed() > Params.TimeOutPerAction) {
        StatNumSimulation.Add(i);
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
