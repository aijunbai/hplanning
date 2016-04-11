#include "experiment.h"
#include "hierarchicalmcts.h"
#include "flatmcts.h"
#include "boost/timer.hpp"

using namespace std;

EXPERIMENT::PARAMS::PARAMS()
    : NumRuns(1000), NumSteps(100000), TimeOut(3600), MinDoubles(0),
      MaxDoubles(20), TransformDoubles(-4), TransformAttempts(1000),
      Accuracy(0.001), UndiscountedHorizon(1000) { }

EXPERIMENT::EXPERIMENT(const SIMULATOR &real, const SIMULATOR &simulator,
                       const string &outputFile, EXPERIMENT::PARAMS &expParams,
                       MCTS::PARAMS &searchParams)
    : Real(real), Simulator(simulator), ExpParams(expParams),
      SearchParams(searchParams),
      OutputFile(outputFile.c_str(), fstream::out | fstream::app) {
  MCTS::InitFastUCB(); //初始化 UCB 表格缓存
}

void EXPERIMENT::Run() {
  boost::timer timer;
  MCTS *mcts = 0;

  if (SearchParams.UseFlatPlanning) {
    mcts = new FlatMCTS(Simulator, SearchParams);
    cerr << "using flatplanning" << endl;
  }
  else {
    if (SearchParams.UseActionAbstraction && Simulator.mStateAbstraction) {
      mcts = new HierarchicalMCTS(Simulator, SearchParams, true);
      cerr << "using hplanning w/ action abstraction" << endl;
    }
    else {
      mcts = new HierarchicalMCTS(Simulator, SearchParams, false);
      cerr << "using hplanning wo/ action abstraction" << endl;
    }
  }

  double undiscountedReturn = 0.0;
  double discountedReturn = 0.0;
  double discount = 1.0;
  bool terminal = false;
  bool outOfParticles = false;
  int t = 0;

  STATE *state = Real.CreateStartState(); //真实的世界状态

  for (t = 0; t < ExpParams.NumSteps; t++) {
    int observation;
    double reward;

    if (SearchParams.Verbose >= 1) {
      cout << "\nStep " << t << " of " << ExpParams.NumSteps << endl;
      Real.DisplayState(*state, cout);
    }

    boost::timer timer_per_action;
    int action = mcts->SelectAction(); // 用 Monte Carlo 方法选择一个动作
    Results.TimePerAction.Add(timer_per_action.elapsed());

    terminal = Real.Step(
        *state, action, observation,
        reward); //根据 state 和 action 转移到下一个状态，获得实际观察和回报
    Results.Reward.Add(reward);
    undiscountedReturn += reward;
    discountedReturn += reward * discount;
    discount *= Real.GetDiscount();

    Results.ExploredNodes.Add(mcts->TreeSize);
    Results.ExploredDepth.Add(mcts->TreeDepth);

    if (SearchParams.Verbose >= 1) {
      cout << "Action: #" << action << " ";
      Real.DisplayAction(action, cout);
      Real.DisplayObservation(*state, observation, cout);
      Real.DisplayReward(reward, cout);
    }

    if (terminal) {
      cout << "Terminated" << endl;
      break;
    }

    outOfParticles = !mcts->Update(
        action, observation, *state); //更新历史信息，得到新的 Root 节点，设置好初始信念状态
    if (outOfParticles) {
      assert(!Real.mHierarchicalPlanning);
      break; // Out of particles, finishing episode with SelectRandom
    }

    if (timer.elapsed() > ExpParams.TimeOut) {
      cout << "Timed out after " << t << " steps in " << Results.Time.GetTotal()
      << "seconds" << endl;
      break;
    }
  }

  if (outOfParticles) //特殊情况处理
  {
    cout << "Out of particles, finishing episode with SelectRandom" << endl;
    while (++t < ExpParams.NumSteps) {
      int observation;
      double reward;

      // This passes real state into simulator!
      // SelectRandom must only use fully observable state
      // to avoid "cheating"
      int action = SimpleRNG::ins().Random(Simulator.GetNumActions());
      terminal = Real.Step(*state, action, observation, reward);

      Results.Reward.Add(reward);
      undiscountedReturn += reward;
      discountedReturn += reward * discount;
      discount *= Real.GetDiscount();

      if (SearchParams.Verbose >= 1) {
        Real.DisplayAction(action, cout);
        Real.DisplayState(*state, cout);
        Real.DisplayObservation(*state, observation, cout);
        Real.DisplayReward(reward, cout);
      }

      if (terminal) {
        cout << "Terminated" << endl;
        break;
      }
    }
  }

  Real.FreeState(state);

  Results.Time.Add(timer.elapsed());
  Results.UndiscountedReturn.Add(undiscountedReturn);
  Results.DiscountedReturn.Add(discountedReturn);

  cout << "\n#End of experiment:" << endl;
  cout << "#Discounted return = " << discountedReturn
  << ", average = " << Results.DiscountedReturn.GetMean() << endl;
  cout << "#Undiscounted return = " << undiscountedReturn
  << ", average = " << Results.UndiscountedReturn.GetMean() << endl;

  delete mcts;
}

void EXPERIMENT::MultiRun() {
  for (int n = 0; n < ExpParams.NumRuns; n++) //实验次数
  {
    cout << "Starting run " << n + 1 << " with " << SearchParams.NumSimulations
    << " simulations... " << endl;

    Run();
    assert(VNODE::GetNumAllocated() == 0);

    if (Results.Time.GetTotal() > ExpParams.TimeOut) {
      cout << "Timed out after " << n << " runs in " << Results.Time.GetTotal()
      << "seconds" << endl;
      break;
    }
  }
}

void EXPERIMENT::DiscountedReturn() {
  cout << "Main runs" << endl;
  OutputFile << "#Simulations\tRuns\tUndiscountedReturn\tUndiscountedError\t"
      "DiscountedReturn\tDiscountedError\t"
      "Time\tTimePerAction\tExploredNodes\tExploredNodesError\t"
      "ExploredDepth\tExploredDepthError\n";

  SearchParams.MaxDepth = Simulator.GetHorizon(
      ExpParams.Accuracy,
      ExpParams.UndiscountedHorizon); //搜索过程中的最大深度
  ExpParams.NumSteps = Real.GetHorizon(
      ExpParams.Accuracy, ExpParams.UndiscountedHorizon); //实验的最大步长

  for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++) {
    SearchParams.NumSimulations = 1 << i; //迭代次数 iterations

    if (SearchParams.TimeOutPerAction < 0.0) { //非anytime模式
      SearchParams.NumStartStates = 1 << i;    //初始粒子数
    }

    if (i + ExpParams.TransformDoubles >= 0)
      SearchParams.NumTransforms = 1
                                   << (i + ExpParams.TransformDoubles); // 1/16
    else
      SearchParams.NumTransforms = 1;
    SearchParams.MaxAttempts =
        SearchParams.NumTransforms * ExpParams.TransformAttempts;

    Results.Clear();
    MultiRun();

    cout << "#Simulations = " << SearchParams.NumSimulations << endl
    << "#Runs = " << Results.Time.GetCount() << endl
    << "#Undiscounted return = " << Results.UndiscountedReturn.GetMean()
    << " +- " << Results.UndiscountedReturn.GetStdErr() << endl
    << "#Discounted return = " << Results.DiscountedReturn.GetMean()
    << " +- " << Results.DiscountedReturn.GetStdErr() << endl
    << "#Time = " << Results.Time.GetMean() << endl
    << "#TimePerAction = " << Results.TimePerAction.GetMean() << endl
    << "#ExploredNodes = " << Results.ExploredNodes.GetMean() << " +- "
    << Results.ExploredNodes.GetStdErr() << endl
    << "#ExploredDepth = " << Results.ExploredDepth.GetMean() << " +- "
    << Results.ExploredDepth.GetStdErr() << endl;

    OutputFile << SearchParams.NumSimulations << "\t" << Results.Time.GetCount()
    << "\t" << Results.UndiscountedReturn.GetMean() << "\t"
    << Results.UndiscountedReturn.GetStdErr() << "\t"
    << Results.DiscountedReturn.GetMean() << "\t"
    << Results.DiscountedReturn.GetStdErr() << "\t"
    << Results.Time.GetMean() << "\t"
    << Results.TimePerAction.GetMean() << "\t"
    << Results.ExploredNodes.GetMean() << "\t"
    << Results.ExploredNodes.GetStdErr() << "\t"
    << Results.ExploredDepth.GetMean() << "\t"
    << Results.ExploredDepth.GetStdErr() << "\t" << endl;
  }
}