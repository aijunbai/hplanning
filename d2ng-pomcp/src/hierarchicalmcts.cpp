#include "hierarchicalmcts.h"

using namespace std;

HierarchicalMCTS::HierarchicalMCTS(const SIMULATOR &simulator,
                                   const PARAMS &params)
    : MCTS(simulator, params) {}

HierarchicalMCTS::~HierarchicalMCTS() {}

bool HierarchicalMCTS::Update(int action, int observation, STATE &state)
{
  PRINT_VALUE("HierarchicalMCTS::Update");
  VNODE::Free(Root, Simulator);
  History.Add(action, observation, Params.MemorySize);  //更新历史
  Root = ExpandNode(&state, History);
  AddSample(Root, state);

  if (Params.Verbose >= 1) Simulator.DisplayBeliefs(Root->Beliefs(), cout);

  return true;
}
