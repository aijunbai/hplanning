#include "hierarchicalmcts.h"

using namespace std;

HierarchicalMCTS::HierarchicalMCTS(const SIMULATOR &simulator,
                                   const PARAMS &params)
    : MetaMCTS(simulator, params) {}

HierarchicalMCTS::~HierarchicalMCTS() {}

bool HierarchicalMCTS::Update(int action, int observation, STATE &state)
{
  return true;
}

int HierarchicalMCTS::SelectAction()
{
  return SimpleRNG::ins().Random(Simulator.GetNumActions());
}

void HierarchicalMCTS::SearchImp()
{

}
