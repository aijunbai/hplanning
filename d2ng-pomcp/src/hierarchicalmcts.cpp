#include "hierarchicalmcts.h"

HierarchicalMCTS::HierarchicalMCTS(const SIMULATOR &simulator,
                                   const PARAMS &params)
    : MCTS(simulator, params) {}

HierarchicalMCTS::~HierarchicalMCTS() {}
