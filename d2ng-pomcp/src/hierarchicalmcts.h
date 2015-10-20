#ifndef HIERARCHICALMCTS_H
#define HIERARCHICALMCTS_H

#include "mcts.h"

/**
 * @brief The HierarchicalMCTS class
 *
 * MCTS algorithm for MDP with state abstraction
 */
class HierarchicalMCTS : public MCTS {
 public:
  HierarchicalMCTS(const SIMULATOR &simulator, const PARAMS &params);
  virtual ~HierarchicalMCTS();

  virtual bool Update(int action, int observation, STATE &state);
};

#endif  // HIERARCHICALMCTS_H
