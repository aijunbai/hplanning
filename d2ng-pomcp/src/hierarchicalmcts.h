#ifndef HIERARCHICALMCTS_H
#define HIERARCHICALMCTS_H

#include "metamcts.h"

/**
 * @brief The HierarchicalMCTS class
 *
 * MCTS algorithm for MDP with state abstraction
 */
class HierarchicalMCTS : public MetaMCTS {
 public:
  HierarchicalMCTS(const SIMULATOR &simulator, const PARAMS &params);
  virtual ~HierarchicalMCTS();

  virtual int SelectAction();
  virtual void SearchImp();
  virtual bool Update(int action, int observation, STATE &state);
};

#endif  // HIERARCHICALMCTS_H
