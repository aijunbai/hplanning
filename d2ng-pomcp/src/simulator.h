#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "history.h"
#include "node.h"
#include "utils.h"
#include <string>
#include <iostream>
#include <math.h>
#include <sstream>

class BELIEF_STATE;
class VNODE;

class STATE : public MEMORY_OBJECT {
 public:
  virtual ~STATE() {}

  virtual size_t hash() const = 0;
};

class SIMULATOR {
 public:
  struct KNOWLEDGE {
    enum { PURE, LEGAL, SMART, NUM_LEVELS };

    KNOWLEDGE();

    int TreeLevel;     //标记使用何种方式初始化为 vnode
    int RolloutLevel;  //标记使用何种方式 rollout
    int SmartTreeCount;
    double SmartTreeValue;

    int Level(int phase) const {
      assert(phase < STATUS::NUM_PHASES);
      if (phase == STATUS::TREE)
        return TreeLevel;
      else
        return RolloutLevel;
    }
  };

  struct STATUS {
    STATUS();

    enum { TREE, ROLLOUT, NUM_PHASES };

    enum { CONSISTENT, INCONSISTENT, RESAMPLED, OUT_OF_PARTICLES };

    int Phase;
    int Particles;
  };

  SIMULATOR();
  SIMULATOR(int numActions, int numObservations, double discount = 1.0);
  virtual ~SIMULATOR();

  // Create start state (can be stochastic)
  virtual STATE *CreateStartState() const = 0;

  // Free memory for state
  virtual void FreeState(STATE *state) const = 0;

  // Update state according to action, and get observation and reward.
  // Return value of true indicates termination of episode (if episodic)
  virtual bool Step(STATE &state, int action, int &observation,
                    double &reward) const = 0;

  // Create new state and copy argument (must be same type)
  virtual STATE *Copy(const STATE &state) const = 0;

  // Sanity check
  virtual void Validate(const STATE &state) const;

  // Modify state stochastically to some related state
  virtual bool LocalMove(STATE &state, const HISTORY &history, int stepObs,
                         const STATUS &status) const;

  // Use domain knowledge to assign prior value and confidence to actions
  // Should only use fully observable state variables
  void Prior(const STATE *state, const HISTORY &history, VNODE *vnode,
             const STATUS &status) const;

  // Use domain knowledge to select actions stochastically during rollouts
  // Should only use fully observable state variables
  int SelectRandom(const STATE &state, const HISTORY &history,
                   const STATUS &status) const;

  // Generate set of legal actions
  virtual void GenerateLegal(const STATE &state, /* const HISTORY& history,
                                                    //XXX 为什么跟 history
                                                    相关？*/
                             std::vector<int> &actions,
                             const STATUS &status) const;

  // Generate set of preferred actions
  virtual void GeneratePreferred(const STATE &state, const HISTORY &history,
                                 std::vector<int> &actions,
                                 const STATUS &status) const;

  // Textual display
  virtual void DisplayBeliefs(const BELIEF_STATE &beliefState,
                              std::ostream &ostr) const;
  virtual void DisplayState(const STATE &state, std::ostream &ostr) const;
  virtual void DisplayAction(int action, std::ostream &ostr) const;
  virtual void DisplayObservation(const STATE &state, int observation,
                                  std::ostream &ostr) const;
  virtual void DisplayReward(double reward, std::ostream &ostr) const;

  // Problem name
  std::string Name() const { return mName.str(); }

  // Accessors
  void SetKnowledge(const KNOWLEDGE &knowledge) { Knowledge = knowledge; }
  int GetNumActions() const { return NumActions; }
  int GetNumObservations() const { return NumObservations; }
  bool IsEpisodic() const { return false; }
  double GetDiscount() const { return Discount; }
  double GetHorizon(double accuracy,
                    int undiscountedHorizon = 100) const;  // XXX 这是怎么算的？

 protected:
  int NumActions, NumObservations;
  double Discount;
  KNOWLEDGE Knowledge;
  std::stringstream mName;

 public:
  bool mHierarchicalPlanning;
};

#endif  // SIMULATOR_H
