#ifndef CONTINIOUSROOMS_H
#define CONTINIOUSROOMS_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"

class ContinousROOMS_STATE : public STATE {
 public:
  Vector AgentPos;

  virtual size_t hash() const {
    using boost::hash_combine;

    // Start with a hash value of 0    .
    std::size_t seed = 0;

    // Modify 'seed' by XORing and bit-shifting in
    // one member of 'Key' after the other:
    hash_combine(seed, hash_value(AgentPos));

    // Return the result.
    return seed;
  }
};

class ContinousROOMS : public SIMULATOR {
 public:
  ContinousROOMS(const char *map_name, bool state_abstraction, bool action_abstraction);
  virtual ~ContinousROOMS();

  virtual STATE *Copy(const STATE &state) const;
  virtual void Validate(const STATE &state) const;
  virtual STATE *CreateStartState() const;
  virtual void FreeState(STATE *state) const;
  virtual bool Step(STATE &state, int action, int &observation,
                    double &reward) const;

  virtual void GenerateLegal(const STATE &state,
                             std::vector<int> &legal) const;
  virtual void GeneratePreferred(const STATE &state, const HISTORY &history,
                                 std::vector<int> &legal) const;
  virtual bool LocalMove(STATE &state, const HISTORY &history,
                         int stepObservation) const;

  virtual void DisplayBeliefs(const BELIEF_STATE &beliefState,
                              std::ostream &ostr) const;
  virtual void DisplayState(const STATE &state, std::ostream &ostr) const;
  virtual void DisplayObservation(const STATE &state, int observation,
                                  std::ostream &ostr) const;
  virtual void DisplayAction(int action, std::ostream &ostr) const;

 protected:
  bool IsValid(const Vector &pos) const;
  void Parse(const char *file_name);
  int GetObservation(const ContinousROOMS_STATE &state) const;

  GRID<int> *mGrid;  // grid map of continous environment
  int mRooms;
  Vector mStartPos;
  Vector mGoalPos;
  double mThreshold;
  double mFieldLength;
  double mFieldWidth;
  double mSizePerGrid;

  COORD Position2Grid(const Vector &o) const;
  Vector Grid2Position(const COORD &o) const;

 private:
  mutable MEMORY_POOL<ContinousROOMS_STATE> mMemoryPool;
};

#endif // CONTINIOUSROOMS_H
