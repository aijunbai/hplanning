#ifndef ROOMS_H
#define ROOMS_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"
#include <utility>

class ROOMS_STATE : public STATE {
 public:
  COORD AgentPos;
  COORD AgentVel;

  ROOMS_STATE(): AgentPos(0, 0), AgentVel(0, 0) {

  }

  static int EncodeInt(int i) {
    int sign = i > 0? 1: 0;
    return sign << 7 | abs(i);
  }

  static int DecodeInt(int c) {
    int v = c & 0x7F;
    return c >> 7? v: -v;
  }

  int Encode() const {
    int code = 0;
    code = code << 8 | EncodeInt(AgentPos.X);
    code = code << 8 | EncodeInt(AgentPos.Y);
    code = code << 8 | EncodeInt(AgentVel.X);
    code = code << 8 | EncodeInt(AgentVel.Y);
    return code;
  }

  static std::pair<COORD, COORD> Decode(int code) {
    COORD pos, vel;
    vel.Y = DecodeInt(code & 0xFF); code >>= 8;
    vel.X = DecodeInt(code & 0xFF); code >>= 8;
    pos.Y = DecodeInt(code & 0xFF); code >>= 8;
    pos.X = DecodeInt(code & 0xFF); code >>= 8;
    return std::make_pair(pos, vel);
  }

  virtual size_t hash() const {
    using boost::hash_combine;

    // Start with a hash value of 0    .
    std::size_t seed = 0;

    // Modify 'seed' by XORing and bit-shifting in
    // one member of 'Key' after the other:
    hash_combine(seed, hash_value(AgentPos));
    hash_combine(seed, hash_value(AgentVel));

    // Return the result.
    return seed;
  }
};

class ROOMS : public SIMULATOR {
 public:
  ROOMS(const char *map_name, bool state_abstraction);
  virtual ~ROOMS();

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

  virtual int AbstractionFunction(const STATE &state) const {
    const ROOMS_STATE &rstate = safe_cast<const ROOMS_STATE &>(state);
    return GetObservation(rstate);
  }

  virtual int SuggestAction(STATE &state, STATE &exit) const;

 protected:
  void Parse(const char *file_name);
  int GetObservation(const ROOMS_STATE &state) const;

  GRID<int> *mGrid;
  int mRooms;
  COORD mStartPos;
  COORD mGoalPos;

 private:
  mutable MEMORY_POOL<ROOMS_STATE> mMemoryPool;
};

#endif  // ROOMS_H
