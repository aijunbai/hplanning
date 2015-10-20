#include "redundantobject.h"
#include "utils.h"
#include "distribution.h"
#include <fstream>
#include <vector>
#include <algorithm>
#include <unordered_map>

using namespace std;
using namespace UTILS;

REDUNDANT_OBJECT::REDUNDANT_OBJECT(int size, bool state_abstraction)
    : mStateAbstraction(state_abstraction),
      mGrid(size, size),
      mStartPos(0, 0),
      mGoalPos(size - 1, size - 1) {
  NumActions = 4;  //动作数
  int grids = mGrid.GetSize();
  NumObservations = mStateAbstraction ? grids : grids * grids;
  Discount = 0.95;
  RewardRange = 9.0;
  mName << "redundant_object_" << size << "_" << state_abstraction;
  mHierarchicalPlanning = true;
}

REDUNDANT_OBJECT::~REDUNDANT_OBJECT() {}

STATE *REDUNDANT_OBJECT::Copy(const STATE &state) const {
  const REDUNDANT_OBJECT_STATE &rstate =
      safe_cast<const REDUNDANT_OBJECT_STATE &>(state);
  REDUNDANT_OBJECT_STATE *newstate = mMemoryPool.Allocate();
  *newstate = rstate;
  return newstate;
}

void REDUNDANT_OBJECT::Validate(const STATE &state) const {
  const REDUNDANT_OBJECT_STATE &rstate =
      safe_cast<const REDUNDANT_OBJECT_STATE &>(state);
  assert(mGrid.Inside(rstate.AgentPos));
  assert(mGrid.Inside(rstate.ObjectPos));
}

STATE *REDUNDANT_OBJECT::CreateStartState() const {
  REDUNDANT_OBJECT_STATE *rstate = mMemoryPool.Allocate();
  rstate->AgentPos = mStartPos;
  rstate->ObjectPos = mGoalPos;
  return rstate;
}

void REDUNDANT_OBJECT::FreeState(STATE *state) const {
  REDUNDANT_OBJECT_STATE *rstate = safe_cast<REDUNDANT_OBJECT_STATE *>(state);
  mMemoryPool.Free(rstate);
}

bool REDUNDANT_OBJECT::Step(STATE &state, int action, int &observation, double &reward) const
{
  assert(action < NumActions);

  REDUNDANT_OBJECT_STATE &rstate = safe_cast<REDUNDANT_OBJECT_STATE &>(state);
  reward = -1.0;

  if (SimpleRNG::ins().Bernoulli(0.1)) {  // fail
    action = coord::Opposite(action);
  }

  COORD pos = rstate.AgentPos + coord::Compass[action];
  if (!mGrid.Inside(pos)) {
    pos = rstate.AgentPos;
  }
  rstate.AgentPos = pos;

  action = SimpleRNG::ins().Random(NumActions);
  pos = rstate.ObjectPos + coord::Compass[action];
  if (!mGrid.Inside(pos)) {
    pos = rstate.ObjectPos;
  }
  rstate.ObjectPos = pos;

  if (rstate.AgentPos == rstate.ObjectPos) {
    reward = -10.0;
  }

  observation = GetObservation(rstate);
  if (rstate.AgentPos == mGoalPos) {
    return true;
  }

  return false;  // not terminated
}

bool REDUNDANT_OBJECT::LocalMove(STATE &state, const HISTORY &history, int,
                                 const STATUS &) const  //局部扰动
{
  REDUNDANT_OBJECT_STATE rstate = safe_cast<REDUNDANT_OBJECT_STATE &>(state);
  if (GetObservation(rstate) == history.Back().Observation) {
    return true;
  }
  return false;
}

void REDUNDANT_OBJECT::GenerateLegal(const STATE &state, /*const HISTORY& ,*/
                                     vector<int> &legal, const STATUS &) const {
  const REDUNDANT_OBJECT_STATE &rstate =
      safe_cast<const REDUNDANT_OBJECT_STATE &>(state);

  assert(mGrid.Inside(rstate.AgentPos));

  legal.push_back(COORD::E_NORTH);
  legal.push_back(COORD::E_EAST);
  legal.push_back(COORD::E_SOUTH);
  legal.push_back(COORD::E_WEST);
}

void REDUNDANT_OBJECT::GeneratePreferred(
    const STATE &state, const HISTORY &,               //手工策略
    vector<int> &actions, const STATUS &status) const  //获得优先动作
{
  GenerateLegal(state, actions, status);
}

int REDUNDANT_OBJECT::Encode(const REDUNDANT_OBJECT_STATE &rstate) const {
  int index = mGrid.Index(rstate.AgentPos) * mGrid.GetSize() +
              mGrid.Index(rstate.ObjectPos);
  return index;
}

REDUNDANT_OBJECT_STATE REDUNDANT_OBJECT::Decode(int index) const {
  int agent = index / mGrid.GetSize();
  int object = index % mGrid.GetSize();

  REDUNDANT_OBJECT_STATE rstate;
  rstate.AgentPos = mGrid.Coord(agent);
  rstate.ObjectPos = mGrid.Coord(object);
  return rstate;
}

int REDUNDANT_OBJECT::GetObservation(
    const REDUNDANT_OBJECT_STATE &rstate) const {
  return mStateAbstraction ?
        mGrid.Index(rstate.AgentPos) :  // only agent's position
        Encode(rstate);  // agent's and object's positions
}

void REDUNDANT_OBJECT::DisplayBeliefs(const BELIEF_STATE &belief,
                                      std::ostream &ostr) const {
  unordered_map<int, int> m;
  for (int i = 0; i < belief.GetNumSamples(); ++i) {
    const REDUNDANT_OBJECT_STATE &rstate =
        safe_cast<const REDUNDANT_OBJECT_STATE &>(*belief.GetSample(i));
    int index = Encode(rstate);
    m[index] += 1;
  }

  vector<pair<double, int>> sorted;
  for (unordered_map<int, int>::iterator it = m.begin(); it != m.end();
       ++it) {
    double p = double(it->second) / double(belief.GetNumSamples());
    sorted.push_back(make_pair(p, it->first));
  }
  sort(sorted.begin(), sorted.end(), greater<pair<double, int>>());

  ostr << "#Belief: ";
  for (uint i = 0; i < sorted.size(); ++i) {
    REDUNDANT_OBJECT_STATE state = Decode(sorted[i].second);
    ostr << "#" << state.AgentPos << ":" << state.ObjectPos << " ("
         << sorted[i].first << ") ";
  }
  ostr << std::endl;
}

void REDUNDANT_OBJECT::DisplayState(const STATE &state,
                                    std::ostream &ostr) const {
  const REDUNDANT_OBJECT_STATE &rstate =
      safe_cast<const REDUNDANT_OBJECT_STATE &>(state);

  ostr << "Y" << endl;
  for (int y = mGrid.GetYSize() - 1; y >= 0; --y) {
    for (int x = 0; x < mGrid.GetXSize(); ++x) {
      if (rstate.AgentPos == COORD(x, y)) {
        ostr << "@";
      } else if (rstate.ObjectPos == COORD(x, y)) {
        ostr << "x";
      } else {
        ostr << ".";
      }
    }
    if (y == 0) {
      ostr << "X" << endl;
    } else {
      ostr << endl;
    }
  }
  ostr << endl;
}

void REDUNDANT_OBJECT::DisplayObservation(const STATE &, int observation,
                                          std::ostream &ostr) const {
  if (mStateAbstraction) {
    ostr << "Observation: "
         << "Agent " << mGrid.Coord(observation) << endl;
  } else {
    REDUNDANT_OBJECT_STATE rstate = Decode(observation);
    ostr << "Observation: "
         << "Agent " << rstate.AgentPos << " Object " << rstate.ObjectPos
         << endl;
  }
}

void REDUNDANT_OBJECT::DisplayAction(int action, std::ostream &ostr) const {
  ostr << coord::CompassString[action] << endl;
}
