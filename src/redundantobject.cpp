#include "redundantobject.h"
#include "hierarchicalmcts.h"

using namespace std;
using namespace utils;

REDUNDANT_OBJECT::REDUNDANT_OBJECT(int size, bool state_abstraction)
  : mGrid(size, size),
    mStartPos(0, 0),
    mGoalPos(size - 1, size - 1) {
  NumActions = 8;
  Discount = 0.98;
  RewardRange = 20.0;

  if (state_abstraction) {
    mName << "redundant_object_" << size << " w/ state abstraction";
  }
  else {
    mName << "redundant_object_" << size << " wo/ state abstraction";
  }

  mHierarchicalPlanning = true;
  mFullyObservable = true;
  mStateAbstraction = state_abstraction;
}

REDUNDANT_OBJECT::~REDUNDANT_OBJECT() { }

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
  for (auto & o: rstate->ObjectPos) {
    o = mGoalPos;
  }
  return rstate;
}

void REDUNDANT_OBJECT::FreeState(STATE *state) const {
  REDUNDANT_OBJECT_STATE *rstate = safe_cast<REDUNDANT_OBJECT_STATE *>(state);
  mMemoryPool.Free(rstate);
}

bool REDUNDANT_OBJECT::Step(STATE &state, int action, int &observation, double &reward) const {
  assert(action < NumActions);
  Validate(state);

  REDUNDANT_OBJECT_STATE &rstate = safe_cast<REDUNDANT_OBJECT_STATE &>(state);
  reward = -1.0;

  if (SimpleRNG::ins().Bernoulli(0.2)) { // fail
    action = SimpleRNG::ins().Random(GetNumActions());
  }

  COORD pos = rstate.AgentPos + coord::Compass[action];
  if (!mGrid.Inside(pos)) {
    pos = rstate.AgentPos;
  }
  rstate.AgentPos = pos;

  for (auto & o: rstate.ObjectPos) {
    action = SimpleRNG::ins().Random(GetNumActions());
    pos = o + coord::Compass[action];
    if (!mGrid.Inside(pos)) {
      pos = o;
    }
    o = pos;
  }

  observation = GetObservation(rstate);

  if (rstate.AgentPos == mGoalPos) {
    reward = 10.0;
    return true;
  }

  return false;
}

bool REDUNDANT_OBJECT::LocalMove(STATE &state, const HISTORY &history, int) const
{
  REDUNDANT_OBJECT_STATE rstate = safe_cast<REDUNDANT_OBJECT_STATE &>(state);
  if (GetObservation(rstate) == history.Back().Observation) {
    return true;
  }
  return false;
}

void REDUNDANT_OBJECT::GenerateLegal(const STATE &state, vector<int> &legal) const {
  Validate(state);

  legal.push_back(coord::E_NORTH);
  legal.push_back(coord::E_EAST);
  legal.push_back(coord::E_SOUTH);
  legal.push_back(coord::E_WEST);
  legal.push_back(coord::E_NORTHEAST);
  legal.push_back(coord::E_NORTHWEST);
  legal.push_back(coord::E_SOUTHEAST);
  legal.push_back(coord::E_SOUTHWEST);
}

void REDUNDANT_OBJECT::GeneratePreferred(
    const STATE &state, const HISTORY &,
    vector<int> &actions) const
{
  GenerateLegal(state, actions);
}

int REDUNDANT_OBJECT::GetObservation(
    const REDUNDANT_OBJECT_STATE &rstate) const {
  if (mStateAbstraction) {
    if (rstate.AgentPos == mGoalPos) {
      return HierarchicalMCTS::ABSTRACT_GOAL; // special case
    }
    else {
      return HierarchicalMCTS::ABSTRACT_GOAL + 1 + mGrid.Index(rstate.AgentPos);
    }
  }
  else {
    return std::abs(int(rstate.hash() % numeric_limits<int>::max())); // full state
  }
}

void REDUNDANT_OBJECT::DisplayBeliefs(const BELIEF_STATE &/*belief*/,
                                      std::ostream &/*ostr*/) const {
//  unordered_map<int, int> m;
//  for (int i = 0; i < belief.GetNumSamples(); ++i) {
//    const REDUNDANT_OBJECT_STATE &rstate =
//        safe_cast<const REDUNDANT_OBJECT_STATE &>(*belief.GetSample(i));
//    int index = rstate.Encode();
//    m[index] += 1;
//  }

//  vector<pair<double, int>> sorted;
//  for (unordered_map<int, int>::iterator it = m.begin(); it != m.end();
//       ++it) {
//    double p = double(it->second) / double(belief.GetNumSamples());
//    sorted.push_back(make_pair(p, it->first));
//  }
//  sort(sorted.begin(), sorted.end(), greater<pair<double, int>>());

//  ostr << "#Belief: ";
//  for (uint i = 0; i < sorted.size(); ++i) {
//    auto s = REDUNDANT_OBJECT_STATE::Decode(sorted[i].second);
//    ostr << "#" << s.first << ":" << s.second << " ("
//         << sorted[i].first << ") ";
//  }
//  ostr << std::endl;
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
      } else if (std::find(rstate.ObjectPos.begin(), rstate.ObjectPos.end(), COORD(x, y)) != rstate.ObjectPos.end()) {
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
    if (observation == HierarchicalMCTS::ABSTRACT_GOAL) {
      ostr << "Observation: "
           << "Agent " << mGoalPos << endl;
    }
    else {
      ostr << "Observation: "
           << "Agent " << mGrid.Coord(observation - '1') << endl;
    }
  } else {
    ostr << "Observation: " << observation << endl;
  }
}

void REDUNDANT_OBJECT::DisplayAction(int action, std::ostream &ostr) const {
  ostr << coord::CompassString[action] << endl;
}
