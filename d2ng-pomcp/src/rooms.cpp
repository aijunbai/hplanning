#include "rooms.h"
#include "utils.h"
#include "distribution.h"
#include <fstream>
#include <vector>
#include <algorithm>
#include <unordered_map>

using namespace std;
using namespace UTILS;

ROOMS::ROOMS(const char *map_name, bool state_abstraction, bool action_abstraction)
    :mGrid(0), mRooms(0) {
  Parse(map_name);

  NumActions = 4;  //动作数
  NumObservations =
      state_abstraction ? mRooms : mGrid->GetXSize() * mGrid->GetYSize();
  Discount = 0.95;
  RewardRange = 10.0;
  mName << "rooms_" << map_name << "_" << state_abstraction << "_" << action_abstraction;

  mHierarchicalPlanning = true;
  mStateAbstraction = state_abstraction;
  mActionAbstraction = action_abstraction;
}

ROOMS::~ROOMS() { delete mGrid; }

void ROOMS::Parse(const char *file_name) {
  ifstream fin(file_name);

  if (!fin) {
    cerr << "can not open map file: " << file_name << endl;
    return;
  }

  uint xsize, ysize;

  fin.ignore(LINE_MAX, ' ');
  fin >> xsize >> ysize;
  fin.ignore(LINE_MAX, ' ');
  fin >> mRooms;
  fin.ignore(LINE_MAX, ' ');
  fin >> mStartPos.X >> mStartPos.Y;
  fin.ignore(LINE_MAX, ' ');
  fin >> mGoalPos.X >> mGoalPos.Y;

  mGrid = new GRID<int>(xsize, ysize);

  int row = 0;
  string line;

  while (getline(fin, line)) {
    if (line.size() < ysize) {
      continue;
    }

    for (uint col = 0; col < line.size(); ++col) {
      int x = col;
      int y = ysize - 1 - row;
      mGrid->operator()(x, y) = line[col];
    }
    row += 1;
  }
}

STATE *ROOMS::Copy(const STATE &state) const {
  const ROOMS_STATE &rstate = safe_cast<const ROOMS_STATE &>(state);
  ROOMS_STATE *newstate = mMemoryPool.Allocate();
  *newstate = rstate;
  return newstate;
}

void ROOMS::Validate(const STATE &state) const {
  const ROOMS_STATE &rstate = safe_cast<const ROOMS_STATE &>(state);
  assert(mGrid->Inside(rstate.AgentPos));
}

STATE *ROOMS::CreateStartState() const {
  ROOMS_STATE *rstate = mMemoryPool.Allocate();
  rstate->AgentPos = mStartPos;
  return rstate;
}

void ROOMS::FreeState(STATE *state) const {
  ROOMS_STATE *rstate = safe_cast<ROOMS_STATE *>(state);
  mMemoryPool.Free(rstate);
}

bool ROOMS::Step(STATE &state, int action, int &observation, double &reward) const
{
  assert(action < NumActions);

  ROOMS_STATE &rstate = safe_cast<ROOMS_STATE &>(state);
  reward = -1.0;

  if (SimpleRNG::ins().Bernoulli(1.0 / 3.0)) {  // fail
    action = SimpleRNG::ins().Random(NumActions);
  }

  COORD pos = rstate.AgentPos + coord::Compass[action];
  if (mGrid->operator()(pos) != 'x') {  // not wall
    rstate.AgentPos = pos;
  }
  observation = GetObservation(rstate);

  if (rstate.AgentPos == mGoalPos) {
    return true;
  }

  return false;  // not terminated
}

bool ROOMS::LocalMove(STATE &state, const HISTORY &history, int) const  //局部扰动
{
  ROOMS_STATE rstate = safe_cast<ROOMS_STATE &>(state);
  if (GetObservation(rstate) == history.Back().Observation) {
    return true;
  }
  return false;
}

void ROOMS::GenerateLegal(const STATE &state, vector<int> &legal) const {
  const ROOMS_STATE &rstate = safe_cast<const ROOMS_STATE &>(state);

  assert(mGrid->Inside(rstate.AgentPos));

  legal.push_back(COORD::E_NORTH);
  legal.push_back(COORD::E_EAST);
  legal.push_back(COORD::E_SOUTH);
  legal.push_back(COORD::E_WEST);
}

void ROOMS::GeneratePreferred(const STATE &state, const HISTORY &,  //手工策略
                              vector<int> &actions) const  //获得优先动作
{
  GenerateLegal(state, actions);
}

int ROOMS::GetObservation(const ROOMS_STATE &rstate) const {
  return mStateAbstraction ? mGrid->operator()(rstate.AgentPos) - '0'  // room number
                           : mGrid->Index(rstate.AgentPos);  // full position
}

void ROOMS::DisplayBeliefs(const BELIEF_STATE &belief, std::ostream &ostr) const {
  unordered_map<COORD, int> m;
  for (int i = 0; i < belief.GetNumSamples(); ++i) {
    const ROOMS_STATE &state =
        safe_cast<const ROOMS_STATE &>(*belief.GetSample(i));
    m[state.AgentPos] += 1;
  }

  vector<pair<double, const COORD *>> sorted;
  for (unordered_map<COORD, int>::iterator it = m.begin(); it != m.end();
       ++it) {
    double p = double(it->second) / double(belief.GetNumSamples());
    sorted.push_back(make_pair(p, &(it->first)));
  }
  sort(sorted.begin(), sorted.end(), greater<pair<double, const COORD *>>());

  ostr << "#Belief: ";
  for (uint i = 0; i < sorted.size(); ++i) {
    ostr << "#" << *(sorted[i].second) << " (" << sorted[i].first << ") ";
  }
  ostr << std::endl;
}

void ROOMS::DisplayState(const STATE &state, std::ostream &ostr) const {
  const ROOMS_STATE &rstate = safe_cast<const ROOMS_STATE &>(state);

  ostr << "Y" << endl;
  for (int y = mGrid->GetYSize() - 1; y >= 0; --y) {
    for (int x = 0; x < mGrid->GetXSize(); ++x) {
      char cell = mGrid->operator()(x, y);

      if (rstate.AgentPos == COORD(x, y)) {
        ostr << "@";
      } else if (cell != 'x') {
        ostr << ".";
      } else {
        ostr << cell;
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

void ROOMS::DisplayObservation(const STATE &, int observation,
                               std::ostream &ostr) const {
  if (mStateAbstraction)
    ostr << "Observation: "
         << "Room " << char(observation + '0') << endl;
  else
    ostr << "Observation: "
         << "Coord " << mGrid->Coord(observation) << endl;
}

void ROOMS::DisplayAction(int action, std::ostream &ostr) const {
  ostr << coord::CompassString[action] << endl;
}
