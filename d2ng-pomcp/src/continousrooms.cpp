#include "continousrooms.h"
#include "utils.h"
#include "distribution.h"
#include <fstream>
#include <vector>
#include <algorithm>
#include <unordered_map>

using namespace std;
using namespace UTILS;

ContinousROOMS::ContinousROOMS(const char *map_name, bool state_abstraction,
                               bool action_abstraction)
    : mGrid(0), mRooms(0), mThreshold(0.1), mSizePerGrid(1.0) {
  Parse(map_name);

  NumActions = 4; //动作数
  NumObservations =
      state_abstraction ? mRooms : -1 /*infinitely many observations*/;
  Discount = 0.95;
  RewardRange = 20.0;
  mName << "continousrooms_" << map_name << "_" << state_abstraction << "_"
        << action_abstraction;

  mHierarchicalPlanning = true;
  mFullyObservable = true;
  mStateAbstraction = state_abstraction;
  mActionAbstraction = action_abstraction;
}

ContinousROOMS::~ContinousROOMS() { delete mGrid; }

void ContinousROOMS::Parse(const char *file_name) {
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

  COORD start_grid, goal_grid;
  fin >> start_grid.X >> start_grid.Y;
  fin.ignore(LINE_MAX, ' ');
  fin >> goal_grid.X >> goal_grid.Y;

  mStartPos = Grid2Position(start_grid);
  mGoalPos = Grid2Position(goal_grid);

  mGrid = new GRID<int>(xsize, ysize);
  mFieldLength = mSizePerGrid * xsize;
  mFieldWidth = mSizePerGrid * ysize;

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

STATE *ContinousROOMS::Copy(const STATE &state) const {
  const ContinousROOMS_STATE &rstate =
      safe_cast<const ContinousROOMS_STATE &>(state);
  ContinousROOMS_STATE *newstate = mMemoryPool.Allocate();
  *newstate = rstate;
  return newstate;
}

COORD ContinousROOMS::Position2Grid(const Vector &o) const {
  return COORD(floor(o.X() / mSizePerGrid), floor(o.Y() / mSizePerGrid));
}

Vector ContinousROOMS::Grid2Position(const COORD &o) const {
  return Vector((o.X + 0.5) * mSizePerGrid, (o.Y + 0.5) * mSizePerGrid);
}

void ContinousROOMS::Validate(const STATE &state) const {
  const ContinousROOMS_STATE &rstate =
      safe_cast<const ContinousROOMS_STATE &>(state);
  assert(IsValid(rstate.AgentPos));
}

bool ContinousROOMS::IsValid(const Vector &pos) const
{
  return mGrid->Inside(Position2Grid(pos)) &&
      pos.X() >= 0.0 && pos.Y() <= mFieldLength &&
      pos.Y() >= 0.0 && pos.Y() <= mFieldWidth;
}

STATE *ContinousROOMS::CreateStartState() const {
  ContinousROOMS_STATE *rstate = mMemoryPool.Allocate();
  rstate->AgentPos = mStartPos;
  return rstate;
}

void ContinousROOMS::FreeState(STATE *state) const {
  ContinousROOMS_STATE *rstate = safe_cast<ContinousROOMS_STATE *>(state);
  mMemoryPool.Free(rstate);
}

bool ContinousROOMS::Step(STATE &state, int action, int &observation,
                          double &reward) const {
  assert(action < NumActions);
  Validate(state);

  ContinousROOMS_STATE &rstate = safe_cast<ContinousROOMS_STATE &>(state);
  reward = -1.0;

  if (SimpleRNG::ins().Bernoulli(0.2)) { // fail
    action = SimpleRNG::ins().Random(NumActions);
  }

  COORD agent_grid = Position2Grid(rstate.AgentPos);
  COORD target_grid = agent_grid + coord::Compass[action];
  if (!IsValid(Grid2Position(target_grid)) ||
      mGrid->operator ()(target_grid) == 'x') { // not valid
    target_grid = agent_grid;
  }

  do {
    rstate.AgentPos = Grid2Position(target_grid) +
                      Vector(SimpleRNG::ins().GetNormal(0.0, 0.1),
                             SimpleRNG::ins().GetNormal(0.0, 0.1));
  } while(!IsValid(rstate.AgentPos));
  observation = GetObservation(rstate);

  if (rstate.AgentPos.Dist(mGoalPos) < mThreshold) {
    reward = 10.0;
    return true;
  }

  return false; // not terminated
}

bool ContinousROOMS::LocalMove(STATE &state, const HISTORY &history,
                               int) const //局部扰动
{
  ContinousROOMS_STATE rstate = safe_cast<ContinousROOMS_STATE &>(state);
  if (GetObservation(rstate) == history.Back().Observation) {
    return true;
  }
  return false;
}

void ContinousROOMS::GenerateLegal(const STATE &state,
                                   vector<int> &legal) const {
  Validate(state);

  legal.push_back(coord::E_NORTH);
  legal.push_back(coord::E_EAST);
  legal.push_back(coord::E_SOUTH);
  legal.push_back(coord::E_WEST);
}

void ContinousROOMS::GeneratePreferred(
    const STATE &state, const HISTORY &, //手工策略
    vector<int> &actions) const          //获得优先动作
{
  GenerateLegal(state, actions);
}

int ContinousROOMS::GetObservation(const ContinousROOMS_STATE &rstate) const {
  COORD agent_grid = Position2Grid(rstate.AgentPos);
  return mStateAbstraction
             ? mGrid->operator()(agent_grid) - '0'    // room number
             : hash_value(rstate.AgentPos) % INT_MAX; // full position
}

void ContinousROOMS::DisplayBeliefs(const BELIEF_STATE &belief,
                                    std::ostream &ostr) const {
  unordered_map<Vector, int> m;
  for (int i = 0; i < belief.GetNumSamples(); ++i) {
    const ContinousROOMS_STATE &state =
        safe_cast<const ContinousROOMS_STATE &>(*belief.GetSample(i));
    m[state.AgentPos] += 1;
  }

  vector<pair<double, const Vector *>> sorted;
  for (unordered_map<Vector, int>::iterator it = m.begin(); it != m.end();
       ++it) {
    double p = double(it->second) / double(belief.GetNumSamples());
    sorted.push_back(make_pair(p, &(it->first)));
  }
  sort(sorted.begin(), sorted.end(), greater<pair<double, const Vector *>>());

  ostr << "#Belief: ";
  for (uint i = 0; i < sorted.size(); ++i) {
    ostr << "#" << *(sorted[i].second) << " (" << sorted[i].first << ") ";
  }
  ostr << std::endl;
}

void ContinousROOMS::DisplayState(const STATE &state,
                                  std::ostream &ostr) const {
  const ContinousROOMS_STATE &rstate =
      safe_cast<const ContinousROOMS_STATE &>(state);

  ostr << "Y" << endl;
  COORD agent_grid = Position2Grid(rstate.AgentPos);

  for (int y = mGrid->GetYSize() - 1; y >= 0; --y) {
    for (int x = 0; x < mGrid->GetXSize(); ++x) {
      char cell = mGrid->operator()(x, y);

      if (agent_grid == COORD(x, y)) {
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
  ostr << "AgentPos=" << rstate.AgentPos << endl;
}

void ContinousROOMS::DisplayObservation(const STATE &, int observation,
                                        std::ostream &ostr) const {
  if (mStateAbstraction)
    ostr << "Observation: "
         << "Room " << char(observation + '0') << endl;
  else
    ostr << "Observation: "
         << "Hash(AgentPosition) " << observation << endl;
}

void ContinousROOMS::DisplayAction(int action, std::ostream &ostr) const {
  ostr << coord::CompassString[action] << endl;
}
