#include "continousrooms.h"

using namespace std;
using namespace UTILS;

ContinousROOMS::ContinousROOMS(const char *map_name, bool state_abstraction)
    : mGrid(0), mRooms(0), mThreshold(0.25), mMotionUncertainty(0.25),
      mSizePerGrid(1.0) {
  Parse(map_name);

  NumActions = 4; //动作数
  NumObservations = state_abstraction ? mRooms : numeric_limits<int>::max();
  Discount = 0.98;
  RewardRange = 20.0;

  if (state_abstraction) {
    mName << "continousrooms @ " << map_name << " w/ state abstraction";
  }
  else {
    mName << "continousrooms @ " << map_name << " wo/ state abstraction";
  }

  mHierarchicalPlanning = true;
  mFullyObservable = true;
  mStateAbstraction = state_abstraction;
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

bool ContinousROOMS::IsValid(const Vector &pos) const {
  return mGrid->Inside(Position2Grid(pos)) && pos.X() >= 0.0 &&
         pos.Y() <= mFieldLength && pos.Y() >= 0.0 && pos.Y() <= mFieldWidth &&
         mGrid->operator()(Position2Grid(pos)) != 'x';
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

bool ContinousROOMS::Step(STATE &state, int action, int &observation, double &reward) const {
  assert(action < NumActions);
  Validate(state);

  ContinousROOMS_STATE &rstate = safe_cast<ContinousROOMS_STATE &>(state);
  reward = -1.0;

  for (int i = 0; i < Knowledge.mBranchingFactor; ++i) {
    if (SimpleRNG::ins().Bernoulli(0.25)) {  // fail
      action = SimpleRNG::ins().Random(8);
    }

    Vector pos = rstate.AgentPos + Vector(coord::Compass[action].X, coord::Compass[action].Y);
    if (IsValid(pos) && mGrid->operator()(Position2Grid(pos)) != 'x') {  // not wall
      rstate.AgentPos = pos;
    }

    do { // add noise
      pos = rstate.AgentPos + Vector(SimpleRNG::ins().GetNormal(0.0, mMotionUncertainty),
                                     SimpleRNG::ins().GetNormal(0.0, mMotionUncertainty));
    } while (Position2Grid(pos) != Position2Grid(rstate.AgentPos));
    rstate.AgentPos = pos;
  }

//  for (int i = 0; i < Knowledge.mBranchingFactor; ++i) {
//    Vector pos = rstate.AgentPos + Vector(coord::Compass[action].X, coord::Compass[action].Y);
//    if (IsValid(pos) && mGrid->operator()(Position2Grid(pos)) != 'x') {  // not wall
//      rstate.AgentPos = pos;
//
//      if (SimpleRNG::ins().Bernoulli(0.2)) {  // fail
//        if (SimpleRNG::ins().Bernoulli(0.5)) {
//          action = coord::Clockwise(action);
//        }
//        else {
//          action = coord::Anticlockwise(action);
//        }
//
//        pos = rstate.AgentPos + Vector(coord::Compass[action].X, coord::Compass[action].Y);
//        if (IsValid(pos) && mGrid->operator()(Position2Grid(pos)) != 'x') {  // not wall
//          rstate.AgentPos = pos;
//        }
//      }
//    }
//
//    do { // add noise
//      pos = rstate.AgentPos + Vector(SimpleRNG::ins().GetNormal(0.0, mMotionUncertainty),
//                                     SimpleRNG::ins().GetNormal(0.0, mMotionUncertainty));
//    } while (Position2Grid(pos) != Position2Grid(rstate.AgentPos));
//    rstate.AgentPos = pos;
//  }

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
  return mStateAbstraction ?
         mGrid->operator()(Position2Grid(rstate.AgentPos)) : // room number
         hash_value(rstate.AgentPos); // full position
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
  ostr << "GoalDist=" << rstate.AgentPos.Dist(mGoalPos) << endl;
}

void ContinousROOMS::DisplayObservation(const STATE &, int observation,
                                        std::ostream &ostr) const {
  if (mStateAbstraction)
    ostr << "Observation: "
    << "Room " << char(observation) << endl;
  else
    ostr << "Observation: "
         << "Hash(AgentPosition) " << observation << endl;
}

void ContinousROOMS::DisplayAction(int action, std::ostream &ostr) const {
  ostr << coord::CompassString[action] << endl;
}
