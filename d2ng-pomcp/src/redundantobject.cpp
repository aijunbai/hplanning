#include "redundantobject.h"
#include "utils.h"
#include "distribution.h"
#include <fstream>
#include <vector>
#include <algorithm>
#include <boost/unordered_map.hpp>

using namespace std;
using namespace UTILS;

REDUNDANT_OBJECT::REDUNDANT_OBJECT(int size, bool state_abstraction):
    mStateAbstraction(state_abstraction),
    mGrid(size, size),
    mStartPos(0, 0),
    mGoalPos(size-1, size-1)
{
    NumActions = 2; //动作数
    int grids = mGrid.GetSize();
    NumObservations = mStateAbstraction? grids: grids * grids;
    Discount = 0.99;
    mName << "redundant_object_" << size << "_" << state_abstraction;
    mHierarchicalPlanning = true;
}

REDUNDANT_OBJECT::~REDUNDANT_OBJECT()
{

}


STATE* REDUNDANT_OBJECT::Copy(const STATE& state) const
{
    const REDUNDANT_OBJECT_STATE& rstate = safe_cast<const REDUNDANT_OBJECT_STATE&>(state);
    REDUNDANT_OBJECT_STATE* newstate = mMemoryPool.Allocate();
    *newstate = rstate;
    return newstate;
}

void REDUNDANT_OBJECT::Validate(const STATE& state) const
{
    const REDUNDANT_OBJECT_STATE& rstate = safe_cast<const REDUNDANT_OBJECT_STATE&>(state);
    assert(mGrid.Inside(rstate.AgentPos));
    assert(mGrid.Inside(rstate.ObjectPos));
}

STATE* REDUNDANT_OBJECT::CreateStartState() const
{
    REDUNDANT_OBJECT_STATE* rstate = mMemoryPool.Allocate();
    rstate->AgentPos = mStartPos;
    rstate->ObjectPos = mStartPos;
    return rstate;
}

void REDUNDANT_OBJECT::FreeState(STATE* state) const
{
    REDUNDANT_OBJECT_STATE* rstate = safe_cast<REDUNDANT_OBJECT_STATE*>(state);
    mMemoryPool.Free(rstate);
}

bool REDUNDANT_OBJECT::Step(STATE& state, int action,
    int& observation, double& reward) const //进行一步模拟：state, action |-> state, reward, observation
{
    assert(action < NumActions);

    REDUNDANT_OBJECT_STATE& rstate = safe_cast<REDUNDANT_OBJECT_STATE&>(state);
    reward = action == COORD::E_NORTH? -0.5: -1.0;

    if (SimpleRNG::ins().Bernoulli(0.1)) { //fail
        action = action == COORD::E_NORTH? COORD::E_EAST: COORD::E_NORTH;
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

    observation = GetObservation(rstate);

    if (rstate.AgentPos == mGoalPos) {
        return true;
    }

    return false; //not terminated
}

bool REDUNDANT_OBJECT::LocalMove(STATE& state, const HISTORY& history, int, const STATUS& ) const //局部扰动
{
    REDUNDANT_OBJECT_STATE rstate = safe_cast<REDUNDANT_OBJECT_STATE&>(state);
    if (GetObservation(rstate) == history.Back().Observation) {
        return true;
    }
    return false;
}

void REDUNDANT_OBJECT::GenerateLegal(const STATE& state, /*const HISTORY& ,*/
    vector<int>& legal, const STATUS& ) const
{
    const REDUNDANT_OBJECT_STATE& rstate =
            safe_cast<const REDUNDANT_OBJECT_STATE&>(state);

    assert(mGrid.Inside(rstate.AgentPos));

    legal.push_back(COORD::E_NORTH);
    legal.push_back(COORD::E_EAST);
}

void REDUNDANT_OBJECT::GeneratePreferred(const STATE& state, const HISTORY&, //手工策略
    vector<int>& actions, const STATUS& status) const //获得优先动作
{
    GenerateLegal(state, actions, status);
}

int REDUNDANT_OBJECT::GetObservation(const REDUNDANT_OBJECT_STATE& rstate) const
{
    return mStateAbstraction? mGrid.Index(rstate.AgentPos):
                              (mGrid.Index(rstate.AgentPos) * mGrid.GetSize() + mGrid.Index(rstate.ObjectPos));
}

void REDUNDANT_OBJECT::DisplayBeliefs(const BELIEF_STATE& belief,
    std::ostream& ostr) const
{
    boost::unordered_map<int, int> m;
    for (int i = 0; i < belief.GetNumSamples(); ++i) {
        const REDUNDANT_OBJECT_STATE& state = safe_cast<const REDUNDANT_OBJECT_STATE&>(*belief.GetSample(i));
        int index = mGrid.Index(state.AgentPos) * mGrid.GetSize() + mGrid.Index(state.ObjectPos);
        m[index] += 1;
    }

    vector<pair<double, int> > sorted;
    for (boost::unordered_map<int, int>::iterator it = m.begin(); it != m.end(); ++it) {
        double p = double(it->second) / double(belief.GetNumSamples());
        sorted.push_back(make_pair(p, it->first));
    }
    sort(sorted.begin(), sorted.end(), greater<pair<double, int> >());

    ostr << "#Belief: ";
    for (uint i = 0; i < sorted.size(); ++i) {
        int index = sorted[i].second;
        int agent = index / mGrid.GetSize();
        int object = index % mGrid.GetSize();
        ostr << "#" << mGrid.Coord(agent) << ":" << mGrid.Coord(object) << " (" << sorted[i].first << ") ";
    }
    ostr << std::endl;
}

void REDUNDANT_OBJECT::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const REDUNDANT_OBJECT_STATE& REDUNDANT_OBJECT_state = safe_cast<const REDUNDANT_OBJECT_STATE&>(state);

    ostr << "Y" << endl;
    for (int y = mGrid.GetYSize() - 1; y >= 0; --y) {
        for (int x = 0; x < mGrid.GetXSize(); ++x) {
            char cell = mGrid.operator()(x, y);

            if (REDUNDANT_OBJECT_state.AgentPos == COORD(x, y)) {
                ostr << "@";
            }
            else if (cell != 'x') {
                ostr << ".";
            }
            else {
                ostr << cell;
            }
        }
        if (y == 0) {
            ostr << "X" << endl;
        }
        else {
            ostr << endl;
        }
    }
    ostr << endl;
}

void REDUNDANT_OBJECT::DisplayObservation(const STATE&, int observation, std::ostream& ostr) const
{
  if (mStateAbstraction) {
    ostr << "Observation: " << "Agent " << mGrid.Coord(observation) << endl;
  }
  else {
      int agent = observation / mGrid.GetSize();
      int object = observation % mGrid.GetSize();
    ostr << "Observation: " << "Agent " << mGrid.Coord(agent)
         << " Object " << mGrid.Coord(object) << endl;
  }
}

void REDUNDANT_OBJECT::DisplayAction(int action, std::ostream& ostr) const
{
    ostr << coord::CompassString[action] << endl;
}
