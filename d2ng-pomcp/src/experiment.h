#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include "hierarchicalmcts.h"
#include "simulator.h"
#include "statistic.h"
#include <fstream>

//----------------------------------------------------------------------------

struct RESULTS
{
    void Clear();

    STATISTIC Time;
    STATISTIC TimePerAction;
    STATISTIC Reward;
    STATISTIC DiscountedReturn;
    STATISTIC UndiscountedReturn;
};

inline void RESULTS::Clear()
{
    Time.Initialise();
    TimePerAction.Initialise();
    Reward.Initialise();
    DiscountedReturn.Initialise();
    UndiscountedReturn.Initialise();
}

//----------------------------------------------------------------------------

class EXPERIMENT
{
public:

    struct PARAMS
    {
        PARAMS();

        int NumRuns;
        int NumSteps;
        double TimeOut;
        int MinDoubles, MaxDoubles;
        int TransformDoubles;
        int TransformAttempts;
        double Accuracy;
        int UndiscountedHorizon;
    };

    EXPERIMENT(const SIMULATOR& real, const SIMULATOR& simulator,
        const std::string& outputFile,
        EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams);

    void Run();
    void MultiRun();
    void DiscountedReturn();
//    void AverageReward();

private:

    const SIMULATOR& Real;
    const SIMULATOR& Simulator;
    EXPERIMENT::PARAMS& ExpParams;
    MCTS::PARAMS& SearchParams;
    RESULTS Results;

    std::ofstream OutputFile;
};

//----------------------------------------------------------------------------

#endif // EXPERIMENT_H
