# hplanning

[![Build Status](https://travis-ci.org/aijunbai/hplanning.svg?branch=master)](https://travis-ci.org/aijunbai/hplanning)  

This is the code release of paper:
* [Markovian State and Action Abstractions for MDPs via Hierarchical MCTS](http://aijunbai.github.io/publications/IJCAI16-Bai.pdf), Aijun Bai, Siddharth Srivastava, and Stuart Russell, Proceedings of the 25th International Joint Conference on Artificial Intelligence (IJCAI), New York, 2016. 

# Dependencies
- qt4-qmake 
- libboost-dev 
- libboost-program-options-dev

# Usages
- `make` to build `hplanning`
- `./run.sh` to run a problem instance with default settings
- `./debug.sh` to build and run using debug version
- `./release.sh` to build and run using release version

# Options
Allowed options of `hplanning`:
```
  --help                         produce help message
  --test                         run unit tests
  --problem arg                  problem to run
  --map arg                      map to use for (continus) rooms domain
  --outputfile arg (=output.txt) summary output file
  --size arg                     size of problem (problem specific)
  --number arg                   number of elements in problem (problem 
                                 specific)
  --timeout arg                  timeout (seconds)
  --mindoubles arg               minimum power of two simulations
  --maxdoubles arg               maximum power of two simulations
  --runs arg                     number of runs
  --accuracy arg                 accuracy level used to determine horizon
  --horizon arg                  horizon to use when not discounting
  --num steps arg                number of steps to run when using average 
                                 reward
  --verbose arg                  verbosity level
  --usetransforms arg            Use transforms
  --useparticlefilter arg        Use particle fileter
  --transformdoubles arg         Relative power of two for transforms compared 
                                 to simulations
  --transformattempts arg        Number of attempts for each transform
  --treeknowledge arg            Knowledge level in tree (0=Pure, 1=Legal, 
                                 2=Smart)
  --rolloutknowledge arg         Knowledge level in rollouts (0=Pure, 1=Legal, 
                                 2=Smart)
  --smarttreecount arg           Prior count for preferred actions during smart
                                 tree search
  --smarttreevalue arg           Prior value for preferred actions during smart
                                 tree search
  --reusetree arg                Reuse tree generated during previous search
  --seeding arg                  Use pid as random seed
  --thompsonsampling arg         use Thompson Sampling instead of UCB1
  --timeoutperaction arg         timeout per action (seconds)
  --polling arg                  use polling rollout for hplanning
  --stack arg                    use call stack for hplanning
  --localreward arg              use local reward
  --hplanning arg                use hplanning when possible
  --actionabstraction arg        use hplanning w/ action abstraction when 
                                 possible
  --memoryless arg               find a memoryless policy in hplanning
```

