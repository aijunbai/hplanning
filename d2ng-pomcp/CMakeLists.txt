cmake_minimum_required(VERSION 2.8)
project(d2ng_pomcp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -pipe -std=c++11 -Wall -Wextra -O2 -Wall -W")

find_package( Boost COMPONENTS program_options REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )

set(SOURCE_FILES
    src/battleship.cpp
    src/battleship.h
    src/beliefstate.cpp
    src/beliefstate.h
    src/continousrooms.cpp
    src/continousrooms.h
    src/coord.cpp
    src/coord.h
    src/distribution.cpp
    src/distribution.h
    src/dot_graph.h
    src/experiment.cpp
    src/experiment.h
    src/flatmcts.cpp
    src/flatmcts.h
    src/grid.h
    src/hierarchicalmcts.cpp
    src/hierarchicalmcts.h
    src/history.h
    src/main.cpp
    src/mcts.cpp
    src/mcts.h
    src/memorypool.h
    src/network.cpp
    src/network.h
    src/node.cpp
    src/node.h
    src/pocman.cpp
    src/pocman.h
    src/prettyprint.h
    src/redundantobject.cpp
    src/redundantobject.h
    src/rocksample.cpp
    src/rocksample.h
    src/rooms.cpp
    src/rooms.h
    src/simulator.cpp
    src/simulator.h
    src/statistic.h
    src/tag.cpp
    src/tag.h
    src/testsimulator.cpp
    src/testsimulator.h
    src/utils.cpp
    src/utils.h)

add_executable(d2ng_pomcp ${SOURCE_FILES})
target_link_libraries(d2ng_pomcp ${Boost_LIBRARIES})
