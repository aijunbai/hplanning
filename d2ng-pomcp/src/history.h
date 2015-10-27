#ifndef HISTORY_H
#define HISTORY_H

#include <vector>
#include <ostream>
#include <assert.h>
#include <cstdlib>
#include <unordered_set>
#include <boost/functional/hash.hpp>
#include "utils.h"

class HISTORY {
 public:
  struct ENTRY {
    ENTRY(): Action(-1), Observation(-1), BeliefHash(0) {}
    ENTRY(int action, int obs)
      : Action(action), Observation(obs), BeliefHash(0) {}

    int Action;
    int Observation;
    size_t BeliefHash;  // hash value of the history up to current position (size dependant)
  };

  bool operator==(const HISTORY &history) const {
    assert(0);
    return BeliefHash() == history.BeliefHash();
  }

  void Add(int action, int obs) {
    History.push_back(ENTRY(action, obs));
    History.back().BeliefHash = belief_hash();
  }

  size_t BeliefHash() const {
    if (History.size()) {
      return History.back().BeliefHash;
    }
    return 0;
  }

  void Pop() { History.pop_back(); }

  void Truncate(int t) { History.resize(t); }

  void Clear() { History.clear(); }

  int Size() const { return History.size(); }

  ENTRY &operator[](uint t) {
    assert(t < History.size());
    return History.at(t);
  }

  const ENTRY &operator[](uint t) const {
    assert(t < History.size());
    return History.at(t);
  }

  ENTRY &Back() {
    assert(History.size() > 0);
    return History.back();
  }

  const ENTRY &Back() const {
    assert(History.size() > 0);
    return History.back();
  }

  void Display(std::ostream &ostr) const {
    for (uint t = 0; t < History.size(); ++t) {
      ostr << "a=" << History[t].Action << ", ";
      if (History[t].Observation >= 0)
        ostr << "o=" << History[t].Observation << ", ";
    }
  }

private:
  size_t belief_hash() const {
    assert(Size() >= 1);
    using boost::hash_combine;

    std::size_t seed = Size() >= 2? History[Size() - 2].BeliefHash: 0;
    hash_combine(seed, boost::hash_value(History[Size() - 1].Observation));
    hash_combine(seed, boost::hash_value(History[Size() - 1].Action));

    return seed;
  }

 private:
  std::vector<ENTRY> History;
};

#endif  // HISTORY
