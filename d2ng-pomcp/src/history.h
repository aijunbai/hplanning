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
    ENTRY(int action = -1, int obs = -1)
      : Action(action), Observation(obs), MemoryHash(0) {}

    int Action;
    int Observation;
    size_t MemoryHash;  // hash value of the history up to current position
//    std::unordered_set<int> Visited;
  };

  bool operator==(const HISTORY &history) const {
    assert(0);
    return BeliefHash() == history.BeliefHash();
  }

  void Add(int action, int obs, int memory_size) {
//    std::unordered_set<int> visited;
//    if (Size()) {
//      visited = History.back().Visited;
//    }
//    visited.insert(obs);

    History.push_back(ENTRY(action, obs));
    History.back().MemoryHash = memory_hash(memory_size);
//    History.back().Visited = visited;
  }

//  bool Visited(int obs) const {
//    if (Size()) {
//      return History.back().Visited.count(obs);
//    }
//    return false;
//  }

  size_t BeliefHash() const {
    using boost::hash_combine;
    std::size_t seed = 0;

    if (Size()) {
      hash_combine(seed, History.back().MemoryHash); //fixed memory hash
      hash_combine(seed, boost::hash_value(Size())); //depth dependent
    }

    return seed;
  }

  void Pop() { History.pop_back(); }

  void Truncate(int t) { History.resize(t); }

  void Clear() { History.clear(); }

  int Size() const { return History.size(); }

  ENTRY &operator[](uint t) {
    assert(t < History.size());
    return History[t];
  }

  const ENTRY &operator[](uint t) const {
    assert(t < History.size());
    return History[t];
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
  virtual size_t memory_hash(int memory_size) const {
    assert(Size() >= 1);

    using boost::hash_combine;
    std::size_t seed = 0;

    if (memory_size < 0) {  // use whole history
      seed = Size() >= 2? History[Size() - 2].MemoryHash: 0;
      hash_combine(seed, boost::hash_value(History[Size() - 1].Observation));
      hash_combine(seed, boost::hash_value(History[Size() - 1].Action));
    }
    else {
      int size = History.size();
      for (int i = size - 1; i >= 0 && i > size - 1 - memory_size; --i) {
        hash_combine(seed, boost::hash_value(History[i].Observation));
        if (i + 1 < size) {
          hash_combine(seed, boost::hash_value(History[i+1].Action));
        }
      }
    }

    return seed;
  }

 private:
  std::vector<ENTRY> History;
};

#endif  // HISTORY
