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
    ENTRY() : Action(-1), Observation(-1) { }
    ENTRY(int action, int obs)
        : Action(action), Observation(obs) { }

    int Action;
    int Observation;
  };

  void Add(int action, int obs) {
    History.push_back(ENTRY(action, obs));
  }

  int LastObservation() const {
    if (History.size()) {
      return History.back().Observation;
    }
    return -1;
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
  std::vector<ENTRY> History;
};

#endif  // HISTORY
