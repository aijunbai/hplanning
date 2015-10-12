#ifndef HISTORY_H
#define HISTORY_H

#include <vector>
#include <ostream>
#include <assert.h>
#include <cstdlib>
#include <boost/functional/hash.hpp>
#include "utils.h"

class HISTORY {
 public:
  struct ENTRY {
    ENTRY(int action = -1, int obs = -1)
      : Action(action), Observation(obs), Hash(0) {}

    int Action;
    int Observation;
    size_t Hash;
  };

  bool operator==(const HISTORY &history) const {
    assert(0);
    return Hash() == history.Hash();

//    if (history.History.size() != History.size()) return false;
//    for (uint i = 0; i < History.size(); ++i)
//      if (history.History[i].Action != History[i].Action ||
//          history.History[i].Observation != History[i].Observation)
//        return false;
//    return true;
  }

  void Add(int action, int obs, int memory_size) {
    History.push_back(ENTRY(action, obs));
    History.back().Hash = hash_value(memory_size);
//    PRINT_VALUE(Hash());
  }

  size_t Hash() const {
    return Size()? History.back().Hash: 0;
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
      ostr << "a=" << History[t].Action << " ";
      if (History[t].Observation >= 0)
        ostr << "o=" << History[t].Observation << " ";
    }
  }

private:
  virtual size_t hash_value(int memory_size) const {
    assert(Size() >= 1);

    using boost::hash_combine;
    std::size_t seed = 0;

    if (memory_size < 0) {  // use whole history
      seed = Size() >= 2? History[Size() - 2].Hash: 0;
      hash_combine(seed, History[Size() - 1].Observation);
      hash_combine(seed, History[Size() - 1].Action);
    }
    else {
      int size = History.size();
      for (int i = size - 1; i >= 0 && i > size - 1 - memory_size; --i) {
        hash_combine(seed, History[i].Action);
        hash_combine(seed, History[i].Observation);
//        std::cerr << "hash " << size - 1 - i << " " << History[i].Action << " " << History[i].Observation << " " << seed << std::endl;
      }
    }

    return seed;
  }

 private:
  std::vector<ENTRY> History;
};

#endif  // HISTORY
