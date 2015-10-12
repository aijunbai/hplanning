#ifndef HISTORY_H
#define HISTORY_H

#include <vector>
#include <ostream>
#include <assert.h>
#include <cstdlib>

class HISTORY {
public:
  struct ENTRY {
    ENTRY(int action = -1, int obs = -1) : Action(action), Observation(obs) {}

    int Action;
    int Observation;
  };

  bool operator==(const HISTORY &history) const {
    if (history.History.size() != History.size())
      return false;
    for (uint i = 0; i < History.size(); ++i)
      if (history.History[i].Action != History[i].Action ||
          history.History[i].Observation != History[i].Observation)
        return false;
    return true;
  }

  void Add(int action, int obs = -1) { History.push_back(ENTRY(action, obs)); }

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
  std::vector<ENTRY> History;
};

#endif // HISTORY
