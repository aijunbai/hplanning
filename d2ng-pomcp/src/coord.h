#ifndef COORD_H
#define COORD_H

#include <stdlib.h>
#include <assert.h>
#include <ostream>
#include <math.h>
#include <boost/functional/hash.hpp>

struct COORD {
  int X, Y;

  COORD(int x = -1, int y = -1) : X(x), Y(y) {}

  bool Valid() const { return X >= 0 && Y >= 0; }

  bool operator==(const COORD &rhs) const { return X == rhs.X && Y == rhs.Y; }

  bool operator!=(const COORD &rhs) const { return X != rhs.X || Y != rhs.Y; }

  void operator+=(const COORD &offset) {
    X += offset.X;
    Y += offset.Y;
  }

  COORD operator+(const COORD &rhs) const {
    return COORD(X + rhs.X, Y + rhs.Y);
  }

  COORD operator*(int mul) const { return COORD(X * mul, Y * mul); }

  enum {
    E_NORTH,      // 0
    E_EAST,       // 1
    E_SOUTH,      // 2
    E_WEST,       // 3
    E_NORTHEAST,  // 4
    E_SOUTHEAST,  // 5
    E_SOUTHWEST,  // 6
    E_NORTHWEST   // 7
  };

  friend std::ostream &operator<<(std::ostream &os, const COORD &o) {
    return os << "[" << o.X << ", " << o.Y << "]";
  }

  static void UnitTest();
};

inline std::size_t hash_value(const COORD &v) {
  using boost::hash_combine;

  // Start with a hash value of 0.
  std::size_t seed = 0;

  // Modify 'seed' by XORing and bit-shifting in
  // one member of 'Key' after the other:
  hash_combine(seed, boost::hash_value(v.X));
  hash_combine(seed, boost::hash_value(v.Y));

  // Return the result.
  return seed;
}

namespace std {
template<>
struct hash<COORD>
{
  size_t operator()(const COORD &o) const {
    return hash_value(o);
  }
};
}

namespace coord {
extern const COORD Null;
extern const COORD North, East, South, West;
extern const COORD NorthEast, SouthEast, SouthWest, NorthWest;
extern const COORD Compass[8];
extern const char *CompassString[8];

inline int Clockwise(int dir) { return (dir + 1) % 4; }
inline int Opposite(int dir) { return (dir + 2) % 4; }
inline int Anticlockwise(int dir) { return (dir + 3) % 4; }

inline double EuclideanDistance(const COORD &lhs, const COORD &rhs) {
  return sqrt((lhs.X - rhs.X) * (lhs.X - rhs.X) +
              (lhs.Y - rhs.Y) * (lhs.Y - rhs.Y));
}

inline int ManhattanDistance(const COORD &lhs, const COORD &rhs) {
  return abs(lhs.X - rhs.X) + abs(lhs.Y - rhs.Y);
}

inline int DirectionalDistance(const COORD &lhs, const COORD &rhs,
                               int direction) {
  switch (direction) {
    case COORD::E_NORTH:
      return rhs.Y - lhs.Y;
    case COORD::E_EAST:
      return rhs.X - lhs.X;
    case COORD::E_SOUTH:
      return lhs.Y - rhs.Y;
    case COORD::E_WEST:
      return lhs.X - rhs.X;
    default:
      assert(false);
      return 0;
  }
}
};

inline std::ostream &operator<<(std::ostream &ostr, COORD &coord) {
  ostr << "(" << coord.X << ", " << coord.Y << ")";
  return ostr;
}

#endif  // COORD_H
