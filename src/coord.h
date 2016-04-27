#ifndef COORD_H
#define COORD_H

#include "utils.h"
#include <stdlib.h>
#include <assert.h>
#include <ostream>
#include <vector>
#include <math.h>
#include <boost/functional/hash.hpp>
#include "distribution.h"

struct COORD {
  int X, Y;

  COORD(int x = -1, int y = -1) : X(x), Y(y) { }

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
struct hash<COORD> {
  size_t operator()(const COORD &o) const {
    return hash_value(o);
  }
};
}

class Vector {
public:
  explicit Vector(const double x = 0.0, const double y = 0.0) : mX(x), mY(y) {
    assertion();
  }

  Vector(const Vector &v) : mX(v.X()), mY(v.Y()) {
    assertion();
  }

  const Vector &operator=(const Vector &v) {
    SetX(v.X());
    SetY(v.Y());

    return *this;
  }

  const double &X() const { return mX; }

  const double &Y() const { return mY; }

  void SetX(const double &x) {
    mX = x;
    assert(!utils::IsNan(mX));
  }

  void SetY(const double &y) {
    mY = y;
    assert(!utils::IsNan(mY));
  }

  void SetValue(const double &x, const double &y) {
    mX = x;
    mY = y;
    assertion();
  }

  void SetValuePolar(const double &r, const utils::AngleDeg &theta) {
    utils::SinCosT value = utils::SinCos(theta);

    mX = r * utils::Cos(value);
    mY = r * utils::Sin(value);

    assertion();
  }

  Vector operator-() const { return Vector(-mX, -mY); }

  Vector operator+(const Vector &v) const { return Vector(mX + v.mX, mY + v.mY); }

  Vector operator-(const Vector &v) const { return Vector(mX - v.mX, mY - v.mY); }

  Vector operator*(const double &a) const { return Vector(mX * a, mY * a); }

  Vector operator/(const double &a) const { return Vector(mX / a, mY / a); }

  void operator+=(const Vector &v) {
    mX += v.mX;
    mY += v.mY;
    assertion();
  }

  void operator-=(const Vector &v) {
    mX -= v.mX;
    mY -= v.mY;
    assertion();
  }

  void operator*=(const double &a) {
    mX *= a;
    mY *= a;
    assertion();
  }

  void operator/=(const double &a) {
    mX /= a;
    mY /= a;
    assertion();
  }

  bool operator!=(const Vector &v) const { return (mX != v.mX) || (mY != v.mY); }

  bool operator==(const Vector &v) const { return (mX == v.mX) && (mY == v.mY); }

  friend std::ostream &operator<<(std::ostream &os, const Vector &v) {
    return os << "(" << v.mX << ", " << v.mY << ")";
  }

  double Mod() const { return utils::Sqrt(mX * mX + mY * mY); }

  double Mod2() const { return mX * mX + mY * mY; }

  double Dist(const Vector &v) const { return (*this - v).Mod(); }

  double Dist2(const Vector &v) const { return (*this - v).Mod2(); }

  utils::AngleDeg Dir() const { return utils::ATan2(mY, mX); }

  /**
   * \return a Vector with length r at the same direction, or Vector (0, 0) if the original Vector was (0, 0).
   */
  Vector Normalize(const double r = 1.0) const {
    const double mod = Mod();

    if (mod > 0.0) {
      return (*this) * (r / mod);
    }

    return Vector(0.0, 0.0);
  }

  /**
   * \return a Vector rotated by angle.
   */
  Vector Rotate(const utils::AngleDeg &angle) const {
    return Rotate(utils::SinCos(angle));
  }

  Vector Rotate(const utils::SinCosT &value) const {
    return Vector(mX * utils::Cos(value) - mY * utils::Sin(value), mY * utils::Cos(value) + mX * utils::Sin(value));
  }

  /**
   * check if a point is approximate equal to *this;
   * @param point to be checked.
   * return true when they are approximate equal, false else;
   */
  bool ApproxEqual(const Vector &v) const {
    return fabs(mX - v.X()) < FLOAT_EPS && fabs(mY - v.Y()) < FLOAT_EPS;
  }

public:
  void assertion() const {
    assert(!utils::IsNan(mX));
    assert(!utils::IsNan(mY));
  }

private:
  double mX;
  double mY;
};

inline Vector Polar2Vector(const double &mod, const utils::AngleDeg &ang) {
  utils::SinCosT value = utils::SinCos(ang);
  return Vector(mod * utils::Cos(value), mod * utils::Sin(value));
}

inline std::size_t hash_value(const Vector &v) {
  using boost::hash_combine;

  // Start with a hash value of 0.
  std::size_t seed = 0;

  // Modify 'seed' by XORing and bit-shifting in
  // one member of 'Key' after the other:
  hash_combine(seed, boost::hash_value(v.X()));
  hash_combine(seed, boost::hash_value(v.Y()));

  // Return the result.
  return seed;
}

namespace std {
template<>
struct hash<Vector> {
  size_t operator()(const Vector &o) const {
    return hash_value(o);
  }
};
}

namespace coord {
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
    case coord::E_NORTH:
      return rhs.Y - lhs.Y;
    case coord::E_EAST:
      return rhs.X - lhs.X;
    case coord::E_SOUTH:
      return lhs.Y - rhs.Y;
    case coord::E_WEST:
      return lhs.X - rhs.X;
    default:
      assert(false);
      return 0;
  }
}

inline int MoveTo(const COORD &pos, const COORD &target, int num_actions = 4 /*or 8*/)
{
  static std::vector<int> actions;
  actions.clear();

  if (pos == target) {
    return SimpleRNG::ins().Random(num_actions);
  }

  int dx = target.X - pos.X;
  int dy = target.Y - pos.Y;

  if (dx > 0) {
    actions.push_back(E_EAST);
  }
  else if (dx < 0) {
    actions.push_back(E_WEST);
  }

  if (dy > 0) {
    actions.push_back(E_NORTH);
  }
  else if (dy < 0) {
    actions.push_back(E_SOUTH);
  }

  if (num_actions == 8 && actions.size() == 2) {
    if (actions[0] == E_EAST) {
      if (actions[1] == E_NORTH) {
        actions.push_back(E_NORTHEAST);
      }
      else {
        actions.push_back(E_SOUTHEAST);
      }
    }
    else {
      if (actions[1] == E_NORTH) {
        actions.push_back(E_NORTHWEST);
      }
      else {
        actions.push_back(E_SOUTHWEST);
      }
    }
  }

  assert(actions.size());
  return SimpleRNG::ins().Sample(actions);
}
};

inline std::ostream &operator<<(std::ostream &ostr, COORD &coord) {
  ostr << "(" << coord.X << ", " << coord.Y << ")";
  return ostr;
}

#endif  // COORD_H
