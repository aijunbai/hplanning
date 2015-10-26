#ifndef COORD_H
#define COORD_H

#include "utils.h"
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

class Vector
{
public:
  explicit Vector(const double x = 0.0, const double y = 0.0): mX(x), mY(y) {
    assertion();
  }

  Vector(const Vector & v) : mX(v.X()), mY(v.Y()) {
    assertion();
  }

  const Vector & operator=(const Vector & v) {
    SetX(v.X());
    SetY(v.Y());

    return *this;
  }

  const double & X() const { return mX; }
  const double & Y() const { return mY; }

  void SetX(const double & x) { mX = x; assert(!IsNan(mX)); }
  void SetY(const double & y) { mY = y; assert(!IsNan(mY)); }

  void SetValue(const double & x, const double & y) { mX = x; mY = y; assertion(); }
  void SetValuePolar(const double & r, const AngleDeg & theta) {
    SinCosT value = SinCos(theta);

    mX = r * Cos(value);
    mY = r * Sin(value);

    assertion();
  }

  Vector operator-() const { return Vector(-mX, -mY); }
  Vector operator+(const Vector &v) const { return Vector(mX + v.mX, mY + v.mY); }
  Vector operator-(const Vector &v) const { return Vector(mX - v.mX, mY - v.mY); }
  Vector operator*(const double &a) const { return Vector(mX * a, mY * a); }
  Vector operator/(const double &a) const { return Vector(mX / a, mY / a); }

  void operator+=(const Vector &v) { mX += v.mX; mY += v.mY; assertion(); }
  void operator-=(const Vector &v) { mX -= v.mX; mY -= v.mY; assertion(); }
  void operator*=(const double &a) { mX *= a; mY *= a; assertion(); }
  void operator/=(const double &a) { mX /= a; mY /= a; assertion(); }

  bool operator!=(const Vector &v) const { return (mX != v.mX) || (mY != v.mY); }
  bool operator==(const Vector &v) const { return (mX == v.mX) && (mY == v.mY); }

  friend std::ostream& operator<<(std::ostream & os, const Vector & v) { return os << "(" << v.mX << ", " << v.mY << ")"; }

  double Mod() const { return Sqrt(mX * mX + mY * mY); }
  double Mod2() const { return mX * mX + mY * mY; }
  double Dist(const Vector &v) const { return (*this - v).Mod(); }
  double Dist2(const Vector &v) const { return (*this - v).Mod2(); }

  AngleDeg Dir() const { return ATan2(mY, mX); }

  /**
   * \return a Vector with length r at the same direction, or Vector (0, 0) if the original Vector was (0, 0).
   */
  Vector Normalize(const double r = 1.0) const
  {
    const double mod = Mod();

    if (mod > 0.0) {
      return (*this) * (r / mod);
    }

    return Vector(0.0, 0.0);
  }

  /**
   * \return a Vector rotated by angle.
   */
  Vector Rotate(const AngleDeg & angle) const
  {
    return Rotate(SinCos(angle));
  }

  Vector Rotate(const SinCosT & value) const
  {
    return Vector(mX * Cos(value) - mY * Sin(value), mY * Cos(value) + mX * Sin(value));
  }

  /**
   * check if a point is approximate equal to *this;
   * @param point to be checked.
   * return true when they are approximate equal, false else;
   */
  bool ApproxEqual(const Vector & v) const
  {
    return fabs(mX-v.X()) < FLOAT_EPS && fabs(mY-v.Y()) < FLOAT_EPS;
  }

public:
  void assertion() const {
    assert(!IsNan(mX));
    assert(!IsNan(mY));
  }

private:
  double mX;
  double mY;
};

inline Vector Polar2Vector(const double & mod, const AngleDeg & ang)
{
  SinCosT value = SinCos(ang);
  return Vector(mod * Cos(value), mod * Sin(value));
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
struct hash<Vector>
{
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
};

inline std::ostream &operator<<(std::ostream &ostr, COORD &coord) {
  ostr << "(" << coord.X << ", " << coord.Y << ")";
  return ostr;
}

#endif  // COORD_H
