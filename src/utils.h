#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "memorypool.h"
#include "prettyprint.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <cmath>

#define LargeInteger 1000000
#define Infinity 1e+10
#define Tiny 1e-10

#ifdef DEBUG
#define safe_cast dynamic_cast
#else
#define safe_cast static_cast
#endif

#define PRINT_ERROR(error)                                                   \
  do {                                                                       \
    std::cerr << __FILE__ << ":" << __LINE__ << " : " << error << std::endl; \
  } while (0)

#define PRINT_VALUE(x)                               \
  do {                                               \
    std::cerr << #x " = '" << x << "'" << std::endl; \
  } while (0)

namespace utils {

inline int Sign(int x) { return (x > 0) - (x < 0); }

inline bool Near(double x, double y, double tol) { return fabs(x - y) <= tol; }

inline bool CheckFlag(int flags, int bit) { return (flags & (1 << bit)) != 0; }

inline void SetFlag(int &flags, int bit) { flags = (flags | (1 << bit)); }

template<class T>
inline bool Contains(std::vector<T> &vec, const T &item) {
  return std::find(vec.begin(), vec.end(), item) != vec.end();
}

void UnitTest();

inline int EncodeInt(int i) {
  int sign = i > 0? 1: 0;
  return sign << 7 | abs(i);
}

inline int DecodeInt(int c) {
  int v = c & 0x7F;
  return c >> 7? v: -v;
}

#define FLOAT_EPS 1.0e-6

inline bool IsNan(const double &x) {
  return std::isnan(x) || std::isinf(x);
}

inline double Sqr(const double &x) {
  return x * x;
}

inline double Sqrt(const double &x) {
  assert(!IsNan(x));
  assert(x >= 0.0);

  return std::sqrt(x);
}

typedef double AngleRad;
typedef double AngleDeg;
typedef std::pair<double, double> SinCosT;

template<typename _Tp>
inline const _Tp &
Max(const _Tp &x, const _Tp &y) {
  return std::max(x, y);
}

template<typename _Tp>
inline const _Tp &
Min(const _Tp &x, const _Tp &y) {
  return std::min(x, y);
}

template<typename _Tp>
inline const _Tp &
MinMax(const _Tp &min, const _Tp &x, const _Tp &max) {
  return Min(Max(min, x), max);
}

template<typename _Tp>
inline int
Sign(const _Tp &x) {
  return x >= 0 ? 1 : -1;
}

inline AngleDeg Rad2Deg(const AngleRad &x) {
  return x * 180.0 / M_PI;
}

inline AngleRad Deg2Rad(const AngleDeg &x) {
  return x * M_PI / 180.0;
}

inline double Sin(const AngleDeg &x) {
  return sin(Deg2Rad(x));
}

inline double Cos(const AngleDeg &x) {
  return cos(Deg2Rad(x));
}

inline SinCosT SinCos(const AngleDeg &x) {
  double sine, cosine;

  sincos(Deg2Rad(x), &sine, &cosine); //faster way to calculate sine and cosine of the same angle x simultaneously

  return std::make_pair(sine, cosine);
}

inline const double &Sin(const SinCosT &value) {
  return value.first;
}

inline const double &Cos(const SinCosT &value) {
  return value.second;
}

inline double Tan(const AngleDeg &x) {
  return tan(Deg2Rad(x));
}

inline AngleDeg ACos(const double &x) {
  return ((x) >= 1.0 - 0.000006 ? 0.0 : ((x) <= -1.0 + 0.000006 ? 180.0 : (Rad2Deg(acos(x)))));
}

inline AngleDeg ASin(const double &x) {
  return ((x) >= 1.0 - 0.000006 ? 90.0 : ((x) <= -1.0 + 0.000006 ? -90.0 : (Rad2Deg(asin(x)))));
}

inline AngleDeg ATan(const double &x) //[-90.0, 90.0]
{
  return (Rad2Deg(atan(x)));
}

inline AngleDeg ATan2(const double &y, const double &x) //[-180.0, 180.0]
{
  return ((fabs(x) < 0.000006 && fabs(y) < 0.000006) ? 0 : (Rad2Deg(atan2(y, x))));
}
}

#endif  // UTILS_H
