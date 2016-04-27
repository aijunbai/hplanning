#ifndef GRID_H
#define GRID_H

#include "coord.h"
#include "distribution.h"

template <class T> class GRID {
public:
  GRID(int xsize = 0, int ysize = 0) : XSize(xsize), YSize(ysize) {
    Grid.resize(xsize * ysize);
  }

  void Resize(int xsize, int ysize) {
    XSize = xsize;
    YSize = ysize;
    Grid.resize(xsize * ysize);
  }

  int GetXSize() const { return XSize; }

  int GetYSize() const { return YSize; }

  int GetSize() const { return XSize * YSize; }

  T &operator()(int index) {
    assert(index >= 0 && index < GetSize());
    return Grid[index];
  }

  const T &operator()(int index) const {
    assert(index >= 0 && index < GetSize());
    return Grid[index];
  }

  T &operator()(const COORD &coord) {
    assert(Inside(coord));
    return Grid[Index(coord)];
  }

  const T &operator()(const COORD &coord) const {
    assert(Inside(coord));
    return Grid[Index(coord)];
  }

  T &operator()(int x, int y) {
    assert(Inside(COORD(x, y)));
    return Grid[Index(x, y)];
  }

  const T &operator()(int x, int y) const {
    assert(Inside(COORD(x, y)));
    return Grid[Index(x, y)];
  }

  int Index(const COORD &coord) const { return XSize * coord.Y + coord.X; }

  int Index(int x, int y) const {
    assert(Inside(COORD(x, y)));
    return XSize * y + x;
  }

  bool Inside(const COORD &coord) const {
    return coord.X >= 0 && coord.Y >= 0 && coord.X < XSize && coord.Y < YSize;
  }

  int DistToEdge(const COORD &coord, int direction) {
    assert(Inside(coord));
    switch (direction) {
    case coord::E_NORTH:
      return YSize - 1 - coord.Y;
    case coord::E_EAST:
      return XSize - 1 - coord.X;
    case coord::E_SOUTH:
      return coord.Y;
    case coord::E_WEST:
      return coord.X;
    default:
      assert(false);
      return -1;
    }
  }

  void SetAllValues(const T &value) {
    for (int x = 0; x < XSize; x++)
      for (int y = 0; y < YSize; y++)
        Grid[Index(x, y)] = value;
  }

  void SetRow(int y, T *values) {
    for (int x = 0; x < XSize; x++)
      Grid[Index(x, y)] = values[x];
  }

  void SetCol(int x, T *values) {
    for (int y = 0; y < YSize; y++)
      Grid[Index(x, y)] = values[y];
  }

  COORD Coord(int index) const {
    assert(index >= 0 && index < XSize * YSize);
    return COORD(index % XSize, index / XSize);
  }

  COORD RandomPos() const {
    return COORD(SimpleRNG::ins().Random(XSize),
                 SimpleRNG::ins().Random(YSize));
  }

  bool ValidPos(const COORD &pos) const {
    return Inside(pos) && this->operator()(pos) != 'x';
  }

  bool ValidPos(int x, int y) const { return ValidPos(COORD(x, y)); }

  int ValidPath(const COORD &pos, const COORD &next, const COORD &goal,
                COORD &final) const {
    return ValidPath(pos.X, pos.Y, next.X, next.Y, goal.X, goal.Y, final.X,
                     final.Y);
  }

  // based on Bresenham's line algorithm
  int ValidPath(int x1, int y1, int x2, int y2, int gx, int gy, int &ox,
                int &oy) const {
    assert(ValidPos(x1, y1));
    int rv = 0, ix = x1, iy = y1, lx = x1, ly = y1;

    const bool steep = (std::fabs(y2 - y1) > std::fabs(x2 - x1));
    if (steep) {
      std::swap(x1, y1);
      std::swap(x2, y2);
    }

    if (x1 > x2) {
      std::swap(x1, x2);
      std::swap(y1, y2);
    }

    const float dx = x2 - x1;
    const float dy = std::fabs(y2 - y1);

    float error = dx / 2.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = (int)y1;

    const int maxX = (int)x2;

    for (int x = (int)x1; rv == 0 && x <= maxX; x++) {
      lx = ix;
      ly = iy;
      ix = x, iy = y;
      if (steep) {
        std::swap(ix, iy);
      }

      ix = utils::MinMax(0, ix, XSize - 1);
      iy = utils::MinMax(0, iy, YSize - 1);

      if (!ValidPos(ix, iy))
        rv = 1;
      else if (ix == gx && iy == gy)
        rv = 2;

      error -= dy;
      if (error < 0) {
        y += ystep;
        error += dx;
      }
    }

    if (rv == 1) {
      ox = lx;
      oy = ly;
    } else if (rv == 2) {
      ox = ix;
      oy = iy;
    }
    return rv;
  }

public:
  int XSize, YSize;
  std::vector<T> Grid;
};

template <class T> inline std::size_t hash_value(const GRID<T> &v) {
  using boost::hash_value;
  using boost::hash_combine;

  // Start with a hash value of 0    .
  std::size_t seed = 0;

  // Modify 'seed' by XORing and bit-shifting in
  // one member of 'Key' after the other:
  hash_combine(seed, boost::hash_value(v.XSize));
  hash_combine(seed, boost::hash_value(v.YSize));
  hash_combine(seed, boost::hash_value(v.Grid));

  // Return the result.
  return seed;
}

#endif // GRID_H
