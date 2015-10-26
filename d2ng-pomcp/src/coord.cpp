#include "coord.h"
#include "utils.h"

namespace coord {
const COORD Null(-1, -1);
const COORD North(0, 1);
const COORD East(1, 0);
const COORD South(0, -1);
const COORD West(-1, 0);
const COORD NorthEast(1, 1);
const COORD SouthEast(1, -1);
const COORD SouthWest(-1, -1);
const COORD NorthWest(-1, 1);

const COORD Compass[8] = {North,     East,      South,     West,
                          NorthEast, SouthEast, SouthWest, NorthWest};

const char *CompassString[8] = {"N", "E", "S", "W", "NE", "SE", "SW", "NW"};
};

void COORD::UnitTest() {
  assert(COORD(3, 3) + COORD(2, 2) == COORD(5, 5));
  COORD coord(5, 2);
  coord += COORD(2, 5);
  assert(coord == COORD(7, 7));
  assert(COORD(2, 2) + coord::North == COORD(2, 3));
  assert(COORD(2, 2) + coord::East == COORD(3, 2));
  assert(COORD(2, 2) + coord::South == COORD(2, 1));
  assert(COORD(2, 2) + coord::West == COORD(1, 2));
  assert(coord::Compass[coord::E_NORTH] == coord::North);
  assert(coord::Compass[coord::E_EAST] == coord::East);
  assert(coord::Compass[coord::E_WEST] == coord::West);
  assert(coord::Compass[coord::E_SOUTH] == coord::South);
  assert(coord::Clockwise(coord::E_NORTH) == coord::E_EAST);
  assert(coord::Clockwise(coord::E_EAST) == coord::E_SOUTH);
  assert(coord::Clockwise(coord::E_SOUTH) == coord::E_WEST);
  assert(coord::Clockwise(coord::E_WEST) == coord::E_NORTH);
  assert(coord::Opposite(coord::E_NORTH) == coord::E_SOUTH);
  assert(coord::Opposite(coord::E_EAST) == coord::E_WEST);
  assert(coord::Opposite(coord::E_SOUTH) == coord::E_NORTH);
  assert(coord::Opposite(coord::E_WEST) == coord::E_EAST);
  assert(coord::Anticlockwise(coord::E_NORTH) == coord::E_WEST);
  assert(coord::Anticlockwise(coord::E_EAST) == coord::E_NORTH);
  assert(coord::Anticlockwise(coord::E_SOUTH) == coord::E_EAST);
  assert(coord::Anticlockwise(coord::E_WEST) == coord::E_SOUTH);
  assert(coord::ManhattanDistance(COORD(3, 2), COORD(-4, -7)) == 16);
  assert(coord::DirectionalDistance(COORD(3, 2), COORD(-4, -7), coord::E_NORTH) == -9);
  assert(coord::DirectionalDistance(COORD(3, 2), COORD(-4, -7), coord::E_EAST) == -7);
  assert(coord::DirectionalDistance(COORD(3, 2), COORD(-4, -7), coord::E_SOUTH) == 9);
  assert(coord::DirectionalDistance(COORD(3, 2), COORD(-4, -7), coord::E_WEST) == 7);
}
