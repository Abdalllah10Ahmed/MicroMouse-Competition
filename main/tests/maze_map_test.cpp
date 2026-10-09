#include <cassert>
#include <cstdio>

#include "../MazeMap.h"

int main() {
  MazeMap maze;
  maze.reset();

  assert(maze.cell(0, 0).distance == 14);
  assert(maze.cell(7, 7).distance == 0);
  assert((maze.cell(0, 0).walls & (1U << (uint8_t)Direction::West)) != 0);
  assert((maze.cell(0, 0).walls & (1U << (uint8_t)Direction::South)) != 0);

  Direction next = Direction::West;
  assert(maze.chooseNext(0, 0, Direction::North, next));
  assert(next == Direction::North);

  // Facing north at the start: front wall, left incorrectly reported open at
  // the maze boundary, and right open. The boundary remains closed.
  const int conflicts =
      maze.observe(0, 0, Direction::North, true, false, false);
  assert(conflicts == 1);
  assert((maze.cell(0, 0).walls & (1U << (uint8_t)Direction::North)) != 0);
  assert((maze.cell(1, 0).known & (1U << (uint8_t)Direction::West)) != 0);
  assert((maze.cell(1, 0).walls & (1U << (uint8_t)Direction::West)) == 0);
  maze.floodFill();
  assert(maze.chooseNext(0, 0, Direction::North, next));
  assert(next == Direction::East);

  maze.markTraversed(0, 0, Direction::East);
  assert((maze.cell(0, 0).walls & (1U << (uint8_t)Direction::East)) == 0);
  assert((maze.cell(1, 0).walls & (1U << (uint8_t)Direction::West)) == 0);
  assert(maze.observe(1, 0, Direction::North, false, true, false) == 1);
  assert((maze.cell(0, 0).walls & (1U << (uint8_t)Direction::East)) == 0);
  assert((maze.cell(1, 0).walls & (1U << (uint8_t)Direction::West)) == 0);

  assert(MazeMap::leftOf(Direction::North) == Direction::West);
  assert(MazeMap::rightOf(Direction::North) == Direction::East);
  assert(MazeMap::opposite(Direction::East) == Direction::West);
  assert(MazeMap::dx(Direction::West) == -1);
  assert(MazeMap::dy(Direction::North) == 1);
  assert(maze.isGoal(7, 8));
  assert(!maze.isGoal(6, 8));

  // NVS round-trip through the fake Preferences shim.
  assert(maze.save());
  MazeMap restored;
  assert(restored.load());
  for (int y = 0; y < MAZE_SIZE; ++y) {
    for (int x = 0; x < MAZE_SIZE; ++x) {
      const MazeCell &before = maze.cell(x, y);
      const MazeCell &after = restored.cell(x, y);
      assert(before.walls == after.walls);
      assert(before.known == after.known);
      assert(before.traversed == after.traversed);
      assert(before.visited == after.visited);
    }
  }
  assert(restored.cell(7, 7).distance == 0);
  assert((restored.cell(0, 0).walls & (1U << (uint8_t)Direction::West)) != 0);

  std::puts("PASS: maze walls, boundaries, flood distances and next-cell choice");
}
