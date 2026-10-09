# 01 — Maze Model & Flood-Fill Algorithm

Source: `main/MazeMap.{h,cpp}`, `main/main.ino` (`runFloodStep`,
`planSpeedRoute`).

---

## 1. Data model

`MAZE_SIZE = 16`. Per cell (`MazeCell`):

| Field | Type | Meaning |
|---|---|---|
| `walls` | `uint8_t` | bit per absolute direction that has a **known wall** |
| `known` | `uint8_t` | bit per direction whose state has ever been measured |
| `traversed` | `uint8_t` | bit per direction the robot has **physically driven across** |
| `visited` | `bool` | robot has occupied this cell |
| `distance` | `uint16_t` | flood-fill distance to nearest goal (`0xFFFF` = unreachable) |

Direction encoding (absolute, not relative to the robot):

```
North = 0, East = 1, South = 2, West = 3
wallBit(d) = 1 << d
```

Coordinate convention: start `(0,0)`, **north = +Y, east = +X**. Note this is a
screen/logic convention chosen by the authors; the physical maze is rotated to
match (documented in `README.md`).

```
      y
      ↑   (7,8)(8,8)
      |   (7,7)(8,7)  ← 2×2 goal
      |
      +----------→ x
   (0,0)
```

## 2. Boundary walls

`reset()` memsets every cell, then closes the outer ring:
- `setWall(x,0,South,true)`, `setWall(x,15,North,true)` for all x
- `setWall(0,y,West,true)`, `setWall(15,y,East,true)` for all y

Then `floodFill()`. `setWall` refuses to remove a boundary wall
(`if (!inBounds(nx,ny)) present = true`) and flags that as a `conflict`.

## 3. `setWall` — the conflict policy

```
setWall(x,y,dir,present):
  if out of bounds -> clamp present=true (boundary)
  if present && (cell.traversed & mask): return true (refuse to re-close)
  conflict = boundaryConflict || (already known && value flips)
  known |= mask; walls |= or &= mask
  mirror the opposite bit into the neighbour cell
  return conflict
```

Two important rules:
- **`traversed` wins over sensors.** Once the robot has crossed an edge, a later
  reading claiming a wall there is recorded as a *conflict* but ignored. This
  protects the map from a single bad measurement.
- **Conflicts are counted, not fatal.** `observe()` returns the conflict count;
  the app logs `MAP WARNING` and continues.

## 4. `observe` and `markTraversed`

```
observe(x,y,heading, frontWall,leftWall,rightWall):
  visited = true
  setWall(heading,      frontWall)
  setWall(leftOf(head), leftWall)
  setWall(rightOf(head),rightWall)
  return number of conflicts

markTraversed(x,y,dir):
  setWall(x,y,dir,false)               // force it open
  cell(x,y).traversed |= (1<<dir)
  cell(neighbour).traversed |= (1<<opposite(dir))
```

`sensors report "open"` → `frontWall = !frontOpen`, etc.

## 5. Flood fill (BFS from the goal)

`floodFill()` is a multi-source BFS, **not** a recursive flood:

```
set every distance = FLOOD_UNREACHABLE
seed the four goal cells with distance 0, push them
while queue not empty:
    c = pop
    next = distance(c) + 1
    for each direction d:
        if canTravel(c,d)            // in bounds AND (walls & bit)==0
            n = neighbour
            if distance(n) > next:   // relax
                distance(n) = next
                push n
```

`canTravel` is symmetric because wall bits are mirrored into both cells. Cost =
1 per orthogonal step; unknown edges count as open (no wall bit set), so the
robot always has a candidate path to explore.

**Complexity:** O(256) cells × 4 directions, trivial on ESP32; called after every
scan and every move.

## 6. Choosing the next cell (`chooseNext`)

```
preference order = [heading, rightOf(heading), leftOf(heading), opposite(heading)]
best = none
for d in preference:
    if canTravel(x,y,d):
        cand = neighbour cell
        if !best
           || cand.distance < bestDistance
           || (cand.distance == bestDistance && bestWasVisited && !cand.visited):
            best = d; bestDistance = cand.distance; bestWasVisited = cand.visited
return best && bestDistance != UNREACHABLE
```

- Primary key: **lowest flood distance**.
- Tie-break: prefer a cell not yet visited (encourages covering new ground).
- `preference` only breaks ties among *equal* distances in the order
  straight → right → left → back. So the robot naturally prefers going straight.
- The selected edge is re-verified after the turn; if a wall now blocks it, the
  step is abandoned and the loop replans from the same cell.

## 7. Goal detection

```
isGoal(x,y) = (x==7 || x==8) && (y==7 || y==8)
```

The four centre cells. `floodFill` seeds all four, so "distance 0" is the goal
region.

## 8. Persistence (NVS)

- Namespace `"micromouse"`, key `"maze"`.
- `SavedMaze` = magic `0x4D4D0101` + 16×16×4 bytes (`walls, known, traversed,
  visited`). `distance` is recomputed by `floodFill()` on load.
- `save()` is called once at goal; `load()` at boot. If load fails the app calls
  `resetMap()`.
- Because `visited` is stored, the tie-break survives a power cycle.

## 9. Speed-route planner (`planSpeedRoute`, main.ino)

A second BFS, this time **from the start over traversed edges only**:

```
previous[] = -1; previous[0] = 0; queue=[0]
while queue:
    c = pop
    if isGoal(c): goal = c; break
    for d in 0..3:
        if !(cell(c).traversed & (1<<d)): continue
        n = neighbour; if visited skip
        previous[n] = c; entered[n] = d; push n
if no goal: return false
walk back goal→start collecting `entered`, then reverse
```

`runSpeedStep` then merges consecutive equal directions into a straight segment
and drives it with `demoMoveStraightCells(n)`.

**Why this is safe:** only edges the robot has *already driven* are eligible, so
the speed run can never command an unverified edge. The cost is that the route
is only as good as the exploration path, not a fresh shortest path over the whole
known map.

## 10. Correctness notes / improvement candidates

- The exploration flood fill and the speed-route BFS both use `distance` vs
  `traversed` respectively — deliberate (safety vs optimality).
- `chooseNext` treats unknown edges as open; combined with the post-turn
  verification this is safe but can cost an extra turn at a fake opening.
- A known limitation: `floodFill` uses wall bits only, so it cannot distinguish
  "known open" from "unknown"; both are passable. That is intended for
  exploration.
- The maze is hardcoded 16×16 and the goal hardcoded to the 2×2 centre. Both are
  easy to parameterise for porting (see `06-PORTING-AND-LESSONS.md`).
