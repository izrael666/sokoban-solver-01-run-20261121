// Room-decomposed push-optimal Sokoban solver (MIT).
//
// Strategy: the generated levels are a hallway plus 1-4 small rooms (3-4
// boxes each). A box pushed out of its room can never return, so each room
// is solved independently and push-optimally, and the rooms' optima add up
// to the level optimum. Each room is solved with A* over pushes (state =
// sorted box squares + the player's reachable region, named by its lowest
// square), with an admissible heuristic (minimum-cost assignment of boxes
// to goals under shortest-path floor distances), dead-square pruning and
// frozen 2x2-block pruning. All budgets are state counts, never wall
// clock, so two runs write the same bytes.
//
// First chunk: parsing, distances, heuristic, push A* search.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

type Level = {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
};

/** [dx, dy, step letter, push letter]. */
const DIRS: [number, number, string, string][] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

/** Parse XSB text. Short rows and the outer frame count as walls. */
function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x]!;
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++)
      if (x === 0 || y === 0 || x === w - 1 || y === h - 1) wall[y * w + x] = 1;
  return { w, h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

/** Squares from which a box alone can be pushed onto some goal (pulled back from the goals over open floor). */
function liveSquares(level: Level): Uint8Array {
  const { w, wall, goals } = level;
  const live = new Uint8Array(wall.length);
  const queue = [...goals];
  for (const g of goals) live[g] = 1;
  for (let qi = 0; qi < queue.length; qi++) {
    const x = queue[qi]!;
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (!wall[y] && !wall[y - d] && !live[y]) {
        live[y] = 1;
        queue.push(y);
      }
    }
  }
  return live;
}

/** All-pairs shortest-path distances over open floor (ignoring boxes). dist[a * size + b]. -1 when unreachable. */
function floorDistances(level: Level): Int32Array {
  const { w, wall } = level;
  const size = wall.length;
  const dist = new Int32Array(size * size).fill(-1);
  const queue = new Int32Array(size);
  for (let s = 0; s < size; s++) {
    if (wall[s]) continue;
    let qh = 0;
    let qt = 0;
    queue[qt++] = s;
    dist[s * size + s] = 0;
    while (qh < qt) {
      const p = queue[qh++]!;
      const d0 = dist[s * size + p]!;
      for (const [dx, dy] of DIRS) {
        const q = p + dy * w + dx;
        if (wall[q] || dist[s * size + q] !== -1) continue;
        dist[s * size + q] = d0 + 1;
        queue[qt++] = q;
      }
    }
  }
  return dist;
}

/**
 * Admissible heuristic: the minimum, over assignments of boxes to distinct
 * goals, of the summed floor distance. Each push moves one box one floor
 * step, so the true remaining push count is at least this. Exact assignment
 * by DP over goal subsets for n <= 8, else the (still admissible) sum of
 * per-box minima.
 */
function heuristic(
  boxes: number[],
  goals: number[],
  dist: Int32Array,
  size: number,
): number {
  const n = boxes.length;
  if (n === 0) return 0;
  if (n > 8) {
    let s = 0;
    for (const b of boxes) {
      let m = Infinity;
      for (const g of goals) {
        const d = dist[b * size + g]!;
        if (d >= 0 && d < m) m = d;
      }
      if (m === Infinity) return Infinity;
      s += m;
    }
    return s;
  }
  const m = goals.length;
  const dp = new Float64Array(1 << m).fill(Infinity);
  dp[0] = 0;
  for (let mask = 0; mask < 1 << m; mask++) {
    const k = popcount(mask); // boxes assigned so far
    if (k >= n || dp[mask] === Infinity) continue;
    const b = boxes[k]!;
    for (let j = 0; j < m; j++) {
      if (mask & (1 << j)) continue;
      const d = dist[b * size + goals[j]!]!;
      if (d < 0) continue;
      const nm = mask | (1 << j);
      const v = dp[mask]! + d;
      if (v < dp[nm]!) dp[nm] = v;
    }
  }
  let best = Infinity;
  for (let mask = 0; mask < 1 << m; mask++)
    if (popcount(mask) === n && dp[mask]! < best) best = dp[mask]!;
  return best;
}

function popcount(x: number): number {
  let c = 0;
  while (x) {
    x &= x - 1;
    c++;
  }
  return c;
}

/** Lowest square of the player's reachable region from `from` (boxes block), via flood fill with stamp arrays. */
function reachLow(
  level: Level,
  occupied: Uint8Array,
  from: number,
  seen: Int32Array,
  stamp: number,
  stack: Int32Array,
): number {
  const { w, wall } = level;
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n]!;
    if (p < low) low = p;
    for (let k = 0; k < 4; k++) {
      const q = k === 0 ? p - w : k === 1 ? p + w : k === 2 ? p - 1 : p + 1;
      if (wall[q] === 0 && occupied[q] === 0 && seen[q] !== stamp) {
        seen[q] = stamp;
        stack[n++] = q;
      }
    }
  }
  return low;
}

/** Whether the box just pushed to `t` sits in a frozen 2x2 block (walls/boxes) holding a box off its goal. */
function frozenBlock(
  level: Level,
  occupied: Uint8Array,
  goal: Uint8Array,
  t: number,
): boolean {
  const { w, wall } = level;
  for (const a of [t, t - 1, t - w, t - w - 1]) {
    const b = a + 1;
    const c = a + w;
    const d = c + 1;
    if (!(wall[a] || occupied[a]) || !(wall[b] || occupied[b]) || !(wall[c] || occupied[c]) || !(wall[d] || occupied[d]))
      continue;
    if (
      (occupied[a] && !goal[a]) ||
      (occupied[b] && !goal[b]) ||
      (occupied[c] && !goal[c]) ||
      (occupied[d] && !goal[d])
    )
      return true;
  }
  return false;
}

/** Shortest walk from `from` to `to` around walls and boxes (lower LURD letters, u first on ties). */
function walkTo(level: Level, occupied: Uint8Array, from: number, to: number): string {
  const { w, wall } = level;
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how: string[] = new Array(wall.length);
  const queue = [from];
  prev[from] = from;
  for (let qi = 0; qi < queue.length; qi++) {
    const p = queue[qi]!;
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const q = p + dy * w + dx;
      if (wall[q] || occupied[q] || prev[q] !== -1) continue;
      prev[q] = p;
      how[q] = letter;
      queue.push(q);
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: push square out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]!) s = how[p]! + s;
  return s;
}

/** Square the player stands on after `moves` (case aside) from `from` on a board `w` wide. */
function endOf(w: number, from: number, moves: string): number {
  let p = from;
  for (const ch of moves.toLowerCase())
    p += ch === 'u' ? -w : ch === 'd' ? w : ch === 'l' ? -1 : 1;
  return p;
}

/**
 * Push-optimal (weight 1) or weighted A* over pushes for one level.
 * Returns {pushes, moves} (moves in LURD, upper letter = a push) or null
 * when the state budget runs out or the level is unsolvable. Deterministic:
 * fixed direction order, heap ties by (f, h, insertion order), state-count
 * budgets only, no clock or randomness.
 */
function solvePush(
  level: Level,
  maxStates: number,
  weight: number,
): { pushes: number; moves: string } | null {
  const { w, wall, goals } = level;
  const size = wall.length;
  const goal = new Uint8Array(size);
  for (const g of goals) goal[g] = 1;
  const live = liveSquares(level);
  if (level.boxes.some((b) => !live[b])) return null;
  const dist = floorDistances(level);
  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;
  const solved = (boxes: number[]): boolean => boxes.every((b) => goal[b]);
  const keyOf = (boxes: number[], low: number): number => {
    let k = low;
    for (const b of boxes) k = k * size + b;
    return k;
  };

  const sBoxes: number[][] = [level.boxes];
  const sPlayer: number[] = [level.player];
  const parent: number[] = [-1];
  const pushFrom: number[] = [-1];
  const pushTo: number[] = [-1];
  const pushDir: number[] = [-1];
  const sg: number[] = [0];
  const bestG = new Map<number, number>();
  for (const b of level.boxes) occupied[b] = 1;
  bestG.set(keyOf(level.boxes, reachLow(level, occupied, level.player, seen, ++stamp, stack)), 0);
  for (const b of level.boxes) occupied[b] = 0;

  // Binary heap of [f, h, seq, state], ordered by f, then h, then seq.
  const heap: [number, number, number, number][] = [];
  const less = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!less(heap[i]!, heap[p]!)) break;
      [heap[i], heap[p]] = [heap[p]!, heap[i]!];
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && less(heap[c + 1]!, heap[c]!)) c++;
        if (!less(heap[c]!, heap[i]!)) break;
        [heap[i], heap[c]] = [heap[c]!, heap[i]!];
        i = c;
      }
    }
    return top;
  };

  let seq = 0;
  const h0 = weight === 0 ? 0 : heuristic(level.boxes, goals, dist, size);
  if (h0 === Infinity) return null;
  if (solved(level.boxes)) return { pushes: 0, moves: '' };
  heapPush([weight * h0, h0, seq++, 0]);

  let found = -1;
  while (heap.length > 0) {
    const [, , , s] = heapPop();
    const boxes = sBoxes[s]!;
    const g = sg[s]!;
    // Stale entry (a better path to this state was found since).
    const curLow = (() => {
      for (const b of boxes) occupied[b] = 1;
      const l = reachLow(level, occupied, sPlayer[s]!, seen, ++stamp, stack);
      for (const b of boxes) occupied[b] = 0;
      return l;
    })();
    if (bestG.get(keyOf(boxes, curLow)) !== g) continue;
    for (const b of boxes) occupied[b] = 1;
    reachLow(level, occupied, sPlayer[s]!, seen, ++stamp, stack);
    const here = stamp;
    for (let bi = 0; bi < boxes.length && found < 0; bi++) {
      const b = boxes[bi]!;
      for (let di = 0; di < 4; di++) {
        const [dx, dy] = DIRS[di]!;
        const d = dy * w + dx;
        const t = b + d;
        if (seen[b - d] !== here || wall[t] || occupied[t] || !live[t]) continue;
        occupied[b] = 0;
        occupied[t] = 1;
        const frozen = frozenBlock(level, occupied, goal, t);
        let next: number[] | null = null;
        let key = -1;
        let low = -1;
        if (!frozen) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((x, y) => x - y);
          low = reachLow(level, occupied, b, mark, ++stamp, stack);
          key = keyOf(next, low);
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (frozen || next === null) continue;
        const ng = g + 1;
        const prevG = bestG.get(key);
        if (prevG !== undefined && prevG <= ng) continue;
        if (sBoxes.length >= maxStates) return null;
        bestG.set(key, ng);
        const idx = sBoxes.length;
        sBoxes.push(next);
        sPlayer.push(b);
        parent.push(s);
        pushFrom.push(b);
        pushTo.push(t);
        pushDir.push(di);
        sg.push(ng);
        if (solved(next)) {
          found = idx;
          break;
        }
        const h = weight === 0 ? 0 : heuristic(next, goals, dist, size);
        if (h === Infinity) continue;
        heapPush([ng + weight * h, h, seq++, idx]);
      }
    }
    for (const x of boxes) occupied[x] = 0;
  }
  if (found < 0) return null;

  // Reconstruct the push path, then expand each push into a walk + push.
  const path: number[] = [];
  for (let s = found; s > 0; s = parent[s]!) path.push(s);
  path.reverse();
  let moves = '';
  let player = level.player;
  let boxes = level.boxes;
  for (const s of path) {
    const from = pushFrom[s]!;
    const to = pushTo[s]!;
    const d = to - from;
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(level, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += DIRS[pushDir[s]!]![3];
    player = from;
    const nextBoxes = sBoxes[s]!;
    boxes = nextBoxes;
  }
  return { pushes: path.length, moves };
}

// Room decomposition (same cut as the kit's reference solver): cut the dead
// squares of the player's row (the hallway: a box there reaches no goal)
// out of the floor; each remaining part holding a box is a room. Null when
// some part holds unequal boxes/goals, or a box starts in the hallway.
function roomsOf(
  level: Level,
): { squares: number[]; boxes: number[]; goals: number[]; done: boolean }[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(level);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) if (!wall[i] && !live[i]) cut[i] = 1;
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: { squares: number[]; boxes: number[]; goals: number[]; done: boolean }[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      for (const [dx, dy] of DIRS) {
        const q = squares[i]! + dy * w + dx;
        if (!wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [], done: false });
  }
  for (const b of boxes) rooms[part[b]!]!.boxes.push(b);
  for (const g of goals) if (part[g]! >= 0) rooms[part[g]!]!.goals.push(g);
  if (goals.some((g) => part[g]! < 0) || rooms.some((r) => r.boxes.length !== r.goals.length))
    return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

/** Solve one room's sub-level, trying optimal A* first, then weighted, then uniform-cost. */
function solveRoom(
  sub: Level,
): { pushes: number; moves: string } | null {
  return (
    solvePush(sub, 400_000, 1) ??
    solvePush(sub, 400_000, 3) ??
    solvePush(sub, 1_200_000, 0)
  );
}

/**
 * Solve room by room: each room's push-optimal solution in turn, the other
 * rooms' boxes standing as walls (on their goals once their room is done),
 * the player walking on from where the last room left it. A box never
 * leaves its room, so the rooms' optima add up to the level's.
 */
function solveByRooms(level: Level): { pushes: number; moves: string } | null {
  const rooms = roomsOf(level);
  if (rooms === null) return null;
  let player = level.player;
  let pushes = 0;
  let moves = '';
  for (const room of rooms) {
    const wall = level.wall.slice();
    for (const other of rooms)
      if (other !== room)
        for (const b of other.done ? other.goals : other.boxes) wall[b] = 1;
    const sub: Level = {
      w: level.w,
      h: level.h,
      wall,
      goals: room.goals.slice().sort((a, b) => a - b),
      boxes: room.boxes.slice().sort((a, b) => a - b),
      player,
    };
    const solved = solveRoom(sub);
    if (solved === null) return null;
    pushes += solved.pushes;
    moves += solved.moves;
    player = endOf(level.w, player, solved.moves);
    room.done = true;
  }
  return { pushes, moves };
}

function solveLevel(level: Level): string {
  const byRooms = solveByRooms(level);
  if (byRooms !== null) return byRooms.moves;
  // Level is no set of rooms: fall back to a whole-level search.
  const whole =
    solvePush(level, 400_000, 1) ??
    solvePush(level, 400_000, 3) ??
    solvePush(level, 800_000, 0);
  return whole === null ? '' : whole.moves;
}

for (const id of readdirSync(I).sort()) {
  const level = parseLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  const moves = solveLevel(level);
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
