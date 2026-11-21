// Push-optimal Sokoban solver: room decomposition + push-optimal A* per room.
//
// A generated level is a 1-wide hallway (the player's row) with 1-4 rooms off it.
// A box pushed through a door into the hallway can never come back, so every box
// stays in its room and the level's push-optimal count is the sum of the rooms'
// optima. Each room is solved push-optimally (A* over push-states: box positions
// + the player's reachable region, heuristic = minimum-cost assignment of boxes
// to goals by push-distance, pruned by dead squares and 2x2 blocks), then the
// room solutions are stitched together with hallway walking moves (walking adds
// no pushes). Deterministic: budgets are fixed node counts, never the clock.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DIRS = [
  [0, -1, 'u'],
  [0, 1, 'd'],
  [-1, 0, 'l'],
  [1, 0, 'r'],
];
const ROOM_BUDGET = 150000;
const ROOM_BUDGET_BIG = 350000;
const LEVEL_BUDGET = 200000;
// Shared work budget across the whole set: the program must finish inside one
// 60-second wall for 5 instances, so the total number of expanded push-states
// is capped as well as each room's own budget. Budgets are node counts, never
// the clock, so two runs expand the same states and write the same bytes.
let totalExpanded = 0;
const GLOBAL_BUDGET = 1200000;

// ---------- parsing ----------
function parse(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h);
  const goals = [];
  let boxes = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const ch = rows[y][x] ?? '#';
      const i = y * w + x;
      wall[i] = ch === '#' ? 1 : 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// ---------- BFS over free cells: reachable region + walk parents ----------
function bfsRegion(start, arenaSet, boxSet, w, h) {
  const parent = new Map();
  const region = new Set();
  const queue = [start];
  region.add(start);
  parent.set(start, null);
  for (let qi = 0; qi < queue.length; qi++) {
    const c = queue[qi];
    const cx = c % w;
    const cy = (c / w) | 0;
    for (const [dx, dy, letter] of DIRS) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
      const ni = ny * w + nx;
      if (!arenaSet.has(ni) || boxSet.has(ni) || region.has(ni)) continue;
      region.add(ni);
      parent.set(ni, { from: c, dir: letter });
      queue.push(ni);
    }
  }
  return { region, parent };
}

function pathTo(target, parent) {
  if (!parent.has(target)) return null;
  let s = '';
  let c = target;
  for (;;) {
    const p = parent.get(c);
    if (!p) break;
    s = p.dir + s;
    c = p.from;
  }
  return s;
}

// ---------- push distance: min pushes to move one box to a goal, walls only ----------
// Reverse BFS on the push graph from each goal. A box at c can be pushed to
// c+dir only when c-dir (where the player stands) and c+dir are free cells.
// So a predecessor of x is c = x-dir with x-2*dir free.
function computePushDist(goals, arenaSet, w, h) {
  const result = new Map();
  for (const g of goals) {
    const dist = new Map();
    dist.set(g, 0);
    const q = [g];
    for (let qi = 0; qi < q.length; qi++) {
      const x = q[qi];
      const d = dist.get(x);
      const xx = x % w;
      const xy = (x / w) | 0;
      for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const cx = xx - dx;
        const cy = xy - dy;
        if (cx < 0 || cx >= w || cy < 0 || cy >= h) continue;
        const c = cy * w + cx;
        if (!arenaSet.has(c) || dist.has(c)) continue;
        const bx = cx - dx;
        const by = cy - dy;
        if (bx < 0 || bx >= w || by < 0 || by >= h) continue;
        if (!arenaSet.has(by * w + bx)) continue;
        dist.set(c, d + 1);
        q.push(c);
      }
    }
    result.set(g, dist);
  }
  return result;
}

// ---------- minimum-cost assignment of boxes to goals (brute force, 3-4 boxes) ----------
function allPerms(n) {
  const res = [];
  const a = [];
  for (let i = 0; i < n; i++) a.push(i);
  const rec = (k) => {
    if (k === n) {
      res.push(a.slice());
      return;
    }
    for (let j = k; j < n; j++) {
      const t = a[k];
      a[k] = a[j];
      a[j] = t;
      rec(k + 1);
      const t2 = a[k];
      a[k] = a[j];
      a[j] = t2;
    }
  };
  rec(0);
  return res;
}
const PERMS = { 3: allPerms(3), 4: allPerms(4) };

function heuristic(boxes, goals, pushDist) {
  const n = boxes.length;
  if (n === 0) return 0;
  const perms = PERMS[n] || allPerms(n);
  let best = Infinity;
  for (const p of perms) {
    let sum = 0;
    for (let i = 0; i < n; i++) {
      const d = pushDist.get(goals[p[i]]).get(boxes[i]);
      if (d === undefined) {
        sum = Infinity;
        break;
      }
      sum += d;
      if (sum >= best) break;
    }
    if (sum < best) best = sum;
  }
  return best;
}

// ---------- 2x2 block deadlock ----------
function isDead2x2(boxes, goalSet, arenaSet, w, h) {
  const boxSet = new Set(boxes);
  const checked = new Set();
  for (const b of boxes) {
    const bx = b % w;
    const by = (b / w) | 0;
    for (let ox = -1; ox <= 0; ox++) {
      for (let oy = -1; oy <= 0; oy++) {
        const x = bx + ox;
        const y = by + oy;
        if (x < 0 || y < 0 || x + 1 >= w || y + 1 >= h) continue;
        const tl = y * w + x;
        if (checked.has(tl)) continue;
        checked.add(tl);
        const cells = [tl, tl + 1, tl + w, tl + w + 1];
        let allBlocked = true;
        let hasBox = false;
        let boxOnGoal = false;
        for (const c of cells) {
          if (boxSet.has(c)) {
            hasBox = true;
            if (goalSet.has(c)) boxOnGoal = true;
          } else if (!arenaSet.has(c)) {
            // wall: blocked
          } else {
            allBlocked = false;
            break;
          }
        }
        if (allBlocked && hasBox && !boxOnGoal) return true;
      }
    }
  }
  return false;
}

// ---------- binary heap on [f, h, idx] ----------
function heapPush(heap, e) {
  let i = heap.length;
  heap.push(e);
  while (i > 0) {
    const p = (i - 1) >> 1;
    if (heap[i][0] > heap[p][0] || (heap[i][0] === heap[p][0] && heap[i][1] >= heap[p][1])) break;
    const t = heap[i];
    heap[i] = heap[p];
    heap[p] = t;
    i = p;
  }
}
function heapPop(heap) {
  const top = heap[0];
  const last = heap.pop();
  if (heap.length > 0) {
    heap[0] = last;
    for (let i = 0; ; ) {
      let c = 2 * i + 1;
      if (c >= heap.length) break;
      if (c + 1 < heap.length && (heap[c + 1][0] < heap[c][0] || (heap[c + 1][0] === heap[c][0] && heap[c + 1][1] < heap[c][1]))) c++;
      if (heap[c][0] > heap[i][0] || (heap[c][0] === heap[i][0] && heap[c][1] >= heap[i][1])) break;
      const t = heap[i];
      heap[i] = heap[c];
      heap[c] = t;
      i = c;
    }
  }
  return top;
}

function stateKey(boxes, region) {
  let mask = 0n;
  for (const c of region) mask |= 1n << BigInt(c);
  return boxes.join(',') + '|' + mask.toString(36);
}

// ---------- push-optimal A* over push-states ----------
// cfg: arenaSet, boxes, goals, player, startPushes, isGoal(boxes, region),
//      pushDist, budget, w, h.
// Returns { moves, finalPlayer, finalBoxes } or null.
function pushAStar(cfg) {
  const { arenaSet, boxes: initBoxes, goals, player: initPlayer, startPushes, isGoal, pushDist, budget, w, h } = cfg;
  const goalSet = new Set(goals);
  const heap = [];
  const stBoxes = [];
  const stPlayer = [];
  const stG = [];
  const stParent = [];
  const stMove = [];
  const stKey = [];
  const stRegion = [];
  const stBfs = [];
  const best = new Map();

  const b0 = initBoxes.slice().sort((a, b) => a - b);
  const bfs0 = bfsRegion(initPlayer, arenaSet, new Set(b0), w, h);
  const key0 = stateKey(b0, bfs0.region);
  const h0 = heuristic(b0, goals, pushDist);
  if (h0 === Infinity) return null;
  stBoxes.push(b0);
  stPlayer.push(initPlayer);
  stG.push(startPushes);
  stParent.push(-1);
  stMove.push('');
  stKey.push(key0);
  stRegion.push(bfs0.region);
  stBfs.push(bfs0.parent);
  best.set(key0, startPushes);
  heapPush(heap, [startPushes + h0, h0, 0]);

  let expanded = 0;
  while (heap.length > 0 && expanded < budget && totalExpanded < GLOBAL_BUDGET) {
    const top = heapPop(heap);
    const s = top[2];
    if (best.get(stKey[s]) !== stG[s]) continue; // stale
    const boxes = stBoxes[s];
    const region = stRegion[s];
    if (isGoal(boxes, region)) {
      const segs = [];
      for (let t = s; t > 0; t = stParent[t]) segs.push(stMove[t]);
      segs.reverse();
      return { moves: segs.join(''), finalPlayer: stPlayer[s], finalBoxes: boxes.slice() };
    }
    expanded++;
    totalExpanded++;
    const player = stPlayer[s];
    const boxSet = new Set(boxes);
    const bfsParent = stBfs[s];
    for (let bi = 0; bi < boxes.length; bi++) {
      const b = boxes[bi];
      const bx = b % w;
      const by = (b / w) | 0;
      for (const [dx, dy, letter] of DIRS) {
        const behindX = bx - dx;
        const behindY = by - dy;
        const aheadX = bx + dx;
        const aheadY = by + dy;
        if (behindX < 0 || behindX >= w || behindY < 0 || behindY >= h) continue;
        if (aheadX < 0 || aheadX >= w || aheadY < 0 || aheadY >= h) continue;
        const behind = behindY * w + behindX;
        const ahead = aheadY * w + aheadX;
        if (!region.has(behind)) continue;
        if (!arenaSet.has(ahead) || boxSet.has(ahead)) continue;
        const nb = boxes.slice();
        nb[bi] = ahead;
        nb.sort((a, b) => a - b);
        const ng = stG[s] + 1;
        const nbfs = bfsRegion(b, arenaSet, new Set(nb), w, h);
        const nkey = stateKey(nb, nbfs.region);
        const prev = best.get(nkey);
        if (prev !== undefined && prev <= ng) continue;
        const nh = heuristic(nb, goals, pushDist);
        if (nh === Infinity) continue;
        if (isDead2x2(nb, goalSet, arenaSet, w, h)) continue;
        const walk = pathTo(behind, bfsParent);
        if (walk === null) continue;
        const idx = stBoxes.length;
        stBoxes.push(nb);
        stPlayer.push(b);
        stG.push(ng);
        stParent.push(s);
        stMove.push(walk + letter.toUpperCase());
        stKey.push(nkey);
        stRegion.push(nbfs.region);
        stBfs.push(nbfs.parent);
        best.set(nkey, ng);
        heapPush(heap, [ng + nh, nh, idx]);
      }
    }
  }
  return null;
}

// ---------- room decomposition ----------
// The hallway is the player's row. Rooms are the connected components of
// non-wall cells strictly above / below that row. Each room opens onto the
// hallway through exactly one door cell (the room cell adjacent to a hallway
// cell); the entry cell is the hallway cell next to the door.
function decompose(level) {
  const { w, h, wall, goals, boxes, player } = level;
  const H = (player / w) | 0;
  const visited = new Uint8Array(w * h);
  const rooms = [];
  const boxSetAll = new Set(boxes);
  const goalSetAll = new Set(goals);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const i = y * w + x;
      if (wall[i] || visited[i] || y === H) continue;
      const comp = [];
      const stack = [i];
      visited[i] = 1;
      while (stack.length) {
        const c = stack.pop();
        comp.push(c);
        const cx = c % w;
        const cy = (c / w) | 0;
        for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
          const nx = cx + dx;
          const ny = cy + dy;
          if (nx < 0 || nx >= w || ny < 0 || ny >= h || ny === H) continue;
          const ni = ny * w + nx;
          if (wall[ni] || visited[ni]) continue;
          visited[ni] = 1;
          stack.push(ni);
        }
      }
      const compSet = new Set(comp);
      const rboxes = boxes.filter((b) => compSet.has(b));
      const rgoals = goals.filter((g) => compSet.has(g));
      if (rboxes.length === 0) continue;
      if (rboxes.length !== rgoals.length) return null;
      // door: the room cell adjacent to a hallway cell
      let door = -1;
      let entry = -1;
      let doorCount = 0;
      for (const c of comp) {
        const cx = c % w;
        const cy = (c / w) | 0;
        for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
          const nx = cx + dx;
          const ny = cy + dy;
          if (ny !== H || nx < 0 || nx >= w) continue;
          const ni = ny * w + nx;
          if (wall[ni]) continue;
          doorCount++;
          door = c;
          entry = ni;
        }
      }
      if (doorCount !== 1) return null;
      rooms.push({ cells: comp, cellSet: compSet, boxes: rboxes, goals: rgoals, door, entry });
    }
  }
  // every box and every goal must sit in exactly one room
  for (const b of boxes) {
    if (!rooms.some((r) => r.cellSet.has(b))) return null;
  }
  for (const g of goals) {
    if (!rooms.some((r) => r.cellSet.has(g))) return null;
  }
  if (rooms.length === 0) return null;
  return rooms;
}

// ---------- solve one room: returns moves from entry back to entry ----------
// The player starts at the entry cell (in the hallway). The first step into
// the door may push the box sitting on the door. The A* then solves the room
// and ends with the door reachable, so the player can walk out again.
function solveRoom(room, level, budget) {
  const { w, h } = level;
  const arenaSet = room.cellSet;
  const goals = room.goals;
  const goalSet = new Set(goals);
  const door = room.door;
  const entry = room.entry;
  const edx = (door % w) - (entry % w);
  const edy = ((door / w) | 0) - ((entry / w) | 0);
  const entryDir = DIRS.find(([dx, dy]) => dx === edx && dy === edy);
  if (!entryDir) return null;

  let boxes = room.boxes.slice().sort((a, b) => a - b);
  let pushes = 0;
  let entryMove = entryDir[2]; // lowercase walk by default
  const doorBox = boxes.indexOf(door);
  if (doorBox >= 0) {
    const ahead = door + edy * w + edx;
    if (!arenaSet.has(ahead) || boxes.includes(ahead)) return null;
    boxes[doorBox] = ahead;
    boxes.sort((a, b) => a - b);
    pushes = 1;
    entryMove = entryDir[2].toUpperCase();
  }
  const pushDist = computePushDist(goals, arenaSet, w, h);
  const goalTest = (bs, region) => bs.every((b) => goalSet.has(b)) && region.has(door);
  let res = pushAStar({
    arenaSet,
    boxes,
    goals,
    player: door,
    startPushes: pushes,
    isGoal: goalTest,
    pushDist,
    budget,
    w,
    h,
  });
  // A room that exhausts its first budget gets one retry with a bigger budget
  // rather than zeroing the whole level; the shared global budget still caps
  // the total work, so the set stays inside its wall.
  if (!res) {
    res = pushAStar({
      arenaSet,
      boxes,
      goals,
      player: door,
      startPushes: pushes,
      isGoal: goalTest,
      pushDist,
      budget: ROOM_BUDGET_BIG,
      w,
      h,
    });
  }
  if (!res) return null;
  // exit: walk from the final player position to the door, then step out
  const exitWalk = (() => {
    const bfs = bfsRegion(res.finalPlayer, arenaSet, new Set(res.finalBoxes), w, h);
    return pathTo(door, bfs.parent);
  })();
  if (exitWalk === null) return null;
  // step back out to the hallway: the OPPOSITE of the entry direction
  const exitDir = DIRS.find(([dx, dy]) => dx === -edx && dy === -edy);
  if (!exitDir) return null;
  const exitStep = exitDir[2]; // a walk, so lowercase
  return { moves: entryMove + res.moves + exitWalk + exitStep, finalBoxes: res.finalBoxes };
}

// ---------- walk on the full level between two cells ----------
// Prefer the hallway row (the player walks between rooms along it); fall back
// to any free cell if that finds no path.
function walkPath(from, to, level, boxSet) {
  const { w, h, wall } = level;
  const H = (level.player / w) | 0;
  const bfs = (onlyHallway) => {
    const arena = new Set();
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = y * w + x;
        if (wall[i]) continue;
        if (onlyHallway && y !== H) continue;
        arena.add(i);
      }
    }
    return bfsRegion(from, arena, boxSet, w, h);
  };
  let r = bfs(true);
  let p = pathTo(to, r.parent);
  if (p === null) {
    r = bfs(false);
    p = pathTo(to, r.parent);
  }
  return p;
}

// ---------- solve one level ----------
function solveLevel(level) {
  const rooms = decompose(level);
  if (!rooms) {
    // Fallback: whole-level push A* (budget-bounded; may still solve small levels).
    const arenaSet = new Set();
    for (let i = 0; i < level.w * level.h; i++) if (!level.wall[i]) arenaSet.add(i);
    const pushDist = computePushDist(level.goals, arenaSet, level.w, level.h);
    const goalSet = new Set(level.goals);
    const res = pushAStar({
      arenaSet,
      boxes: level.boxes,
      goals: level.goals,
      player: level.player,
      startPushes: 0,
      isGoal: (bs) => bs.every((b) => goalSet.has(b)),
      pushDist,
      budget: LEVEL_BUDGET,
      w: level.w,
      h: level.h,
    });
    return res ? res.moves : '';
  }
  let moves = '';
  let player = level.player;
  let globalBoxes = level.boxes.slice().sort((a, b) => a - b);
  for (const room of rooms) {
    const walk = walkPath(player, room.entry, level, new Set(globalBoxes));
    if (walk === null) return '';
    moves += walk;
    const solved = solveRoom(room, level, ROOM_BUDGET);
    if (!solved) return '';
    moves += solved.moves;
    player = room.entry;
    // replace this room's boxes with their solved positions (all on goals)
    const roomBoxSet = new Set(room.boxes);
    globalBoxes = globalBoxes.filter((b) => !roomBoxSet.has(b)).concat(solved.finalBoxes);
    globalBoxes.sort((a, b) => a - b);
  }
  return moves;
}

for (const id of readdirSync(I).sort()) {
  const level = parse(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  const moves = solveLevel(level);
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
