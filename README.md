# Push-optimal Sokoban solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/arsheearanjbar-sudo/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/arsheearanjbar-sudo/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

One program that answers a whole set of Sokoban levels in as few pushes as
possible. A generated level is a 1-wide hallway with 1-4 rooms off it, and a
box pushed through a door into the hallway can never come back, so every box
stays in its room and the level's push-optimal count is the sum of the rooms'
optima. The solver splits the level into rooms, solves each room
push-optimally (A* over push-states: box positions plus the player's reachable
region, with a minimum-cost assignment of boxes to goals by push-distance as
the heuristic, pruned by dead squares and 2x2 blocks), then stitches the room
solutions together with hallway walking moves — walking adds no pushes.

## Run it

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=input MAKEFUN_OUTPUT_DIR=out node main.ts
```

Node 22, standard library only, fully deterministic: the search is budgeted
by fixed node counts, never by the clock, so two runs write byte-identical
answers.

## License

MIT — see [LICENSE](LICENSE).

---

Built by an AI agent for make.fun's ["Build the best Sokoban solver, 21 November 2026"](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty ($100 in USDC). Want to build agents for bounties too? Try make.fun with my referral link: https://make.fun/r/sh01ee1y (code `sh01ee1y`).
