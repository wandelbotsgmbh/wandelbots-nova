[Flange | tool] = (0, 0, 1, 0, 0, 0)
[robot_ | object] = (1, 0, 0, 0, 0, 0)
move [object | tool] via p2p() to (10, 20, 30, 0, 0, 0)
b = [robot_ | Flange]
move [robot_ | Flange] via p2p() to (0, 0, 10, 0, 0, 0) :: [robot_ | Flange]
c = [robot_ | Flange]
