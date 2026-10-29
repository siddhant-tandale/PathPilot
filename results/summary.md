# Benchmark results

20 maps per scenario, 50x50 cells. Path cost vs A* is the median ratio over maps where both succeeded. Runtime, checks, and nodes are medians over all runs.

## All blocks maps

| Planner | Success | Path cost vs A* | Runtime (ms) | Collision checks | Nodes |
|---|---:|---:|---:|---:|---:|
| A* | 100% | 1.000 | 3.3 | 3,498 | 424 |
| Dijkstra | 100% | 1.000 | 11.3 | 14,561 | 1,850 |
| Weighted A* (w=2) | 100% | 1.011 | 0.8 | 836 | 72 |
| Theta* | 100% | 0.951 | 16.8 | 5,431 | 432 |
| RRT | 100% | 1.175 | 16.2 | 809 | 204 |
| RRT* | 100% | 0.945 | 422.2 | 10,662 | 2,114 |
| PRM | 98% | 1.001 | 80.4 | 4,330 | 602 |

## blocks, 15% obstacles (20 maps)

| Planner | Success | Path cost vs A* | Runtime (ms) | Collision checks | Nodes |
|---|---:|---:|---:|---:|---:|
| A* | 100% | 1.000 | 2.9 | 2,560 | 291 |
| Dijkstra | 100% | 1.000 | 13.3 | 16,602 | 2,100 |
| Weighted A* (w=2) | 100% | 1.000 | 0.7 | 703 | 58 |
| Theta* | 100% | 0.963 | 22.3 | 5,349 | 406 |
| RRT | 100% | 1.173 | 13.3 | 504 | 181 |
| RRT* | 100% | 0.962 | 583.7 | 11,721 | 2,469 |
| PRM | 100% | 1.008 | 86.7 | 4,210 | 602 |

## blocks, 25% obstacles (20 maps)

| Planner | Success | Path cost vs A* | Runtime (ms) | Collision checks | Nodes |
|---|---:|---:|---:|---:|---:|
| A* | 100% | 1.000 | 3.7 | 4,018 | 478 |
| Dijkstra | 100% | 1.000 | 11.2 | 14,561 | 1,850 |
| Weighted A* (w=2) | 100% | 1.012 | 0.7 | 838 | 71 |
| Theta* | 100% | 0.948 | 15.9 | 5,318 | 409 |
| RRT | 100% | 1.167 | 14.6 | 756 | 242 |
| RRT* | 100% | 0.941 | 418.1 | 10,712 | 2,131 |
| PRM | 100% | 0.988 | 80.2 | 4,330 | 602 |

## blocks, 35% obstacles (20 maps)

| Planner | Success | Path cost vs A* | Runtime (ms) | Collision checks | Nodes |
|---|---:|---:|---:|---:|---:|
| A* | 100% | 1.000 | 3.4 | 3,950 | 470 |
| Dijkstra | 100% | 1.000 | 9.3 | 12,392 | 1,584 |
| Weighted A* (w=2) | 100% | 1.014 | 1.1 | 980 | 87 |
| Theta* | 100% | 0.946 | 15.8 | 5,810 | 480 |
| RRT | 100% | 1.175 | 20.6 | 1,280 | 234 |
| RRT* | 100% | 0.935 | 334.6 | 9,566 | 1,736 |
| PRM | 95% | 1.003 | 78.3 | 4,490 | 602 |

## noise, 25% obstacles (20 maps)

| Planner | Success | Path cost vs A* | Runtime (ms) | Collision checks | Nodes |
|---|---:|---:|---:|---:|---:|
| A* | 100% | 1.000 | 4.2 | 5,160 | 646 |
| Dijkstra | 100% | 1.000 | 10.7 | 14,147 | 1,854 |
| Weighted A* (w=2) | 100% | 1.019 | 0.8 | 955 | 92 |
| Theta* | 100% | 0.959 | 12.5 | 7,756 | 734 |
| RRT | 75% | 1.180 | 70.3 | 3,768 | 538 |
| RRT* | 60% | 0.929 | 232.8 | 10,722 | 1,052 |
| PRM | 50% | 1.179 | 67.0 | 4,283 | 602 |
