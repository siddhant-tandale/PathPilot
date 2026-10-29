# Benchmark results

20 maps per scenario, 50x50 cells. Path cost vs A* is the median ratio over maps where both succeeded. Runtime, checks, and nodes are medians over all runs.

## All blocks maps

| Planner | Success | Path cost vs A* | Runtime (ms) | Collision checks | Nodes |
|---|---:|---:|---:|---:|---:|
| A* | 100% | 1.000 | 3.4 | 3,498 | 424 |
| Dijkstra | 100% | 1.000 | 11.4 | 14,561 | 1,850 |
| Weighted A* (w=2) | 100% | 1.011 | 0.7 | 836 | 72 |
| RRT | 100% | 1.175 | 14.3 | 809 | 204 |
| RRT* | 100% | 0.945 | 421.5 | 10,662 | 2,114 |
| PRM | 98% | 1.001 | 80.0 | 4,330 | 602 |

## blocks, 15% obstacles (20 maps)

| Planner | Success | Path cost vs A* | Runtime (ms) | Collision checks | Nodes |
|---|---:|---:|---:|---:|---:|
| A* | 100% | 1.000 | 2.6 | 2,560 | 291 |
| Dijkstra | 100% | 1.000 | 13.3 | 16,602 | 2,100 |
| Weighted A* (w=2) | 100% | 1.000 | 0.6 | 703 | 58 |
| RRT | 100% | 1.173 | 10.1 | 504 | 181 |
| RRT* | 100% | 0.962 | 522.4 | 11,721 | 2,469 |
| PRM | 100% | 1.008 | 80.1 | 4,210 | 602 |

## blocks, 25% obstacles (20 maps)

| Planner | Success | Path cost vs A* | Runtime (ms) | Collision checks | Nodes |
|---|---:|---:|---:|---:|---:|
| A* | 100% | 1.000 | 3.4 | 4,018 | 478 |
| Dijkstra | 100% | 1.000 | 11.0 | 14,561 | 1,850 |
| Weighted A* (w=2) | 100% | 1.012 | 0.7 | 838 | 71 |
| RRT | 100% | 1.167 | 14.4 | 756 | 242 |
| RRT* | 100% | 0.941 | 421.5 | 10,712 | 2,131 |
| PRM | 100% | 0.988 | 80.8 | 4,330 | 602 |

## blocks, 35% obstacles (20 maps)

| Planner | Success | Path cost vs A* | Runtime (ms) | Collision checks | Nodes |
|---|---:|---:|---:|---:|---:|
| A* | 100% | 1.000 | 3.6 | 3,950 | 470 |
| Dijkstra | 100% | 1.000 | 9.1 | 12,392 | 1,584 |
| Weighted A* (w=2) | 100% | 1.014 | 0.8 | 980 | 87 |
| RRT | 100% | 1.175 | 19.5 | 1,280 | 234 |
| RRT* | 100% | 0.935 | 328.3 | 9,566 | 1,736 |
| PRM | 95% | 1.003 | 78.9 | 4,490 | 602 |

## noise, 25% obstacles (20 maps)

| Planner | Success | Path cost vs A* | Runtime (ms) | Collision checks | Nodes |
|---|---:|---:|---:|---:|---:|
| A* | 100% | 1.000 | 4.2 | 5,160 | 646 |
| Dijkstra | 100% | 1.000 | 10.8 | 14,147 | 1,854 |
| Weighted A* (w=2) | 100% | 1.019 | 0.8 | 955 | 92 |
| RRT | 75% | 1.180 | 71.0 | 3,768 | 538 |
| RRT* | 60% | 0.929 | 222.0 | 10,722 | 1,052 |
| PRM | 50% | 1.179 | 68.9 | 4,283 | 602 |
