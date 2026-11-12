<p align="center">
  <img src="assets/logo.svg" width="88" alt="voxpath logo"/>
</p>

# voxpath

High-performance 2D grid pathfinding (A*, Jump Point Search), crowd flow fields, and path smoothing for Dart games.

Built specifically for pure-Dart game engines and real-time interactive simulations.

## Features

- **A\* Pathfinding (`AStarFinder`)**: Fast priority-heap A* supporting Manhattan, Euclidean, Chebyshev, and Octile heuristics with diagonal corner-cutting controls and terrain costs.
- **Jump Point Search (`JumpPointSearch`)**: Symmetric path pruning and line jumping across open uniform-cost grids, yielding orders-of-magnitude speedups over standard A*.
- **Crowd Flow Fields (`FlowField`)**: Dijkstra integration distance waves and gradient vector fields steering hundreds of agents to common targets with $O(1)$ per-agent lookup.
- **Path Smoothing (`PathSmoother`)**: Raycasting string-pulling turning jagged tile steps into minimal direct line-of-sight waypoints.
- **Flexible Grid Representation (`NavGrid`)**: Dynamic walkable flags, custom terrain traversal multipliers, ASCII map parsing, and Bresenham line-of-sight checks.
- **Zero External Dependencies**: Pure Dart 3, runs everywhere.

## Quick Start

```dart
import 'package:voxpath/voxpath.dart';

void main() {
  // Parse ASCII map
  final grid = NavGrid.fromAscii([
    '..........',
    '..######..',
    '..#....#..',
    '..#....#..',
    '..........',
  ]);

  // A* Pathfinding
  const astar = AStarFinder();
  final path = astar.findPath(grid, GridPoint(0, 0), GridPoint(9, 4));
  print('A* Path length: ${path.length}');

  // Path smoothing
  final smoothed = PathSmoother.smooth(grid, path);
  print('Smoothed waypoints: $smoothed');

  // Crowd Flow Field
  final flowField = FlowField.calculate(grid, GridPoint(5, 2));
  final dir = flowField.getDirection(0, 0);
  print('Vector towards target: (${dir.dx}, ${dir.dy})');
}
```

## License

MIT
