import 'point.dart';

/// Heuristic distance function signature used by A*.
typedef HeuristicFunc = double Function(GridPoint a, GridPoint b);

/// Standard heuristics for grid pathfinding.
abstract final class Heuristics {
  /// Manhattan distance (best for 4-directional grids).
  static double manhattan(GridPoint a, GridPoint b) => a.manhattanDistanceTo(b).toDouble();

  /// Euclidean straight-line distance.
  static double euclidean(GridPoint a, GridPoint b) => a.distanceTo(b);

  /// Octile distance (best for 8-directional grids where diagonals cost ~1.414).
  static double octile(GridPoint a, GridPoint b) => a.octileDistanceTo(b);

  /// Chebyshev distance (best for 8-directional grids where diagonal movement costs 1).
  static double chebyshev(GridPoint a, GridPoint b) => a.chebyshevDistanceTo(b).toDouble();
}
