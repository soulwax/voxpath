import 'grid.dart';
import 'point.dart';

/// Simplifies and smooths grid paths using line-of-sight raycasting.
abstract final class PathSmoother {
  /// Reduces a multi-step [path] into minimal straight-line waypoints
  /// by skipping intermediate nodes where a clear [NavGrid.hasLineOfSight] exists.
  static List<GridPoint> smooth(NavGrid grid, List<GridPoint> path) {
    if (path.length <= 2) return List.of(path);

    final smoothed = <GridPoint>[path[0]];
    var currentIdx = 0;

    while (currentIdx < path.length - 1) {
      var nextIdx = currentIdx + 1;
      // Look ahead as far as possible while line of sight is clear
      for (var testIdx = path.length - 1; testIdx > currentIdx + 1; testIdx--) {
        if (grid.hasLineOfSight(path[currentIdx], path[testIdx])) {
          nextIdx = testIdx;
          break;
        }
      }

      smoothed.add(path[nextIdx]);
      currentIdx = nextIdx;
    }

    return smoothed;
  }
}
