import 'dart:collection';
import 'dart:math' as math;
import 'dart:typed_data';
import 'grid.dart';
import 'point.dart';

/// 2D Flow Field calculating integration distances and directional vectors for crowd pathfinding.
class FlowField {
  final NavGrid grid;
  final Float32List _integration;
  final Float32List _vectors; // 2 floats per cell: [dx, dy]

  FlowField._(this.grid, this._integration, this._vectors);

  /// Computes a flow field leading to [target].
  factory FlowField.calculate(NavGrid grid, GridPoint target) {
    return FlowField.calculateMulti(grid, [target]);
  }

  /// Computes a flow field leading to any of the specified [targets].
  factory FlowField.calculateMulti(NavGrid grid, Iterable<GridPoint> targets) {
    final w = grid.width;
    final h = grid.height;
    final totalCells = w * h;

    final integration = Float32List(totalCells);
    integration.fillRange(0, totalCells, double.infinity);

    final queue = Queue<GridPoint>();

    for (final t in targets) {
      if (grid.isWalkable(t.x, t.y)) {
        final idx = t.y * w + t.x;
        integration[idx] = 0.0;
        queue.add(t);
      }
    }

    const sqrt2 = 1.4142135623730951;

    // Dijkstra flood-fill to compute integration field
    while (queue.isNotEmpty) {
      final current = queue.removeFirst();
      final currIdx = current.y * w + current.x;
      final currentCost = integration[currIdx];

      final neighbors = grid.getNeighbors(current, allowDiagonal: true, dontCrossCorners: true);
      for (final next in neighbors) {
        final nextIdx = next.y * w + next.x;
        final isDiag = (next.x != current.x) && (next.y != current.y);
        final stepCost = (isDiag ? sqrt2 : 1.0) * grid.getCost(next.x, next.y);
        final newCost = currentCost + stepCost;

        if (newCost < integration[nextIdx]) {
          integration[nextIdx] = newCost;
          queue.add(next);
        }
      }
    }

    // Compute direction vectors from the integration gradient
    final vectors = Float32List(totalCells * 2);

    for (var y = 0; y < h; y++) {
      for (var x = 0; x < w; x++) {
        final cellIdx = y * w + x;
        if (!grid.isWalkable(x, y) || integration[cellIdx] == 0.0 || integration[cellIdx].isInfinite) {
          vectors[cellIdx * 2] = 0.0;
          vectors[cellIdx * 2 + 1] = 0.0;
          continue;
        }

        var minCost = integration[cellIdx];
        var bestDx = 0.0;
        var bestDy = 0.0;

        final neighbors = grid.getNeighbors(GridPoint(x, y), allowDiagonal: true, dontCrossCorners: true);
        for (final n in neighbors) {
          final nIdx = n.y * w + n.x;
          final nCost = integration[nIdx];
          if (nCost < minCost) {
            minCost = nCost;
            bestDx = (n.x - x).toDouble();
            bestDy = (n.y - y).toDouble();
          }
        }

        final len = math.sqrt(bestDx * bestDx + bestDy * bestDy);
        if (len > 1e-6) {
          vectors[cellIdx * 2] = bestDx / len;
          vectors[cellIdx * 2 + 1] = bestDy / len;
        } else {
          vectors[cellIdx * 2] = 0.0;
          vectors[cellIdx * 2 + 1] = 0.0;
        }
      }
    }

    return FlowField._(grid, integration, vectors);
  }

  /// Returns integration distance to nearest goal, or `double.infinity` if unreachable.
  double getIntegrationCost(int x, int y) {
    if (!grid.isInBounds(x, y)) return double.infinity;
    return _integration[y * grid.width + x];
  }

  /// Returns the normalized 2D movement vector `(dx, dy)` at grid coordinate `(x, y)`.
  ({double dx, double dy}) getDirection(int x, int y) {
    if (!grid.isInBounds(x, y)) return (dx: 0.0, dy: 0.0);
    final idx = (y * grid.width + x) * 2;
    return (dx: _vectors[idx], dy: _vectors[idx + 1]);
  }
}
