import 'dart:typed_data';
import 'point.dart';

/// A 2D uniform navigation grid containing traversability flags and movement costs.
class NavGrid {
  final int width;
  final int height;
  final Uint8List _walkable;
  final Float32List _costs;

  NavGrid(this.width, this.height, {bool defaultWalkable = true})
      : _walkable = Uint8List(width * height),
        _costs = Float32List(width * height) {
    if (defaultWalkable) {
      _walkable.fillRange(0, _walkable.length, 1);
    }
    _costs.fillRange(0, _costs.length, 1.0);
  }

  /// Parses a multi-line ASCII layout where [obstacleChars] are unwalkable obstacles.
  factory NavGrid.fromAscii(
    List<String> lines, {
    String obstacleChars = '#X',
  }) {
    final h = lines.length;
    final w = lines.isNotEmpty ? lines[0].length : 0;
    final grid = NavGrid(w, h, defaultWalkable: true);

    for (var y = 0; y < h; y++) {
      final line = lines[y];
      for (var x = 0; x < w && x < line.length; x++) {
        if (obstacleChars.contains(line[x])) {
          grid.setWalkable(x, y, false);
        }
      }
    }
    return grid;
  }

  /// Returns `true` if coordinates `(x, y)` are inside grid bounds.
  bool isInBounds(int x, int y) => x >= 0 && x < width && y >= 0 && y < height;

  /// Returns `true` if cell `(x, y)` is within bounds and traversable.
  bool isWalkable(int x, int y) {
    if (!isInBounds(x, y)) return false;
    return _walkable[y * width + x] != 0;
  }

  /// Sets traversability of cell `(x, y)`.
  void setWalkable(int x, int y, bool walkable) {
    if (!isInBounds(x, y)) return;
    _walkable[y * width + x] = walkable ? 1 : 0;
  }

  /// Gets movement cost multiplier for cell `(x, y)` (defaults to 1.0).
  double getCost(int x, int y) {
    if (!isInBounds(x, y)) return double.infinity;
    return _costs[y * width + x];
  }

  /// Sets custom terrain movement cost for cell `(x, y)`.
  void setCost(int x, int y, double cost) {
    if (!isInBounds(x, y)) return;
    _costs[y * width + x] = cost;
  }

  /// Gets reachable neighbors for [point].
  List<GridPoint> getNeighbors(
    GridPoint p, {
    bool allowDiagonal = true,
    bool dontCrossCorners = true,
  }) {
    final results = <GridPoint>[];
    final x = p.x;
    final y = p.y;

    final n = isWalkable(x, y - 1);
    final s = isWalkable(x, y + 1);
    final w = isWalkable(x - 1, y);
    final e = isWalkable(x + 1, y);

    if (n) results.add(GridPoint(x, y - 1));
    if (s) results.add(GridPoint(x, y + 1));
    if (w) results.add(GridPoint(x - 1, y));
    if (e) results.add(GridPoint(x + 1, y));

    if (allowDiagonal) {
      // NW
      if (dontCrossCorners ? (n && w) : (n || w)) {
        if (isWalkable(x - 1, y - 1)) results.add(GridPoint(x - 1, y - 1));
      }
      // NE
      if (dontCrossCorners ? (n && e) : (n || e)) {
        if (isWalkable(x + 1, y - 1)) results.add(GridPoint(x + 1, y - 1));
      }
      // SW
      if (dontCrossCorners ? (s && w) : (s || w)) {
        if (isWalkable(x - 1, y + 1)) results.add(GridPoint(x - 1, y + 1));
      }
      // SE
      if (dontCrossCorners ? (s && e) : (s || e)) {
        if (isWalkable(x + 1, y + 1)) results.add(GridPoint(x + 1, y + 1));
      }
    }

    return results;
  }

  /// Tests clear line-of-sight between points [a] and [b] using Bresenham's algorithm.
  bool hasLineOfSight(GridPoint a, GridPoint b) {
    var x0 = a.x;
    var y0 = a.y;
    final x1 = b.x;
    final y1 = b.y;

    final dx = (x1 - x0).abs();
    final dy = (y1 - y0).abs();
    final sx = x0 < x1 ? 1 : -1;
    final sy = y0 < y1 ? 1 : -1;
    var err = dx - dy;

    while (true) {
      if (!isWalkable(x0, y0)) return false;
      if (x0 == x1 && y0 == y1) break;

      final e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x0 += sx;
      }
      if (e2 < dx) {
        err += dx;
        y0 += sy;
      }
    }
    return true;
  }
}
