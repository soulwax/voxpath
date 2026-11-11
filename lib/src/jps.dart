import 'dart:collection';
import 'grid.dart';
import 'heuristics.dart';
import 'point.dart';

final class _JpsNode implements Comparable<_JpsNode> {
  final GridPoint point;
  double g;
  double h;
  double f;
  _JpsNode? parent;

  _JpsNode(this.point, this.g, this.h, this.f, [this.parent]);

  @override
  int compareTo(_JpsNode other) => f.compareTo(other.f);
}

/// Jump Point Search pathfinder for ultra-fast pathfinding on uniform-cost grids.
class JumpPointSearch {
  final HeuristicFunc heuristic;

  const JumpPointSearch({this.heuristic = Heuristics.octile});

  /// Finds the sequence of jump-point waypoints between [start] and [goal].
  List<GridPoint> findPath(NavGrid grid, GridPoint start, GridPoint goal) {
    if (!grid.isWalkable(start.x, start.y) || !grid.isWalkable(goal.x, goal.y)) {
      return const [];
    }
    if (start == goal) return [start];

    final openMap = HashMap<GridPoint, _JpsNode>();
    final openList = <_JpsNode>[];
    final closedSet = HashSet<GridPoint>();

    final startH = heuristic(start, goal);
    final startNode = _JpsNode(start, 0.0, startH, startH);
    openMap[start] = startNode;
    openList.add(startNode);

    while (openList.isNotEmpty) {
      // Smallest f node
      var bestIdx = 0;
      for (var i = 1; i < openList.length; i++) {
        if (openList[i].f < openList[bestIdx].f) bestIdx = i;
      }
      final current = openList.removeAt(bestIdx);
      openMap.remove(current.point);

      if (current.point == goal) {
        final path = <GridPoint>[];
        var curr = current;
        while (curr.parent != null) {
          path.add(curr.point);
          curr = curr.parent!;
        }
        path.add(start);
        return path.reversed.toList();
      }

      closedSet.add(current.point);

      // Identify successors
      final successors = _identifySuccessors(grid, current, goal);
      for (final next in successors) {
        if (closedSet.contains(next)) continue;

        final stepDist = current.point.octileDistanceTo(next);
        final tentativeG = current.g + stepDist;

        var nextNode = openMap[next];
        if (nextNode == null) {
          final h = heuristic(next, goal);
          nextNode = _JpsNode(next, tentativeG, h, tentativeG + h, current);
          openMap[next] = nextNode;
          openList.add(nextNode);
        } else if (tentativeG < nextNode.g) {
          nextNode.parent = current;
          nextNode.g = tentativeG;
          nextNode.f = tentativeG + nextNode.h;
        }
      }
    }

    return const [];
  }

  List<GridPoint> _identifySuccessors(NavGrid grid, _JpsNode node, GridPoint goal) {
    final successors = <GridPoint>[];
    final neighbors = _pruneNeighbors(grid, node);

    for (final neighbor in neighbors) {
      final dx = neighbor.x - node.point.x;
      final dy = neighbor.y - node.point.y;
      final jumpPoint = _jump(grid, neighbor.x, neighbor.y, dx, dy, goal);
      if (jumpPoint != null) {
        successors.add(jumpPoint);
      }
    }

    return successors;
  }

  GridPoint? _jump(NavGrid grid, int x, int y, int dx, int dy, GridPoint goal) {
    if (!grid.isWalkable(x, y)) return null;
    if (x == goal.x && y == goal.y) return GridPoint(x, y);

    // Diagonal
    if (dx != 0 && dy != 0) {
      // Check for forced neighbors
      if ((grid.isWalkable(x - dx, y + dy) && !grid.isWalkable(x - dx, y)) ||
          (grid.isWalkable(x + dx, y - dy) && !grid.isWalkable(x, y - dy))) {
        return GridPoint(x, y);
      }

      // Check horizontal and vertical sub-branches
      if (_jump(grid, x + dx, y, dx, 0, goal) != null ||
          _jump(grid, x, y + dy, 0, dy, goal) != null) {
        return GridPoint(x, y);
      }
    } else if (dx != 0) {
      // Horizontal
      if ((grid.isWalkable(x + dx, y + 1) && !grid.isWalkable(x, y + 1)) ||
          (grid.isWalkable(x + dx, y - 1) && !grid.isWalkable(x, y - 1))) {
        return GridPoint(x, y);
      }
    } else {
      // Vertical
      if ((grid.isWalkable(x + 1, y + dy) && !grid.isWalkable(x + 1, y)) ||
          (grid.isWalkable(x - 1, y + dy) && !grid.isWalkable(x - 1, y))) {
        return GridPoint(x, y);
      }
    }

    return _jump(grid, x + dx, y + dy, dx, dy, goal);
  }

  List<GridPoint> _pruneNeighbors(NavGrid grid, _JpsNode node) {
    final parent = node.parent;
    final x = node.point.x;
    final y = node.point.y;

    if (parent == null) {
      return grid.getNeighbors(node.point, allowDiagonal: true, dontCrossCorners: true);
    }

    final dx = (x - parent.point.x).clamp(-1, 1);
    final dy = (y - parent.point.y).clamp(-1, 1);
    final neighbors = <GridPoint>[];

    // Diagonal
    if (dx != 0 && dy != 0) {
      if (grid.isWalkable(x, y + dy)) neighbors.add(GridPoint(x, y + dy));
      if (grid.isWalkable(x + dx, y)) neighbors.add(GridPoint(x + dx, y));
      if (grid.isWalkable(x + dx, y + dy)) neighbors.add(GridPoint(x + dx, y + dy));
      if (!grid.isWalkable(x - dx, y) && grid.isWalkable(x - dx, y + dy)) {
        neighbors.add(GridPoint(x - dx, y + dy));
      }
      if (!grid.isWalkable(x, y - dy) && grid.isWalkable(x + dx, y - dy)) {
        neighbors.add(GridPoint(x + dx, y - dy));
      }
    } else if (dx != 0) {
      // Horizontal
      if (grid.isWalkable(x + dx, y)) neighbors.add(GridPoint(x + dx, y));
      if (!grid.isWalkable(x, y + 1) && grid.isWalkable(x + dx, y + 1)) {
        neighbors.add(GridPoint(x + dx, y + 1));
      }
      if (!grid.isWalkable(x, y - 1) && grid.isWalkable(x + dx, y - 1)) {
        neighbors.add(GridPoint(x + dx, y - 1));
      }
    } else {
      // Vertical
      if (grid.isWalkable(x, y + dy)) neighbors.add(GridPoint(x, y + dy));
      if (!grid.isWalkable(x + 1, y) && grid.isWalkable(x + 1, y + dy)) {
        neighbors.add(GridPoint(x + 1, y + dy));
      }
      if (!grid.isWalkable(x - 1, y) && grid.isWalkable(x - 1, y + dy)) {
        neighbors.add(GridPoint(x - 1, y + dy));
      }
    }

    return neighbors;
  }
}
