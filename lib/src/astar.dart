import 'dart:collection';
import 'grid.dart';
import 'heuristics.dart';
import 'point.dart';

final class _AStarNode implements Comparable<_AStarNode> {
  final GridPoint point;
  double g;
  double h;
  double f;
  _AStarNode? parent;

  _AStarNode(this.point, this.g, this.h, this.f, [this.parent]);

  @override
  int compareTo(_AStarNode other) => f.compareTo(other.f);
}

/// A min-heap priority queue for optimal node expansion in A*.
class _MinHeap<T extends Comparable<T>> {
  final List<T> _data = [];

  bool get isNotEmpty => _data.isNotEmpty;
  int get length => _data.length;

  void push(T item) {
    _data.add(item);
    _siftUp(_data.length - 1);
  }

  T pop() {
    final top = _data[0];
    final last = _data.removeLast();
    if (_data.isNotEmpty) {
      _data[0] = last;
      _siftDown(0);
    }
    return top;
  }

  void _siftUp(int index) {
    var i = index;
    while (i > 0) {
      final parent = (i - 1) >> 1;
      if (_data[i].compareTo(_data[parent]) < 0) {
        final tmp = _data[i];
        _data[i] = _data[parent];
        _data[parent] = tmp;
        i = parent;
      } else {
        break;
      }
    }
  }

  void _siftDown(int index) {
    var i = index;
    final len = _data.length;
    while ((i << 1) + 1 < len) {
      var child = (i << 1) + 1;
      final right = child + 1;
      if (right < len && _data[right].compareTo(_data[child]) < 0) {
        child = right;
      }
      if (_data[child].compareTo(_data[i]) < 0) {
        final tmp = _data[i];
        _data[i] = _data[child];
        _data[child] = tmp;
        i = child;
      } else {
        break;
      }
    }
  }
}

/// High-performance A* pathfinder.
class AStarFinder {
  final bool allowDiagonal;
  final bool dontCrossCorners;
  final HeuristicFunc heuristic;
  final double weight;

  const AStarFinder({
    this.allowDiagonal = true,
    this.dontCrossCorners = true,
    this.heuristic = Heuristics.octile,
    this.weight = 1.0,
  });

  /// Finds shortest path on [grid] between [start] and [goal].
  /// Returns empty list if unreachable or if start/goal is unwalkable.
  List<GridPoint> findPath(NavGrid grid, GridPoint start, GridPoint goal) {
    if (!grid.isWalkable(start.x, start.y) || !grid.isWalkable(goal.x, goal.y)) {
      return const [];
    }

    if (start == goal) return [start];

    final openHeap = _MinHeap<_AStarNode>();
    final openMap = HashMap<GridPoint, _AStarNode>();
    final closedSet = HashSet<GridPoint>();

    final startH = heuristic(start, goal) * weight;
    final startNode = _AStarNode(start, 0.0, startH, startH);
    openHeap.push(startNode);
    openMap[start] = startNode;

    const sqrt2 = 1.4142135623730951;

    while (openHeap.isNotEmpty) {
      final current = openHeap.pop();
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

      final neighbors = grid.getNeighbors(
        current.point,
        allowDiagonal: allowDiagonal,
        dontCrossCorners: dontCrossCorners,
      );

      for (final next in neighbors) {
        if (closedSet.contains(next)) continue;

        final isDiag = (next.x != current.point.x) && (next.y != current.point.y);
        final baseDist = isDiag ? sqrt2 : 1.0;
        final stepCost = baseDist * grid.getCost(next.x, next.y);
        final tentativeG = current.g + stepCost;

        var nextNode = openMap[next];
        if (nextNode == null) {
          final h = heuristic(next, goal) * weight;
          nextNode = _AStarNode(next, tentativeG, h, tentativeG + h, current);
          openHeap.push(nextNode);
          openMap[next] = nextNode;
        } else if (tentativeG < nextNode.g) {
          nextNode.parent = current;
          nextNode.g = tentativeG;
          nextNode.f = tentativeG + nextNode.h;
          // Re-push updated node to heap
          openHeap.push(nextNode);
        }
      }
    }

    return const [];
  }
}
