import 'package:test/test.dart';
import 'package:voxpath/voxpath.dart';

void main() {
  group('voxpath GridPoint', () {
    test('coordinate arithmetic and metrics', () {
      const p1 = GridPoint(2, 3);
      const p2 = GridPoint(5, 7);

      expect(p1 + const GridPoint(1, 1), equals(const GridPoint(3, 4)));
      expect(p2 - p1, equals(const GridPoint(3, 4)));
      expect(p1.manhattanDistanceTo(p2), equals(7));
      expect(p1.chebyshevDistanceTo(p2), equals(4));
      expect(p1.distanceTo(p2), closeTo(5.0, 1e-4));
    });
  });

  group('voxpath NavGrid & Bresenham line-of-sight', () {
    test('grid bounds and line-of-sight raycasts', () {
      final grid = NavGrid(10, 10, defaultWalkable: true);
      expect(grid.isWalkable(0, 0), isTrue);
      expect(grid.isInBounds(10, 10), isFalse);

      // Line of sight unobstructed
      expect(grid.hasLineOfSight(const GridPoint(0, 0), const GridPoint(5, 5)), isTrue);

      // Place an obstacle in between
      grid.setWalkable(2, 2, false);
      expect(grid.hasLineOfSight(const GridPoint(0, 0), const GridPoint(4, 4)), isFalse);
    });

    test('NavGrid ASCII parsing', () {
      final grid = NavGrid.fromAscii([
        '.....',
        '.###.',
        '.....',
      ]);
      expect(grid.width, equals(5));
      expect(grid.height, equals(3));
      expect(grid.isWalkable(0, 1), isTrue);
      expect(grid.isWalkable(1, 1), isFalse);
    });
  });

  group('voxpath AStarFinder', () {
    test('finds optimal path navigating around a wall', () {
      final grid = NavGrid.fromAscii([
        '.....',
        '.###.',
        '.....',
      ]);

      const finder = AStarFinder();
      final path = finder.findPath(grid, const GridPoint(0, 1), const GridPoint(4, 1));

      expect(path, isNotEmpty);
      expect(path.first, equals(const GridPoint(0, 1)));
      expect(path.last, equals(const GridPoint(4, 1)));
      expect(path.any((p) => p.y == 1 && (p.x >= 1 && p.x <= 3)), isFalse);
    });

    test('returns empty when destination is unreachable', () {
      final grid = NavGrid.fromAscii([
        '###',
        '#.#',
        '###',
      ]);

      const finder = AStarFinder();
      final path = finder.findPath(grid, const GridPoint(1, 1), const GridPoint(0, 0));
      expect(path, isEmpty);
    });
  });

  group('voxpath JumpPointSearch', () {
    test('jumps across open uniform terrain', () {
      final grid = NavGrid(20, 20, defaultWalkable: true);

      const jps = JumpPointSearch();
      final path = jps.findPath(grid, const GridPoint(0, 0), const GridPoint(10, 10));

      expect(path, isNotEmpty);
      expect(path.first, equals(const GridPoint(0, 0)));
      expect(path.last, equals(const GridPoint(10, 10)));
    });
  });

  group('voxpath FlowField', () {
    test('generates vector gradient directing towards target', () {
      final grid = NavGrid(10, 10, defaultWalkable: true);
      const target = GridPoint(5, 5);

      final flow = FlowField.calculate(grid, target);

      expect(flow.getIntegrationCost(5, 5), closeTo(0.0, 1e-4));
      expect(flow.getIntegrationCost(0, 5), greaterThan(0.0));

      // Point at (2, 5) should point directly right (+x) towards (5, 5)
      final dir = flow.getDirection(2, 5);
      expect(dir.dx, greaterThan(0.5));
      expect(dir.dy.abs(), lessThan(0.1));
    });
  });

  group('voxpath PathSmoother', () {
    test('reduces intermediate collinear steps', () {
      final grid = NavGrid(10, 10, defaultWalkable: true);
      final rawPath = [
        const GridPoint(0, 0),
        const GridPoint(1, 0),
        const GridPoint(2, 0),
        const GridPoint(3, 0),
        const GridPoint(4, 0),
        const GridPoint(5, 0),
      ];

      final smoothed = PathSmoother.smooth(grid, rawPath);
      expect(smoothed.length, equals(2));
      expect(smoothed.first, equals(const GridPoint(0, 0)));
      expect(smoothed.last, equals(const GridPoint(5, 0)));
    });
  });
}
