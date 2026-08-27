# Obstacles

This package owns the public obstacle constructors, combination, and occupancy
query, then converts canonical histories into immutable cached geometry for
planning. It preserves original and protected geometry, applies a requested
safety margin exactly once during construction, and never selects a route.

```matlab
obstacles = azElObstacles.makeAzElObstacleData(...);
obstacles = azElObstacles.combineAzElObstacles(obstacles);
isOccupied = azElObstacles.queryAzElTimeObstacle(...);
```
