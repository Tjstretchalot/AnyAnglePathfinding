# Any-Angle Pathfinding

This package is a lightweight any-angle pathfinding algoirthm that depends on
[SharpMath2](https://github.com/Tjstretchalot/sharpmath2). This implementation
tends to have faster performance and a simpler implementation compared to Theta*
Pathfinding.

This package does not require a graph system and does not give optimal paths,
however it seems that for a 2D rts style game this is sufficiently fast and
gives sufficiently good paths. Importantly, the difficult of calculating the
path does not depend on the map size but rather the complexity of the path
produced. This ensures that even when the paths are slow it is intuitive *why*
the paths are slow and thus easier for level designers to avoid pathological
cases.

## Videos

The following video demonstrates paths from this library. ***Click for higher
quality***

[![Paths
example](docs/example.gif)](https://www.youtube.com/watch?v=bt-QorgXXWQ)

## Installation

### Without Monogame / With NuGet

https://www.nuget.org/packages/AAPathfinding_Nuget_Nomono/

### With Monogame / Without NuGet

Open either a [git for windows terminal](https://gitforwindows.org/) (windows)
or the standard terminal (linux, mac). Move into the solution folder (i.e., `cd`
into the folder containing the .sln folder for the project you want to use this
pathfinder in). Run the following:

```
mkdir SharpMath2
cd SharpMath2
git init
git remote add origin "git@github.com:Tjstretchalot/SharpMath2.git"
git pull origin master
cd ..
mkdir AnyAnglePathfinding
cd AnyAnglePathfinding
git init
git remote add origin "git@github.com:Tjstretchalot/AnyAnglePathfinding.git"
git pull origin master
```

Then open up your project, right click the solution, "Add existing project",
navigate to the sharpmath2.shproj file and add it. Repeat for the AnyAnglePathfinding
package.

Right click "References" inside your main project. Under shared projects (left),
check "SharpMath2" and "AnyAnglePathfinding".

Add the NuGet package [High-Speed-Priority-Queue](https://github.com/BlueRaja/High-Speed-Priority-Queue-for-C-Sharp)

## Quickstart

```csharp
var map = new RectPartitionAAMap<AACollidable>(200, 100);

var entity = new AACollidable()
{
    Position = new Vector2(50, 70),
    Bounds = ShapeUtils.CreateCircle(radius: 10, segments: 7)
};
map.Register(entity);

var entity2 = new TSCollidable()
{
    Position = new Vector2(100, 70),
    Bounds = ShapeUtils.CreateCircle(radius: 7, segments: 5)
};
map.Register(entity2);

var excludeIds = new HashSet<int>();
excludeIds.Add(entity.ID);

List<Vector2> path = new AAPathfinder(
        map, entity.Bounds, entity.Position,
        new Vector2(150, 70), excludeIds, 0
    ).CalculatePath();

Console.WriteLine($"entity 1 at {entity.Position}");
foreach(var loc in path)
{
    Console.WriteLine($"moved entity 1 to {loc}");
}
```

## Usage

You will need a map class that implements `AAMap<T>`. This class will need to be
able to trace polygons as they move through your world. In general your
implementation can mirror the `SimpleAAMap<T>` class, except you should
incorporate some space partitioning system in order to prevent the n^2 collision
detection costs. Luckily you can get a great amount of benefit from the general
implementation provided with `RectPartitionAAMap<T>` which will automatically
partition your map into rectangles calculated from the location of entities:

![Partitioning example](docs/rectpartition.png)

The most direct way to incorporate ThetaStarSharp into an existing project:

- Have your entity class extend (or include) `AACollidable`
- Have your map class include a `RectPartitionAAMap<T>` instance where `T` is
your Entity class
- Calculate paths via `new AAPathfinder(map, bounds, start, end, excludeIds,
  excludeFlags).CalculatePath()`
- Move entities with `Map.Move(int id, Vector2 pos)` - some work needs to be done
  to keep the partitions accurate. Failing to do this will cause cascading errors.

## Common Issues

When using the `RectPartitionAAMap<T>` it is essential that you do not modify the
position of collidables directly, instead using `Move(int id, Vector2 position)` where
id is the id of the collidable to move. If you see the pathfinder **attempting to path
through another entity**, then the partition has broken. First use Visual Studio Code to
find all references to the Position for your AACollidable class and verify it's not being
modified anywhere. If it is not, turn on the compiler flag `PARTITION_DEBUG` and re-run.
This will verify the state of the partition before and after each operation, which will
allow you to isolate where the problem is occurring. If it happens on the second check
within a function, it is a bug with this project. If it happens on the first check within
a function (i.e., before it has done anything), it is a bug with your project.

If it doesn't catch any issues, then it must be something else. For this particular bug,
verify that the entity is actually added to the pathfinder. This is as simple as checking
`Map.CollidablesLookup.ContainsKey(ent.ID)`.

## Comparison to other pathfinding algorithms

This is not a grid-based pathfinding algorithm. Furthermore, tracing polygons
across arbitrary distances requires polygon collision detection which is not
trivial to implement on your own. By incorporating this project you get the
flexibility of any-angle pathfinding on a plane in just a few lines of code.
Furthermore you no longer have to worry about grid size as a hyperparameter in
your pathfinding performance, which improves generalizability.

This package is not suitable if any of the following apply:

- Movement is not identical on a plane (i.e. sidescrollers cannot use this
  package)
- Your entities can only move in a limited number of directions (i.e. cardinal
  directions)
- It is more expensive to move over some parts of the map then others, **and you
  want this taken into account during pathfinding**
- Optimal pathing is more important than quickly finding paths
- There are relatively few paths that will get you to the destination. In other
  words, the world is not relatively open.
  - Navigating through a maze will work but is not going to be fast using this
    algorithm

This package is meant for you (and you should submit issues to address any
problems) if:

- Your entities can move in any angle (i.e. an RTS)
- Collidables are relatively sparse
- You prefer human-looking paths to optimal paths
- Paths will tend to have only a few nodes or be impossible
- Some classes of objects do not collide with other classes of objects
  - For example, you want to move to attack a building and ignore any *enemy*
    that is in the way
- Some paths will not collide with a small set of objects
  - For example a door that only one unit can walk through

## Flags example

```csharp
// define your flags by incrementing the number on the right.
// this gives you up to 63 flags. You can use an enum with [Flags]
// which subtypes long as well
long TEAM_1 = 1 << 0; // 2^0
long TEAM_2 = 1 << 1; // 2^1

var map = new RectPartitionAAMap<AACollidable>(2000, 1000);
var entity = new AACollidable()
{
    Position = new Vector2(50, 70),
    Bounds = ShapeUtils.CreateCircle(radius: 10, segments: 7),
    Flags = TEAM_1
};
map.Register(entity);

var entity2 = new AACollidable()
{
    Position = new Vector2(100, 70),
    Bounds = ShapeUtils.CreateCircle(radius: 7, segments: 5),
    FLAGS = TEAM_2
};
map.Register(entity2);

var excludeIds = new HashSet<int>();
excludeIds.Add(entity.ID);

List<Vector2> path = new AAPathfinder(
        map, entity.Bounds, entity.Position,
        new Vector2(150, 70), excludeIds, TEAM_2 // ignores team 2 (i.e., entity2) -> direct path
    ).CalculatePath();

Console.WriteLine($"entity 1 at {entity.Position}");
foreach(var loc in path)
{
    Console.WriteLine($"moved entity 1 to {loc}");
}
```
