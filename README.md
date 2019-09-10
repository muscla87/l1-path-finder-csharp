# l1-path-finder-csharp
C# porting of original l1-path-finder project (https://github.com/mikolalysenko/l1-path-finder)

[<img src="https://github.com/mikolalysenko/l1-path-finder/raw/master/img/logo.png">](https://mikolalysenko.github.io/l1-path-finder/www)

# Example

```csharp
using L1PathFinder;

Point start = new Point(0, 0);
Point target = new Point(7, 6);

int[,] grid =
{
    {0, 1, 0, 0, 0, 0, 0,},
    {0, 1, 0, 1, 0, 0, 0,},
    {0, 1, 0, 1, 1, 1, 0,},
    { 0, 1, 0, 1, 0, 0, 0,},
    {0, 1, 0, 1, 0, 0, 0,},
    { 0, 1, 0, 1, 0, 0, 0,},
    {0, 1, 0, 1, 0, 1, 1,},
    {0, 0, 0, 1, 0, 0, 0}
};

var planner = L1PathPlanner.CreatePlanner(grid);

var dist = planner.Search(target, start, out List<Point> path);

//Log output
Console.WriteLine($"path length={dist}");
Console.WriteLine($"path = {string.Join(" ",path)}");
```

Output:

```
path length=31
path = (0,0) (7,0) (7,2) (0,2) (0,4) (1,4) (1,6) (3,6) (5,6) (5,4) (7,4) (7,6)
```
