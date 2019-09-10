using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization.Formatters.Binary;
using System.Text;
using L1PathFinder;

namespace L1PathFinder.TestCli
{
    class Program
    {
        static void Main(string[] args)
        {
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

        }
    }
}
