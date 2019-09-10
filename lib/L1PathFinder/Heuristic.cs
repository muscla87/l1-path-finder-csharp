using System;

namespace L1PathFinder
{
    static class Heuristic
    {
        public static double Run(double[] tdist, int tx, int ty, Vertex node)
        {
            var nx = node.X;
            var ny = node.Y;
            double pi = Math.Abs(nx - tx) + Math.Abs(ny - ty);
            var ndist = node.Landmark;
            for (var i = 0; i < LandmarksHelper.NUM_LANDMARKS; ++i)
            {
                pi = Math.Max(pi, tdist[i] - ndist[i]);
            }
            return 1.0000009536743164 * pi;
        }
    }
}