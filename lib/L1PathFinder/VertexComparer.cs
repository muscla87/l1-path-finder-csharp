using System.Collections;

namespace L1PathFinder
{
    class VertexComparer : IComparer
    {
        public int Compare(object x, object y)
        {
            var a = x as Vertex;
            var b = y as Vertex;
            var d = a.X - b.X;
            if (d != 0)
            {
                return d;
            }
            return a.Y - b.Y;
        }
    }
}