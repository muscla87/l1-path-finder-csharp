
using System;
using System.Collections;
using System.Collections.Generic;

namespace L1PathFinder
{
    internal class Countour2D
    {
        public class Segment
        {
            public int Start { get; set; }
            public int End { get; set; }
            public int Direction { get; set; }
            public int Height { get; set; }
            public bool Visited { get; set; }
            public Segment Next { get; set; }
            public Segment Prev { get; set; }

            public Segment(int start, int end, int direction, int height)
            {
                this.Start = start;
                this.End = end;
                this.Direction = direction;
                this.Height = height;
                this.Visited = false;
                this.Next = null;
                this.Prev = null;
            }
        }

        public class Vertex
        {
            public int X { get; set; }
            public int Y { get; set; }
            public Segment Segment { get; set; }
            public int Orientation { get; set; }

            public Vertex(int x, int y, Segment segment, int orientation)
            {
                this.X = x;
                this.Y = y;
                this.Segment = segment;
                this.Orientation = orientation;
            }
        }

        public class Point<T>
        {
            public T X { get; set; }
            public T Y { get; set; }

            public Point(T x, T y)
            {
                this.X = x;
                this.Y = y;
            }

            public T this[int index]
            {
                get => index == 0 ? X : Y;
                set
                {
                    if (index == 0)
                    {
                        X = value;
                    }
                    else
                    {
                        Y = value;
                    }
                }
            }

            public static implicit operator T[](Point<T> point)
            {
                return new[] { point.X, point.Y };
            }
        }

        public class Polygon
        {
            public List<Point<int>> Points { get; set; }

            public Polygon()
            {
                this.Points = new List<Point<int>>();
            }
        }

        static List<Segment> GetParallelCountours(int[,] array, int direction)
        {
            var n = array.GetLength(0);
            var m = array.GetLength(1);
            var contours = new List<Segment>();
            //Scan top row
            var a = false;
            var b = false;
            var c = false;
            var d = false;
            var x0 = 0;
            int i = 0, j = 0;
            for (j = 0; j < m; ++j)
            {
                b = array[0, j] != 0;

                if (b == a)
                {
                    continue;

                }
                if (a)
                {
                    contours.Add(new Segment(x0, j, direction, 0));
                }
                if (b)
                {
                    x0 = j;
                }
                a = b;
            }
            if (a)
            {
                contours.Add(new Segment(x0, j, direction, 0));
            }
            //Scan center
            for (i = 1; i < n; ++i)
            {
                a = false;
                b = false;
                x0 = 0;

                for (j = 0; j < m; ++j)
                {
                    c = array[i - 1, j] != 0;
                    d = array[i, j] != 0;

                    if (c == a && d == b)
                    {
                        continue;

                    }
                    if (a != b)
                    {
                        if (a)
                        {
                            contours.Add(new Segment(j, x0, direction, i));
                        }
                        else
                        {
                            contours.Add(new Segment(x0, j, direction, i));
                        }
                    }
                    if (c != d)
                    {
                        x0 = j;
                    }

                    a = c;
                    b = d;
                }
                if (a != b)
                {
                    if (a)
                    {
                        contours.Add(new Segment(j, x0, direction, i));
                    }
                    else
                    {
                        contours.Add(new Segment(x0, j, direction, i));
                    }
                }
            }
            //Scan bottom row
            a = false;
            x0 = 0;
            for (j = 0; j < m; ++j)
            {
                b = array[n - 1, j] != 0;

                if (b == a)
                {
                    continue;

                }
                if (a)
                {
                    contours.Add(new Segment(j, x0, direction, n));
                }
                if (b)
                {
                    x0 = j;
                }
                a = b;
            }
            if (a)
            {
                contours.Add(new Segment(j, x0, direction, n));
            }
            return contours;
        }

        static Vertex[] GetVertices(List<Segment> contours)
        {
            var vertices = new Vertex[contours.Count * 2];
            for (var i = 0; i < contours.Count; ++i)
            {
                var h = contours[i];

                if (h.Direction == 0)
                {
                    vertices[2 * i] = new Vertex(h.Start, h.Height, h, 0);
                    vertices[2 * i + 1] = new Vertex(h.End, h.Height, h, 1);
                }
                else
                {
                    vertices[2 * i] = new Vertex(h.Height, h.Start, h, 0);
                    vertices[2 * i + 1] = new Vertex(h.Height, h.End, h, 1);
                }
            }
            return vertices;
        }

        static Polygon Walk(Segment v, bool clockwise)
        {
            var result = new Polygon();
            while (!v.Visited)
            {
                v.Visited = true;

                if (v.Direction > 0)
                {
                    result.Points.Add(new Point<int>(v.Height, v.End));
                }
                else
                {
                    result.Points.Add(new Point<int>(v.Start, v.Height));
                }
                if (clockwise)
                {
                    v = v.Next;
                }
                else
                {
                    v = v.Prev;
                }
            }
            return result;
        }

        public static List<Polygon> GetContours(int[,] array, bool clockwise = false)
        {
            //First extract horizontal contours and vertices
            var hcontours = GetParallelCountours(array, 0);
            var hvertices = GetVertices(hcontours);
            Array.Sort(hvertices, new VertexComparer());

            //Extract vertical contours and vertices
            var vcontours = GetParallelCountours(array.Transpose(), 1);
            var vvertices = GetVertices(vcontours);
            Array.Sort(vvertices, new VertexComparer());

            //Glue horizontal and vertical vertices together
            var nv = hvertices.Length;
            for (var i = 0; i < nv; ++i)
            {
                var h = hvertices[i];

                var v = vvertices[i];

                if (h.Orientation == 1)
                {
                    h.Segment.Next = v.Segment;
                    v.Segment.Prev = h.Segment;
                }
                else
                {
                    h.Segment.Prev = v.Segment;
                    v.Segment.Next = h.Segment;
                }
            }

            //Unwrap loops
            var loops = new List<Polygon>();
            for (var i = 0; i < hcontours.Count; ++i)
            {
                var h = hcontours[i];

                if (!h.Visited)
                {
                    loops.Add(Walk(h, clockwise));
                }
            }

            //Return
            return loops;
        }


        class VertexComparer : IComparer
        {
            public int Compare(object x, object y)
            {
                Vertex a = x as Vertex;
                Vertex b = y as Vertex;
                var d = a.X - b.X;
                if (d != 0)
                {
                    return d;
                }
                d = a.Y - b.Y;
                if (d != 0)
                {
                    return d;
                }
                return (a.Orientation - b.Orientation);
            }
        }
    }
}