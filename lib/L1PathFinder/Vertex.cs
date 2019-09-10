using System.Diagnostics;
using System.Collections.Generic;

namespace L1PathFinder
{
    [DebuggerDisplay("({x}, {y})")]
    public class Vertex
    {
        public int X { get; set; }
        public int Y { get; set; }
        public int State { get; set; }
        public int Component { get; set; }
        public double Heuristic { get; set; }
        public double Weight { get; set; }

        public Vertex Left { get; set; }
        public Vertex Right { get; set; }
        public Vertex Parent { get; set; }
        public Vertex Pred { get; set; }
        public Vertex NextFree { get; set; }
        public double[] Landmark { get; set; }
        public List<Vertex> Edges { get; set; }

        private static Vertex _nil;
        public static Vertex NIL
        {
            get
            {
                return _nil;
            }
        }

        static Vertex()
        {
            _nil = new Vertex(int.MaxValue, int.MaxValue);
            _nil.Weight = double.MinValue;
            _nil.Left = _nil.Right = _nil.Parent = _nil;
        }

        public Vertex(int x, int y)
        {
            //User data
            X = x;
            Y = y;

            //Priority queue info
            Heuristic = 0.25;
            Weight = 0.25;
            Left = NIL;
            Right = NIL;
            Parent = NIL;

            //Visit tags
            State = 0;
            Pred = null;

            //Free list
            NextFree = null;

            //Adjacency info
            Edges = new List<Vertex>();

            //Landmark data
            Landmark = LandmarksHelper.BuildEmptyLandMarks();

            //Connected component label
            Component = 0;
        }

        public void AddEdge(Vertex other)
        {
            Edges.Add(other);
            other.Edges.Add(this);
        }

        //Heap insertion
        private Vertex Link(Vertex b)
        {
            var al = this.Left;
            b.Right = al;
            al.Parent = b;
            b.Parent = this;
            Left = b;
            Right = NIL;
            return this;
        }

        public Vertex Merge(Vertex b)
        {
            if (this == NIL)
            {
                return b;
            }
            else if (b == NIL)
            {
                return this;
            }
            else if (this.Weight < b.Weight)
            {
                return this.Link(b);
            }
            else
            {
                return b.Link(this);
            }
        }

        public Vertex Push(Vertex node)
        {
            if (this == NIL)
            {
                return node;
            }
            else if (this.Weight < node.Weight)
            {
                var l = this.Left;
                node.Right = l;
                l.Parent = node;
                node.Parent = this;
                this.Left = node;
                return this;
            }
            else
            {
                var l = node.Left;
                this.Right = l;
                l.Parent = this;
                this.Parent = node;
                node.Left = this;
                return node;
            }
        }

        public Vertex Pop()
        {
            Vertex returnValue = this;
            var p = returnValue.Left;
            returnValue.Left = NIL;
            returnValue = p;

            while (true)
            {
                var q = returnValue.Right;
                if (q == NIL)
                {
                    break;
                }
                p = returnValue;

                var r = q.Right;
                var s = p.Merge(q);
                returnValue = s;

                while (true)
                {
                    p = r;
                    q = r.Right;
                    if (q == NIL)
                    {
                        break;
                    }
                    r = q.Right;
                    s = s.Right = p.Merge(q);
                }
                s.Right = NIL;
                if (p != NIL)
                {
                    p.Right = returnValue;
                    returnValue = p;
                }
            }
            returnValue.Parent = NIL;
            return returnValue;
        }

        public Vertex DecreaseKey(Vertex p)
        {
            var q = p.Parent;
            if (q.Weight < p.Weight)
            {
                return this;
            }

            var r = p.Right;
            r.Parent = q;
            if (q.Left == p)
            {
                q.Left = r;
            }
            else
            {
                q.Right = r;
            }

            if (Weight <= p.Weight)
            {
                var l = Left;
                l.Parent = p;
                p.Right = l;
                Left = p;
                p.Parent = this;
                return this;
            }
            else
            {
                var l = p.Left;
                Right = l;
                l.Parent = this;
                p.Left = this;
                Parent = p;
                p.Right = p.Parent = NIL;
                return p;
            }
        }

        public void Clear()
        {
            Vertex v = this;
            while (v != null)
            {
                var next = v.NextFree;
                v.State = 0;
                v.Left = v.Right = v.Parent = Vertex.NIL;
                v.NextFree = null;
                v = next;
            }
        }

        ////Free list functions
        public Vertex Insert(Vertex node)
        {
            if (node.NextFree != null)
            {
                return this;
            }
            node.NextFree = this;
            return node;
        }
    }
}