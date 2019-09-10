using System.Collections.Generic;

namespace L1PathFinder
{
    public class Bucket
    {
        public int Y0 { get; set; }
        public int Y1 { get; set; }

        internal Vertex Top { get; private set; }
        internal Vertex Bottom { get; private set; }
        internal List<Vertex> Left { get; private set; }
        internal List<Vertex> Right { get; private set; }
        internal List<Vertex> On { get; private set; }

        public Bucket(int y0, int y1, Vertex top, Vertex bottom, List<Vertex> left, List<Vertex> right, List<Vertex> on)
        {
            this.Y0 = y0;
            this.Y1 = y1;
            this.Top = top;
            this.Bottom = bottom;
            this.Left = left;
            this.Right = right;
            this.On = on;
        }
    }
};