using System.Collections.Generic;

namespace L1PathFinder
{
    public class Node
    {
        public int X { get; set; }
        public List<Bucket> Buckets { get; set; }
        public Node Left { get; set; }
        public Node Right { get; set; }
        public List<Vertex> Verts { get; set; } = null;
        public bool Leaf { get; set; }

        public Node(int x, List<Bucket> buckets, Node left, Node right)
        {
            this.Buckets = buckets;
            this.X = x;
            this.Left = left;
            this.Right = right;
            this.Leaf = false;
        }
    }
};