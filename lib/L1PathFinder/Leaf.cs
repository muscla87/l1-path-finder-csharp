using System.Collections.Generic;

namespace L1PathFinder
{
    public class Leaf : Node
    {
        public Leaf(List<Vertex> verts) : base(0, null, null, null)
        {
            this.Verts = verts;
            this.Leaf = true;
        }
    }
};