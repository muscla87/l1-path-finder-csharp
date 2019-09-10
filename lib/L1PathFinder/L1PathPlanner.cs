using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace L1PathFinder
{
    public class L1PathPlanner
    {
        const int LEAF_CUTOFF = 64;
        const int BUCKET_SIZE = 32;

        public Graph Graph { get; private set; }

        private Geometry _geometry;
        private Node _root;

        internal L1PathPlanner(Geometry geometry, Graph graph, Node root)
        {
            _geometry = geometry;
            Graph = graph;
            _root = root;
        }

        private int CompareBucket(Bucket bucket, int y)
        {
            return bucket.Y0 - y;
        }

        private void ConnectList(List<Vertex> nodes, Geometry geom, Graph graph, bool isTarget, int x, int y)
        {
            for (var i = 0; i < nodes.Count; ++i)
            {
                var v = nodes[i];
                if (!geom.StabBox(v.X, v.Y, x, y))
                {
                    if (isTarget)
                    {
                        graph.AddTarget(v);
                    }
                    else
                    {
                        graph.AddSource(v);
                    }
                }
            }
        }

        void ConnectNodes(Geometry geom, Graph graph, Node node, bool isTarget, int x, int y)
        {
            //Mark target nodes
            Vertex v;
            while (node != null)
            {
                //Check leaf case
                if (node.Leaf)
                {
                    var vv = node.Verts;
                    var nn = vv.Count;
                    for (var i = 0; i < nn; ++i)
                    {
                        v = vv[i];
                        if (!geom.StabBox(v.X, v.Y, x, y))
                        {
                            if (isTarget)
                            {
                                graph.AddTarget(v);
                            }
                            else
                            {
                                graph.AddSource(v);
                            }
                        }
                    }
                    break;
                }

                //Otherwise, glue into buckets
                var buckets = node.Buckets;

                //var idx = bsearch.lt(buckets, y, compareBucket);
                var idx = FindFirstIndexGreaterThanOrEqualTo(buckets, y, CompareBucket);

                if (idx >= 0)
                {
                    var bb = buckets[idx];
                    if (y < bb.Y1)
                    {
                        //Common case:
                        if (node.X >= x)
                        {
                            //Connect right
                            ConnectList(bb.Right, geom, graph, isTarget, x, y);
                        }
                        if (node.X <= x)
                        {
                            //Connect left
                            ConnectList(bb.Left, geom, graph, isTarget, x, y);
                        }
                        //Connect on
                        ConnectList(bb.On, geom, graph, isTarget, x, y);
                    }
                    else
                    {
                        //Connect to bottom of bucket above
                        v = buckets[idx].Bottom;
                        if (v != null && !geom.StabBox(v.X, v.Y, x, y))
                        {
                            if (isTarget)
                            {
                                graph.AddTarget(v);
                            }
                            else
                            {
                                graph.AddSource(v);
                            }
                        }
                        //Connect to top of bucket below
                        if (idx + 1 < buckets.Count)
                        {
                            v = buckets[idx + 1].Top;
                            if (v != null && !geom.StabBox(v.X, v.Y, x, y))
                            {
                                if (isTarget)
                                {
                                    graph.AddTarget(v);
                                }
                                else
                                {
                                    graph.AddSource(v);
                                }
                            }
                        }
                    }
                }
                else
                {
                    //Connect to top of box
                    v = buckets[0].Top;
                    if (v != null && !geom.StabBox(v.X, v.Y, x, y))
                    {
                        if (isTarget)
                        {
                            graph.AddTarget(v);
                        }
                        else
                        {
                            graph.AddSource(v);
                        }
                    }
                }
                if (node.X > x)
                {
                    node = node.Left;
                }
                else if (node.X < x)
                {
                    node = node.Right;
                }
                else
                {
                    break;
                }
            }
        }

        private static int FindFirstIndexGreaterThanOrEqualTo<T, U>(IList<T> list, U value, Func<T, U, int> comparer)
        {
            if (list == null)
                throw new ArgumentNullException("list");
            var comp = Comparer<T>.Default;
            int lo = 0, hi = list.Count - 1;
            while (lo < hi)
            {
                int m = (hi + lo) / 2;  // this might overflow; be careful.
                if (comparer(list[m], value) < 0) lo = m + 1;
                else hi = m - 1;
            }
            if (comparer(list[lo], value) < 0) lo++;
            return lo;
        }


        public double Search(Point start, Point target, out List<Point> outPath)
        {
            outPath = new List<Point>();
            int tx = target.X;
            int ty = target.Y;
            int sx = start.X;
            int sy = start.Y;

            var geom = this._geometry;
            //Degenerate case:  s and t are equal
            if (tx == sx && ty == sy)
            {
                if (!geom.StabBox(tx, ty, sx, sy))
                {
                    if (outPath != null)
                    {
                        //outPath.push(sx, sy);
                        outPath.Add(new Point(sx, sy));
                    }
                    return 0;
                }
                return double.MaxValue;
            }

            //Check easy case - s and t directly connected
            if (!geom.StabBox(tx, ty, sx, sy))
            {
                if (outPath != null)
                {
                    if (sx != tx && sy != ty)
                    {
                        //outPath.push(tx, ty, sx, ty, sx, sy);
                        outPath.Add(new Point(tx, ty));
                        outPath.Add(new Point(sx, ty));
                        outPath.Add(new Point(sx, sy));
                    }
                    else
                    {
                        //outPath.push(tx, ty, sx, sy);
                        outPath.Add(new Point(tx, ty));
                        outPath.Add(new Point(sx, sy));
                    }
                }
                return Math.Abs(tx - sx) + Math.Abs(ty - sy);
            }

            //Prepare graph
            var graph = this.Graph;
            graph.SetSourceAndTarget(sx, sy, tx, ty);
            //Mark target
            ConnectNodes(geom, graph, this._root, true, tx, ty);
            //Mark source
            ConnectNodes(geom, graph, this._root, false, sx, sy);

            //Run A*
            var dist = graph.Search();
            //Recover path
            if (outPath != null && dist < double.MaxValue)
            {
                graph.GetPath(outPath);
            }

            return dist;
        }

        private class PairComparer : IComparer
        {
            public int Compare(object x, object y)
            {
                var a = x as int[];
                var b = y as int[];
                var d = a[1] - b[1];
                if (d != 0)
                {
                    return d;
                }
                return a[0] - b[0];
            }
        }

        private static (int x, List<int[]> left, List<int[]> right, List<int[]> on, List<int[]> vis) MakePartition(int x, List<int[]> corners, Geometry geom, object edges = null)
        {
            var left = new List<int[]>();
            var right = new List<int[]>();
            var on = new List<int[]>();

            //Intersect rays along x horizontal line
            for (var i = 0; i < corners.Count; ++i)
            {
                var c = corners[i];
                if (!geom.StabRay(c[0], c[1], x))
                {
                    on.Add(c);
                }
                if (c[0] < x)
                {
                    left.Add(c);
                }
                else if (c[0] > x)
                {
                    right.Add(c);
                }
            }

            //Sort on events by y then x
            var onArray = on.ToArray();
            Array.Sort(onArray, new PairComparer());
            on = onArray.ToList();

            //Construct vertices and horizontal edges
            var vis = new List<int[]>();
            var rem = new List<int[]>();

            for (var i = 0; i < on.Count;)
            {
                var l = x;
                var r = x;
                var v = on[i];
                var y = v[1];
                while (i < on.Count && on[i][1] == y && on[i][0] < x)
                {
                    l = on[i++][0];
                }
                if (l < x)
                {
                    vis.Add(new[] { l, y });
                }
                while (i < on.Count && on[i][1] == y && on[i][0] == x)
                {
                    rem.Add(on[i]);
                    vis.Add(on[i]);
                    ++i;
                }
                if (i < on.Count && on[i][1] == y)
                {
                    r = on[i++][0];
                    while (i < on.Count && on[i][1] == y)
                    {
                        ++i;
                    }
                }
                if (r > x)
                {
                    vis.Add(new[] { r, y });
                }
            }

            return (x: x, left: left, right: right, on: rem, vis: vis);
        }

        public static L1PathPlanner CreatePlanner(int[,] grid)
        {
            var geom = Geometry.CreateGeometry(grid);
            var graph = new Graph();
            var verts = new Dictionary<Tuple<int, int>, Vertex>();
            var edges = new List<int[][]>();

            Vertex MakeVertex(int[] pair)
            {
                if (pair == null)
                {
                    return null;
                }
                verts.TryGetValue(Tuple.Create(pair[0], pair[1]), out Vertex res);
                if (res != null)
                {
                    return res;
                }
                return verts[Tuple.Create(pair[0], pair[1])] = graph.AddVertex(pair[0], pair[1]);
            }

            //function makeLeaf(corners, x0, x1)
            Leaf MakeLeaf(List<int[]> corners, int x0, int x1)
            {
                var localVerts = new List<Vertex>();
                for (var i = 0; i < corners.Count; ++i)
                {
                    var u = corners[i];
                    var ux = graph.AddVertex(u[0], u[1]);
                    localVerts.Add(ux);
                    verts[Tuple.Create(u[0], u[1])] = ux;
                    for (var j = 0; j < i; ++j)
                    {
                        var v = corners[j];
                        if (!geom.StabBox(u[0], u[1], v[0], v[1]))
                        {
                            edges.Add(new[] { u, v });
                        }
                    }
                }
                return new Leaf(localVerts);
            }

            (List<int[]> left, List<int[]> right, List<int[]> on, int[] steiner0, int[] steiner1, int y0, int y1)
                    MakeBucket(List<int[]> corners, int x)
            {
                //Split visible corners into 3 cases
                var left = new List<int[]>();
                var right = new List<int[]>();
                var on = new List<int[]>();

                for (var i = 0; i < corners.Count; ++i)
                {
                    if (corners[i][0] < x)
                    {
                        left.Add(corners[i]);
                    }
                    else if (corners[i][0] > x)
                    {
                        right.Add(corners[i]);
                    }
                    else
                    {
                        @on.Add(corners[i]);
                    }
                }

                //Add Steiner vertices if needed
                int[] AddSteiner(int y, bool first)
                {
                    if (!geom.StabTile(x, y))
                    {
                        for (var i = 0; i < @on.Count; ++i)
                        {
                            if (@on[i][0] == x && @on[i][1] == y)
                            {
                                return @on[i];
                            }
                        }
                        var pair = new int[] { x, y };

                        if (first)
                        {
                            @on.Insert(0, pair);
                        }
                        else
                        {
                            @on.Add(pair);
                        }
                        if (!verts.ContainsKey(Tuple.Create(pair[0], pair[1])))
                        {
                            verts[Tuple.Create(pair[0], pair[1])] = graph.AddVertex(x, y);
                        }
                        return pair;
                    }
                    return null;
                }

                var y0 = corners[0][1];
                var y1 = corners[corners.Count - 1][1];
                var loSteiner = AddSteiner(y0, true);
                var hiSteiner = AddSteiner(y1, false);

                void Bipartite(List<int[]> a, List<int[]> b)
                {
                    for (var i = 0; i < a.Count; ++i)
                    {
                        var u = a[i];
                        for (var j = 0; j < b.Count; ++j)
                        {
                            var v = b[j];
                            if (!geom.StabBox(u[0], u[1], v[0], v[1]))
                            {
                                edges.Add(new[] { u, v });
                            }
                        }
                    }
                }

                Bipartite(left, right);
                Bipartite(@on, left);
                Bipartite(@on, right);

                //Connect vertical edges
                for (var i = 1; i < @on.Count; ++i)
                {
                    var u = @on[i - 1];

                    var v = @on[i];

                    if (!geom.StabBox(u[0], u[1], v[0], v[1]))
                    {
                        edges.Add(new[] { u, v });
                    }
                }

                return (
                    left: left,
                    right: right,
                    on: on,
                    steiner0: loSteiner,
                    steiner1: hiSteiner,
                    y0: y0,
                    y1: y1
                );
            }

            //Make tree
            Node MakeTree(List<int[]> corners, int x0, int x1)
            {
                if (corners.Count == 0)
                {
                    return null;
                }

                if (corners.Count < LEAF_CUTOFF)
                {
                    return MakeLeaf(corners, x0, x1);
                }

                var x = corners[corners.Count >> 1][0];
                var partition = MakePartition(x, corners, geom, edges);
                var left = MakeTree(partition.left, x0, x);
                var right = MakeTree(partition.right, x, x1);

                //Construct vertices
                for (var i = 0; i < partition.on.Count; ++i)
                {
                    var key = Tuple.Create(partition.on[i][0], partition.on[i][1]);
                    verts[key] = graph.AddVertex(partition.on[i][0], partition.on[i][1]);
                }

                //Build buckets
                var vis = partition.vis;
                var buckets = new List<Bucket>();
                int[] lastSteiner = null;
                for (var i = 0; i < vis.Count;)
                {
                    var v0 = i;
                    var v1 = Math.Min(i + BUCKET_SIZE - 1, vis.Count - 1);
                    while (++v1 < vis.Count && vis[v1 - 1][1] == vis[v1][1]) { }
                    i = v1;
                    //vis.slice(v0, v1)
                    var bb = MakeBucket(vis.Skip(v0).Take(v1 - v0).ToList(), x);

                    if (lastSteiner != null && bb.steiner0 != null &&
                        !geom.StabBox(lastSteiner[0], lastSteiner[1], bb.steiner0[0], bb.steiner0[1]))
                    {
                        edges.Add(new[] { lastSteiner, bb.steiner0 });
                    }
                    lastSteiner = bb.steiner1;
                    buckets.Add(new Bucket(
                        bb.y0,
                        bb.y1,
                        MakeVertex(bb.steiner0),
                        MakeVertex(bb.steiner1),
                        bb.left.Select(MakeVertex).ToList(),
                        bb.right.Select(MakeVertex).ToList(),
                        bb.on.Select(MakeVertex).ToList()
                    ));
                }
                return new Node(x, buckets, left, right);
            }

            var root = MakeTree(geom.corners, int.MinValue, int.MaxValue);

            //Link edges
            for (var i = 0; i < edges.Count; ++i)
            {
                var firstKey = Tuple.Create(edges[i][0][0], edges[i][0][1]);
                var secondKey = Tuple.Create(edges[i][1][0], edges[i][1][1]);
                graph.Link(verts[firstKey], verts[secondKey]);
            }

            //Initialized graph
            graph.Init();

            //Return resulting tree
            return new L1PathPlanner(geom, graph, root);
        }
    }
};