using System;
using System.Collections.Generic;

namespace L1PathFinder
{
	public partial class Graph
	{
		private Vertex _target, _freeList, _toVisit, _lastS, _lastT;
		public List<Vertex> Verts { get; private set; }
		private List<Vertex> _landmarks;
		private double[] _landmarkDist;
		private int _srcX, _srcY, _dstX, _dstY;

		public Graph()
		{
			Verts = new List<Vertex>();
			_target = new Vertex(0, 0);
            _freeList = _target;
			_toVisit = Vertex.NIL;
			_lastS = null;
			_lastT = null;
			_srcX = 0;
			_srcY = 0;
			_dstX = 0;
			_dstY = 0;
			_landmarks = new List<Vertex>();
			_landmarkDist = LandmarksHelper.BuildEmptyLandMarks();
		}

		public Vertex AddVertex(int x, int y)
		{
			var v = new Vertex(x, y);
			Verts.Add(v);
			return v;
		}

		public void Link(Vertex u, Vertex v)
		{
			u.AddEdge(v);
		}

		public void SetSourceAndTarget(int sx, int sy, int tx, int ty)
		{
			_srcX = sx;
			_srcY = sy;
			_dstX = tx;
			_dstY = ty;
		}

		//Mark vertex connected to source
		public void AddSource(Vertex v)
		{
			if ((v.State & 2) == 0)
			{
				v.Heuristic = Heuristic.Run(_landmarkDist, _dstX, _dstY, v);
				v.Weight = Math.Abs(_srcX - v.X) + Math.Abs(_srcY - v.Y) + v.Heuristic;
				v.State |= 2;
				v.Pred = null;
				_toVisit = _toVisit.Push(v);
				_freeList = _freeList.Insert(v);
				_lastS = v;
			}
		}

		//Mark vertex connected to target
		public void AddTarget(Vertex v)
		{
			if ((v.State & 1) == 0)
			{
				v.State |= 1;
				_freeList = _freeList.Insert(v);
				_lastT = v;

				//Update heuristic
				var d = Math.Abs(v.X - _dstX) + Math.Abs(v.Y - _dstY);
				var vdist = v.Landmark;
				var tdist = _landmarkDist;
				for (var i = 0; i < LandmarksHelper.NUM_LANDMARKS; ++i)
				{
					tdist[i] = Math.Min(tdist[i], vdist[i] + d);
				}
			}
		}

		//Retrieves the path from dst->src
		public List<Point> GetPath(List<Point> outList)
		{
			var prevX = _dstX;
			var prevY = _dstY;

			outList.Add(new Point(prevX, prevY));

			var head = _target.Pred;

			while (head != null)
			{
				if (prevX != head.X && prevY != head.Y)
				{
					outList.Add(new Point(head.X, prevY));
				}
				if (prevX != head.X || prevY != head.Y)
				{
					outList.Add(new Point(head.X, head.Y));
				}
				prevX = head.X;
				prevY = head.Y;
				head = head.Pred;
			}
			if (prevX != _srcX && prevY != _srcY)
			{
				outList.Add(new Point(_srcX, prevY));

			}
			if (prevX != _srcX || prevY != _srcY)
			{
				outList.Add(new Point(_srcX, _srcY));
			}
			return outList;
		}

		List<Vertex[]> FindComponents()
		{
			var n = Verts.Count;

			for (var i = 0; i < n; ++i)
			{
				Verts[i].Component = -1;
			}

			var components = new List<Vertex[]>();
			for (var i = 0; i < n; ++i)
			{
				var root = Verts[i];
				if (root.Component >= 0)
				{
					continue;
				}

				var label = components.Count;
				root.Component = label;
				var toVisit = new List<Vertex>() { root };

				var ptr = 0;
				while (ptr < toVisit.Count)
				{
					var v = toVisit[ptr++];
					var adj = v.Edges;

					for (var j = 0; j < adj.Count; ++j)
					{
						var u = adj[j];
						if (u.Component >= 0)
						{
							continue;

						}
						u.Component = label;
						toVisit.Add(u);

					}
				}
				components.Add(toVisit.ToArray());
			}
			return components;
		}

		//For each connected component compute a set of landmarks
		private void FindLandmarks(Vertex[] component)
		{
			Array.Sort(component, new VertexComparer());
			var v = component[component.Length >> 1];

			for (var k = 0; k < LandmarksHelper.NUM_LANDMARKS; ++k)
			{
				v.Weight = 0.0;
				_landmarks.Add(v);

				for (var toVisit = v; toVisit != Vertex.NIL;)
				{
					v = toVisit;
					v.State = 2;
					toVisit = toVisit.Pop();

					var w = v.Weight;
					var adj = v.Edges;
					for (var i = 0; i < adj.Count; ++i)
					{
						var u = adj[i];
						if (u.State == 2)
						{
							continue;
						}
						var d = w + Math.Abs(v.X - u.X) + Math.Abs(v.Y - u.Y);
						if (u.State == 0)
						{
							u.State = 1;
							u.Weight = d;
							toVisit = toVisit.Push(u);
						}
						else if (d < u.Weight)
						{
							u.Weight = d;
							toVisit = toVisit.DecreaseKey(u);
						}
					}
				}
				double farthestD = 0;

				for (var i = 0; i < component.Length; ++i)
				{
					var u = component[i];
					u.State = 0;
					u.Landmark[k] = u.Weight;

					var s = double.MaxValue;
					for (var j = 0; j <= k; ++j)
					{
						s = Math.Min(s, u.Landmark[j]);
					}
					if (s > farthestD)
					{
						v = u;
						farthestD = s;
					}
				}
			}
		}

		public void Init()
		{
			var components = this.FindComponents();
			for (var i = 0; i < components.Count; ++i)
			{
				FindLandmarks(components[i]);
			}
		}


		//Runs a* on the graph
		public double Search()
		{
			var target = this._target;
			var freeList = this._freeList;
			var tdist = _landmarkDist;

			//Initialize target properties
			double dist = Double.MaxValue;

			//Test for case where S and T are disconnected
			if (_lastS != null && _lastT != null && _lastS.Component == _lastT.Component)
			{
				var sx = _srcX;
				var sy = _srcY;
				var tx = _dstX;
				var ty = _dstY;

				for (var toVisit = this._toVisit; toVisit != Vertex.NIL;)
				{
					var node = toVisit;
					var nx = node.X;
					var ny = node.Y;

					var d = Math.Floor(node.Weight - node.Heuristic);
					if (node.State == 3)
					{
						//If node is connected to target, exit
						dist = d + Math.Abs(tx - nx) + Math.Abs(ty - ny);
						target.Pred = node;
						break;
					}

					//Mark node closed
					node.State = 4;

					//Pop node from toVisit queue
					toVisit = toVisit.Pop();

					var adj = node.Edges;
					var n = adj.Count;
					for (var i = 0; i < n; ++i)
					{
						var v = adj[i];
						var state = v.State;
						if (state == 4)
						{
							continue;
						}

						var vd = d + Math.Abs(nx - v.X) + Math.Abs(ny - v.Y);
						if (state < 2)
						{
							var vh = Heuristic.Run(tdist, tx, ty, v);
							v.State |= 2;
							v.Heuristic = vh;
							v.Weight = vh + vd;
							v.Pred = node;
							toVisit = toVisit.Push(v);
							freeList = freeList.Insert(v);
						}
						else
						{
							var vw = vd + v.Heuristic;
							if (vw < v.Weight)
							{
								v.Weight = vw;
								v.Pred = node;
								toVisit = toVisit.DecreaseKey(v);
							}
						}
					}
				}
			}

			//Clear the free list & priority queue
			freeList.Clear();

			//Reset pointers
			this._freeList = target;
			this._toVisit = Vertex.NIL;
			this._lastS = this._lastT = null;

			//Reset landmark distance
			for (var i = 0; i < LandmarksHelper.NUM_LANDMARKS; ++i)
			{
				tdist[i] = int.MaxValue;
			}

			//Return target distance
			return dist;
		}
	}
}