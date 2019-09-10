using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace L1PathFinder
{
	internal class Geometry
	{
		public List<int[]> corners;
		private int[,] grid;

		public Geometry(List<int[]> corners, int[,] grid)
		{
			this.corners = corners;
			this.grid = grid;
		}

		public bool StabRay(int vx, int vy, int x)
		{
			return this.StabBox(vx, vy, x, vy);
		}

		public bool StabTile(int x, int y)
		{
			return this.StabBox(x, y, x, y);
		}

		private int integrate(int x, int y)
		{
			if (x < 0 || y < 0)
			{
				return 0;
			}
			return this.grid[
				Math.Min(x, this.grid.GetLength(0) - 1) | 0,
				Math.Min(y, this.grid.GetLength(1) - 1) | 0];
		}

		public bool StabBox(int ax, int ay, int bx, int by)
		{
			var lox = Math.Min(ax, bx);
			var loy = Math.Min(ay, by);
			var hix = Math.Max(ax, bx);
			var hiy = Math.Max(ay, by);



			var s = this.integrate(lox - 1, loy - 1)
					- this.integrate(lox - 1, hiy)
					- this.integrate(hix, loy - 1)
					+ this.integrate(hix, hiy);
			return s > 0;
		}

		public static Geometry CreateGeometry(int[,] grid)
		{
			var loops = Countour2D.GetContours(grid.Transpose());

			//Extract corners
			var corners = new HashSet<Tuple<int, int>>();
			foreach (var polygon in loops)
			{
				for (var pIndex = 0; pIndex < polygon.Points.Count; ++pIndex)
				{
					var a = polygon.Points[(pIndex + polygon.Points.Count - 1) % polygon.Points.Count];

					var b = polygon.Points[pIndex];

					var c = polygon.Points[(pIndex + 1) % polygon.Points.Count];

					var orientation = Orientation.Orientation3(a, b, c);
					if (orientation > 0)
					{
						var offset = new[] { 0, 0 };

						for (var j = 0; j < 2; ++j)
						{
							if (b[j] - a[j] != 0)
							{
								offset[j] = b[j] - a[j];
							}
							else
							{
								offset[j] = b[j] - c[j];
							}
							offset[j] = b[j] + Math.Min((int)Math.Round(offset[j] / (double)Math.Abs(offset[j])) | 0, 0);

						}
						if (offset[0] >= 0 && offset[0] < grid.GetLength(0) &&
							offset[1] >= 0 && offset[1] < grid.GetLength(1) &&
							grid[offset[0], offset[1]] == 0)
						{
							corners.Add(Tuple.Create(offset[0], offset[1]));
						}
					}
				}
			}

			//porting: corners is now an hashset
			//Remove duplicate corners
			//uniq(corners, comparePair); 

			//Create integral image
			var img = new int[grid.GetLength(0), grid.GetLength(1)];
			for (int i = 0; i < grid.GetLength(0); i++)
			{
				for (int j = 0; j < grid.GetLength(1); j++)
				{
					img[i, j] = grid[i, j] > 0 ? 1 : 0;
				}
			}

			img = img.IntegralSum();
			var cornersArray = corners.ToArray();
			Array.Sort(cornersArray, new CornerCompare());

			//Return resulting geometry
			return new Geometry(cornersArray.Select(corner => new int[] { corner.Item1, corner.Item2 }).ToList(), img);
		}

		private static void saveMatrix(int[,] img, string filename)
		{
			StringBuilder sbBuilder = new StringBuilder();
			for (int x = 0; x < img.GetLength(0); x++)
			{
				for (int y = 0; y < img.GetLength(1); y++)
				{
					sbBuilder.Append(img[x, y]);
					if (y != img.GetLength(1) - 1)
					{
						sbBuilder.Append(" ");
					}
					else
					{
						sbBuilder.AppendLine("");
					}
				}
			}
			File.WriteAllText(filename, sbBuilder.ToString());
		}

		class CornerCompare : IComparer
		{
			public int Compare(object x, object y)
			{
				var a = x as Tuple<int, int>;
				var b = y as Tuple<int, int>;
				var d = a.Item1 - b.Item1;
				if (d != 0)
				{
					return d;
				}
				return a.Item2 - b.Item2;

			}
		}
	}
}