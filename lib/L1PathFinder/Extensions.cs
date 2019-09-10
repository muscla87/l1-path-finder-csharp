using System;
using System.Collections.Generic;
using System.Text;

namespace L1PathFinder
{
	static class Extensions
	{
		public static T[,] Transpose<T>(this T[,] matrix)
		{
			int w = matrix.GetLength(0);
			int h = matrix.GetLength(1);

			T[,] result = new T[h, w];

			for (int i = 0; i < w; i++)
			{
				for (int j = 0; j < h; j++)
				{
					result[j, i] = matrix[i, j];
				}
			}

			return result;
		}

		public static int[,] IntegralSum(this int[,] matrix)
		{
			int w = matrix.GetLength(0);
			int h = matrix.GetLength(1);

			int[,] result = new int[w, h];

			for (int x = 0; x < w; x++)
			{
				for (int y = 0; y < h; y++)
				{
					int sum = 0;
					for (int xx = x; xx >= 0; xx--)
					{
						for (int yy = y; yy >= 0; yy--)
						{
							sum += matrix[xx, yy];
						}
					}
					result[x, y] = sum;
				}
			}
			return result;
		}

	}
}
