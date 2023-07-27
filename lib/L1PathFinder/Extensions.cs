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

			result[0, 0] = matrix[0, 0];
			for (int x = 1; x < w; x++)
				result[x, 0] = matrix[x, 0] + result[x - 1, 0];
			for (int y = 1; y < h; y++)
				result[0, y] = matrix[0, y] + result[0, y-1];

			for (int x = 1; x < w; x++)
				for (int y = 1; y < h; y++)
					result[x, y] = matrix[x, y] + result[x, y - 1]+ result[x-1, y] - result[x - 1, y-1];
			return result;
		}

	}
}
