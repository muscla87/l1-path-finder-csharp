using System;

namespace L1PathFinder
{
    public class Orientation
    {
        private const int NUM_EXPAND = 5;

        private const double EPSILON = 1.1102230246251565e-16;
        private const double ERRBOUND3 = (3.0 + 16.0 * EPSILON) * EPSILON;
        private const double ERRBOUND4 = (7.0 + 56.0 * EPSILON) * EPSILON;

        private static readonly double SPLITTER = +(Math.Pow(2, 27) + 1.0);

        static double[] Prod(double a, double b)
        {
            var x = a * b;

            var c = SPLITTER * a;
            var abig = c - a;
            var ahi = c - abig;
            var alo = a - ahi;

            var d = SPLITTER * b;
            var bbig = d - b;
            var bhi = d - bbig;
            var blo = b - bhi;

            var err1 = x - (ahi * bhi);
            var err2 = err1 - (alo * bhi);
            var err3 = err2 - (ahi * blo);

            var y = alo * blo - err3;

            return new[] { y, x };
        }

        static double Orientation3Exact(int[] m0, int[] m1, int[] m2)
        {
            Func<double[], double[], double[]> sum = RobustSum.Sum;
            Func<double[], double[], double[]> sub = RobustDiff.Subtract;

            var p = sum(sum(Prod(m1[1], m2[0]), Prod(-m2[1], m1[0])), sum(Prod(m0[1], m1[0]), Prod(-m1[1], m0[0])));
            var n = sum(Prod(m0[1], m2[0]), Prod(-m2[1], m0[0]));
            var d = sub(p, n);
            return d[d.Length - 1];
        }

        public static double Orientation3(int[] a, int[] b, int[] c)
        {
            var l = (double)(a[1] - c[1]) * (b[0] - c[0]);
            var r = (double)(a[0] - c[0]) * (b[1] - c[1]);
            var det = l - r;
            double s;
            if (l > 0)
            {
                if (r <= 0)
                {
                    return det;
                }
                else
                {
                    s = l + r;
                }
            }
            else if (l < 0)
            {
                if (r >= 0)
                {
                    return det;
                }
                else
                {
                    s = -(l + r);
                }
            }
            else
            {
                return det;
            }
            var tol = ERRBOUND3 * s;
            if (det >= tol || det <= -tol)
            {
                return det;
            }
            return Orientation3Exact(a, b, c);
        }
    }

    class RobustSum
    {
        //Easy case: Add two scalars
        public static double[] ScalarScalar(double a, double b)
        {
            var x = a + b;
            var bv = x - a;
            var av = x - bv;
            var br = b - bv;
            var ar = a - av;
            var y = ar + br;
            if (y != 0)
            {
                return new[] { y, x };
            }
            return new[] { x };
        }

        public static double[] Sum(double[] e, double[] f)
        {
            var ne = e.Length | 0;
            var nf = f.Length | 0;
            if (ne == 1 && nf == 1)
            {
                return ScalarScalar(e[0], f[0]);
            }
            var n = ne + nf;
            var g = new double[n];
            var count = 0;
            var eptr = 0;
            var fptr = 0;
            Func<double, double> abs = Math.Abs;
            var ei = e[eptr];
            var ea = abs(ei);
            var fi = f[fptr];
            var fa = abs(fi);
            double a, b;
            if (ea < fa)
            {
                b = ei;
                eptr += 1;
                if (eptr < ne)
                {
                    ei = e[eptr];
                    ea = abs(ei);
                }
            }
            else
            {
                b = fi;
                fptr += 1;
                if (fptr < nf)
                {
                    fi = f[fptr];
                    fa = abs(fi);
                }
            }
            if ((eptr < ne && ea < fa) || (fptr >= nf))
            {
                a = ei;
                eptr += 1;
                if (eptr < ne)
                {
                    ei = e[eptr];
                    ea = abs(ei);
                }
            }
            else
            {
                a = fi;
                fptr += 1;
                if (fptr < nf)
                {
                    fi = f[fptr];
                    fa = abs(fi);
                }
            }
            var x = a + b;
            var bv = x - a;
            var y = b - bv;
            var q0 = y;
            var q1 = x;
            double _x, _bv, _av, _br, _ar;
            while (eptr < ne && fptr < nf)
            {
                if (ea < fa)
                {
                    a = ei;
                    eptr += 1;
                    if (eptr < ne)
                    {
                        ei = e[eptr];
                        ea = abs(ei);
                    }
                }
                else
                {
                    a = fi;
                    fptr += 1;
                    if (fptr < nf)
                    {
                        fi = f[fptr];
                        fa = abs(fi);
                    }
                }
                b = q0;
                x = a + b;
                bv = x - a;
                y = b - bv;
                if (y != 0)
                {
                    g[count++] = y;
                }
                _x = q1 + x;
                _bv = _x - q1;
                _av = _x - _bv;
                _br = x - _bv;
                _ar = q1 - _av;
                q0 = _ar + _br;
                q1 = _x;
            }
            while (eptr < ne)
            {
                a = ei;
                b = q0;
                x = a + b;
                bv = x - a;
                y = b - bv;
                if (y != 0)
                {
                    g[count++] = y;
                }
                _x = q1 + x;
                _bv = _x - q1;
                _av = _x - _bv;
                _br = x - _bv;
                _ar = q1 - _av;
                q0 = _ar + _br;
                q1 = _x;
                eptr += 1;
                if (eptr < ne)
                {
                    ei = e[eptr];
                }
            }
            while (fptr < nf)
            {
                a = fi;
                b = q0;
                x = a + b;
                bv = x - a;
                y = b - bv;
                if (y != 0)
                {
                    g[count++] = y;
                }
                _x = q1 + x;
                _bv = _x - q1;
                _av = _x - _bv;
                _br = x - _bv;
                _ar = q1 - _av;
                q0 = _ar + _br;
                q1 = _x;
                fptr += 1;
                if (fptr < nf)
                {
                    fi = f[fptr];
                }
            }
            if (q0 != 0)
            {
                g[count++] = q0;
            }
            if (q1 != 0)
            {
                g[count++] = q1;
            }
            if (count == 0)
            {
                g[count++] = 0.0;
            }
            Array.Resize(ref g, count);
            //g.length = count;
            return g;
        }

    }

    class RobustDiff
    {
        //Easy case: Add two scalars
        static double[] ScalarScalar(double a, double b)
        {
            var x = a + b;
            var bv = x - a;
            var av = x - bv;
            var br = b - bv;
            var ar = a - av;
            var y = ar + br;
            if (y != 0)
            {
                return new[] { y, x };
            }
            return new[] { x };
        }

        public static double[] Subtract(double[] e, double[] f)
        {
            var ne = e.Length;
            var nf = f.Length;
            if (ne == 1 && nf == 1)
            {
                return ScalarScalar(e[0], -f[0]);
            }
            var n = ne + nf;
            var g = new double[n];
            var count = 0;
            var eptr = 0;
            var fptr = 0;
            Func<double, double> abs = Math.Abs;
            var ei = e[eptr];
            var ea = abs(ei);
            var fi = -f[fptr];
            var fa = abs(fi);
            double a, b;
            if (ea < fa)
            {
                b = ei;
                eptr += 1;
                if (eptr < ne)
                {
                    ei = e[eptr];
                    ea = abs(ei);
                }
            }
            else
            {
                b = fi;
                fptr += 1;
                if (fptr < nf)
                {
                    fi = -f[fptr];
                    fa = abs(fi);
                }
            }
            if ((eptr < ne && ea < fa) || (fptr >= nf))
            {
                a = ei;
                eptr += 1;
                if (eptr < ne)
                {
                    ei = e[eptr];
                    ea = abs(ei);
                }
            }
            else
            {
                a = fi;
                fptr += 1;
                if (fptr < nf)
                {
                    fi = -f[fptr];
                    fa = abs(fi);
                }
            }
            var x = a + b;
            var bv = x - a;
            var y = b - bv;
            var q0 = y;
            var q1 = x;
            double _x, _bv, _av, _br, _ar;
            while (eptr < ne && fptr < nf)
            {
                if (ea < fa)
                {
                    a = ei;
                    eptr += 1;
                    if (eptr < ne)
                    {
                        ei = e[eptr];
                        ea = abs(ei);
                    }
                }
                else
                {
                    a = fi;
                    fptr += 1;
                    if (fptr < nf)
                    {
                        fi = -f[fptr];
                        fa = abs(fi);
                    }
                }
                b = q0;
                x = a + b;
                bv = x - a;
                y = b - bv;
                if (y != 0)
                {
                    g[count++] = y;
                }
                _x = q1 + x;
                _bv = _x - q1;
                _av = _x - _bv;
                _br = x - _bv;
                _ar = q1 - _av;
                q0 = _ar + _br;
                q1 = _x;
            }
            while (eptr < ne)
            {
                a = ei;
                b = q0;
                x = a + b;
                bv = x - a;
                y = b - bv;
                if (y != 0)
                {
                    g[count++] = y;
                }
                _x = q1 + x;
                _bv = _x - q1;
                _av = _x - _bv;
                _br = x - _bv;
                _ar = q1 - _av;
                q0 = _ar + _br;
                q1 = _x;
                eptr += 1;
                if (eptr < ne)
                {
                    ei = e[eptr];
                }
            }
            while (fptr < nf)
            {
                a = fi;
                b = q0;
                x = a + b;
                bv = x - a;
                y = b - bv;
                if (y != 0)
                {
                    g[count++] = y;
                }
                _x = q1 + x;
                _bv = _x - q1;
                _av = _x - _bv;
                _br = x - _bv;
                _ar = q1 - _av;
                q0 = _ar + _br;
                q1 = _x;
                fptr += 1;
                if (fptr < nf)
                {
                    fi = -f[fptr];
                }
            }
            if (q0 != 0)
            {
                g[count++] = q0;
            }
            if (q1 != 0)
            {
                g[count++] = q1;
            }
            if (count == 0)
            {
                g[count++] = 0.0;
            }
            Array.Resize(ref g, count);
            //g.length = count;
            return g;
        }
    }

}