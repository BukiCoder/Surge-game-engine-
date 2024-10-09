using OpenTK;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal class Geometry //! tмтоды для работы с геометрией
    {
        public static double GetAngle(Vector2 v1, Vector2 v2) //! угол между 2 прямыми
        {
            return Math.Acos(Vector2.Dot(v1, v2) / (v1.Length * v2.Length));
        }

        public static Vector2 RotateVector(Vector2 vec, float angle) //! вращение прямой
        {
            float cos = (float)Math.Cos(angle);
            float sin = (float)Math.Sin(angle);
            return new Vector2(vec.X * cos - vec.Y * sin, vec.Y * cos + vec.X * sin);
        }

        public static Vector2 GetNormal(Vector2 vec) //! перпендикуляр к прямой
        {
            return new Vector2(-vec.Y, vec.X);
        }

        public static float Length(Vector2 v1, Vector2 v2) //! расстояние между 2 точками
        {
            return (float)Math.Sqrt(Math.Pow(v1.X - v2.X, 2) + Math.Pow(v1.Y - v2.Y, 2));
        }

        public static float NSqrtLength(Vector2 v1, Vector2 v2) //! (расстояние между 2 точками)^2
        {
            return (float)(Math.Pow(v1.X - v2.X, 2) + Math.Pow(v1.Y - v2.Y, 2));
        }

        public static bool IsPointInPolygon(Vector2[] p, Vector2 point) //! находится ли точка в многоугольниеке
        {
            bool result = false;
            int j = p.Length - 1;
            for (int i = 0; i < p.Length; i++)
            {
                if ((p[i].Y < point.Y && p[j].Y >= point.Y || p[j].Y < point.Y && p[i].Y >= point.Y) && (p[i].X + (point.Y - p[i].Y) / (p[j].Y - p[i].Y) * (p[j].X - p[i].X) < point.X))
                {
                    result = !result;
                }

                j = i;
            }
            return result;
        }

        private static int Mf(float x) //! сигнум(sgn)
        {
            if (x == 0)
            {
                return 0;
            }
            else if (x < 0)
            {
                return -1;
            }
            else
            {
                return 1;
            }
        }

        public static bool IsConvex(Polygon p) //! выпуклый многоугольник?
        {
            int zn = Mf(Vector2.Dot(p.edges[0], p.edges[1]));
            for (int i = 2; i < p.edges.Length - 1; i++)
            {
                if (zn != Mf(Vector2.Dot(p.edges[i], p.edges[++i])))
                {
                    return false;
                }
            }
            if (zn != Mf(Vector2.Dot(p.edges[p.edges.Length - 1], p.edges[0])))
            {
                return false;
            }
            return true;
        }

        //private float GetRadiansFromGrads(float grads)
        //{
        //    return grads /= 57.295779f;
        //}
    }
}