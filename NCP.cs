using OpenTK;
using OpenTK.Graphics;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal class NCP : Shape //! Невыпукый многоугольник
    {
        public Polygon[] elements;      //! выпуклые многоугольники, из которых состоит невыпуклый
        public Circle[] contactCircles; //! котнактные круги
        public Vector2[] positions;     //! позиции

        public NCP(Polygon p, Scene.types type = Scene.types.NCP) : base(type = Scene.types.NCP)
        {
            elements = CutAll(p);
            contactCircles = new Circle[elements.Length + 1];//! оставляем место под главный круг

            /*   foreach (var item in elements)
               {
                   foreach (var item1 in item.vertexs)
                   {
                       ShapeObject.Create(new Point(item1, 5, Color4.Red), new Physic(0.1f, 0, 0, 0, new Material(0, 0, 0)), "u");
                   }
               }*/
        }

        public static Polygon[] CutAll(Polygon p)//! режет невупуклый многоугольник
        {
            Vector2[] fefel = p.vertexs;

            List<Polygon> els = new List<Polygon>();
            Tuple<Polygon, Polygon> fk = Cut(p);
            if (fk is null)
            {
                return null;
            }
            els.Add(fk.Item1);
            els.Add(fk.Item2);
            List<bool> cutbl = new List<bool>();
            cutbl.Add(true);
            cutbl.Add(true);
            while (cutbl.Any(i => i))
            {
                for (int i = 0; i < els.Count; i++)
                {
                    if (cutbl[i])
                    {
                        Tuple<Polygon, Polygon> ct = Cut(els[i]);
                        if (ct == null)
                        {
                            cutbl[i] = false;
                        }
                        else
                        {
                            els[i] = ct.Item1;
                            els.Add(ct.Item2);
                            cutbl.Add(true);
                        }
                    }
                }
            }
            return els.ToArray();
        }

        private static Vector2[] RemoveParallels(Vector2[] points)//! удаляет 1 трчку из 3 лежащих на 1 прямой
        {
            List<Vector2> pns = points.ToList();
            for (int i = 0; i < points.Length; i++)
            {
                int ni = NextIndex(i, points.Length);
                Vector2 v1 = points[i] - points[ni];
                Vector2 v2 = points[ni] - points[NextIndex(ni, points.Length)];
                v1.Normalize();
                v2.Normalize();
                Console.WriteLine(Math.Abs(Vector2.Dot(v1, v2)));
                Vector2 v = v1 - v2;
                if (Math.Abs(v.X + v.Y) < 0.000000001f)
                {
                    pns.Remove(points[ni]);
                }
            }
            return pns.ToArray();
        }

        private static Tuple<Polygon, Polygon> Cut(Polygon p)//! режет 1 невыпуклый многоугольник на 2
        {
            Vector2[] edges = p.edges;
            Edge[] edges1 = p.edges1;

            Vector2[] pn1 = GetIP(edges1, edges, p);

            if (pn1 == null)
            {
                return null;
            }
            else
            {
                List<Vector2> pn2 = new List<Vector2>();

                bool f = true;
                for (int i = 0; i < p.vertexs.Length; i++)
                {
                    if (!pn1.Contains(p.vertexs[i]))
                    {
                        if (!f)
                        {
                            pn2.Add(pn1.Last());
                            pn2.Add(p.vertexs[i]);
                            f = true;
                        }
                        else
                        {
                            pn2.Add(p.vertexs[i]);
                        }
                    }
                    else
                    {
                        if (f)
                        {
                            pn2.Add(pn1[0]);
                            f = false;
                        }
                    }
                }
                pn1 = RemoveParallels(pn1);
                Vector2[] pn2a = RemoveParallels(pn2.ToArray());
                return new Tuple<Polygon, Polygon>(new Polygon(pn1), new Polygon(pn2a));
            }
        }

        private static Vector2[] GetIP(Edge[] edges1, Vector2[] edges, Polygon p)
        {
            int it = 0;
            List<Vector2> pns = new List<Vector2>();
            if (p.vertexs.Length == 3)
            {
                return null;
            }
            for (int i = 0; i < edges.Length; i++)
            {
                if (it == 2)
                {
                    return pns.ToArray();
                }
                else
                {
                    pns = new List<Vector2>();
                }
                int ni = NextIndex(i, edges.Length);
                int pi = PrevIndex(i, edges.Length);
                for (int j = 0; j < edges.Length; j++)
                {
                    float d = Math.Abs(Vector2.Dot(edges[i], edges[j]));

                    if (!(d < 1.00001f && d > 0.99999f))
                    {
                        Vector2 ip = GetIntersectPoint(edges1[i].p1, edges1[i].p2, edges1[j].p1, edges1[j].p2);
                        //   Vector2 ip1 = GetIntersectPoint(edges1[i].p1, edges1[i].p2, edges1[j].p1, edges1[j].p2);

                        if (j == ni || j == pi)
                        {
                        }
                        if (isPointOnEdge(ip, edges1[j].p1, edges1[j].p2))
                        {
                            //if (it == 1)
                            //{
                            //    //   ShapeObject.Create(new Point(ip, 5, Color4.Violet), new Physic(0.1f, 0, 0, 0, new Material(0, 0, 0)), "u");
                            //}
                            //else
                            //{
                            //    /*   ShapeObject.Create(new Point(new Vector2(0, 0.15f), 5, Color4.Orange), new Physic(0.1f, 0, 0, 0, new Material(0, 0, 0)), "u");
                            //       ShapeObject.Create(new Point(new Vector2(0.1f, 0.1f), 5, Color4.Orange), new Physic(0.1f, 0, 0, 0, new Material(0, 0, 0)), "u");*/
                            //    //  ShapeObject.Create(new Point(ip, 5, Color4.Red), new Physic(0.1f, 0, 0, 0, new Material(0, 0, 0)), "u");
                            //}
                            it++;
                            //       ShapeObject.Create(new Point(ip, 5, Color4.Green), new Physic(0.1f, 0, 0, 0, new Material(0, 0, 0)), "u");
                            if (ip == edges1[j].p1 || ip == edges1[j].p2)
                            {
                                if (it != 1)
                                {
                                    pns.Add(p.vertexs[j]);
                                    pns.Add(ip);

                                    break;
                                }
                                else
                                {
                                    Vector2 v = new Vector2(0, 0);
                                    if (ip == edges1[j].p1)
                                    {
                                        v = edges1[j].p2;
                                    }
                                    else
                                    {
                                        v = edges1[j].p1;
                                    }
                                    //Console.WriteLine(Math.Abs(Geometry.GetAngle(ip - p.vertexs[PrevIndex(j, p.vertexs.Length)], p.vertexs[PrevIndex(j, p.vertexs.Length)] - p.vertexs[PrevIndex(PrevIndex(j, p.vertexs.Length), p.vertexs.Length)])) * 57.295779f);
                                    if (Ipoint(ip, (edges1[i].p1 + edges1[i].p1) / 2, p))
                                    {
                                        pns.Add(ip);
                                    }
                                    else
                                    {
                                        it--;
                                    }
                                }
                            }
                            else
                            {
                                pns.Add(p.vertexs[j]);
                                pns.Add(ip);

                                break;
                            }
                        }
                        else
                        {
                            if (it == 1)
                            {
                                pns.Add(p.vertexs[j]);
                            }
                        }
                    }
                    else
                    {
                        if (it == 1)
                        {
                            pns.Add(p.vertexs[j]);
                        }
                    }
                }
            }
            return null;
        }

        private static int NextIndex(int i, int length) //! след индекс
        {
            return (i == length - 1 ? 0 : i + 1);
        }

        private static int PrevIndex(int i, int length) //! пред индекс
        {
            return (i == 0 ? length - 1 : i - 1);
        }

        private static bool Ipoint(Vector2 ip, Vector2 pn, Polygon p)
        {
            Vector2 v = pn - ip;
            v.Normalize();
            v *= 0.0000000001f;
            return Geometry.IsPointInPolygon(p.vertexs, ip += v);
        }

        private static Vector2 GetIntersectPoint(Vector2 v11, Vector2 v12, Vector2 v21, Vector2 v22) //! точка пересечения прямых
        {
            float x1 = v11.X;
            float x2 = v12.X;
            float x3 = v21.X;
            float x4 = v22.X;
            float y1 = v11.Y;
            float y2 = v12.Y;
            float y3 = v21.Y;
            float y4 = v22.Y;
            float x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
            float y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
            return new Vector2(x, y);
        }

        private static bool isPointOnEdge(Vector2 pn, Vector2 v1, Vector2 v2) //! точка на отрезке?
        {
            Vector2 v11 = v1 - pn;
            Vector2 v12 = v2 - pn;

            if (v11 != new Vector2(0, 0))
            {
                v11.Normalize();
            }
            else
            {
                return true;
            }

            if (v12 != new Vector2(0, 0))
            {
                v12.Normalize();
            }
            else
            {
                return true;
            }
            float d = Vector2.Dot(v11, v12);
            return (d > -1.00001f && d < -0.99999f);
        }
    }
}