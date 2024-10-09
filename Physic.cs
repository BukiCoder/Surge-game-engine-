using OpenTK;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Surge
{
    internal class Physic
    {
        public float mass { get; set; } //! масса

        public float gravity { get; set; } //! гравитация
        public Vector2 velocity { get; set; }

        public float inertia; //! инерция

        private Vector2 rotatevector;
        public Vector2 speed { get; set; } //! скорость

        public Material material; //! материал

        private float angularVelocity;

        public Vector2 impuls { get; set; }

        public float lost { get; set; }

        public Physic(float mass, float gravity, float lost, float inertia, Material material)
        {
            this.lost = lost;
            this.mass = mass;
            this.gravity = gravity;
            this.inertia = inertia;
            this.material = material;
        }

        public static bool overlapAABBvsAABB(ShapeObject obj, ShapeObject other)
        {
            AABB a = obj.box as AABB;
            AABB b = other.box as AABB;
            if (a.max.X > b.min.X || a.min.X < b.max.X) return false;
            if (a.max.Y > b.min.Y || a.min.Y < b.max.Y) return false;
            return true;
        }

        private Collision FindAxisLeastPenetration(Vector2[] Av, Circle cr, Vector2[] normals)
        {
            float bestDistance = float.NegativeInfinity;
            int bestIndex = 0;
            for (int i = 0; i < Av.Length; ++i)
            {
                Vector2 n = normals[i];
                Vector2 s = GFP(cr, -n);
                Vector2 v = Av[i];
                float d = Vector2.Dot(n, s - v);
                if (d > bestDistance)
                {
                    bestDistance = d;
                    bestIndex = i;
                }
            }
            return new Collision(bestDistance, normals[bestIndex]);
        }

        private Collision FindAxisLeastPenetration(Vector2[] Av, Vector2[] Bv, Vector2[] normals)
        {
            float bestDistance = float.NegativeInfinity;
            int bestIndex = 0;
            for (int i = 0; i < Av.Length; ++i)
            {
                Vector2 n = normals[i];
                Vector2 s = Bv[GetSupport(Bv, -n)];
                Vector2 v = Av[i];
                float d = Vector2.Dot(n, s - v);
                if (d > bestDistance)
                {
                    bestDistance = d;
                    bestIndex = i;
                }
            }
            return new Collision(bestDistance, normals[bestIndex]);
        }

        public void AddImpulse(Vector2 force)
        {
            this.velocity += force / mass;
        }

        private Vector2[] GetNormals(Vector2[] edges)
        {
            return edges.Select(e => Geometry.GetNormal(e)).ToArray();
        }

        private Vector2 GJKSupport(Vector2[] plg1, Vector2[] plg2, Vector2 dir)
        {
            return GFP(plg1, dir) - GFP(plg2, -dir);
        }

        private Vector2 GJKSupport(Vector2[] plg, Circle cr, Vector2 dir)
        {
            return GFP(plg, dir) - GFP(cr, -dir);
        }

        private Vector2 TripleProduct(Vector2 a, Vector2 b, Vector2 c)
        {
            Vector2 r;
            float ac = a.X * c.X + a.Y * c.Y;
            float bc = b.X * c.X + b.Y * c.Y;
            r.X = b.X * ac - a.X * bc;
            r.Y = b.Y * ac - a.Y * bc;
            return r;
        }

        private Vector2 ContactPointSupport(Vector2[] intersectTriangle1, Vector2[] intersectTriangle2, Vector2 v11, Vector2 v12, Vector2 v21)
        {
            bool vd1 = Geometry.IsPointInPolygon(intersectTriangle2, v11);
            bool vd2 = Geometry.IsPointInPolygon(intersectTriangle2, v12);
            bool vd3 = Geometry.IsPointInPolygon(intersectTriangle1, v21);
            if (vd2 && vd1)
            {
                //    ShapeObject sh = ShapeObject.Create(new Point(v11, 4, Color4.Violet), null, "uhk");
                //    ShapeObject sh1 = ShapeObject.Create(new Point(v12, 4, Color4.Violet), null, "gjy");
                return (v11 + v12) / 2;
            }
            else if (vd2 && vd3)
            {
                //ShapeObject sh = ShapeObject.Create(new Point(v11, 4, Color4.Violet), null, "uhk");
                //ShapeObject sh1 = ShapeObject.Create(new Point(v21, 4, Color4.Violet), null, "gjy");
                return (v11 + v21) / 2;
            }
            else if (vd3 && vd1)
            {
                //ShapeObject sh = ShapeObject.Create(new Point(v21, 4, Color4.Violet), null, "uhk");
                //ShapeObject sh1 = ShapeObject.Create(new Point(v11, 4, Color4.Violet), null, "gjy");
                return (v11 + v21) / 2;
            }
            else
            {
                //ShapeObject sh = ShapeObject.Create(new Point(v21, 4, Color4.Red), null, "uhk");
                //ShapeObject sh1 = ShapeObject.Create(new Point(v21, 4, Color4.Red), null, "gjy");
                return (v21 + v21) / 2;
            }
        }

        private Vector2 GetContactPoint(Vector2 MTV, Circle cr, Vector2[] vr)
        {
            float min = float.PositiveInfinity;
            int bi = 0;
            for (int i = 0; i < vr.Length; i++)
            {
                float ln = Geometry.NSqrtLength(cr.center, vr[i]);
                if (min > ln)
                {
                    min = ln;
                    bi = i;
                }
                else if (ln == min)
                {
                    return (vr[i] + vr[i - 1]) / 2;
                }
            }
            return vr[bi];
        }

        private Vector2 GetContactPoint(Vector2 MTV, Vector2[] vr1, Vector2[] vr2)
        {
            int v1i = GetSupport(vr1, MTV);
            Vector2 v1 = vr1[v1i];
            Vector2 v1n = NextVertex(vr1, v1i);
            Vector2 v1p = PrevVertex(vr1, v1i);
            int v2i = GetSupport(vr2, -MTV);
            Vector2 v2 = vr2[v2i];
            Vector2 v2n = NextVertex(vr2, v2i);
            Vector2 v2p = PrevVertex(vr2, v2i);
            Vector2[] intersectTriangle1 = new Vector2[] { v1p, v1, v1n };
            Vector2[] intersectTriangle2 = new Vector2[] { v2p, v2, v2n };
            Vector2 v1pe = v1p - v1;
            Vector2 v1e = v1 - v1n;
            Vector2 v1ne = v1n - v1p;
            Vector2 v2pe = v2p - v2;
            Vector2 v2e = v2 - v2n;
            Vector2 v2ne = v2n - v2p;
            v1pe.Normalize();
            v1e.Normalize();
            v1ne.Normalize();
            v2pe.Normalize();
            v2e.Normalize();
            v2ne.Normalize();
            if (Vector2.Dot(v1ne, v2ne) == 1 && Vector2.Dot(v1pe, v2ne) == 1)
            {
                return v1;
            }
            else if (Vector2.Dot(v1pe, v2pe) == 0)
            {
                return ContactPointSupport(intersectTriangle1, intersectTriangle2, v1, v1p, v2);
            }
            else if (Vector2.Dot(v1e, v2e) == 0)
            {
                return ContactPointSupport(intersectTriangle1, intersectTriangle2, v1, v1n, v2);
            }
            else if (Vector2.Dot(v1e, v2pe) == 0)
            {
                return ContactPointSupport(intersectTriangle1, intersectTriangle2, v1, v1n, v2);
            }
            else if (Vector2.Dot(v1e, v2ne) == 0)
            {
                return ContactPointSupport(intersectTriangle1, intersectTriangle2, v1, v1n, v2n);
            }
            else if (Vector2.Dot(v1pe, v2e) == 0)
            {
                return ContactPointSupport(intersectTriangle1, intersectTriangle2, v1, v1p, v2);
            }
            else if (Vector2.Dot(v1pe, v2ne) == 0)
            {
                return ContactPointSupport(intersectTriangle1, intersectTriangle2, v1, v1p, v2n);
            }
            else if (Vector2.Dot(v1ne, v2e) == 0)
            {
                return ContactPointSupport(intersectTriangle1, intersectTriangle2, v1n, v1p, v2);
            }
            else if (Vector2.Dot(v1ne, v2pe) == 0)
            {
                return ContactPointSupport(intersectTriangle1, intersectTriangle2, v1n, v1p, v2);
            }
            else
            {
                foreach (var item in intersectTriangle1)
                {
                    //   ShapeObject sh = ShapeObject.Create(new Point(item, 4, Color4.Cyan), null, "uhk");
                    if (Geometry.IsPointInPolygon(intersectTriangle2, item))
                    {
                        return (item);
                    }
                }
                foreach (var item in intersectTriangle2)
                {
                    if (Geometry.IsPointInPolygon(intersectTriangle1, item))
                    {
                        return item;
                    }
                }
            }
            return new Vector2(0, 0);
        }

        private bool GJK(Vector2[] plg1, Vector2[] plg2, Vector2 position1, Vector2 position2)
        {
            Vector2[] simplex = new Vector2[plg1.Length + plg2.Length];
            int index = 0;
            Vector2 a, b, c, d, ao, ab, ac, abperp, acperp;
            d = (position1 - position2);
            if ((d.X == 0) && (d.Y == 0))
                d.X = 1f;
            a = simplex[0] = GJKSupport(plg1, plg2, d);
            if (Vector2.Dot(a, d) <= 0)
            {
                return false;
            }
            int iter_count = 0;
            d = -a;
            while (true)
            {
                iter_count++;
                a = simplex[++index] = GJKSupport(plg1, plg2, d);
                if (Vector2.Dot(a, d) <= 0)
                    return false;
                ao = -(a);
                if (index < 2)
                {
                    b = simplex[0];
                    ab = b - a;
                    d = TripleProduct(ab, ao, ab);
                    if (d.Length == 0)
                        d = Geometry.GetNormal(ab);
                    continue;
                }

                b = simplex[1];
                c = simplex[0];
                ab = b - a;
                ac = c - a;
                acperp = TripleProduct(ab, ac, ac);
                if (Vector2.Dot(acperp, ao) >= 0)
                {
                    d = acperp;
                }
                else
                {
                    abperp = TripleProduct(ac, ab, ab);
                    if (Vector2.Dot(abperp, ao) < 0)
                        return true;
                    simplex[0] = simplex[1];
                    d = abperp;
                }
                simplex[1] = simplex[2];
                --index;
            }
            return false;
        }

        private bool GJK(Vector2[] plg, Circle cr, Vector2 position1, Vector2 position2)
        {
            Vector2[] simplex = new Vector2[plg.Length];
            int index = 0;
            Vector2 a, b, c, d, ao, ab, ac, abperp, acperp;
            d = (position1 - position2);
            if ((d.X == 0) && (d.Y == 0))
                d.X = 1f;
            a = simplex[0] = GJKSupport(plg, cr, d);
            if (Vector2.Dot(a, d) <= 0)
            {
                return false;
            }
            int iter_count = 0;
            d = -a;
            while (true)
            {
                iter_count++;
                a = simplex[++index] = GJKSupport(plg, cr, d);
                if (Vector2.Dot(a, d) <= 0)
                    return false;
                ao = -(a);
                if (index < 2)
                {
                    b = simplex[0];
                    ab = b - a;
                    d = TripleProduct(ab, ao, ab);
                    if (d.Length == 0)
                        d = Geometry.GetNormal(ab);
                    continue;
                }

                b = simplex[1];
                c = simplex[0];
                ab = b - a;
                ac = c - a;
                acperp = TripleProduct(ab, ac, ac);
                if (Vector2.Dot(acperp, ao) >= 0)
                {
                    d = acperp;
                }
                else
                {
                    abperp = TripleProduct(ac, ab, ab);
                    if (Vector2.Dot(abperp, ao) < 0)
                        return true;
                    simplex[0] = simplex[1];
                    d = abperp;
                }
                simplex[1] = simplex[2];
                --index;
            }
            return false;
        }

        private Vector2 GFP(Vector2[] plg, Vector2 dir)
        {
            Vector2 farthest = new Vector2();

            float lastDot = float.NegativeInfinity;

            for (int i = 0; i < plg.Length; i++)
            {
                float dot = Vector2.Dot(dir, plg[i]);
                if (dot > lastDot)
                {
                    farthest = plg[i];
                    lastDot = dot;
                }
            }
            return farthest;
        }

        private Vector2 GFP(Circle cr, Vector2 dir)
        {
            dir.Normalize();

            return cr.center + dir * cr.radius;
        }

        private int GetSupport(Vector2[] vertexs, Vector2 dir)
        {
            float bestProjection = float.NegativeInfinity;
            int index = 0;

            for (int i = 0; i < vertexs.Length; ++i)
            {
                Vector2 v = vertexs[i];
                float projection = Vector2.Dot(v, dir);

                if (projection > bestProjection)
                {
                    index = i;
                    bestProjection = projection;
                }
            }

            return index;
        }

        public void DoCollision(ShapeObject obj1, ShapeObject obj2, Vector2 normal)
        {
            /*float m1 = obj1.phys.mass;
            float m2 = obj2.phys.mass;
            Vector2 v1 = obj1.phys.speed;
            Vector2 v2 = obj2.phys.speed;
            Vector2 pointSpeed1 = ((m1 - m2) * v1 + 2 * m2 * v2) / (m1 + m2);
            Vector2 radius1 = pn - obj1.contactCircle.center;
            float cos = Math.Cos((double)Geometry.GetAngle(pointSpeed1, radius1));*/
            Vector2 rv = obj1.phys.impuls - obj2.phys.impuls;
            rv += obj1.rotateVector - obj2.rotateVector;
            float velAlongNormal = Vector2.Dot(rv, normal);
            if (velAlongNormal > 0)
            {
                return;
            }
            float e = 0;
            if (obj1.phys.material.restution > obj2.phys.material.restution)
            {
                e = obj2.phys.material.restution;
            }
            else
            {
                e = obj1.phys.material.restution;
            }
            float j = -(1 + e) * velAlongNormal;
            j /= obj1.phys.mass + obj2.phys.mass;
            Vector2 impulse = j * normal;
            float mass_sum = obj1.phys.mass + obj2.phys.mass;
            float ratio = mass_sum / obj1.phys.mass;
            obj1.phys.impuls += impulse * ratio;
            //Console.WriteLine("i1   " + obj1.phys.impuls);
            ratio = mass_sum / obj2.phys.mass;
            obj2.phys.impuls -= ratio * impulse;
            //Console.WriteLine("i2   " + obj2.phys.impuls);
            Vector2 tangent = rv - Vector2.Dot(rv, normal) * normal;
            tangent.Normalize();
            Vector2 t = Geometry.GetNormal(normal);
            float jt = -Vector2.Dot(rv, t);
            jt /= obj1.phys.mass + obj2.phys.mass;
            float mu = obj1.phys.material.staticFriction * obj1.phys.material.staticFriction + obj2.phys.material.staticFriction * obj2.phys.material.staticFriction;
            Vector2 frictionImpulse;
            float dynamicFriction;
            if (Math.Abs(jt) < j * mu)
            {
                frictionImpulse = jt * t;
            }
            else
            {
                dynamicFriction = obj1.phys.material.dynamicFriction * obj1.phys.material.dynamicFriction + obj2.phys.material.dynamicFriction * obj2.phys.material.dynamicFriction;
                frictionImpulse = -j * t * dynamicFriction;
            }
            obj1.phys.impuls += obj1.phys.mass * frictionImpulse;
            obj2.phys.impuls -= obj2.phys.mass * frictionImpulse;
            obj1.TranslateD(obj1.phys.impuls / obj1.phys.mass);
            obj2.TranslateD(obj2.phys.impuls / obj2.phys.mass);
        }

        private Vector2[] abedges = new Vector2[] { new Vector2(0, 1), new Vector2(0, -1), new Vector2(1, 0), new Vector2(-1, 0) };

        public void RefreshImpuls(ShapeObject obj)
        {
            Project.container.scene.NRIS.Remove(obj);
            obj.TranslateD(speed);
            speed += velocity;
            obj.Rotate(angularVelocity);
        }

        private Vector2 NextVertex(Vector2[] vertexs, int currentIndex)
        {
            int i = currentIndex + 1;
            return vertexs[i == vertexs.Length ? 0 : i];
        }

        private Vector2 PrevVertex(Vector2[] vertexs, int currentIndex)
        {
            int i = currentIndex - 1;
            return vertexs[i < 0 ? vertexs.Length - 1 : i];
        }

        private void DoCollisionPlgvsPlg(ShapeObject o1, ShapeObject o2, Vector2[] v1, Vector2[] v2)
        {
            Collision collision = FindAxisLeastPenetration(v1, v2, GetNormals(v1));

            collision.MTV.Normalize();
            o1.TranslateD(-Math.Abs(collision.overlap) * collision.MTV * 0.99f);
            Vector2 cp = GetContactPoint(collision.MTV, v1, v2);
            o1.TranslateD(-Math.Abs(collision.overlap) * collision.MTV * 0.01f);
            DoCollision(o1, o2, -collision.MTV);

            Vector2 b = o2.phys.impuls;
            b.Normalize();
            Vector2 t = cp - o2.position;
            t.Normalize();
            float k = Vector2.Dot(t + b, b);

            Vector2 k1 = cp - o1.position;
            k1.Normalize();
            Vector2 k2 = cp - o2.position;
            k2.Normalize();
            //Console.WriteLine("yy   " + (Vector2.Dot(MathS.GetNormal(k1), pair.obj1.phys.impuls)) * (k < 0.1f ? 1 : -1));
            o2.phys.angularVelocity += (2 / o2.phys.inertia) * (Vector2.Dot(Geometry.GetNormal(k2), o2.phys.impuls)) * (k < 0.0f ? 1 : -1);
            o1.phys.angularVelocity += (2 / o1.phys.inertia) * (Vector2.Dot(Geometry.GetNormal(k1), o1.phys.impuls)) * (k < 0.0f ? 1 : -1);
        }

        private void DoCollisionCirclevsPlg(ShapeObject cr, ShapeObject o2, Vector2[] v2)
        {
            Collision collision = FindAxisLeastPenetration(v2, o2.box as Circle, abedges);

            collision.MTV.Normalize();
            cr.TranslateD(-Math.Abs(collision.overlap) * collision.MTV * 0.99f);
            Vector2 cp = GetContactPoint(collision.MTV, cr.box as Circle, v2);
            cr.TranslateD(-Math.Abs(collision.overlap) * collision.MTV * 0.01f);
            DoCollision(cr, o2, -collision.MTV);

            Vector2 b = o2.phys.impuls;
            b.Normalize();
            Vector2 t = cp - o2.position;
            t.Normalize();
            float k = Vector2.Dot(t + b, b);

            Vector2 k1 = cp - cr.position;
            k1.Normalize();
            Vector2 k2 = cp - o2.position;
            k2.Normalize();
            //Console.WriteLine("yy   " + (Vector2.Dot(MathS.GetNormal(k1), pair.obj1.phys.impuls)) * (k < 0.1f ? 1 : -1));

            o2.phys.angularVelocity += (2 / o2.phys.inertia) * (Vector2.Dot(Geometry.GetNormal(k2), o2.phys.impuls)) * (k < 0.0f ? 1 : -1);

            cr.phys.angularVelocity += (2 / cr.phys.inertia) * (Vector2.Dot(Geometry.GetNormal(k1), cr.phys.impuls)) * (k < 0.0f ? 1 : -1);
        }

        private void DoCollisionCirclevsCircle(ShapeObject cr, ShapeObject cr1)
        {
            Circle c1 = cr.box as Circle;
            Circle c2 = cr1.box as Circle;
            float overlap = c1.radius + c2.radius - Geometry.Length(c1.center, c2.center);
            Vector2 MTV = c1.center - c2.center;
            MTV.Normalize();
            //Thread.Sleep(2000);

            MTV.Normalize();
            cr.TranslateD(-Math.Abs(overlap) * MTV * 0.99f);
            Vector2 cp = c1.center + MTV * c1.radius;
            cr.TranslateD(-Math.Abs(overlap) * MTV * 0.01f);
            DoCollision(cr, cr1, -MTV);
            //ShapeObject sh = ShapeObject.Create(new Point(cp, 4, Color4.Aqua), null, "uhk");
            Vector2 b = cr1.phys.impuls;
            b.Normalize();
            Vector2 t = cp - cr1.position;
            t.Normalize();
            float k = Vector2.Dot(t + b, b);

            Vector2 k1 = cp - cr.position;
            k1.Normalize();
            Vector2 k2 = cp - cr1.position;
            k2.Normalize();
            //Console.WriteLine("yy   " + (Vector2.Dot(MathS.GetNormal(k1), pair.obj1.phys.impuls)) * (k < 0.1f ? 1 : -1));
            cr1.phys.angularVelocity += (2 / cr1.phys.inertia) * (Vector2.Dot(Geometry.GetNormal(k2), cr1.phys.impuls)) * (k < 0.0f ? 1 : -1);
            cr.phys.angularVelocity += (2 / cr.phys.inertia) * (Vector2.Dot(Geometry.GetNormal(k1), cr.phys.impuls)) * (k < 0.0f ? 1 : -1);
        }

        public void DoAllCollisions()
        {
            List<Pair> pairs = GenerateContactPairs();
            foreach (var pair in pairs)
            {
                if (pair.obj1.box.type == Scene.types.Polygon)
                {
                    switch (pair.obj2.box.type)
                    {
                        case Scene.types.Polygon:
                            Polygon p1 = (pair.obj1.box as Polygon);
                            Polygon p2 = (pair.obj2.box as Polygon);
                            if (GJK(p1.vertexs, p2.vertexs, pair.obj1.position, pair.obj2.position))
                            {
                                DoCollisionPlgvsPlg(pair.obj1, pair.obj2, p1.vertexs, p2.vertexs);
                            }
                            break;

                        case Scene.types.AABB:
                            Polygon p = (pair.obj1.box as Polygon);
                            AABB ab = (pair.obj2.box as AABB);
                            if (GJK(ab.vertexs, p.vertexs, pair.obj2.position, pair.obj1.position))
                            {
                                DoCollisionPlgvsPlg(pair.obj2, pair.obj1, ab.vertexs, p.vertexs);
                            }
                            break;

                        case Scene.types.NCP:
                            Polygon p3 = (pair.obj1.box as Polygon);
                            NCP ncp = (pair.obj2.box as NCP);
                            for (int i = 0; i < ncp.elements.Length; i++)
                            {
                                if (IsCirclesIntersect(pair.obj1.contactCircle, ncp.contactCircles[i]))
                                {
                                    if (GJK(p3.vertexs, ncp.elements[i].vertexs, pair.obj1.position, ncp.positions[i]))
                                    {
                                        DoCollisionPlgvsPlg(pair.obj1, pair.obj2, p3.vertexs, ncp.elements[i].vertexs);
                                        break;
                                    }
                                }
                            }
                            //   DoNCPvsPolygonCollision(pair.obj2, (pair.obj1.box as Polygon).vertexs, pair.obj1);
                            break;

                        case Scene.types.OBB:
                            Polygon p4 = (pair.obj1.box as Polygon);
                            OBB ob = (pair.obj2.box as OBB);
                            if (GJK(ob.vertexs, p4.vertexs, pair.obj2.position, pair.obj1.position))
                            {
                                DoCollisionPlgvsPlg(pair.obj2, pair.obj1, ob.vertexs, p4.vertexs);
                            }
                            break;

                        default:
                            Polygon p5 = (pair.obj1.box as Polygon);
                            Circle cr = (pair.obj2.box as Circle);
                            if (GJK(p5.vertexs, cr, pair.obj1.position, cr.center))
                            {
                                DoCirclevsPolygonCollision(pair.obj2, pair.obj1, p5.vertexs, p5.edges);
                            }

                            break;
                    }
                }
                else if (pair.obj1.box.type == Scene.types.NCP)
                {
                    switch (pair.obj2.box.type)
                    {
                        case Scene.types.Polygon:
                            Polygon p3 = (pair.obj2.box as Polygon);
                            NCP ncp = (pair.obj1.box as NCP);
                            for (int i = 0; i < ncp.elements.Length; i++)
                            {
                                if (IsCirclesIntersect(pair.obj2.contactCircle, ncp.contactCircles[i]))
                                {
                                    if (GJK(p3.vertexs, ncp.elements[i].vertexs, pair.obj2.position, ncp.positions[i]))
                                    {
                                        DoCollisionPlgvsPlg(pair.obj2, pair.obj1, p3.vertexs, ncp.elements[i].vertexs);
                                        break;
                                    }
                                }
                            }

                            break;

                        case Scene.types.AABB:
                            NCP ncp1 = (pair.obj1.box as NCP);
                            AABB ab = (pair.obj2.box as AABB);
                            for (int i = 0; i < ncp1.elements.Length; i++)
                            {
                                if (IsCirclesIntersect(ncp1.contactCircles[i], pair.obj2.contactCircle))
                                {
                                    if (GJK(ab.vertexs, ncp1.elements[i].vertexs, pair.obj2.position, ncp1.positions[i]))
                                    {
                                        DoCollisionPlgvsPlg(pair.obj2, pair.obj1, ab.vertexs, ncp1.elements[i].vertexs);
                                        break;
                                    }
                                }
                            }

                            break;

                        case Scene.types.NCP:
                            NCP n1 = pair.obj1.box as NCP;
                            NCP n2 = pair.obj2.box as NCP;

                            for (int i = 0; i < n1.contactCircles.Length; i++)
                            {
                                for (int j = 0; j < n2.contactCircles.Length; j++)
                                {
                                    if (IsCirclesIntersect(n1.contactCircles[i], n2.contactCircles[j]))
                                    {
                                        if (GJK(n1.elements[i].vertexs, n2.elements[j].vertexs, n1.positions[i], n2.positions[j]))
                                        {
                                            DoCollisionPlgvsPlg(pair.obj1, pair.obj2, n1.elements[i].vertexs, n2.elements[j].vertexs);
                                            i = n1.contactCircles.Length;
                                            break;
                                        }
                                    }
                                }
                            }
                            break;

                        case Scene.types.OBB:

                            NCP n = pair.obj1.box as NCP;
                            OBB ob = pair.obj2.box as OBB;

                            for (int i = 0; i < n.contactCircles.Length; i++)
                            {
                                if (IsCirclesIntersect(n.contactCircles[i], pair.obj2.contactCircle))
                                {
                                    if (GJK(ob.vertexs, n.elements[i].vertexs, pair.obj2.position, n.positions[i]))
                                    {
                                        DoCollisionPlgvsPlg(pair.obj1, pair.obj2, n.elements[i].vertexs, ob.vertexs);
                                    }
                                }
                            }
                            break;

                        default:
                            NCP ncp2 = (pair.obj1.box as NCP);
                            Circle cr = (pair.obj2.box as Circle);
                            for (int i = 0; i < ncp2.contactCircles.Length; i++)
                            {
                                if (IsCirclesIntersect(ncp2.contactCircles[i], pair.obj2.contactCircle))
                                {
                                    if (GJK(ncp2.elements[i].vertexs, cr, ncp2.positions[i], cr.center))
                                    {
                                        DoCirclevsPolygonCollision(pair.obj2, pair.obj1, ncp2.elements[i].vertexs, ncp2.elements[i].normals);
                                        break;
                                    }
                                }
                            }

                            break;
                    }
                }
                else if (pair.obj1.box.type == Scene.types.OBB)
                {
                    switch (pair.obj2.box.type)
                    {
                        case Scene.types.Polygon:
                            Polygon p4 = (pair.obj2.box as Polygon);
                            OBB ob = (pair.obj1.box as OBB);
                            if (GJK(ob.vertexs, p4.vertexs, pair.obj1.position, pair.obj2.position))
                            {
                                DoCollisionPlgvsPlg(pair.obj1, pair.obj2, ob.vertexs, p4.vertexs);
                            }

                            break;

                        case Scene.types.AABB:
                            OBB p = (pair.obj1.box as OBB);
                            AABB ab = (pair.obj2.box as AABB);
                            if (GJK(ab.vertexs, p.vertexs, pair.obj2.position, pair.obj1.position))
                            {
                                DoCollisionPlgvsPlg(pair.obj2, pair.obj1, ab.vertexs, p.vertexs);
                            }
                            break;

                        case Scene.types.NCP:
                            OBB p3 = (pair.obj1.box as OBB);
                            NCP ncp = (pair.obj2.box as NCP);
                            for (int i = 0; i < ncp.elements.Length; i++)
                            {
                                if (IsCirclesIntersect(pair.obj1.contactCircle, ncp.contactCircles[i]))
                                {
                                    if (GJK(p3.vertexs, ncp.elements[i].vertexs, pair.obj1.position, ncp.positions[i]))
                                    {
                                        DoCollisionPlgvsPlg(pair.obj1, pair.obj2, p3.vertexs, ncp.elements[i].vertexs);
                                        break;
                                    }
                                }
                            }
                            //   DoNCPvsPolygonCollision(pair.obj2, (pair.obj1.box as Polygon).vertexs, pair.obj1);
                            break;

                        case Scene.types.OBB:
                            OBB o1 = (pair.obj1.box as OBB);
                            OBB o2 = (pair.obj2.box as OBB);
                            if (GJK(o1.vertexs, o2.vertexs, pair.obj1.position, pair.obj2.position))
                            {
                                DoCollisionPlgvsPlg(pair.obj1, pair.obj2, o1.vertexs, o2.vertexs);
                            }
                            break;

                        default:
                            OBB p5 = (pair.obj1.box as OBB);
                            Circle cr = (pair.obj2.box as Circle);
                            if (GJK(p5.vertexs, cr, pair.obj1.position, cr.center))
                            {
                                DoCirclevsPolygonCollision(pair.obj2, pair.obj1, p5.vertexs, p5.normals);
                            }

                            break;
                    }
                }
                else if (pair.obj1.box.type == Scene.types.AABB)
                {
                    switch (pair.obj2.box.type)
                    {
                        case Scene.types.Polygon:
                            AABB p4 = (pair.obj2.box as AABB);
                            OBB ob = (pair.obj1.box as OBB);
                            if (GJK(ob.vertexs, p4.vertexs, pair.obj1.position, pair.obj2.position))
                            {
                                DoCollisionPlgvsPlg(pair.obj1, pair.obj2, ob.vertexs, p4.vertexs);
                            }

                            break;

                        case Scene.types.AABB:

                            AABB a1 = pair.obj1.box as AABB;
                            AABB a2 = pair.obj2.box as AABB;

                            if (overlapAABBvsAABB(pair.obj1, pair.obj2))
                            {
                                Vector2 dir = pair.obj1.position - pair.obj2.position;
                                dir.Normalize();
                                if (dir.Y < 0.5 && dir.Y > -0.5)
                                {
                                    float trv = (Math.Abs(a1.max.X - a1.min.X) + Math.Abs(a2.max.X - a2.min.X)) / 2 - Math.Abs(pair.obj1.position.X - pair.obj2.position.X);
                                    if (dir.X < 0)
                                    {
                                        pair.obj1.TranslateD(new Vector2(trv, 0));
                                        pair.obj2.TranslateD(new Vector2(-trv, 0));
                                        DoCollision(pair.obj1, pair.obj2, new Vector2(-1, 0));
                                    }
                                    else
                                    {
                                        pair.obj1.TranslateD(new Vector2(-trv, 0));
                                        pair.obj2.TranslateD(new Vector2(trv, 0));
                                        DoCollision(pair.obj1, pair.obj2, new Vector2(1, 0));
                                    }
                                }
                                else if (dir.X < 0.5 && dir.X > -0.5)
                                {
                                    float trv = (Math.Abs(a1.max.Y - a1.min.Y) + Math.Abs(a2.max.Y - a2.min.Y)) / 2 - Math.Abs(pair.obj1.position.Y - pair.obj2.position.Y);
                                    if (dir.Y > 0)
                                    {
                                        pair.obj1.TranslateD(new Vector2(-trv, 0));
                                        pair.obj2.TranslateD(new Vector2(trv, 0));
                                        DoCollision(pair.obj1, pair.obj2, new Vector2(0, 1));
                                    }
                                    else
                                    {
                                        pair.obj1.TranslateD(new Vector2(trv, 0));
                                        pair.obj2.TranslateD(new Vector2(-trv, 0));
                                        DoCollision(pair.obj1, pair.obj2, new Vector2(0, -1));
                                    }
                                }
                                else
                                {
                                    float trv1 = (Math.Abs(a1.max.Y - a1.min.Y) + Math.Abs(a2.max.Y - a2.min.Y)) / 2 - Math.Abs(pair.obj1.position.Y - pair.obj2.position.Y);
                                    float trv = (Math.Abs(a1.max.X - a1.min.X) + Math.Abs(a2.max.X - a2.min.X)) / 2 - Math.Abs(pair.obj1.position.X - pair.obj2.position.X);
                                    int tr = dir.X < 0 ? -1 : 1;
                                    int tr1 = dir.Y < 0 ? -1 : 1;
                                    pair.obj1.TranslateD(new Vector2(trv * tr, trv1 * tr1));
                                    pair.obj2.TranslateD(new Vector2(-trv * tr, -trv1 * tr1));
                                    DoCollision(pair.obj1, pair.obj2, new Vector2(tr, tr1));
                                }
                            }

                            break;

                        case Scene.types.NCP:
                            AABB p3 = (pair.obj1.box as AABB);
                            NCP ncp = (pair.obj2.box as NCP);
                            for (int i = 0; i < ncp.elements.Length; i++)
                            {
                                if (IsCirclesIntersect(pair.obj1.contactCircle, ncp.contactCircles[i]))
                                {
                                    if (GJK(p3.vertexs, ncp.elements[i].vertexs, pair.obj1.position, ncp.positions[i]))
                                    {
                                        DoCollisionPlgvsPlg(pair.obj1, pair.obj2, p3.vertexs, ncp.elements[i].vertexs);
                                        break;
                                    }
                                }
                            }

                            break;

                        case Scene.types.OBB:
                            AABB o1 = (pair.obj1.box as AABB);
                            OBB o2 = (pair.obj2.box as OBB);
                            if (GJK(o1.vertexs, o2.vertexs, pair.obj1.position, pair.obj2.position))
                            {
                                DoCollisionPlgvsPlg(pair.obj1, pair.obj2, o1.vertexs, o2.vertexs);
                            }
                            break;

                        default:
                            AABB p5 = (pair.obj1.box as AABB);
                            Circle cr = (pair.obj2.box as Circle);
                            if (GJK(p5.vertexs, cr, pair.obj1.position, cr.center))
                            {
                                DoCirclevsPolygonCollision(pair.obj2, pair.obj1, p5.vertexs, p5.normals);
                            }

                            break;
                    }
                }
                else if (pair.obj1.box.type == Scene.types.Circle)
                {
                    switch (pair.obj2.box.type)
                    {
                        case Scene.types.Polygon:
                            Polygon p5 = (pair.obj2.box as Polygon);
                            Circle cr = (pair.obj1.box as Circle);
                            if (GJK(p5.vertexs, cr, pair.obj2.position, cr.center))
                            {
                                DoCirclevsPolygonCollision(pair.obj1, pair.obj2, p5.vertexs, p5.normals);
                            }

                            break;

                        case Scene.types.AABB:

                            AABB ab = (pair.obj2.box as AABB);
                            Circle cr1 = (pair.obj1.box as Circle);
                            if (GJK(ab.vertexs, cr1, pair.obj2.position, cr1.center))
                            {
                                DoCirclevsPolygonCollision(pair.obj1, pair.obj2, ab.vertexs, ab.normals);
                            }

                            break;

                        case Scene.types.NCP:
                            NCP ncp2 = (pair.obj2.box as NCP);
                            Circle cr2 = (pair.obj1.box as Circle);
                            for (int i = 0; i < ncp2.contactCircles.Length; i++)
                            {
                                if (IsCirclesIntersect(ncp2.contactCircles[i], pair.obj1.contactCircle))
                                {
                                    if (GJK(ncp2.elements[i].vertexs, cr2, ncp2.positions[i], cr2.center))
                                    {
                                        DoCirclevsPolygonCollision(pair.obj1, pair.obj2, ncp2.elements[i].vertexs, ncp2.elements[i].normals);
                                        break;
                                    }
                                }
                            }

                            break;

                        case Scene.types.OBB:
                            OBB ob = (pair.obj2.box as OBB);
                            Circle cr3 = (pair.obj1.box as Circle);
                            if (GJK(ob.vertexs, cr3, pair.obj2.position, cr3.center))
                            {
                                DoCirclevsPolygonCollision(pair.obj1, pair.obj2, ob.vertexs, ob.normals);
                            }

                            break;

                        default:

                            DoCollisionCirclevsCircle(pair.obj1, pair.obj2);

                            break;
                    }
                }
            }

            bool IsCirclesIntersect(Circle c1, Circle c2)
            {
                return Geometry.NSqrtLength(c1.center, c2.center) <= Math.Pow(c1.radius + c2.radius, 2);
            }

            void DoCirclevsPolygonCollision(ShapeObject cr, ShapeObject plgo, Vector2[] vrtx, Vector2[] normals)
            {
                Circle cr1 = (cr.box as Circle);

                Collision collision = FindAxisLeastPenetration(vrtx, cr1, normals);
                //Thread.Sleep(2000);
                Vector2 cp = GetContactPoint(collision.MTV, cr1, vrtx);
                collision.MTV.Normalize();
                cr.TranslateD(-Math.Abs(collision.overlap) * collision.MTV);
                DoCollision(cr, plgo, -collision.MTV);
                //ShapeObject sh = ShapeObject.Create(new Point(cp, 4, Color4.Aqua), null, "uhk");
                Vector2 b = plgo.phys.impuls;
                b.Normalize();
                Vector2 t = cp - plgo.position;
                t.Normalize();
                float k = Vector2.Dot(t + b, b);

                Vector2 k1 = cp - cr.position;
                k1.Normalize();
                Vector2 k2 = cp - plgo.position;
                k2.Normalize();
                //Console.WriteLine("yy   " + (Vector2.Dot(MathS.GetNormal(k1), pair.obj1.phys.impuls)) * (k < 0.1f ? 1 : -1));
                plgo.phys.angularVelocity += (2 / plgo.phys.inertia) * (Vector2.Dot(Geometry.GetNormal(k2), plgo.phys.impuls)) * (k < 0.1f ? 1 : -1);
                cr.phys.angularVelocity += (2 / cr.phys.inertia) * (Vector2.Dot(Geometry.GetNormal(k1), cr.phys.impuls)) * (k < 0.1f ? 1 : -1);
            }

            List<Pair> GenerateContactPairs()
            {
                List<ShapeObject> shs = Project.container.scene.shapeObjects.ToList();
                List<Pair> contactshs = new List<Pair>();
                for (int i = 0; i < shs.Count - 1; i++)
                {
                    ShapeObject sh = shs[i];
                    shs.RemoveAt(i);
                    for (int j = 0; j < shs.Count; j++)
                    {
                        if (sh.contactCircle != null && shs[j].contactCircle != null)
                        {
                            if (Geometry.NSqrtLength(sh.contactCircle.center, sh.contactCircle.center) <= Math.Pow(sh.contactCircle.radius + sh.contactCircle.radius, 2))
                            {
                                contactshs.Add(new Pair(sh, shs[j]));
                            }
                        }
                    }
                }
                return contactshs;
            }
        }
    }
}