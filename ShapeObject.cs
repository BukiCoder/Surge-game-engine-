using OpenTK;

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using System.Drawing.Imaging;

namespace Surge
{
    internal class ShapeObject
    {
        public Texture texture; //! текстура

        public Shape box; //! коллайдер

        public Physic phys; //! физический компонент

        public Circle contactCircle;

        public bool enabled = true;
        public string name { get; set; }
        public Vector2 position { get; set; }

        public Vector2 scale { get; set; }

        public Vector2 rotateVector;

        public float rotation
        {
            get; private set;
        }

        public static void Delete(ShapeObject obj)
        {
            Project.container.scene.shapeObjects.Remove(obj);
        }

        public void SetScale(Vector2 scale)
        {
            if (enabled)
            {
                switch (box.type)
                {
                    case Scene.types.Polygon:
                        ScaleRO();
                        break;

                    case Scene.types.AABB:
                        SetScaleК(scale);
                        break;

                    case Scene.types.NCP:
                        ScaleRO();
                        break;

                    case Scene.types.OBB:
                        ScaleRO();
                        break;
                }
            }
        }

        private void ScaleRO()
        {
            Rotate(-rotation);
            SetScaleК(scale);
            Rotate(rotation);
        }

        private void SetScaleК(Vector2 scale)
        {
            if (enabled)
            {
                float rsX = scale.X / this.scale.X;
                float rsY = scale.Y / this.scale.Y;
                switch (box.type)
                {
                    case Scene.types.Polygon:

                        Polygon plg = box as Polygon;
                        for (int i = 0; i < plg.vertexs.Length; i++)
                        {
                            plg.vertexs[i] = new Vector2((plg.vertexs[i].X - position.X) * rsX + position.X, (plg.vertexs[i].Y - position.Y) * rsY + position.Y);
                        }
                        break;

                    case Scene.types.AABB:
                        AABB ab = box as AABB;
                        Vector2 rvp = ab.min - position;
                        Vector2 rvp1 = ab.max - position;
                        ab.min = new Vector2(rvp.X * rsX + position.X, rvp.Y * rsY + position.Y);
                        ab.max = new Vector2(rvp1.X * rsX + position.X, rvp1.Y * rsY + position.Y);
                        break;

                    case Scene.types.NCP:
                        NCP ncp = (box as NCP);
                        foreach (var p in ncp.elements)
                        {
                            for (int i = 0; i < p.vertexs.Length; i++)
                            {
                                p.vertexs[i] = new Vector2((p.vertexs[i].X - position.X) * rsX + position.X, (p.vertexs[i].Y - position.Y) * rsY + position.Y);
                            }
                        }
                        break;

                    case Scene.types.OBB:
                        SSKOBB(box as OBB, rsX, rsY);
                        break;
                }
                SSKOBB(texture.border, rsX, rsY);
                this.scale = scale;
            }
        }

        private void SSKOBB(OBB ob, float rsX, float rsY)
        {
            for (int i = 0; i < 4; i++)
            {
                ob.vertexs[i] = new Vector2((ob.vertexs[i].X - position.X) * rsX + position.X, (ob.vertexs[i].Y - position.Y) * rsY + position.Y);
            }
        }

        public static Circle CreateContactCircle(ShapeObject sh)
        {
            switch (sh.box.type)
            {
                case Scene.types.Polygon:
                    Polygon p = sh.box as Polygon;

                    double ln = float.NegativeInfinity;
                    foreach (Vector2 v in p.vertexs)
                    {
                        double lnt = (Math.Pow(v.X - sh.position.X, 2) + Math.Pow(v.Y - sh.position.Y, 2));
                        if (lnt > ln)
                        {
                            ln = lnt;
                        }
                    }
                    return new Circle((float)Math.Sqrt(ln), sh.position);

                case Scene.types.AABB:
                    Vector2 v1 = (sh.box as AABB).max;
                    return new Circle((float)Math.Sqrt((Math.Pow(v1.X - sh.position.X, 2) + Math.Pow(v1.Y - sh.position.Y, 2))), sh.position);

                case Scene.types.Circle:
                    return (sh.box as Circle);

                case Scene.types.NCP:
                    List<Vector2> vrtxs = new List<Vector2>();
                    NCP ncp = sh.box as NCP;
                    foreach (var item in ncp.elements)
                    {
                        vrtxs.AddRange(item.vertexs);
                    }
                    return ContactCirclePolygon(vrtxs, sh.position);

                case Scene.types.OBB:
                    Vector2 v2 = (sh.box as OBB).vertexs[0];
                    return new Circle((float)Math.Sqrt((Math.Pow(v2.X - sh.position.X, 2) + Math.Pow(v2.Y - sh.position.Y, 2))), sh.position);
            }
            return null;
        }

        public static int LoadTexture(string file)
        {
            GL.Enable(EnableCap.Texture2D);
            Bitmap bitmap = new Bitmap(Image.FromFile(file));

            int tex;
            GL.Hint(HintTarget.PerspectiveCorrectionHint, HintMode.Nicest);

            GL.GenTextures(1, out tex);
            GL.BindTexture(TextureTarget.Texture2D, tex);

            BitmapData data = bitmap.LockBits(new System.Drawing.Rectangle(0, 0, bitmap.Width, bitmap.Height),
            ImageLockMode.ReadOnly, bitmap.PixelFormat);

            GL.TexImage2D(TextureTarget.Texture2D, 0, PixelInternalFormat.Rgba, data.Width, data.Height, 0,
            OpenTK.Graphics.OpenGL.PixelFormat.Bgra, PixelType.UnsignedByte, data.Scan0);
            bitmap.UnlockBits(data);

            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)TextureMinFilter.Linear);
            GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)TextureMagFilter.Linear);
            //  GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapS, (int)TextureWrapMode.Repeat);
            //  GL.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureWrapT, (int)TextureWrapMode.Repeat);

            return tex;
        }

        public static ShapeObject Create(Shape sh, Physic phys, Texture texture, string name)
        {
            if (sh.type == Scene.types.Polygon && !Geometry.IsConvex(sh as Polygon))
            {
                sh = new NCP(sh as Polygon);
            }

            ShapeObject shape = new ShapeObject();
            shape.scale = new Vector2(1, 1);
            shape.name = name;
            shape.box = sh;
            shape.phys = phys;
            shape.texture = texture;
            Project.container.scene.textures.Add(LoadTexture(texture.path));
            texture.index = Project.container.scene.textures.Last();
            Project.container.scene.shapeObjects.Add(shape);
            shape.position = GeneratePosition(shape);

            shape.contactCircle = CreateContactCircle(shape);

            /*    switch (sh.type)
                {
                    case "plg":
                        ShapeCreating.DrawPolygon(shape.shape as Polygon);
                        break;

                    case "ab":
                        ShapeCreating.DrawAABB(shape.shape as AABB);
                        break;

                    case "ln":
                        ShapeCreating.DrawLine(shape.shape as Line);
                        break;

                    case "pn":
                        ShapeCreating.DrawPoint(shape.shape as Point);
                        break;

                    case "cr":
                        ShapeCreating.DrawCircle(shape.shape as DrawCircle);
                        break;
                }*/

            return shape;
        }

        private static Vector2 CenterQ(Vector2 min, Vector2 max)
        {
            return new Vector2((max.X + min.X) / 2, (max.Y + min.Y) / 2);
        }

        public static Vector2 GeneratePosition(ShapeObject sh)
        {
            switch (sh.box.type)
            {
                case Scene.types.Polygon:

                    return PolygonPos((sh.box as Polygon).vertexs);

                case Scene.types.AABB:
                    AABB ab = sh.box as AABB;
                    return CenterQ(ab.min, ab.max);

                /*   case MainClass.types.Line:
                       Line ln = sh.box as Line;
                       return CenterQ(ln.start, ln.end);

                   case MainClass.types.Point:
                       return (sh.box as Point).point;*/

                case Scene.types.Circle:
                    return (sh.box as Circle).center;

                case Scene.types.NCP:
                    NCP ncp = sh.box as NCP;
                    Vector2[] positions = new Vector2[ncp.elements.Length];

                    for (int i = 1; i < ncp.elements.Length; i++)
                    {
                        positions[i] = PolygonPos(ncp.elements[i].vertexs);
                        ncp.contactCircles[i] = ContactCirclePolygon(ncp.elements[i].vertexs, positions[i]);
                    }
                    ncp.positions = positions;
                    Vector2 position = PolygonPos(positions);

                    return position;

                case Scene.types.OBB:
                    OBB ob = sh.box as OBB;
                    return CenterQ(ob.vertexs[0], ob.vertexs[2]);
            }
            return new Vector2();
        }

        private static Circle ContactCirclePolygon(IEnumerable<Vector2> vrtxs, Vector2 pos)
        {
            double ln = float.NegativeInfinity;
            foreach (Vector2 v in vrtxs)
            {
                double lnt = (Math.Pow(v.X - pos.X, 2) + Math.Pow(v.Y - pos.Y, 2));
                if (lnt > ln)
                {
                    ln = lnt;
                }
            }
            return new Circle((float)Math.Sqrt(ln), pos);
        }

        private static Vector2 PolygonPos(Vector2[] pns)
        {
            float X = 0;
            float Y = 0;
            for (int i = 0; i < pns.Length; i++)
            {
                X += pns[i].X;
                Y += pns[i].Y;
            }
            X /= pns.Length;
            Y /= pns.Length;
            return new Vector2(X, Y);
        }

        public void Rotate(float angle)
        {
            if (enabled)
            {
                if (angle != 0)
                {
                    if (Scene.types.Polygon == box.type)
                    {
                        rotateVector = (this.box as Polygon).vertexs[0];
                    }

                    Rotate(angle, position);
                    if (Scene.types.Polygon == box.type)
                    {
                        rotateVector -= (this.box as Polygon).vertexs[0];
                    }
                    rotation += angle;
                }
            }
        }

        public void SetRotation(float angle)
        {
            if (enabled)
            {
                Rotate(angle - rotation, position);
                position = GeneratePosition(this);
                rotation = angle;
            }
        }

        public void SetRotation(float angle, Vector2 point)
        {
            Rotate(angle - rotation, point);
            position = GeneratePosition(this);
        }

        private Vector2[] RotateVertex(Vector2[] vrtx, float angle)
        {
            angle /= 57.295779f;
            Vector2 X = Geometry.RotateVector(new Vector2(1, 0), angle);
            Vector2 Y = Geometry.GetNormal(X);
            for (int i = 0; i < vrtx.Length; i++)
            {
                vrtx[i] = (X * vrtx[i].X) + (Y * vrtx[i].Y);
            }
            return vrtx;
        }

        private Vector2[] RotateVertex(Vector2[] vrtx, Vector2 X, Vector2 Y)
        {
            for (int i = 0; i < vrtx.Length; i++)
            {
                vrtx[i] = (X * vrtx[i].X) + (Y * vrtx[i].Y);
            }
            return vrtx;
        }

        public void Rotate(float angle, Vector2 point)
        {
            if (enabled)
            {
                rotation += angle;
                switch (box.type)
                {
                    case Scene.types.Polygon:
                        Polygon plg = (box as Polygon);
                        plg.vertexs = RotateVertexs(angle, point, plg.vertexs);
                        for (int i = 0; i < plg.normals.Length; i++)
                        {
                            plg.normals[i] = Geometry.RotateVector(plg.normals[i], angle);
                        }

                        break;

                    case Scene.types.OBB:
                        OBB ob = (box as OBB);
                        ob.vertexs = RotateVertexs(angle, point, ob.vertexs);
                        for (int i = 0; i < ob.normals.Length; i++)
                        {
                            ob.normals[i] = Geometry.RotateVector(ob.normals[i], angle);
                        }
                        break;

                    case Scene.types.NCP:
                        NCP n = (box as NCP);
                        angle /= 57.295779f;
                        Vector2 X = Geometry.RotateVector(new Vector2(1, 0), angle);
                        Vector2 Y = Geometry.GetNormal(X);
                        for (int i = 0; i < n.elements.Length; i++)
                        {
                            n.elements[i].vertexs = RotateVertexs(X, Y, point, n.elements[i].vertexs);
                            for (int j = 0; j < n.elements[i].normals.Length; j++)
                            {
                                n.elements[i].normals[j] = Geometry.RotateVector(n.elements[i].normals[j], angle);
                            }
                        }

                        break;
                }
                OBB ob1 = (texture.border as OBB);
                ob1.vertexs = RotateVertexs(angle, point, ob1.vertexs);
            }
        }

        private Vector2[] RotateVertexs(float angle, Vector2 point, Vector2[] vertexs)
        {
            for (int i = 0; i < vertexs.Length; i++)
            {
                vertexs[i] -= (point);
            }

            vertexs = RotateVertex(vertexs, angle);

            for (int i = 0; i < vertexs.Length; i++)
            {
                vertexs[i] += point;
            }

            return vertexs;
        }

        private Vector2[] RotateVertexs(Vector2 X, Vector2 Y, Vector2 point, Vector2[] vertexs)
        {
            for (int i = 0; i < vertexs.Length; i++)
            {
                vertexs[i] -= (point);
            }

            vertexs = RotateVertex(vertexs, X, Y);

            for (int i = 0; i < vertexs.Length; i++)
            {
                vertexs[i] += point;
            }

            return vertexs;
        }

        public void Translate(Vector2 pos)
        {
            if (enabled)
            {
                Vector2 distance = pos - position;
                TranslateD(distance);
                position = pos;
            }
        }

        public void TranslateD(Vector2 distance)
        {
            if (enabled)
            {
                position += distance;
                switch (box.type)
                {
                    case Scene.types.Polygon:
                        Polygon plg = box as Polygon;
                        for (int i = 0; i < plg.vertexs.Length; i++)
                        {
                            plg.vertexs[i] += distance;
                        }

                        break;

                    case Scene.types.OBB:
                        TDOBB(box as OBB, distance);

                        break;

                    case Scene.types.AABB:
                        AABB ab = box as AABB;
                        for (int i = 0; i < 4; i++)
                        {
                            ab.vertexs[i] += distance;
                        }
                        ab.min = ab.vertexs[0];
                        ab.max += ab.vertexs[2];
                        box = ab;
                        break;

                    case Scene.types.Circle:
                        (box as Circle).center += distance;
                        break;

                    case Scene.types.NCP:
                        NCP n = (box as NCP);
                        for (int i = 0; i < n.elements.Length; i++)
                        {
                            for (int j = 0; j < n.elements[i].vertexs.Length; j++)
                            {
                                n.elements[i].vertexs[j] += distance;
                            }
                        }
                        break;
                }
                TDOBB(texture.border, distance);
            }
        }

        private void TDOBB(OBB ob, Vector2 distance)
        {
            for (int i = 0; i < ob.vertexs.Length; i++)
            {
                ob.vertexs[i] += distance;
            }
        }

        public void AddImpulse(Vector2 impulse)
        {
            if (enabled && phys != null)
            {
                phys.AddImpulse(impulse);
            }
        }

        public void RefreshImpuls()
        {
            if (enabled && phys != null)
            {
                phys.RefreshImpuls(this);
            }
        }
    }
}