using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;

namespace Surge
{
    internal class Polygon : Shape
    {
        public Vector2[] vertexs;
        public Vector2[] edges;
        public Vector2[] normals;
        public Edge[] edges1;

        private void GetEdges()
        {
            edges = new Vector2[vertexs.Length];
            edges1 = new Edge[vertexs.Length];
            int ln = vertexs.Length - 1;
            for (int i = 0; i < ln; i++)
            {
                Vector2 e = vertexs[i] - vertexs[i + 1];
                e.Normalize();
                normals[i] = Geometry.GetNormal(e);
                edges[i] = e;
                edges1[i] = new Edge(vertexs[i], vertexs[i + 1]);
            }
            Vector2 e1 = vertexs[ln] - vertexs[0];
            e1.Normalize();
            edges[ln] = e1;
            edges1[ln] = new Edge(vertexs[ln], vertexs[0]);
        }

        public Polygon(Vector2[] vertexs, Scene.types type = Scene.types.Polygon) : base(type = Scene.types.Polygon)
        {
            this.vertexs = vertexs;
            GetEdges();
        }
    }
}