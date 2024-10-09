using OpenTK;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal class OBB : Shape //! вращающийся прямоугольник
    {
        public Vector2[] vertexs = new Vector2[4];
        public Vector2[] normals = new Vector2[4];

        public OBB(Vector2[] vertexs, Scene.types type = Scene.types.OBB) : base(type = Scene.types.OBB)
        {
            this.vertexs = vertexs;
            for (int i = 0; i < 3; i++)
            {
                normals[i] = vertexs[i] - vertexs[i + 1];
                normals[i].Normalize();
            }
            normals[3] = vertexs[3] - vertexs[0];
            normals[3].Normalize();
        }
    }
}