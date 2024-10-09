using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal class Point : SimpleShape
    {
        public Vector2 point { get; set; }
        public float size { get; set; }
        public Color4 color { get; set; }

        public Point(Vector2 point, float size, Color4 color, Scene.types type = Scene.types.Point) : base(type = Scene.types.Point)
        {
            this.point = point;
            this.size = size;
            this.color = color;
        }
    }
}