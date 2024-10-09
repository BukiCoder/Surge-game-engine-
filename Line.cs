using OpenTK;
using OpenTK.Graphics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal class Line : SimpleShape
    {
        public Vector2 start;
        public Vector2 end;
        public bool solid;
        public Color4[] colors = new Color4[2];
        public float width;

        public Line(Vector2 start, Vector2 end, Color4[] colors, float width, Scene.types type = Scene.types.Line) : base(type = Scene.types.Line)
        {
            this.start = start;
            this.end = end;
            this.colors = colors;
            this.width = width;
            solid = false;
        }

        public Line(Vector2 start, Vector2 end, Color4 color, float width, Scene.types type = Scene.types.Line) : base(type = Scene.types.Line)
        {
            this.start = start;
            this.end = end;
            this.colors[0] = color;
            this.width = width;
            solid = true;
        }
    }
}