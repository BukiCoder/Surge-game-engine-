using OpenTK;
using OpenTK.Graphics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal class Circle : Shape //! круг
    {
        public float radius;   //! радиус
        public Vector2 center; //! центр круга

        public Circle(float radius, Vector2 center, Scene.types type = Scene.types.Circle) : base(type = Scene.types.Circle)
        {
            this.radius = radius;
            this.center = center;
        }
    }
}