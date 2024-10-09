using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal class Shape
    {
        public Scene.types type { get; set; }

        public Shape(Scene.types type)
        {
            this.type = type;
        }
    }
}