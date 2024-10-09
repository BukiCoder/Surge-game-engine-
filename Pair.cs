using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal class Pair
    {
        public ShapeObject obj1;
        public ShapeObject obj2;

        public Pair(ShapeObject obj1, ShapeObject obj2)
        {
            this.obj1 = obj1;
            this.obj2 = obj2;
        }
    }
}