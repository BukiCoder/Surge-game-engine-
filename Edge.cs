using OpenTK;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal class Edge //! грань(отрезок)
    {
        public Vector2 p1; //! первая точка
        public Vector2 p2; //! вторая точка

        public Edge(Vector2 p1, Vector2 p2)
        {
            this.p1 = p1;
            this.p2 = p2;
        }
    }
}