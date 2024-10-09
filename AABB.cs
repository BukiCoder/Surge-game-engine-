using OpenTK;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using OpenTK.Graphics.OpenGL;
using System.Drawing;
using System.Diagnostics;
using OpenTK.Graphics;

namespace Surge
{
    internal class AABB : Shape //! Некрутящийся прямоугольник
    {
        public Vector2 max; //! первая точка
        public Vector2 min; //! вторая точка
        //! max/min - *--------------------
        //!           |                   |
        //!           |                   |
        //!           |                   |
        //!           |                   |
        //!           |                   |
        //!           |___________________* - min/max

        public Vector2[] vertexs; //! список всех вершин, для удобства работы алгоритмов
        public Vector2[] normals = new Vector2[] { new Vector2(0, -1), new Vector2(1, 0), new Vector2(0, 1), new Vector2(-1, 0) }; //! перпендикуляры к сторонам для работы алгоритмов

        private Vector2[] GetVertexs(Vector2 min, Vector2 max) //! получение вершин из макс и мин вершины
        {
            return new Vector2[] { min, new Vector2(max.X, min.Y), max, new Vector2(min.X, max.Y) };
        }

        public AABB(Vector2 max, Vector2 min, Scene.types type = Scene.types.AABB) : base(type = Scene.types.AABB) //! конструктор
        {
            this.max = max;
            this.min = min;
            vertexs = GetVertexs(min, max);
        }
    }
}