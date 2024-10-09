using System.Collections.Generic;
using System.Text.Json;
using System.IO;
using System.Windows.Forms;
using System.Threading;

namespace Surge
{
    internal class Scene
    {
        public List<ShapeObject> NRIS = new List<ShapeObject>();        //! not refresh impulse shapes
        public List<ShapeObject> shapeObjects = new List<ShapeObject>();//! все сложные обьекты
        public List<int> textures = new List<int>();//! текстуры

        public delegate void Update();

        public enum types//! типы обьектоы
        {
            AABB,
            Circle,
            Polygon,
            NCP,
            Line,
            Point,
            OBB
        }
    }
}