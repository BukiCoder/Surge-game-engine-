using OpenTK;
using OpenTK.Graphics;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal class Texture
    {
        public string path;
        public int index = 0;
        public OBB border;

        public Texture(string path, OBB border)
        {
            this.path = path;
            this.border = border;
        }
    }
}