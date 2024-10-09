using System;
using System.Drawing;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;
using System.Linq;
using System.Threading;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

namespace Surge
{
    internal class Script
    {
        public void StartScript(Scene.Update update)
        {
            Project.game.RenderFrame += (sender, e) => { update(); };
        }
    }
}