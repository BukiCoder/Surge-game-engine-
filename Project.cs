using OpenTK;
using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal static class Project
    {
        public static Container container;

        public static void LoadContainer(Scene scene, string path)
        {
            container = new Container(scene, path);
            GL.Scale(new Vector3(0.75f, 1, 1));
            game.RenderFrame += (sender, e) =>
            {
                GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
                GL.MatrixMode(MatrixMode.Projection);
                var sha = Project.container.scene.shapeObjects;
                Project.container.scene.NRIS = sha.ToList();
                foreach (var item in Project.container.scene.shapeObjects.ToList())
                {
                    item.RefreshImpuls();
                    ShapeDrawing.RedrawShape(item);
                }

                game.SwapBuffers();
            };

            game.Run(0.1f);
        }

        public static GameWindow game = new GameWindow();
    }
}