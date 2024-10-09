using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;
using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;

namespace Surge
{
    internal class ShapeDrawing
    {
        public static void DrawLine(Line line)
        {
            GL.Begin(PrimitiveType.Lines);
            if (line.solid)
            {
                Color4 color = line.colors[0];
                GL.Color4(color);
                GL.Vertex2(line.start);
                GL.Vertex2(line.end);
            }
            else
            {
                GL.Color4(line.colors[0]);
                GL.Vertex2(line.start);
                GL.Color4(line.colors[1]);
                GL.Vertex2(line.end);
            }

            GL.End();
            GL.LineWidth(line.width);
        }

        public static void DrawPoint(Point point)
        {
            GL.PointSize(point.size);
            GL.Begin(PrimitiveType.Points);
            GL.Color4(point.color);
            GL.Vertex2(point.point);
            GL.End();
        }

        public static void RedrawShape(ShapeObject obj)
        {
            if (obj.enabled)
            {
                //   GL.Enable(EnableCap.Texture2D);
                // GL.BindTexture(EnableCap.Texture2D, texture.path);
                //    GL.TexImage2D(TextureTarget.Texture2D, 8, PixelInternalFormat.Rgba, 9, 9, 0, PixelFormat.Rgba, PixelType.Bitmap, )
                //  GL.Color3(0, 0, 0);
                GL.Begin(PrimitiveType.Quads);
                //      GL.Enable(EnableCap.Texture2D);
                //    GL.BindTexture(TextureTarget.Texture2D, obj.texture.index);
                GL.TexCoord2(new Vector2(0, 0));
                GL.Vertex2(obj.texture.border.vertexs[0]);
                GL.TexCoord2(new Vector2(0, 1));
                GL.Vertex2(obj.texture.border.vertexs[1]);
                GL.TexCoord2(new Vector2(1, 1));
                GL.Vertex2(obj.texture.border.vertexs[2]);
                GL.TexCoord2(new Vector2(1, 0));
                GL.Vertex2(obj.texture.border.vertexs[3]);
                GL.End();
            }
        }
    }
}