using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Surge
{
    internal class Container
    {
        public Scene scene;        //! активная сцена
        public string folder;      //! папка сцены/проекта
        private string scenePath;  //! путь сцены

        public Container(Scene scene, string folder)
        {
            this.scene = scene;
            this.folder = folder;
            scenePath = folder + @"\scene.json";
        }

        public void Save() //! сохранение сцены
        {
            string json = JsonSerializer.Serialize<Scene>(scene);
            File.WriteAllText(scenePath, json);
        }

        public void Load() //! загрузкаа сцены
        {
            scene = JsonSerializer.Deserialize<Scene>(File.ReadAllText(scenePath));
            scene.textures = new List<int>();                                   //! для OpenGL нам необходимо загрузить текстуры
            foreach (var obj in scene.shapeObjects)
            {
                scene.textures.Add(ShapeObject.LoadTexture(obj.texture.path));
                obj.texture.index = scene.textures.Last();
            }
        }
    }
}