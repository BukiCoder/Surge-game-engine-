using OpenTK;

namespace Surge
{
    internal class Collision //! характеризует столкновение объектоы
    {
        public float overlap;  //! глубина проникновения по вектору
        public Vector2 MTV;    //! вектор проникновения(по нему в разные стороны будут отталкиваться тела)

        public Collision(float overlap, Vector2 smallest)
        {
            this.overlap = overlap;
            this.MTV = smallest;
        }
    }
}