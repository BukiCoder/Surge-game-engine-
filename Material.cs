using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Surge
{
    internal class Material
    {
        public float staticFriction { get; set; } //! трение пококя

        public float dynamicFriction { get; set; } //! динамическое трение

        public float restution { get; set; } //! упругость

        public Material(float staticFriction, float dynamicFriction, float restution)
        {
            this.staticFriction = staticFriction;
            this.dynamicFriction = dynamicFriction;
            this.restution = restution;
        }
    }
}