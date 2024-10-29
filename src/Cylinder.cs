using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RotatingCylinderWorld
{
    public class Cylinder
    {
        private float angle;
        public Cylinder(float angle) { 
            this.angle = angle;
        }

        public void UpdateAngle(float diff)
        {
            this.angle += diff;
            if (this.angle > (2 * MathF.PI))
            {
                this.angle = this.angle % (2 * MathF.PI);
            }
        }

        public float CurrentAngle 
        { 
            get
            {
                return this.angle;
            }
        }
    }
}
