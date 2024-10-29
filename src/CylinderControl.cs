using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RotatingCylinderWorld
{
    public class CylinderControl
    {
        private Cylinder _cylinder;
        private ControlColumn _velocityColumn;
        private ControlColumn _accelerationColumn;
        private ControlColumn _timeDeltaColumn;
        private float radius;
        public CylinderControl(Cylinder cylinder, ControlColumn velocityColumn, ControlColumn accelerationColumn, ControlColumn timeDeltaColumn, float radius)
        {
            this._cylinder = cylinder;
            this._velocityColumn = velocityColumn;
            this._accelerationColumn = accelerationColumn;
            this._timeDeltaColumn = timeDeltaColumn;
            this.radius = radius;
        }
        /// <summary>
        /// Update the cylinder's angle based on the control columns for velocity, modulated acceleration, and time.
        /// The acceleration is modulated by a Bézier curve during the time step.
        /// </summary>
        public void UpdateMotion(int timeIndex)
        {
            float velocity = _velocityColumn.GetValue(timeIndex);
            float deltaTime = _timeDeltaColumn.GetValue(timeIndex);
            // Loop over small increments of time to simulate continuous acceleration over the time step
            int numSubSteps = 10; // Use finer sub-steps to compute modulated acceleration
            float timeIncrement = deltaTime / numSubSteps;
            for (int subStep = 0; subStep < numSubSteps; subStep++)
            {
                float t = (float)subStep / numSubSteps; // Normalized time between 0 and 1
                                                        // Get the modulated acceleration using Bézier curve for the current sub-step
                float modulatedAcceleration = _accelerationColumn.GetBezierModulatedAcceleration(timeIndex, t);
                // Update the velocity incrementally using the modulated acceleration
                velocity += modulatedAcceleration * timeIncrement;
                // Update the cylinder's angle based on the current velocity and time increment
                float angleDelta = velocity * timeIncrement;
                _cylinder.UpdateAngle(angleDelta);
            }
        }
        /// <summary>
        /// Get the current angle of the cylinder.
        /// </summary>
        public float GetCylinderAngle()
        {
            return _cylinder.CurrentAngle;
        }
    }
}
