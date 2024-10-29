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
        /// The acceleration is modulated by a Bézier curve during the time step, and motion is adjusted for radius.
        /// </summary>
        public void UpdateMotion(int timeIndex)
        {
            float tangentialVelocity = _velocityColumn.GetValue(timeIndex);
            float deltaTime = _timeDeltaColumn.GetValue(timeIndex);

            // Convert tangential velocity to angular velocity (ω = v/r)
            float angularVelocity = tangentialVelocity / radius;

            int numSubSteps = 10;
            float timeIncrement = deltaTime / numSubSteps;

            for (int subStep = 0; subStep < numSubSteps; subStep++)
            {
                float t = (float)subStep / numSubSteps;

                // Get modulated tangential acceleration
                float modulatedTangentialAcceleration = _accelerationColumn.GetBezierModulatedAcceleration(timeIndex, t);

                // Convert tangential acceleration to angular acceleration (α = a/r)
                float angularAcceleration = modulatedTangentialAcceleration / radius;

                // Update angular velocity using angular acceleration
                angularVelocity += angularAcceleration * timeIncrement;

                // Update angle using angular velocity
                float angleDelta = angularVelocity * timeIncrement;
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
