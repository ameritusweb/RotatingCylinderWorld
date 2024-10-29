using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace RotatingCylinderWorld
{
    public class ControlColumn
    {
        public float[] Values { get; private set; }
        public Vector3[] BezierControlPoints { get; private set; }
        public ControlColumn(float[] initialValues, Vector3[] bezierControlPoints)
        {
            this.Values = initialValues;
            this.BezierControlPoints = bezierControlPoints;
        }
        /// <summary>
        /// Get the value from the control column at a specific index (e.g., time step).
        /// </summary>
        public float GetValue(int index)
        {
            if (index >= 0 && index < Values.Length)
            {
                return Values[index];
            }
            throw new IndexOutOfRangeException("Invalid index for ControlColumn.");
        }
        /// <summary>
        /// Modulate the initial acceleration using a cubic Bézier curve over the time step.
        /// </summary>
        public float GetBezierModulatedAcceleration(int index, float t)
        {
            Vector3 p0 = new Vector3(Values[index], 0, 0); // The starting point (initial acceleration)
            Vector3 p1 = BezierControlPoints[0];
            Vector3 p2 = BezierControlPoints[1];
            Vector3 p3 = BezierControlPoints[2];
            // Use the cubic Bézier curve to interpolate the acceleration over the time step
            Vector3 modulatedPoint = Bezier.CubicBezier(p0, p1, p2, p3, t);
            return modulatedPoint.X; // Return the modulated acceleration
        }
        /// <summary>
        /// Update the control values. This could be extended for learning or external input.
        /// </summary>
        public void UpdateValues(float[] newValues)
        {
            if (newValues.Length == Values.Length)
            {
                this.Values = newValues;
            }
            else
            {
                throw new ArgumentException("New values must match the length of the original ControlColumn.");
            }
        }
    }
}
