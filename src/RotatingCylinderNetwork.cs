using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RotatingCylinderWorld
{
    public class RotatingCylinderNetwork

    {

        private List<CylinderControl> _cylinderControls;

        private AngularBucket _angularBucket;



        public RotatingCylinderNetwork(int bucketCount)

        {

            _cylinderControls = new List<CylinderControl>();

            _angularBucket = new AngularBucket(bucketCount);

        }



        /// <summary>

        /// Add a CylinderControl object to the network.

        /// </summary>

        public void AddCylinderControl(CylinderControl cylinderControl)

        {

            _cylinderControls.Add(cylinderControl);

        }



        public void Reset()

        {

            _angularBucket.Reset();

        }



        /// <summary>

        /// Perform a forward pass: Update motion, accumulate angular values into buckets, and consider pairwise interactions.

        /// </summary>

        public void Forward(int timeIndex)

        {

            Dictionary<int, float> currentAngles = new Dictionary<int, float>();



            // Step 1: Update motions and store angle-bucket relations

            for (int i = 0; i < _cylinderControls.Count; i++)

            {

                var cylinderControl = _cylinderControls[i];

                cylinderControl.UpdateMotion(timeIndex);

                float angle = cylinderControl.GetCylinderAngle();



                // Store the angle for interaction calculation

                currentAngles[i] = angle;



                // Store this cylinder's relations to buckets

                _angularBucket.Accumulate(angle, i);

            }



            // Step 2: Update buckets using stored relations and current angles

            _angularBucket.UpdateBuckets(currentAngles);

        }



        /// <summary>

        /// Classify based on the angular bucket with the highest accumulation.

        /// </summary>

        public int Classify()
        {

            return _angularBucket.GetMaxBucket();

        }



        /// <summary>

        /// Optional: Get the current bucket values for visualization or analysis.

        /// </summary>

        public float[] GetBucketValues()

        {

            return _angularBucket.GetBucketValues();

        }

    }
}
