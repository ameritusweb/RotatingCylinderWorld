using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RotatingCylinderWorld
{
    public class AngularBucket
    {
        private readonly int _bucketCount;
        private readonly float[] _bucketValues;
        private readonly Dictionary<int, float[]> _angleRelations; // Maps cylinder ID to its bucket relations

        public AngularBucket(int bucketCount)
        {
            _bucketCount = bucketCount;
            _bucketValues = new float[bucketCount];
            _angleRelations = new Dictionary<int, float[]>();
        }

        /// <summary>
        /// Store how this cylinder's angle relates to each bucket center
        /// </summary>
        public void Accumulate(float angle, int cylinderId)
        {
            angle = angle % (2 * MathF.PI);
            if (angle < 0) angle += 2 * MathF.PI;

            float[] relations = new float[_bucketCount];
            for (int i = 0; i < _bucketCount; i++)
            {
                float bucketCenterAngle = (2 * MathF.PI * i) / _bucketCount;

                float diffToBucket = Math.Abs(angle - bucketCenterAngle);

                if (diffToBucket > MathF.PI) diffToBucket = 2 * MathF.PI - diffToBucket;

                relations[i] = MathF.Exp(-diffToBucket * diffToBucket);
            }

            _angleRelations[cylinderId] = relations;
        }

        /// <summary>
        /// Calculate interaction weight between two cylinders
        /// </summary>
        private float CalculateInteraction(float angle1, float angle2)
        {
            float angleDiff = Math.Abs(angle1 - angle2);
            if (angleDiff > MathF.PI) angleDiff = 2 * MathF.PI - angleDiff;

            return MathF.Exp(-angleDiff);
        }

        /// <summary>
        /// Weight the stored angle-bucket relationships by cylinder interactions and update bucket values
        /// </summary>
        public void UpdateBuckets(Dictionary<int, float> currentAngles)
        {
            // For each pair of cylinders
            var cylinderIds = currentAngles.Keys.ToList();
            for (int i = 0; i < cylinderIds.Count; i++)
            {

                for (int j = i + 1; j < cylinderIds.Count; j++)
                {
                    int cyl1 = cylinderIds[i];
                    int cyl2 = cylinderIds[j];

                    // Calculate interaction weight between these cylinders
                    float interaction = CalculateInteraction(currentAngles[cyl1], currentAngles[cyl2]);

                    // For each bucket
                    for (int b = 0; b < _bucketCount; b++)
                    {
                        // Combine their relationships to this bucket, weighted by their interaction
                        float combinedRelation = _angleRelations[cyl1][b] * _angleRelations[cyl2][b] * interaction;
                        _bucketValues[b] += combinedRelation;
                    }
                }
            }

            // Clear stored relations for next time step
            _angleRelations.Clear();
        }

        public int GetMaxBucket()
        {
            float maxValue = float.MinValue;
            int maxIndex = 0;


            for (int i = 0; i < _bucketCount; i++)
            {
                if (_bucketValues[i] > maxValue)
                {
                    maxValue = _bucketValues[i];
                    maxIndex = i;
                }
            }

            return maxIndex;
        }



        public void Reset()

        {
            Array.Clear(_bucketValues, 0, _bucketValues.Length);
            _angleRelations.Clear();
        }



        public float[] GetBucketValues()

        {
            return (float[])_bucketValues.Clone();
        }

    }
}
