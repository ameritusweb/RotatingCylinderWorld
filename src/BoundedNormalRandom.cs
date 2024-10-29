using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RotatingCylinderWorld
{
    public class BoundedNormalRandom : IRandom
    {
        private readonly Random _random;
        private readonly double _mean;
        private readonly double _stdDev;
        private readonly double _noiseLevel;

        public BoundedNormalRandom(double mean, double stdDev, double noiseLevel, int seed)
        {
            _random = new Random(seed);
            _mean = mean;
            _stdDev = stdDev;
            _noiseLevel = noiseLevel;
        }

        public double NextDouble()
        {
            // Box-Muller transform to generate normal distribution
            double u1 = _random.NextDouble();
            double u2 = _random.NextDouble();

            double randStdNormal = Math.Sqrt(-2.0 * Math.Log(u1)) *
                                  Math.Sin(2.0 * Math.PI * u2);

            // Apply mean and standard deviation
            double randNormal = _mean + _stdDev * randStdNormal;

            // Add noise based on noise level
            double noise = (_random.NextDouble() * 2 - 1) * _noiseLevel;
            double result = randNormal + noise;

            // Keep the result bounded within reasonable limits (3 standard deviations)
            double lowerBound = _mean - 3 * _stdDev;
            double upperBound = _mean + 3 * _stdDev;
            var reasonablyBounded = Math.Max(lowerBound, Math.Min(upperBound, result));

            lowerBound = 0d;
            upperBound = 1d;
            return Math.Max(lowerBound, Math.Min(upperBound, reasonablyBounded));
        }
    }
}
