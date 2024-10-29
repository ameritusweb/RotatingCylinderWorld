using System.Numerics;
using System;

namespace RotatingCylinderWorld
{
    internal class Program
    {

        static void Main(string[] args)
        {
            Classifier classifier = new Classifier();
            int seed = 9;
            BoundedNormalRandom rand = new BoundedNormalRandom(0.5d, 0.2d, 0.1d, seed);
            double mean = rand.NextDouble();
            var result = classifier.Classify(1000000, seed, mean);
        }
    }
}
