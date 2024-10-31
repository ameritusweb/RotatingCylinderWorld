using System.Numerics;
using System;
using ParallelReverseAutoDiff.PRAD;

namespace RotatingCylinderWorld
{
    internal class Program
    {

        static void Main(string[] args)
        {
            PradOpExample example = new PradOpExample();
            int seed = 300;
            Random rand = new Random(seed);
            float noiseLevel = 0.05f;
            PradOp initialAngles = new PradOp(new Tensor(new int[] { 1, 10 }, 0.1f).ApplyNoise(rand, noiseLevel));
            PradOp cylinderRadii = new PradOp(new Tensor(new int[] { 1, 10 }, 1f).ApplyNoise(rand, noiseLevel));
            PradOp velocityOverTime = new PradOp(new Tensor(new int[] { 1, 20 }, 0.2f).ApplyNoise(rand, noiseLevel));
            PradOp accelerationOverTime = new PradOp(new Tensor(new int[] { 1, 20 }, 0.1f).ApplyNoise(rand, noiseLevel));
            PradOp deltasOverTime = new PradOp(new Tensor(new int[] { 1, 20 }, 0.1f).ApplyNoise(rand, noiseLevel));
            PradOp cylinderBezier = new PradOp(new Tensor(new int[] { 4, 3 }, 0.1f).ApplyNoise(rand, noiseLevel));
            PradOp result = example.RunExample(20, 10, 12, initialAngles, cylinderRadii, velocityOverTime, accelerationOverTime, deltasOverTime, cylinderBezier);

        }

        static void Main2(string[] args)
        {
            Classifier classifier = new Classifier();
            int seed = 9;
            BoundedNormalRandom rand = new BoundedNormalRandom(0.5d, 0.2d, 0.1d, seed);
            double mean = rand.NextDouble();
            var result = classifier.Classify(1000000, seed, mean);
        }
    }
}
