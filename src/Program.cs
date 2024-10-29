using System.Numerics;
using System;

namespace RotatingCylinderWorld
{
    internal class Program
    {

        static void Main(string[] args)
        {
            Classifier classifier = new Classifier();
            var result = classifier.Classify(100000, 7000, 0.5d);
        }
    }
}
