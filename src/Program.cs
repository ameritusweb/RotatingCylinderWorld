using System.Numerics;
using System;

namespace RotatingCylinderWorld
{
    internal class Program

    {

        /// <summary>

        /// Creates a random Vector3 with components in the specified range.

        /// </summary>

        /// <param name="random"></param>

        /// <param name="min">The minimum value for the vector components.</param>

        /// <param name="max">The maximum value for the vector components.</param>

        /// <returns>A Vector3 with random X, Y, and Z components.</returns>

        public static Vector3 CreateRandomVector3(IRandom random, float min, float max)

        {



            float x = (float)(random.NextDouble() * (max - min) + min);

            float y = (float)(random.NextDouble() * (max - min) + min);

            float z = (float)(random.NextDouble() * (max - min) + min);



            return new Vector3(x, y, z);

        }



        /// <summary>

        /// Creates a RotatingCylinderNetwork with randomized control columns for a specified number of cylinders and time steps.

        /// </summary>

        public static RotatingCylinderNetwork CreateRandomCylinderNetwork(int numCylinders, int N, int bucketCount, int seed, double mean, double noise = 0.1d)

        {

            // Initialize the network with a given bucket count

            RotatingCylinderNetwork network = new RotatingCylinderNetwork(bucketCount);



            for (int i = 0; i < numCylinders; i++)

            {

                BoundedNormalRandom random = new BoundedNormalRandom(mean, 0.1d, noise, seed + i + 1);

                var randN = (int)(random.NextDouble() * 10000);

                BoundedNormalRandom random2 = new BoundedNormalRandom(mean, 0.1d, noise, seed + ((i + 1) * randN * 1000));



                // Generate random velocity, acceleration, and time delta columns for N time steps

                float[] velocityValues = new float[N];

                float[] accelerationValues = new float[N];

                float[] timeDeltaValues = new float[N];



                for (int t = 0; t < N; t++)

                {

                    // Randomize the values within some reasonable ranges

                    velocityValues[t] = (float)(random.NextDouble() * 2.0); // Velocity range [0, 2]

                    accelerationValues[t] = (float)(random.NextDouble() * 0.5); // Acceleration range [0, 0.5]

                    timeDeltaValues[t] = (float)(random.NextDouble() * 0.2 + 0.1); // Time delta range [0.1, 0.3]

                }



                // Create the control columns for velocity, acceleration, and time deltas

                ControlColumn velocityColumn = new ControlColumn(velocityValues, null);

                ControlColumn accelerationColumn = new ControlColumn(accelerationValues, new Vector3[] {

        CreateRandomVector3(random2, (float)random2.NextDouble(), 0.5f),

        CreateRandomVector3(random2, (float)random2.NextDouble(), 0.5f),

        CreateRandomVector3(random2, (float)random2.NextDouble(), 0.5f)

      });

                ControlColumn timeDeltaColumn = new ControlColumn(timeDeltaValues, null);



                // Randomize the radius for the cylinder (e.g., in the range [0.5, 2.5])

                float radius = (float)(random2.NextDouble() * 2.0 + 0.5);



                // Create the cylinder and its control

                Cylinder cylinder = new Cylinder((float)random2.NextDouble() * MathF.PI * 2f);

                CylinderControl cylinderControl = new CylinderControl(cylinder, velocityColumn, accelerationColumn, timeDeltaColumn, radius);



                // Add the cylinder control to the network

                network.AddCylinderControl(cylinderControl);

            }



            return network;

        }



        static void Main(string[] args)

        {

            int timeSpan = 11000;

            RotatingCylinderNetwork network = CreateRandomCylinderNetwork(10, timeSpan, 12, 252, 0.5d, 0.100051d);



            // Simulate forward pass for multiple time steps

            List<int> classifications = new List<int>();

            for (int t = 0; t < timeSpan; t++)

            {

                network.Forward(t);

                classifications.Add(network.Classify());

            }



            var counts = GetConsecutiveCounts(classifications);



            var distinct = classifications.Distinct().ToList();



            var winner = GetWinningTuple(counts);



            var top = GetTopNWinningTuples(counts, distinct.Count);



            // Get classification result based on accumulated angular values

            int classification = network.Classify();

            Console.WriteLine($"Classification Result: {classification}");



            // Optional: Get bucket values for visualization

            float[] bucketValues = network.GetBucketValues();

            Console.WriteLine($"Bucket Values: {string.Join(", ", bucketValues)}");

        }



        /// <summary>

        /// Finds the top N tuples with the highest combined sum of consecutive counts.

        /// </summary>

        /// <param name="consecutiveCounts">A list of tuples containing values and their consecutive counts.</param>

        /// <param name="topN">The number of top results to retrieve.</param>

        /// <returns>A list of tuples representing the values with the highest combined counts.</returns>

        public static List<(int Value, int Count)> GetTopNWinningTuples(List<(int Value, int Count)> consecutiveCounts, int topN)

        {

            // Use a dictionary to aggregate counts for each unique value

            var valueCounts = new Dictionary<int, int>();



            foreach (var tuple in consecutiveCounts)

            {

                if (valueCounts.ContainsKey(tuple.Value))

                {

                    valueCounts[tuple.Value] += tuple.Count;

                }

                else

                {

                    valueCounts[tuple.Value] = tuple.Count;

                }

            }



            // Sort by combined count in descending order and take the top N results

            var topWinningTuples = valueCounts

        .OrderByDescending(entry => entry.Value)

        .Take(topN)

        .Select(entry => (entry.Key, entry.Value))

        .ToList();



            return topWinningTuples;

        }



        /// <summary>

        /// Returns a list of tuples, where each tuple contains a value and the count of its consecutive occurrences.

        /// </summary>

        /// <param name="numbers">The list of integers to process.</param>

        /// <returns>A list of tuples containing values and their consecutive counts.</returns>

        public static List<(int Value, int Count)> GetConsecutiveCounts(List<int> numbers)

        {

            var result = new List<(int Value, int Count)>();



            if (numbers.Count == 0)

                return result;



            int currentValue = numbers[0];

            int currentCount = 1;



            for (int i = 1; i < numbers.Count; i++)

            {

                if (numbers[i] == currentValue)

                {

                    currentCount++;

                }

                else

                {

                    result.Add((currentValue, currentCount));

                    currentValue = numbers[i];

                    currentCount = 1;

                }

            }



            // Add the last group

            result.Add((currentValue, currentCount));



            return result;

        }



        /// <summary>

        /// Finds the tuple with the highest combined sum of consecutive counts.

        /// </summary>

        /// <param name="consecutiveCounts">A list of tuples containing values and their consecutive counts.</param>

        /// <returns>A tuple representing the value with the highest combined count and the sum.</returns>

        public static (int Value, int Count) GetWinningTuple(List<(int Value, int Count)> consecutiveCounts)

        {

            // Use a dictionary to aggregate counts for each unique value

            var valueCounts = new Dictionary<int, int>();



            foreach (var tuple in consecutiveCounts)

            {

                if (valueCounts.ContainsKey(tuple.Value))

                {

                    valueCounts[tuple.Value] += tuple.Count;

                }

                else

                {

                    valueCounts[tuple.Value] = tuple.Count;

                }

            }



            // Find the key with the maximum count

            var winningEntry = valueCounts.Aggregate((max, current) => current.Value > max.Value ? current : max);



            return (winningEntry.Key, winningEntry.Value);

        }

    }
}
