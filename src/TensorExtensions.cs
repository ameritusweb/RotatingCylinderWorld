

namespace RotatingCylinderWorld
{
    using ParallelReverseAutoDiff.PRAD;
    using System;
    using System.Threading.Tasks;

    public static class TensorExtensions
    {
        /// <summary>
        /// Applies Gaussian noise to the tensor in place, maintaining the same shape.
        /// The noise is generated with a mean of 0 and standard deviation specified by the noise level.
        /// </summary>
        /// <param name="tensor">The tensor to which noise will be applied.</param>
        /// <param name="noiseLevel">The standard deviation of the Gaussian noise to be added.</param>
        /// <returns>A new tensor with noise applied.</returns>
        public static Tensor ApplyNoise(this Tensor tensor, Random rand, float noiseLevel)
        {
            if (noiseLevel <= 0)
            {
                throw new ArgumentException("Noise level must be a positive value.");
            }

            // Create a new tensor to hold the noisy result
            var result = new Tensor(tensor.Shape);

            // Apply noise to each element in parallel
            for (int i = 0; i < tensor.Data.Length; ++i)
            {
                // Generate Gaussian noise with mean 0 and standard deviation = noiseLevel
                var u1 = rand.NextDouble();
                var u2 = rand.NextDouble();
                var noise = noiseLevel * MathF.Sqrt(-2.0f * MathF.Log((float)u1)) * MathF.Cos(2.0f * MathF.PI * (float)u2);

                // Add noise to the original value
                result.Data[i] = tensor.Data[i] + noise;
            }

            return result;
        }
    }

}
