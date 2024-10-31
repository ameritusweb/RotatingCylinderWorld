using ParallelReverseAutoDiff.PRAD;

namespace RotatingCylinderWorld
{
    public class PradOpExample
    {
        public PradOp RunExample(int timeSteps, int numCylinders, int numBuckets, PradOp initialAngles, PradOp cylinderRadii, PradOp velocityOverTime, PradOp accelerationOverTime, PradOp deltasOverTime, PradOp cylinderBezier)
        {
            Tensor tensor = new Tensor(new[] { 1, numCylinders }, 0f);
            Tensor time = new Tensor(new[] {1, timeSteps}, 10f);
            PradOp opTime = new PradOp(time);
            PradOp cylinderAngles = new PradOp(tensor);

            Tensor bucketCenters = new Tensor(new int[] { 1, numBuckets }, 0f);
            for (int i = 0; i < numBuckets; i++)
            {
                float bucketCenterAngle = (2 * MathF.PI * i) / numBuckets;
                bucketCenters.Data[i] = bucketCenterAngle;
            }

            PradOp bucketCentersOp = new PradOp(bucketCenters);
            PradOp bucketCentersTiled = bucketCentersOp.Tile(new int[] { numCylinders, 1 }).PradOp.Transpose(1, 0).PradOp;

            PradOp[] bucketCentersOpArray = bucketCentersTiled.Replicate(timeSteps);

            var initializeCylinders = cylinderAngles.Add(initialAngles.CurrentTensor).PradOp;
            var initializeCylindersBranch = initializeCylinders.Branch();

            Tensor bucketInitialTensor = new Tensor(new[] { 1, numBuckets }, 0f);
            PradOp bucketCumulative = new PradOp(bucketInitialTensor);

            var velocityOverTimeArray = velocityOverTime.Replicate(timeSteps);
            var accelerationOverTimeArray = accelerationOverTime.Replicate(timeSteps);
            var deltasOverTimeArray = deltasOverTime.Replicate(timeSteps);
            var opTimeArray = opTime.Replicate(timeSteps);
            var radiiBranchesArray = cylinderRadii.Replicate(timeSteps * numCylinders);
            var cylinderBezierArray = cylinderBezier.Replicate(timeSteps * numCylinders * 4);
            var initializeCylindersArray = initializeCylinders.Replicate(numCylinders);

            for (int i = 0; i < timeSteps; ++i)
            {
                PradOp velocity = velocityOverTimeArray[i].Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp accel = accelerationOverTimeArray[i].Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp delta = deltasOverTimeArray[i].Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp opTimeI = opTimeArray[i].Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp deltaDiv = delta.Div(opTimeI.CurrentTensor).PradOp;
                PradOp[] cylinders = new PradOp[numCylinders];
                if (i > 0)
                {
                    initializeCylindersArray = initializeCylindersBranch.Replicate(numCylinders);
                }

                for (int j = 0; j < numCylinders; ++j)
                {
                    int ind = (i * numCylinders) + j;
                    PradOp cylinderRadius = radiiBranchesArray[ind].Indexer("...", $"{j}:{j + 1}").PradOp;
                    velocity.Div(cylinderRadius.CurrentTensor);

                    int ind2 = (i * numCylinders * 4) + (j * 4);
                    PradOp bezier1 = cylinderBezierArray[ind2].Indexer("0:1", $"...").PradOp;
                    PradOp bezier2 = cylinderBezierArray[ind2 + 1].Indexer("1:2", $"...").PradOp;
                    PradOp bezier3 = cylinderBezierArray[ind2 + 2].Indexer("2:3", $"...").PradOp;
                    PradOp bezier4 = cylinderBezierArray[ind2 + 3].Indexer("3:4", $"...").PradOp;
                    PradOp cylAngle = initializeCylindersArray[j].Indexer("...", $"{j}:{j + 1}").PradOp;

                    for (int k = 0; k < 10; ++k)
                    {
                        float stepI = (float)k / 10;
                        Tensor stepTensor = new Tensor(new int[] { 1, 3 }, stepI);
                        PradOp step = new PradOp(stepTensor);
                        PradOp cubicBezier = this.CubicBezier(step, bezier1, bezier2, bezier3, bezier4);
                        PradOp cubicFirst = cubicBezier.Indexer("...", "0:1").PradOp;
                        PradOp cubicMult = cubicFirst.Mul(opTimeI.CurrentTensor).PradOp;
                        cylAngle.Add(cubicMult.CurrentTensor);
                    }

                    cylinders[j] = cylAngle.Modulus(new Tensor(new[] { 1, 1 }, MathF.PI * 2f)).PradOp;
                }

                PradOp cylindersConcat = cylinders[0].Concat(cylinders.Skip(1).Select(x => x.CurrentTensor).ToArray(), 1).PradOp;

                initializeCylindersBranch.Add(cylindersConcat.CurrentTensor);

                var cylindersConcatBranches = cylindersConcat.BranchStack(numBuckets);

                PradOp cylindersTiled = cylindersConcat.Tile(new int[] { numBuckets, 1 }).PradOp;
                PradOp twoPI = new PradOp(new Tensor(cylindersTiled.CurrentShape, 2 * MathF.PI));
                var added = twoPI.Add(cylindersTiled.CurrentTensor).PradOp;
                var squared = added.Sub(bucketCentersOpArray[i].CurrentTensor).PradOp.Square().PradOp;
                var interaction = bucketCentersOpArray[i].Sub(cylindersTiled.CurrentTensor).PradOp.Square().PradOp;
                var min = interaction.Min(squared.CurrentTensor).PradOp;
                var bucketInteractions = min.Mul(new Tensor(cylindersTiled.CurrentShape, -1f)).PradOp.Exp().PradOp;
                PradOp[] bucketValueDiffs = new PradOp[numBuckets];

                var bucketInteractionsArray = bucketInteractions.Replicate(numBuckets);

                for (int bucketIdx = 0; bucketIdx < numBuckets; bucketIdx++)
                {
                    // Get this bucket's interaction values for all cylinders
                    PradOp bucketInteraction = bucketInteractionsArray[bucketIdx].Indexer($"{bucketIdx}:{bucketIdx + 1}", "...").PradOp;

                    // Create pairs of angles for interaction calculation
                    PradOp anglePairs = cylindersConcatBranches.Pop().SelfPair().PradOp; // [2,M] tensor of angle pairs

                    // Calculate both possible differences
                    PradOp firstRow = anglePairs.Indexer("0:1", "...").PradOp;
                    PradOp secondRow = anglePairs.Indexer("1:2", "...").PradOp;
                    PradOp regularDiff = firstRow.Sub(secondRow.CurrentTensor).PradOp.Abs().PradOp;
                    PradOp twoPI2 = new PradOp(new Tensor(regularDiff.CurrentShape, 2 * MathF.PI));
                    PradOp wrappedAngle = secondRow.Add(twoPI2.CurrentTensor).PradOp;
                    PradOp wrappedDiff = firstRow.Sub(wrappedAngle.CurrentTensor).PradOp.Abs().PradOp;

                    // Take minimum of the two differences
                    PradOp minDiff = regularDiff.Min(wrappedDiff.CurrentTensor).PradOp;

                    // Calculate interaction weights (e^-diff)
                    PradOp interactions = minDiff.Mul(new Tensor(minDiff.CurrentShape, -1f)).PradOp.Exp().PradOp;

                    // Stack bucket relations and interactions
                    PradOp combined = bucketInteraction.SelfPair().PradOp.Concat(new[] { interactions.CurrentTensor }, 0).PradOp;

                    // Multiply columns to get final bucket values
                    PradOp bucketValues = combined.MultiplyColumns().PradOp;

                    PradOp bucketValue = bucketValues.SumRows().PradOp;
                    bucketValueDiffs[bucketIdx] = bucketValue;
                }

                PradOp bucketValueDiff = bucketValueDiffs[0].Concat(bucketValueDiffs.Skip(1).Select(x => x.CurrentTensor).ToArray(), 1).PradOp;

                var bucketValueMax = bucketValueDiff.CurrentTensor.Data.Max();
                PradOp maxReciprocalSquareOp = new PradOp(new Tensor(bucketValueDiff.CurrentShape, bucketValueMax + 1f)).Sub(bucketValueDiff.CurrentTensor).PradOp.Reciprocal().PradOp.Square().PradOp;
                PradOp mulOp = maxReciprocalSquareOp.Mul(new Tensor(maxReciprocalSquareOp.CurrentShape, 10f)).PradOp;

                bucketCumulative.Add(mulOp.CurrentTensor);
            }

            return bucketCumulative;
        }

        private PradOp CubicBezier(PradOp t, PradOp p0, PradOp p1, PradOp p2, PradOp p3)
        {
            // Create t branches for reuse
            var tBranches = t.BranchStack(4);

            // Calculate (1-t) once and create branches for reuse
            var oneMinusT = tBranches.Pop().SubFrom(new Tensor(t.CurrentShape, 1.0f));
            var uBranches = oneMinusT.PradOp.BranchStack(3);

            // Calculate powers of t and (1-t)
            var t2 = t.Square();
            var t3 = t2.Then(PradOp.MulOp, tBranches.Pop().BranchInitialTensor);

            var u2 = uBranches.Pop().Square();
            var u3 = u2.Then(PradOp.MulOp, uBranches.Pop().BranchInitialTensor);

            // Calculate the four Bernstein polynomial terms
            // Term 1: (1-t)³ * p0
            var term1 = u3.Then(result =>
                result.PradOp.Mul(p0.CurrentTensor));

            // Term 2: 3(1-t)²t * p1
            var term2 = u2
                .Then(result => result.PradOp.Mul(tBranches.Pop().BranchInitialTensor))
                .Then(result => result.PradOp.Mul(new Tensor(t.CurrentShape, 3.0f)))
                .Then(result => result.PradOp.Mul(p1.CurrentTensor));

            // Term 3: 3(1-t)t² * p2
            var term3 = uBranches.Pop()
                .Mul(t2.PradOp.CurrentTensor)
                .Then(result => result.PradOp.Mul(new Tensor(t.CurrentShape, 3.0f)))
                .Then(result => result.PradOp.Mul(p2.CurrentTensor));

            // Term 4: t³ * p3
            var term4 = t3.Then(result =>
                result.PradOp.Mul(p3.CurrentTensor));

            // Sum all terms
            return term1
                .Then(result => result.PradOp.Add(term2.Result))
                .Then(result => result.PradOp.Add(term3.Result))
                .Then(result => result.PradOp.Add(term4.Result))
                .PradOp;
        }
    }
}
