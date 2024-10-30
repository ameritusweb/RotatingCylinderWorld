using ParallelReverseAutoDiff.PRAD;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RotatingCylinderWorld
{
    public class PradOpExample
    {
        public void RunExample(int timeSteps, int numCylinders, int numBuckets, PradOp initialAngles, PradOp cylinderRadii, PradOp velocityOverTime, PradOp accelerationOverTime, PradOp deltasOverTime, PradOp cylinderBezier)
        {
            Tensor tensor = new Tensor(new[] { 1, numCylinders }, 0f);
            Tensor time = new Tensor(new[] {1, timeSteps}, 10f);
            PradOp opTime = new PradOp(time);
            PradOp cylinderAngles = new PradOp(tensor);

            Tensor bucketCenters = new Tensor(new int[] { numBuckets, 1 }, 0f);
            for (int i = 0; i < numBuckets; i++)
            {
                float bucketCenterAngle = (2 * MathF.PI * i) / numBuckets;
                bucketCenters.Data[i] = bucketCenterAngle;
            }

            PradOp bucketCentersOp = new PradOp(bucketCenters);
            PradOp bucketCentersTiled = bucketCentersOp.Tile(new int[] { 1, numCylinders }).PradOp;
            BranchStack bucketCentersBranches = bucketCentersTiled.BranchStack(timeSteps - 1);
            PradOp[] bucketCentersOpArray = new PradOp[timeSteps];
            bucketCentersOpArray[0] = bucketCentersTiled;
            for (int ii = 1; ii < timeSteps; ++ii)
            {
                bucketCentersOpArray[ii] = bucketCentersBranches.Pop();
            }

            var initializeCylinders = cylinderAngles.Add(cylinderAngles.CurrentTensor).PradOp;

            for (int i = 0; i < timeSteps; ++i)
            {
                PradOp velocity = velocityOverTime.Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp accel = accelerationOverTime.Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp delta = deltasOverTime.Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp opTimeI = opTime.Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp deltaDiv = delta.Div(opTimeI.CurrentTensor).PradOp;
                PradOp[] cylinders = new PradOp[numCylinders];

                for (int j = 0; j < numCylinders; ++j)
                {
                    PradOp cylinderRadius = cylinderRadii.Indexer("...", $"{j}:{j + 1}").PradOp;
                    velocity.Div(cylinderRadius.CurrentTensor);
                    PradOp bezier1 = cylinderBezier.Indexer("0:1", $"...").PradOp;
                    PradOp bezier2 = cylinderBezier.Indexer("1:2", $"...").PradOp;
                    PradOp bezier3 = cylinderBezier.Indexer("2:3", $"...").PradOp;
                    PradOp bezier4 = cylinderBezier.Indexer("3:4", $"...").PradOp;
                    PradOp cylAngle = initializeCylinders.Indexer("...", $"{j}:{j + 1}").PradOp;

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
                PradOp cylindersTiled = cylindersConcat.Tile(new int[] { numBuckets, 1 }).PradOp;
                PradOp twoPI = new PradOp(new Tensor(cylindersTiled.CurrentShape, 2 * MathF.PI));
                var added = twoPI.Add(cylindersTiled.CurrentTensor).PradOp;
                var squared = added.Sub(bucketCentersOpArray[i].CurrentTensor).PradOp.Square().PradOp;
                var interaction = bucketCentersOpArray[i].Sub(cylindersTiled.CurrentTensor).PradOp.Square().PradOp;
                var min = interaction.Min(squared.CurrentTensor).PradOp;
                var bucketInteractions = min.Mul(new Tensor(cylindersTiled.CurrentShape, -1f)).PradOp.Exp().PradOp;

                // continue with interactions...
            }
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
