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
        public void RunExample(int timeSteps, int numCylinders, PradOp initialAngles, PradOp cylinderRadii, PradOp velocityOverTime, PradOp accelerationOverTime, PradOp deltasOverTime, PradOp cylinderBezier)
        {
            Tensor tensor = new Tensor(new[] { 1, numCylinders }, 0f);
            Tensor time = new Tensor(new[] {1, timeSteps}, 10f);
            PradOp opTime = new PradOp(time);
            PradOp cylinderAngles = new PradOp(tensor);

            var initializeCylinders = cylinderAngles.Add(cylinderAngles.CurrentTensor);

            for (int i = 0; i < timeSteps; ++i)
            {
                PradOp velocity = velocityOverTime.Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp accel = accelerationOverTime.Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp delta = deltasOverTime.Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp opTimeI = opTime.Indexer("...", $"{i}:{i + 1}").PradOp;
                PradOp deltaDiv = delta.Div(opTimeI.CurrentTensor).PradOp;

                for (int j = 0; j < numCylinders; ++j)
                {
                    PradOp cylinderRadius = cylinderRadii.Indexer("...", $"{j}:{j + 1}").PradOp;
                    velocity.Div(cylinderRadius.CurrentTensor);
                    PradOp bezier1 = cylinderBezier.Indexer("0:1", $"...").PradOp;
                    PradOp bezier2 = cylinderBezier.Indexer("1:2", $"...").PradOp;
                    PradOp bezier3 = cylinderBezier.Indexer("2:3", $"...").PradOp;
                    PradOp bezier4 = cylinderBezier.Indexer("3:4", $"...").PradOp;
                    PradOp bezier5 = cylinderBezier.Indexer("4:5", $"...").PradOp;
                    PradOp bezier6 = cylinderBezier.Indexer("5:6", $"...").PradOp;

                    for (int k = 0; k < 10; ++k)
                    {
                        float stepI = (float)k / 10;
                        Tensor stepTensor = new Tensor(new int[] { 1, 1 }, stepI);
                        PradOp step = new PradOp(stepTensor);
                    }
                }
            }
        }
    }
}
