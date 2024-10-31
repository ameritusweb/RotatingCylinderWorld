using ParallelReverseAutoDiff.PRAD;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RotatingCylinderWorld
{
    public static class PradOpExtensions
    {
        public static PradOp[] Replicate(this PradOp pradOp, int n)
        {
            var branches = pradOp.BranchStack(n);
            PradOp[] inst = new PradOp[n];
            inst[0] = pradOp;
            for (int i = 1; i < n; ++i)
            {
                inst[i] = branches.Pop();
            }

            return inst;
        }
    }
}
