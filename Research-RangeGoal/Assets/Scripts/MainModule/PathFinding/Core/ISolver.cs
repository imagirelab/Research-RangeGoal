using System.Collections.Generic;

namespace MainModule.PathFinding.Core
{
    public interface ISolver
    {
        List<int> Solve(int start, int goal);
    }
}