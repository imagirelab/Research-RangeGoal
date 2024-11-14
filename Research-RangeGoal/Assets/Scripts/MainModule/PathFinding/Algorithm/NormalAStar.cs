using System.Collections.Generic;
using System.Linq;
using MainModule.PathFinding.Core;

namespace MainModule.PathFinding.Algorithm
{
    public class NormalAStar : ISolver
    {
        private readonly GridGraphMediator mediator;
        private readonly MultiGoalAStar pathFinder;

        public NormalAStar(Graph graph, GridGraphMediator mediator)
        {
            this.mediator = mediator;

            pathFinder = new MultiGoalAStar(graph, CreateNodes(graph));
        }

        public List<int> Solve(int start, int goal)
        {
            List<Node> path = pathFinder.FindPath(start, new HashSet<int>() { goal }, mediator.GetPos(goal));
            return path.Select(node => node.Index).ToList();
        }

        private List<Node> CreateNodes(Graph graph)
        {
            // 座標からアルゴリズムで使用するノードを作成
            List<Node> nodes = new List<Node>(graph.NodeCount);

            for (int i = 0; i < graph.NodeCount; i++)
            {
                Vector2Int pos = mediator.GetPos(i);
                nodes.Add(new Node(i, new Vector2(pos.x, pos.y)));
            }

            return nodes;
        }
    }
}