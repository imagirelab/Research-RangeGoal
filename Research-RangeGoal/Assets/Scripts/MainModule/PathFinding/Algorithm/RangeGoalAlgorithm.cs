using System;
using System.Collections.Generic;
using System.Linq;
using MainModule.PathFinding.Core;
using Vector2 = MainModule.PathFinding.Core.Vector2;
using Vector2Int = MainModule.PathFinding.Core.Vector2Int;

namespace MainModule.PathFinding.Algorithm
{
    public class RangeGoalAlgorithm : ISolver
    {
        private readonly GridGraphMediator mediator;
        private readonly MultiGoalAStar pathFinder;
        private readonly RangeGoalAStar rangeGoalAStar;
        private readonly RangeGoalFinder rangeGoalFinder;

        public HashSet<int> CorrectGoals;
        public HashSet<int> IncorrectGoals;
        public List<List<int>> DebugPaths = new List<List<int>>();
        public List<Vector2> vs = new List<Vector2>();
        public List<Vector2Int> points = new List<Vector2Int>();

        public RangeGoalAlgorithm(Graph graph, GridGraphMediator mediator)
        {
            this.mediator = mediator;

            pathFinder = new MultiGoalAStar(graph, CreateNodes(graph));
            rangeGoalAStar = new RangeGoalAStar(graph, CreateNodes(graph));
            rangeGoalFinder = new RangeGoalFinder(graph, mediator);
        }

        public List<int> Solve(int start, int goal)
        {
            DebugPaths.Clear();
            points.Clear();
            vs.Clear();

            Vector2Int startPos = mediator.GetPos(start);
            Vector2Int goalPos = mediator.GetPos(goal);

            const float alpha = 0.5f;
            Vector2Int diff = startPos - goalPos;

            // ゴールからスタートの距離 * alphaの長さをゴールの範囲とする
            double distance = Math.Sqrt(diff.x * diff.x + diff.y * diff.y) * alpha;
            int radius = (int)Math.Round(distance);

            // 範囲ゴールを取得
            var rangeGoals = rangeGoalFinder.GetRangeGoals(goalPos, radius);

            // 到達可能な範囲ゴール
            var correctRangeGoal = new List<HashSet<int>>();

            rangeGoalAStar.Reset();

            foreach (HashSet<int> rangeGoal in rangeGoals)
            {
                // 経路を許可する範囲
                // 境界のグリッドをゴール内にするために、はみ出し判定に余裕をもたせる
                float allowRange = radius + 0.5f;

                // 範囲ゴールからゴールまでの経路を求める
                // 収まっていたら到達可能な範囲ゴールに含める
                if (rangeGoalAStar.IsReachableGoal(rangeGoal.First(), goal, allowRange))
                {
                    correctRangeGoal.Add(rangeGoal);
                }
            }

            HashSet<int> correctGoals = correctRangeGoal.SelectMany(item => item).ToHashSet();

            // パスと範囲ゴールのデバッグ
            DebugPaths.Add(rangeGoalAStar.PassableNodes.ToList());
            CorrectGoals = correctGoals;
            HashSet<int> rangeGoalSet = rangeGoals.SelectMany(item => item).ToHashSet();
            rangeGoalSet.ExceptWith(CorrectGoals);
            IncorrectGoals = rangeGoalSet;

            // 到達可能な範囲ゴールに対して経路探索
            List<Node> result = pathFinder.FindPath(start, correctGoals, mediator.GetPos(goal));

            // ノードに変換して結果に追加
            return result.Select(node => node.Index).ToList();
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