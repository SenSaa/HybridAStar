using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using env;
using System.Linq;
using utils;

namespace astar
{

    public class Astar
    {
        // Standard/Wighted A*.

        private env.Grid grid;
        private List<int> start;
        private List<AstarParams> table;
        // *
        public List<List<int>> route;
        // ***
        public int numberOfIterations;

        public Astar(env.Grid grid, List<double> start)
        {
            this.grid = grid;
            this.start = this.grid.to_cell_id(start);
            table = new List<AstarParams>();
            this.route = new List<List<int>>();
            numberOfIterations = 1000;
        }

        private double heuristic(List<int> p1, List<int> p2)
        {
            // Simple Manhattan distance  heuristic.
            return Math.Abs(p2[0] - p1[0]) + Math.Abs(p2[1] - p1[1]);
        }

        private List<List<int>> backtracking(AstarNode node)
        {
            // Backtracking the path.

            var route = new List<List<int>>();
            while (node.parent != null)
            {
                route.Add((node.cell_id));
                node = node.parent;
            }

            route.Add((this.start));

            route.Reverse();
            return route;
        }

        public double search_path(List<double> goal_)
        {
            // Search the path by astar.

            var goal = this.grid.to_cell_id(goal_);

            for(int i=0; i < table.Count; i++)
            {
                //
                if (table[i].cell_id[0] == goal[0] && table[i].cell_id[1] == goal[1])
                {
                    return this.table[this.table.IndexOf(table[i])].g;
                }
            }
            

            var root = new AstarNode(this.start);
            root.g = 0;
            // -----
            // * Change from standard A* to Weighted A*
            //root.f = root.g + self.heuristic(self.start, goal)
            var e = 5;
            root.f = root.g + e * this.heuristic(this.start, goal);
            // -----

            var closed_ = new List<AstarNode>();
            var open_ = new List<AstarNode> { root };

            int iteration = 0;
            while (open_ != null && iteration < numberOfIterations)
            {
                iteration++;

                var best = Extensions.MinBy(open_, x => x.f);
                
                open_.Remove(best);
                closed_.Add(best);

                if (best.cell_id[0] == goal[0] && best.cell_id[1] == goal[1])
                {
                    // *
                    this.route = this.backtracking(best);
                    this.table.Add(new AstarParams(goal, best.g));
                    return best.g;
                }

                var nbs = this.grid.get_neighbors(best.cell_id);

                foreach (var nb in nbs)
                {
                    var child = new AstarNode(nb);
                    child.g = best.g + 1;
                    // -----
                    // * Change from standard A* to Weighted A*
                    //child.f = child.g + self.heuristic(nb, goal)
                    //e = 5
                    child.f = child.g + e * this.heuristic(nb, goal);
                    // -----
                    child.parent = best;

                    if(Utils.NodesListContainsNode(closed_, child))
                    {
                        continue;
                    }

                    if (!Utils.NodesListContainsNode(open_, child))
                    {
                        open_.Add(child);
                    }

                    var nodeExists_cost = Utils.CostOfNodeInNodesList(open_, child);
                    var nodeExists = nodeExists_cost.Item1;
                    var cost = nodeExists_cost.Item2;
                    if (nodeExists)
                    {
                        if (child.g < cost)
                        {
                            open_.Remove(child);
                            open_.Add(child);
                        }
                    }
                }

            }

            return double.PositiveInfinity;
        }

    }

}