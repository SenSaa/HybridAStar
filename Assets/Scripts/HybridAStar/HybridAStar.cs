using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using astar;
using env;
using Dubins;
using utils;
using System.Linq;

namespace hybridAStar
{

    public class HybridAStar
    {
        // Hybrid A* search.

        private SimpleCar car;
        private env.Grid grid;
        private bool reverse;
        private double unit_theta;
        private double dt;
        private int check_dubins;
        private List<double> start;
        private List<double> goal;
        private double r;
        private int drive_steps;
        private double arc;
        private List<double> phil;
        private List<int> ml;
        private List<Tuple<double, double>> comb;
        private DubinsPath dubins;
        private Astar astar;
        private double w1, w2, w3, w4, w5;
        private List<double> thetas;

        public int numberOfIterations;

        public HybridAStar(SimpleCar car, env.Grid grid, bool reverse, double unit_theta= Math.PI / 12, double dt= 1e-2, int check_dubins= 1)
        {
            this.car = car;
            this.grid = grid;
            this.reverse = reverse;
            this.unit_theta = unit_theta;
            this.dt = dt;
            this.check_dubins = check_dubins;

            this.start = this.car.start_pos;
            this.goal = this.car.end_pos;
            Debug.Log("start -> " + Utils.ListToString(start));
            Debug.Log("goal -> " + Utils.ListToString(goal));

            this.r = this.car.l / Math.Tan(this.car.max_phi);
            this.drive_steps = (int)(Math.Sqrt(2) * this.grid.cell_size / this.dt) + 1;
            this.arc = this.drive_steps * this.dt;
            this.phil = new List<double> { -this.car.max_phi, 0, this.car.max_phi };
            this.ml = new List<int> { 1, -1 };

            if (reverse)
            {
                List<double> ml_ = new List<double>();
                for (int i = 0; i < this.ml.Count; i++)
                {
                    ml_.Add(this.ml[i]);
                }
                this.comb = Utils.Product<double>(ml_, this.phil);
            }
            else
            {
                this.comb = Utils.Product<double>(new List<double> { 1 }, this.phil);
            }

            this.dubins = new DubinsPath(this.car);
            var goal_ = new List<double> { this.goal[0], this.goal[1] };
            this.astar = new Astar(this.grid, goal_);

            this.w1 = 0.95; // weight for astar heuristic
            this.w2 = 0.05; // weight for simple heuristic
            this.w3 = 0.30; // weight for extra cost of steering angle change
            this.w4 = 0.10; // weight for extra cost of turning
            this.w5 = 2.00; // weight for extra cost of reversing

            this.thetas = Utils.get_discretized_thetas(this.unit_theta);

            numberOfIterations = 1000;
        }

        private HybridAStarNode construct_node(List<double> pos)
        {
            // Create node for a pos.

            var theta = pos[2];
            var pt = new List<double> { pos[0], pos[1] };

            theta = Utils.round_theta(Utils.Modulus(theta, (2 * Math.PI)), this.thetas);

            var cell_id = this.grid.to_cell_id(pt);

            List<double> grid_pos = new List<double>();
            for (int i=0; i < cell_id.Count; i++)
            {
                grid_pos.Add(cell_id[i]);
            }
            grid_pos.Add(theta);

            var node = new HybridAStarNode(grid_pos, pos);

            return node;
        }

        private double simple_heuristic(List<double> pos)
        {
            // Heuristic by Manhattan distance.
            return Math.Abs(this.goal[0] - pos[0]) + Math.Abs(this.goal[1] - pos[1]);
        }

        private double astar_heuristic(List<double> pos)
        {
            // Heuristic by standard astar.
            var pos_ = new List<double> { pos[0], pos[1] };
            var h1 = this.astar.search_path(pos_) * this.grid.cell_size;
            var h2 = this.simple_heuristic(pos_);
            return this.w1 * h1 + this.w2 * h2;
        }

        private List<(HybridAStarNode, List<(double, List<double>)>)> get_children(HybridAStarNode node, int heu, bool extra)
        {
            // Get successors from a state.

            var children = new List<(HybridAStarNode, List<(double, List<double>)>)>();
            foreach (var m_phi in this.comb)
            {
                var m = m_phi.Item1;
                var phi = m_phi.Item2;

                // don't go back
                if (node.m == phi && node.phi == phi && node.m * m == -1)
                {
                    continue;
                }

                if (node.m == 1 && m == -1)
                {
                    continue;
                }

                var pos = node.pos;
                var branch = new List<(double, List<double>)> { (  m, new List<double> { pos[0], pos[1] } ) };

                for (int _=0; _ < this.drive_steps; _++)
                {
                    pos = this.car.step(pos, phi, m);
                    branch.Add((m, new List<double> { pos[0], pos[1] }));
                }

                // check safety of route-----------------------
                var pos1 = m ==1 ? node.pos : pos;
                var pos2 = m == 1 ? pos : node.pos;
                
                var safe = true;

                if (phi == 0)
                {
                    safe = this.dubins.is_straight_route_safe(pos1, pos2);
                }
                else
                {
                    var d_c_r = this.car.get_params(pos1, phi);
                    var d = d_c_r.Item1;
                    var c = d_c_r.Item2;
                    var r = d_c_r.Item3;
                    safe = this.dubins.is_turning_route_safe(pos1, pos2, d, c, r);
                }
                // --------------------------------------------
            
                if (!safe)
                {
                    continue;
                }


                var child = this.construct_node(pos);
                child.phi = phi;
                child.m = m;
                child.parent = node;
                child.g = node.g + this.arc;
                child.g_ = node.g_ + this.arc;

                if (extra)
                {
                    // extra cost for changing steering angle
                    if (phi != node.phi)
                    {
                        child.g += this.w3 * this.arc;
                    }
                

                    // extra cost for turning
                    if (phi != 0)
                    {
                        child.g += this.w4 * this.arc;
                    }

                    // extra cost for reverse
                    if (m == -1)
                    {
                        child.g += this.w5 * this.arc;
                    }
                }

                if (heu == 0)
                {
                    child.f = child.g + this.simple_heuristic(child.pos);
                }
                if (heu == 1)
                {
                    child.f = child.g + this.astar_heuristic(child.pos);
                }

                children.Add((child, branch));
            }

            return children;
        }

        private (HybridAStarNode, double, List<(List<double>, double, int)>) best_final_shot(List<HybridAStarNode> open_, List<HybridAStarNode> closed_, HybridAStarNode best, double cost, List<(List<double>, double, int)> d_route, int n=10)
        {
            // Search best final shot in open set.

            open_ = open_.OrderBy(x => x.f).ToList();

            int min_Btwn_b_and_open_ = Math.Min(n, open_.Count);
            for (int t=0; t < min_Btwn_b_and_open_; t++)
            {
                var best_ = open_[t];
                var solutions_ = this.dubins.find_tangents(best_.pos, this.goal);
                var d_route_cost_valid_ = this.dubins.best_tangent(solutions_);

                var d_route_ = d_route_cost_valid_.Item1;
                var cost_ = d_route_cost_valid_.Item2;
                var valid_ = d_route_cost_valid_.Item3;

                if (valid_ && cost_ + best_.g_ < cost + best.g_)
                {
                    best = best_;
                    cost = cost_;
                    d_route = d_route_;
                }
            }

            if (open_.Contains(best))
            {
                open_.Remove(best);
                closed_.Add(best);
            }

            return (best, cost, d_route);
        }

        private List<(List<double>,double,int)> backtracking(HybridAStarNode node)
        {
            // Backtracking the path.

            var route = new List<(List<double>, double, int)>();
            while (node.parent != null)
            {
                route.Add((node.pos, node.phi, (int)Math.Round(node.m)));
                node = node.parent;
            }

            route.Reverse();
            return route;
        }

        public (List<CarState>, List<HybridAStarNode>) search_path(int heu= 1, bool extra= false)
        {
            // Hybrid A* pathfinding.

            HybridAStarNode root = this.construct_node(this.start);
            root.g = 0;
            root.g_ = 0;

            if (heu == 0)
            {
                root.f = root.g + this.simple_heuristic(root.pos);
            }
            if (heu == 1)
            {
                root.f = root.g + this.astar_heuristic(root.pos);
            }

            var closed_ = new List<HybridAStarNode>();
            var open_ = new List<HybridAStarNode> { root };

            int count = 0;
            while (open_ != null && count < numberOfIterations)
            {
                count += 1;

                var best = Extensions.MinBy(open_, x => x.f);

                open_.Remove(best);
                closed_.Add(best);

                // check dubins path
                if (count % this.check_dubins == 0)
                {
                    var solutions = this.dubins.find_tangents(best.pos, this.goal);
                    var d_route_cost_valid = this.dubins.best_tangent(solutions);
                    var d_route = d_route_cost_valid.Item1;
                    var cost = d_route_cost_valid.Item2;
                    var valid = d_route_cost_valid.Item3;

                    if (valid)
                    {
                        var best_cost_d_route = this.best_final_shot(open_, closed_, best, cost, d_route);
                        best = best_cost_d_route.Item1;
                        cost = best_cost_d_route.Item2;
                        d_route = best_cost_d_route.Item3;
                        var route = this.backtracking(best);
                        route.AddRange(d_route);
                        var path = this.car.get_path(this.start, route);
                        cost += best.g_;
                        Debug.Log("Shortest path cost: " + cost);
                        Debug.Log("Total iterations: " +  count);

                        return (path, closed_);
                    }
                }

                var children = this.get_children(best, heu, extra);

                foreach (var child_branch in children)
                {
                    var child = child_branch.Item1;
                    var branch = child_branch.Item2;

                    if (closed_.Contains(child))
                    {
                        continue;
                    }

                    if (!open_.Contains(child))
                    {
                        best.branches.Add(branch);
                        open_.Add(child);
                    }

                    else if (child.g < open_[open_.IndexOf(child)].g)
                    {
                        best.branches.Add(branch);

                        var c = open_[open_.IndexOf(child)];
                        var p = c.parent;
                        foreach (var b in p.branches)
                        {
                            var c_pos = new List<double> { c.pos[0], c.pos[1] };
                            if (Utils.same_point(b[b.Count-1].Item2, c_pos))
                            {
                                p.branches.Remove(b);
                                break;
                            }
                        }

                        open_.Remove(child);
                        open_.Add(child);
                    }
                }
            }

            return (null, null);
        }

    }

}