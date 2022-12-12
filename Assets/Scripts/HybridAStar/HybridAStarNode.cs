using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace hybridAStar
{
    public class HybridAStarNode
    {
        // Hybrid A* tree node.

        public List<double> grid_pos;
        public List<double> pos;
        public double g;
        public double g_;
        public double f;
        public HybridAStarNode parent;
        public double phi;
        public double m;
        public List<List<(double, List<double>)>> branches;

        public HybridAStarNode(List<double> grid_pos, List<double> pos)
        {
            this.grid_pos = grid_pos;
            this.pos = pos;
            this.g = double.PositiveInfinity;
            this.g_ = double.PositiveInfinity;
            this.f = double.PositiveInfinity;
            this.parent = null;
            this.phi = 0;
            this.m = 0;
            this.branches = new List<List<(double, List<double>)>>();
        }
    }


}
