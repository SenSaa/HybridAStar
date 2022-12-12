using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace astar
{
    public class AstarNode
    {

        // Standard A* node.

        public List<int> cell_id;
        public double g;
        public double f;
        public AstarNode parent;

        public AstarNode(List<int> cell_id)
        {
            this.cell_id = cell_id;
            this.g = double.MaxValue;
            this.f = double.MaxValue;
            this.parent = null;
        }
    }
}