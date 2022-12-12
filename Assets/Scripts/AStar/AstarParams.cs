using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace astar
{
    public class AstarParams
    {
        // Store the computed costs.

        public List<int> cell_id;
        public double g;

        public AstarParams(List<int> cell_id, double g)
        {
            this.cell_id = cell_id;
            this.g = g;
        }
    }
}