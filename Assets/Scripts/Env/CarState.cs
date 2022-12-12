using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace env
{
    public class CarState
    {

        public List<double> pos;
        public List<(List<double>, double, double, double)> model;

        public CarState(List<double> pos, List<(List<double>, double, double, double)> model)
        {
            this.pos = pos;
            this.model = model;
        }

    }
}
