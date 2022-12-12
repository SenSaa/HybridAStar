using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace Dubins
{
    //  Store parameters for different dubins paths. 
    public class DubinsPathParams
    {

        public List<int> d;                // dubins type
        public List<double> t1;     // first tangent point
        public List<double> t2;     // second tangent point
        public List<double> c1;     // first center point
        public List<double> c2;     // second center point
        public double len;                 // total travel distance

    public DubinsPathParams(List<int> d)
        {
            this.d = d;                                                     // dubins type
            this.t1 = new List<double>();   // first tangent point
            this.t2 = new List<double>();   // second tangent point
            this.c1 = new List<double>();   // first center point
            this.c2 = new List<double>();   // second center point
            this.len = 0;                                                   // total travel distance
        }
        
    }

}
