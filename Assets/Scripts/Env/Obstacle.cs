using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using utils;

namespace env
{
    public class Obstacle
    {

        public double x, y, w, h, safe_dis;
        public List<List<double>> obs;

        // Rectangle obstacles

        public Obstacle(double x, double y, double w, double h, double safe_dis= 0.1)
        {
            this.x = (float)x;
            this.y = (float)y;
            this.w = (float)w;
            this.h = (float)h;

            this.safe_dis = safe_dis;

            
            this.obs = new List<List<double>>() {
                new List<double> { this.x - safe_dis, this.y - safe_dis },
                new List<double> { this.x + this.w + safe_dis, this.y - safe_dis },
                new List<double> { this.x + this.w + safe_dis, this.y + this.h + safe_dis },
                new List<double> { this.x - safe_dis, this.y + this.h + safe_dis }
            };            
        }

        // Check a rectangle object is intersected with an obstacle or not.
        public bool rectangle_safe(List<List<double>> rect)
        {            
            return !Intersection.polygons_overlapping(obs, rect);            
        }

        // Check a ringsector object is intersected with an obstacle or not.
        public bool ringsector_safe(List<double> rs)
        {
            return !Intersection.rectangle_ringsector_intersected(obs, rs, false);            
        }

    }
}
