using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using utils;

namespace env
{
    public class Environment
    {

        public double lx, ly;
        public List<Obstacle> obs = new List<Obstacle>();

        public Environment(List<List<double>> obs = null, double lx = 50, double ly = 50)
        {
            this.lx = (float)lx;
            this.ly = (float)ly;


            if (obs != null || obs.Count > 0)
            {
                foreach(var ob in obs)
                {
                    this.obs.Add(new Obstacle(ob[0], ob[1], ob[2], ob[3]));
                }
            }
            else
            {
                this.obs = new List<Obstacle>();
            }
        }

        // Check rectangle target within the map bounds.
        public bool rectangle_inbounds(List<List<double>> rect, double safe_dis= 0.05)
        {
            foreach (var v in rect)
            {
                if (v[0] < safe_dis)
                {
                    return false;
                }
                if (v[0] > this.lx - safe_dis)
                {
                    return false;
                }
                if (v[1] < safe_dis)
                {
                    return false;
                }
                if (v[1] > this.ly - safe_dis)
                {
                    return false;
                }
            }
            return true;
        }

        // Check ringsector target within the map bounds.
        public bool ringsector_inbounds(List<double> rs, double safe_dis= 0.05)
        {
            var rect = new List<List<double>> {
                new List<double> {0 + safe_dis, 0 + safe_dis},
                new List<double> {this.lx-safe_dis,  0 + safe_dis},
                new List<double> { this.lx-safe_dis, this.ly - safe_dis},
                new List<double> {0 + safe_dis, this.ly - safe_dis}
                };
        
            return !Intersection.rectangle_ringsector_intersected(rect, rs, false);
        }

        // Check rectangle target is obstacle-free or not.
        public bool rectangle_obstacle_free(List<List<double>> rect)
        {
            foreach (var ob in this.obs)
            {
                if (!ob.rectangle_safe(rect))
                {
                    return false;
                }
            }
            return true;
        }

        // Check ringsector target is obstacle-free or not.
        private bool ringsector_obstacle_free(List<double> rs)
        {

            foreach (var ob in this.obs)
            {
                if (!ob.ringsector_safe(rs))
                {
                    return false;
                }
            }

            return true;
        }

        // Check rectangle target is safe or not.
        public bool rectangle_safe(List<List<double>> rect)
        {
            if (this.rectangle_inbounds(rect) && this.rectangle_obstacle_free(rect))
            {
                return true;
            }
            return false;
        }

        // Check ringsector target is safe or not.
        public bool ringsector_safe(List<double> rs)
        {
            if (this.ringsector_inbounds(rs) && this.ringsector_obstacle_free(rs))
            {
                return true;
            }
            return false;
        }

    }
}