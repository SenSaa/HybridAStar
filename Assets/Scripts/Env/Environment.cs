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

        ///public Environment(List<List<double>> obs = null, double lx = 10, double ly = 10)
        public Environment(List<List<double>> obs = null, double lx = 50, double ly = 50)
        ////public Environment(List<List<double>> obs = null, double lx = 500, double ly = 500)
        {
            this.lx = (float)lx;
            this.ly = (float)ly;

            //Debug.Log(obs.Count);

            if (obs != null || obs.Count > 0)
            {
                foreach(var ob in obs)
                {
                    //Debug.Log(ob.Count);
                    this.obs.Add(new Obstacle(ob[0], ob[1], ob[2], ob[3]));
                    //this.obs = new List<List<double>>() { new List<double>() { ob[0], ob[1], ob[2], ob[3] } };
                }
            }
            else
            {
                this.obs = new List<Obstacle>();
                //this.obs = new List<List<double>>();
            }
        }

        // Check rectangle target within the map bounds.
        public bool rectangle_inbounds(List<List<double>> rect, double safe_dis= 0.05)
        {
            // Debug
            for (int i=0; i < rect.Count; i++)
            {
                for (int j = 0; j < rect[i].Count; j++)
                {
                    //Debug.Log("rect_" + i + "_" + j + "_ : " + rect[i][j]);
                }
            }

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
            //Debug.Log("True");
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
            //Debug.Log("Environment - rectangle_obstacle_free");
            //Debug.Log(obs.Count);
            foreach (var ob in this.obs)
            {
                //Debug.Log("Environment - rectangle_obstacle_free");
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
                //Debug.Log("Env - rectangle_safe - True");
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