using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using utils;

namespace env
{
    public class SimpleCar
    {

        public Environment env;
        public List<double> start_pos, end_pos;
        public double l, max_phi;
        public double carl, carw, whll, whlw;
        public List<double> c1, c2, c3, c4;
        public List<double> w1, w2, w3, w4;

        public SimpleCar(Environment env, List<double> start_pos, List<double> end_pos, double l = 0.5, double max_phi = Math.PI / 4)
        {
            this.env = env;
            this.l = (float)l;
            this.max_phi = max_phi;

            this.carl = 1.6 * this.l;
            this.carw = 0.8 * this.l;

            this.whll = 0.4 * this.l;
            this.whlw = 0.2 * this.l;

            if (start_pos != null || start_pos.Count > 0) { this.start_pos = start_pos; }
            else { this.start_pos = this.random_pos(); }
            if (end_pos != null || end_pos.Count > 0) { this.end_pos = end_pos; }
            else { this.end_pos = this.random_pos(); }
        }

        //  Generate a random pos.
        public List<double> random_pos()
        {
            var pos = new List<double>();

            while (true)
            {
                var x = Utils.RandomNumber(-(this.env.lx * 0.9), (this.env.lx * 0.9));
                var y = Utils.RandomNumber(-(this.env.ly * 0.9), (this.env.ly * 0.9));
                var theta = Utils.RandomNumber(-Math.PI, Math.PI);
                pos = new List<double> { x, y, theta };
                var safe = this.is_pos_safe(pos);

                if (safe)
                {
                    break;
                }
            }

            return pos;
        }

        // Get parameters for turning route.
        public (int, List<double>, double) get_params(List<double> pos, double phi)
        {
            var x = pos[0];
            var y = pos[1];
            var theta = pos[2];

            var r = this.l / Math.Abs(Math.Tan(phi));
            var d = phi > 0 ? 1 : -1;
            var id = phi > 0 ? 1 : 2;

            var c = Utils._transform(x, y, 0, r, theta, id);

            return (d, c, r);
        }

        // Get the bounding rectangle of car.
        public List<List<double>> get_car_bounding(List<double> pos)
        {
            var x = pos[0];
            var y = pos[1];
            var theta = pos[2];

            this.c1 = Utils._transform(x, y, 1.3 * this.l, 0.4 * this.l, theta, 1);
            this.c2 = Utils._transform(x, y, 1.3 * this.l, 0.4 * this.l, theta, 2);
            this.c3 = Utils._transform(x, y, 0.3 * this.l, 0.4 * this.l, theta, 3);
            this.c4 = Utils._transform(x, y, 0.3 * this.l, 0.4 * this.l, theta, 4);

            var vertex = new List<List<double>> { this.c1, this.c2, this.c4, this.c3 };

            return vertex;
        }


        // Get the car state according to the pos and steering angle.
        public CarState get_car_state(List<double> pos, double phi= 0)
        {
            var model = new List<(List<double>, double, double, double)>();

            try
            {

                var x = pos[0];
                var y = pos[1];
                var theta = pos[2];

                pos = new List<double> { x, y, theta };
                this.get_car_bounding(pos);

                var c_ = Utils._transform(x, y, this.l, 0.2 * this.l, theta, 1);
                this.w1 = Utils._transform(c_[0], c_[1], 0.2 * this.l, 0.1 * this.l, theta + phi, 4);


                c_ = Utils._transform(x, y, this.l, 0.2 * this.l, theta, 2);
                this.w2 = Utils._transform(c_[0], c_[1], 0.2 * this.l, 0.1 * this.l, theta + phi, 4);


                this.w3 = Utils._transform(x, y, 0.2 * this.l, 0.1 * this.l, theta, 3);
                this.w4 = Utils._transform(x, y, 0.2 * this.l, 0.3 * this.l, theta, 4);

                model = new List<(List<double>, double, double, double)>
                {
                    (this.c4, this.carl, this.carw, (theta)),
                    (this.w1, this.whll, this.whlw, (theta + phi)),
                    (this.w2, this.whll, this.whlw, (theta + phi)),
                    (this.w3, this.whll, this.whlw, (theta)),
                    (this.w4, this.whll, this.whlw, (theta))
                };

            }
            catch (Exception e) { Debug.LogWarning(e); }


            var state = new CarState(pos, model);

            return state;
        }

        // Car dynamics.
        public List<double> step(List<double> pos, double phi, double m = 1, double dt= 1e-2)
        {
            var x = pos[0];
            var y = pos[1];
            var theta = pos[2];
            
            var dx = Math.Cos(theta);
            var dy = Math.Sin(theta);

            var dtheta = Math.Tan(phi) / this.l;

            x += m * dt * dx;
            y += m * dt * dy;
            theta += m * dt * dtheta;

            return new List<double> { x, y, theta };
        }

        // Check pos safety.
        public bool is_pos_safe(List<double> pos)
        {
            var vertex = this.get_car_bounding(pos);
            return this.env.rectangle_safe(vertex);
        }

        // Generate path according to route.
        public List<CarState> get_path(List<double> pos, List<(List<double>, double, int)> route)
        {
            var path = new List<CarState>();
            CarState car_state = null;
            double phi = 0.0;

            foreach (var goal_phi_ml in route)
            {
                var goal = goal_phi_ml.Item1;
                phi = goal_phi_ml.Item2;
                var ml = goal_phi_ml.Item3;
                while (true)
                {
                    car_state = this.get_car_state(pos, phi);
                    path.Add(car_state);
                    
                    pos = this.step(pos, phi, ml);

                    if (Utils.same_point(pos, goal))
                    {
                        pos = goal;
                        break;
                    }
                }
            }

            car_state = this.get_car_state(pos, phi);
            path.Add(car_state);

            return path;
        }

    }
}
