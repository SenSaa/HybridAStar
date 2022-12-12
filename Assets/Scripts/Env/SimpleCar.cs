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
        ///public (double, double) start_pos, end_pos;
        public List<double> start_pos, end_pos;
        public double l, max_phi;
        public double carl, carw, whll, whlw;
        public List<double> c1, c2, c3, c4;
        public List<double> w1, w2, w3, w4;

        ///public SimpleCar(Environment env, (int x, int y)? start_pos = null, (int x, int y)? end_pos = null, double l = 0.5, double max_phi = Math.PI / 5)
        //public SimpleCar(Environment env, List<double> start_pos = null, List<double>? end_pos = null, double l = 0.5, double max_phi = Math.PI / 5)
        public SimpleCar(Environment env, List<double> start_pos, List<double> end_pos, double l = 0.5, double max_phi = Math.PI / 4)
        {
            this.env = env;
            this.l = (float)l;
            this.max_phi = max_phi;

            this.carl = 1.6 * this.l;
            this.carw = 0.8 * this.l;

            this.whll = 0.4 * this.l;
            this.whlw = 0.2 * this.l;

            /*
            if (start_pos != null) { this.start_pos = start_pos.Value;  }
            else { this.start_pos = this.random_pos(); }
            if (end_pos != null) { this.end_pos = start_pos.Value; }
            else { this.end_pos = this.random_pos(); }
            */
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
                ///var x = Utils.RandomNumber((this.env.lx * 0.1), (this.env.lx * 0.9));
                ///var y = Utils.RandomNumber((this.env.ly * 0.1), (this.env.ly * 0.9));
                // ***
                var x = Utils.RandomNumber(-(this.env.lx * 0.9), (this.env.lx * 0.9));
                var y = Utils.RandomNumber(-(this.env.ly * 0.9), (this.env.ly * 0.9));
                var theta = Utils.RandomNumber(-Math.PI, Math.PI);
                /*
                var x = UnityEngine.Random.Range((float)(this.env.lx * 0.1), (float)(this.env.lx * 0.9));
                var y = UnityEngine.Random.Range((float)(this.env.ly * 0.1), (float)(this.env.ly * 0.9));
                var theta = UnityEngine.Random.Range((float)-Math.PI, (float)Math.PI);
                */
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
            //var d = 1 if phi > 0 else -1;
            var d = phi > 0 ? 1 : -1;
            //var id = 1 if phi > 0 else 2;
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

            //vertex = [self.c1.tolist(), self.c2.tolist(), self.c4.tolist(), self.c3.tolist()]
            var vertex = new List<List<double>> { this.c1, this.c2, this.c4, this.c3 };

            return vertex;
        }


        // Get the car state according to the pos and steering angle.
        public CarState get_car_state(List<double> pos, double phi= 0)
        {

            var model = new List<(List<double>, double, double, double)>();

            //Debug.Log(pos.Count);

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


                // !
                /*
                var model = [
                    Rectangle(self.c4, self.carl, self.carw, degrees(theta), fc = 'y', ec = 'k'),
                    Rectangle(self.w1, self.whll, self.whlw, degrees(theta + phi), fc = 'k', ec = 'k'),
                    Rectangle(self.w2, self.whll, self.whlw, degrees(theta + phi), fc = 'k', ec = 'k'),
                    Rectangle(self.w3, self.whll, self.whlw, degrees(theta), fc = 'k', ec = 'k'),
                    Rectangle(self.w4, self.whll, self.whlw, degrees(theta), fc = 'k', ec = 'k'),
                    Arrow(x, y, 1.1 * self.carl * cos(theta), 1.1 * self.carl * sin(theta), width = 0.1, color = 'r')
                ];
                */
                //var model = new List<(List<double>, double, double, double)>
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
            //Debug.Log(pos.Count);

            var x = pos[0];
            var y = pos[1];
            var theta = pos[2];
            
            var dx = Math.Cos(theta);
            var dy = Math.Sin(theta);
            
            /*
            var dx = Math.Sin(theta);
            var dy = Math.Cos(theta);
            */
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
        ////public List<CarState> get_path(List<double> pos, IEnumerable route)
        public List<CarState> get_path(List<double> pos, List<(List<double>, double, int)> route)
        {
            var path = new List<CarState>();
            CarState car_state = null;
            double phi = 0.0;

            foreach (var goal_phi_ml in route)
            {
                var goal = goal_phi_ml.Item1;
                //var phi = goal_phi_ml.Item2;
                phi = goal_phi_ml.Item2;
                var ml = goal_phi_ml.Item3;
                //Debug.Log(goal);
                //Debug.Log(phi);
                //Debug.Log(ml);
                while (true)
                {
                    car_state = this.get_car_state(pos, phi);
                    path.Add(car_state);

                    //Debug.Log(pos.Count);
                        
                    pos = this.step(pos, phi, ml);

                    //pos = new List<double>() { pos[0], pos[1], pos[2] };
                    //goal = new List<double>() { goal[0], goal[1], goal[2] };

                    if (Utils.same_point(pos, goal))
                    {
                        pos = goal;
                        break;
                    }

                    //Debug.Log("car_state pos = " + car_state.pos[0] + " " + car_state.pos[1] + " " + car_state.pos[2]);
                    //Debug.Log("step pos = " + pos[0] + " " + pos[1] + " " + pos[2]);

                }
            }

            car_state = this.get_car_state(pos, phi);
            path.Add(car_state);

            return path;
        }

    }
}
