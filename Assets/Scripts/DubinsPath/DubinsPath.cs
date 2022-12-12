using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using env;
using utils;
using System.Linq;

namespace Dubins
{

    //     Consider four dubins paths
    //     - LSL
    //     - LSR
    //     - RSL
    //     - RSR
    //     and find the shortest obstacle-free one.

    public class DubinsPath
    {

        public SimpleCar car;
        public double r;
        public Dictionary<string, List<int>> direction;
        ///public (double, double, double) start_pos, end_pos;
        public List<double> start_pos, end_pos;
        public (double, double) s, e;
        public List<double> lc1, rc1, lc2, rc2;

        public DubinsPath(SimpleCar car)
        {
            this.car = car;
            r = this.car.l / Math.Tan(this.car.max_phi);

            // turn left: 1, turn right: -1
            /*
            this.direction = new Dictionary<string, (int, int)>();
            direction["LSL"] = (1, 1);
            direction["LSR"] = (1, -1);
            direction["RSL"] = (-1, 1);
            direction["RSR"] = (-1, -1);
            */
            this.direction = new Dictionary<string, List<int>> {
                { "LSL", new List<int> { 1, 1 }   },
                { "LSR", new List<int> { 1, -1 }  },
                { "RSL", new List<int> { -1, 1 }  },                
                { "RSR", new List<int> { -1, -1 } },
            };
        }

        //  Find the tangents of four dubins paths. 
        ///public List<DubinsPathParams> find_tangents((double, double, double) start_pos, (double, double, double) end_pos)
        public List<DubinsPathParams> find_tangents(List<double> start_pos, List<double> end_pos)
        {
            this.start_pos = start_pos;
            this.end_pos = end_pos;

            /*
            var x1 = start_pos.Item1;
            var y1 = start_pos.Item2;
            var theta1 = start_pos.Item3;
            var x2 = end_pos.Item1;
            var y2 = end_pos.Item2;
            var theta2 = end_pos.Item3;
            */
            var x1 = start_pos[0];
            var y1 = start_pos[1];
            var theta1 = start_pos[2];
            var x2 = end_pos[0];
            var y2 = end_pos[1];
            var theta2 = end_pos[2];

            //self.s = np.array(start_pos[:2])
            //self.e = np.array(end_pos[:2])
            /*
            this.s = (start_pos.Item1, start_pos.Item2);
            this.e = (end_pos.Item1, end_pos.Item2);
            */
            this.s = (start_pos[0], start_pos[1]);
            this.e = (end_pos[0], end_pos[1]);

            this.lc1 = Utils._transform(x1, y1, 0, this.r, theta1, 1);
            this.rc1 = Utils._transform(x1, y1, 0, this.r, theta1, 2);
            this.lc2 = Utils._transform(x2, y2, 0, this.r, theta2, 1);
            this.rc2 = Utils._transform(x2, y2, 0, this.r, theta2, 2);

            //Debug.Log("x1_y1_theta_x2_y2_theta2_r: " + x1 + " , " + y1 + " , " + theta1 + " , " + x2 + " , " + y2 + " , " + theta2 + " , " + this.r);
            //Debug.Log("lc1: " + Utils.ListToString(lc1));
            //Debug.Log("rc1: " + Utils.ListToString(rc1));
            //Debug.Log("lc2: " + Utils.ListToString(lc2));
            //Debug.Log("rc2: " + Utils.ListToString(rc2));
            

            //solutions = [self._LSL(), self._LSR(), self._RSL(), self._RSR()]
            var solutions = new List<DubinsPathParams> {
                    this._LSL(),
                    this._LSR(),
                    this._RSL(),
                    this._RSR()
            };
            //solutions = [s for s in solutions if s is not None]
            solutions = (from s in solutions
                         where s != null // *
                         select s).ToList();
            //solutions.sort(key: x => x.len, reverse: false);
            solutions = solutions.OrderBy(x => x.len).ToList();

            return solutions;
        }

        //  Calculate the dubins path length. 
        public DubinsPathParams get_params(
            DubinsPathParams dub,
            List<double> c1,
            List<double> c2,
            List<double> t1,
            List<double> t2)
        {
            /*
            var v1 = this.s - c1;
            var v2 = t1 - c1;
            var v3 = t2 - t1;
            var v4 = t2 - c2;
            var v5 = this.e - c2;
            */
            var v1 = Utils.substractTuples(this.s, c1);
            var v2 = Utils.substractTuples(t1,c1);
            var v3 = Utils.substractTuples(t2, t1);
            var v4 = Utils.substractTuples(t2, c2);
            var v5 = Utils.substractTuples(this.e, c2);

            var delta_theta1 = Utils.directional_theta(v1, v2, dub.d[0]);
            var delta_theta2 = Utils.directional_theta(v4, v5, dub.d[1]);

            var arc1 = Math.Abs(delta_theta1 * this.r);
            //var tangent = np.linalg.norm(v3);
            var tangent = Math.Sqrt(Math.Pow(v3.Item1,2) + Math.Pow(v3.Item2, 2));
            var arc2 = Math.Abs(delta_theta2 * this.r);

            ///var theta = this.start_pos.Item3 + delta_theta1;
            var theta = this.start_pos[2] + delta_theta1;

            t1.Add(theta);
            dub.t1 = t1;
            t2.Add(theta);
            dub.t2 = t2;
            dub.c1 = c1;
            dub.c2 = c2;
            dub.len = arc1 + tangent + arc2;

            return dub;
        }

        public DubinsPathParams _LSL()
        {
            var lsl = new DubinsPathParams(this.direction["LSL"]);
            //var cline = this.lc2 - this.lc1;
            var cline = Utils.substractTuples(this.lc2, this.lc1);
            //var R = np.linalg.norm(cline) / 2;
            var R = (Math.Sqrt(Math.Pow(cline.Item1, 2) + Math.Pow(cline.Item2, 2))) / 2;
            
            var theta = Math.Atan2(cline.Item2, cline.Item1) - Math.Acos(0);
            /*
            var theta = Math.Atan2(cline.Item2, cline.Item1) - Math.Asin(0);
            */
            var t1 = Utils._transform(this.lc1[0], this.lc1[1], this.r, 0, theta, 1);
            var t2 = Utils._transform(this.lc2[0], this.lc2[1], this.r, 0, theta, 1);
            lsl = this.get_params(lsl, this.lc1, this.lc2, t1, t2);
            return lsl;
        }

        public DubinsPathParams _LSR()
        {
            var lsr = new DubinsPathParams(this.direction["LSR"]);
            //var cline = this.rc2 - this.lc1;
            var cline = Utils.substractTuples(this.rc2, this.lc1);
            //var R = np.linalg.norm(cline) / 2;
            var R = (Math.Sqrt(Math.Pow(cline.Item1, 2) + Math.Pow(cline.Item2, 2))) / 2;
            if (R < this.r)
            {
                return null;
            }
            
            var theta = Math.Atan2(cline.Item2, cline.Item1) - Math.Acos(this.r / R);
            /*
            var theta = Math.Atan2(cline.Item2, cline.Item1) - Math.Asin(this.r / R);
            */

            var t1 = Utils._transform(this.lc1[0], this.lc1[1], this.r, 0, theta, 1);
            var t2 = Utils._transform(this.rc2[0], this.rc2[1], this.r, 0, theta + Math.PI, 1);
            lsr = this.get_params(lsr, this.lc1, this.rc2, t1, t2);
            return lsr;
        }

        public DubinsPathParams _RSL()
        {
            var rsl = new DubinsPathParams(this.direction["RSL"]);
            //var cline = this.lc2 - this.rc1;
            var cline = Utils.substractTuples(this.lc2, this.rc1);
            //var R = np.linalg.norm(cline) / 2;
            var R = (Math.Sqrt(Math.Pow(cline.Item1, 2) + Math.Pow(cline.Item2, 2))) / 2;
            if (R < this.r)
            {
                return null;
            }
            
            var theta = Math.Atan2(cline.Item2, cline.Item1) + Math.Acos(this.r / R);
            /*
            var theta = Math.Atan2(cline.Item2, cline.Item1) + Math.Asin(this.r / R);
            */

            var t1 = Utils._transform(this.rc1[0], this.rc1[1], this.r, 0, theta, 1);
            var t2 = Utils._transform(this.lc2[0], this.lc2[1], this.r, 0, theta + Math.PI, 1);
            rsl = this.get_params(rsl, this.rc1, this.lc2, t1, t2);
            return rsl;
        }

        public DubinsPathParams _RSR()
        {
            var rsr = new DubinsPathParams(this.direction["RSR"]);
            //var cline = this.rc2 - this.rc1;
            var cline = Utils.substractTuples(this.rc2, this.rc1);
            //var R = np.linalg.norm(cline) / 2;
            var R = (Math.Sqrt(Math.Pow(cline.Item1, 2) + Math.Pow(cline.Item2, 2))) / 2;
            
            var theta = Math.Atan2(cline.Item2, cline.Item1) + Math.Acos(0);
            /*
            var theta = Math.Atan2(cline.Item2, cline.Item1) + Math.Asin(0);
            */

            var t1 = Utils._transform(this.rc1[0], this.rc1[1], this.r, 0, theta, 1);
            var t2 = Utils._transform(this.rc2[0], this.rc2[1], this.r, 0, theta, 1);
            rsr = this.get_params(rsr, this.rc1, this.rc2, t1, t2);
            return rsr;
        }

        //  Get the shortest obstacle-free dubins path. 
        List<(List<double>, double, int)> route = new List<(List<double>, double, int)>();
        public (List<(List<double>, double, int)>, double, bool) best_tangent(List<DubinsPathParams> solutions)
        {
            var s_len = 0.0;
            var safe = false;

            var pos0 = this.start_pos;
            var pos1 = this.end_pos;

            if (solutions == null || solutions.Count == 0)
            {
                return (null, 0.0, false);
            }

            foreach (var s in solutions)
            {
                s_len = s.len;

                route = this.get_route(s);
                
                safe = this.is_straight_route_safe(s.t1, s.t2);
                //Debug.Log("t1_t2: " + Utils.ListToString(s.t1) + " , " + Utils.ListToString(s.t2));
                //Debug.Log("straight_route safe: " + safe);
                if (!safe)
                {
                    //Debug.Log("straight_route not safe");
                    continue;
                }
                
                safe = this.is_turning_route_safe(pos0, s.t1, s.d[0], s.c1, this.r);
                //Debug.Log("pos0_t1_d[0]_c1_r: " + Utils.ListToString(pos0) + " , " + Utils.ListToString(s.t1) + " , " + s.d[0] + " , " + Utils.ListToString(s.c1) + " , " + this.r);
                //Debug.Log("turning_route safe: " + safe);
                if (!safe)
                {
                    //Debug.Log("turning_route1 not safe");
                    continue;
                }
                
                safe = this.is_turning_route_safe(s.t2, pos1, s.d[1], s.c2, this.r);
                //Debug.Log("t2_pos1_d[1]_c2_r: " + Utils.ListToString(s.t2) + " , " + Utils.ListToString(pos1) + " , " + s.d[1] + " , " + Utils.ListToString(s.c2) + " , " + this.r);
                //Debug.Log("turning_route2 safe: " + safe);
                if (!safe)
                {
                    //Debug.Log("turning_route2 not safe");
                    continue;
                }
                
                if (safe)
                {
                    break;
                }
            }

            /*
            for(int i=0; i < route.Count; i++)
            {
                Debug.Log(route[i].Item1[0] + " " + route[i].Item1[1] + " " + route[i].Item1[2]);
                Debug.Log(route[i].Item2);
                Debug.Log(route[i].Item3);
            }
            */
            //Debug.Log("safe: " + safe);
            
            return (route, s_len, safe);
        }

        //  Check a straight route is safe. 
        public bool is_straight_route_safe(List<double> t1, List<double> t2)
        {
            // a straight route is simply a rectangle
            var vertex1 = this.car.get_car_bounding(t1);
            var vertex2 = this.car.get_car_bounding(t2);

            // Debug
            /*
            for (int i = 0; i < vertex1.Count; i++)
            {
                for (int j = 0; j < vertex1[i].Count; j++)
                {
                    Debug.Log("vertex1_" + i + "_" + j + "_ : " + vertex1[i][j]);
                }
            }
            */

            /*
            for(int i = 0; i < vertex1.Count; i++)
            {
                for (int j = 0; j < vertex1[i].Count; j++)
                {
                    Debug.Log(vertex1[i][j]);
                }
            }
            */

            var vertex = new List<List<double>> {
                    vertex2[0],
                    vertex2[1],
                    vertex1[3],
                    vertex1[2]
                };
            return this.car.env.rectangle_safe(vertex);
        }

        //  Check if a turning route is safe. 
        public bool is_turning_route_safe(
            List<double> start_pos,
            List<double> end_pos,
            int d,
            List<double> c,
            double r)
        {
            // a turning_route is decomposed into:
            //   1. start_pos (checked previously as end_pos)
            //   2. end_pos
            //   3. inner ringsector
            //   4. outer ringsector
            if (!this.car.is_pos_safe(end_pos))
            {
                //Debug.Log("!this.car.is_pos_safe(end_pos)");
                return false;
            }
            var _tup_1 = this.construct_ringsectors(start_pos, end_pos, d, c, r);
            var rs_inner = _tup_1.Item1;
            var rs_outer = _tup_1.Item2;
            //Debug.Log("rs_inner,rs_outer: " + Utils.ListToString(rs_inner) + " , " + Utils.ListToString(rs_outer));
            if (!this.car.env.ringsector_safe(rs_inner))
            {
                //Debug.Log("!this.car.env.ringsector_safe(rs_inner)");
                return false;
            }
            if (!this.car.env.ringsector_safe(rs_outer))
            {
                //Debug.Log("!this.car.env.ringsector_safe(rs_inner)");
                return false;
            }
            return true;
        }

        //  Construct inner and outer ringsectors of a turning route. 
        public (List<double>, List<double>) construct_ringsectors(
            List<double> start_pos,
            List<double> end_pos,
            int d,
            List<double> c,
            double r)
        {
            double start_outer = 0;
            double end_outer = 0;
            double start_inner = 0;
            double end_inner = 0;
            var _tup_1 = start_pos;
            var x = _tup_1[0];
            var y = _tup_1[1];
            var theta = _tup_1[2];
            var delta_theta = end_pos[2] - theta;
            //Debug.Log("delta_theta = " + delta_theta);
            //var p_inner = start_pos[:2];
            var p_inner = new List<double>() { start_pos[0], start_pos[1] };
            this.s = (start_pos[0], start_pos[1]);
            this.e = (end_pos[0], end_pos[1]);
            var id = d == -1 ? 1 : 2;
            //Debug.Log("id = " + id);
            var p_outer = Utils._transform(x, y, 1.3 * this.car.l, 0.4 * this.car.l, theta, id);
            //Debug.Log("p_outer = " + Utils.ListToString(p_outer));
            var r_inner = r - this.car.carw / 2;
            //Debug.Log("r_inner = " + r_inner);
            var r_outer = Intersection.distance(p_outer, c);
            //Debug.Log("r_outer = " + r_outer);
            var v_inner = new List<double> {
                    p_inner[0] - c[0],
                    p_inner[1] - c[1]
                };
            //Debug.Log("v_inner = " + Utils.ListToString(v_inner));
            var v_outer = new List<double> {
                    p_outer[0] - c[0],
                    p_outer[1] - c[1]
                };
            //Debug.Log("v_outer = " + Utils.ListToString(v_outer));
            if (d == -1)
            {
                /*
                end_inner = Math.Atan2(v_inner[1], v_inner[0]) % (2 * Math.PI);
                start_inner = (end_inner + delta_theta) % (2 * Math.PI);
                end_outer = Math.Atan2(v_outer[1], v_outer[0]) % (2 * Math.PI);
                start_outer = (end_outer + delta_theta) % (2 * Math.PI);
                */
                end_inner = Utils.Modulus(Math.Atan2(v_inner[1], v_inner[0]), (2 * Math.PI));
                start_inner = Utils.Modulus((end_inner + delta_theta), (2 * Math.PI));
                end_outer = Utils.Modulus(Math.Atan2(v_outer[1], v_outer[0]), (2 * Math.PI));
                start_outer = Utils.Modulus((end_outer + delta_theta), (2 * Math.PI));
            }
            if (d == 1)
            {
                /*
                start_inner = Math.Atan2(v_inner[1], v_inner[0]) % (2 * Math.PI);
                end_inner = (start_inner + delta_theta) % (2 * Math.PI);
                start_outer = Math.Atan2(v_outer[1], v_outer[0]) % (2 * Math.PI);
                end_outer = (start_outer + delta_theta) % (2 * Math.PI);
                */
                start_inner = Utils.Modulus(Math.Atan2(v_inner[1], v_inner[0]), (2 * Math.PI));
                end_inner = Utils.Modulus((start_inner + delta_theta), (2 * Math.PI));
                start_outer = Utils.Modulus(Math.Atan2(v_outer[1], v_outer[0]), (2 * Math.PI));
                end_outer = Utils.Modulus((start_outer + delta_theta), (2 * Math.PI));
            }
            //Debug.Log(Utils.Modulus(Math.Atan2(v_inner[1], v_inner[0]), (2 * Math.PI)));
            //Debug.Log("start_inner = " + start_inner);
            //Debug.Log("end_inner = " + end_inner);
            //Debug.Log("start_outer = " + start_outer);
            //Debug.Log("end_outer = " + end_outer);
            var rs_inner = new List<double> {
                    c[0],
                    c[1],
                    r_inner,
                    r,
                    start_inner,
                    end_inner
                };
            var rs_outer = new List<double> {
                    c[0],
                    c[1],
                    r,
                    r_outer,
                    start_outer,
                    end_outer
                };
            return (rs_inner, rs_outer);
        }

        //  Get the route of dubins path. 
        public List<(List<double> , double, int)> get_route(DubinsPathParams s)
        {
            var phi1 = s.d[0] == 1 ? this.car.max_phi : -this.car.max_phi;
            var phi2 = s.d[1] == 1 ? this.car.max_phi : -this.car.max_phi;
            var phil = new List<double> {
                    phi1,
                    0,
                    phi2
                };
            var goal = new List<List<double>> {
                    s.t1,
                    s.t2,
                    this.end_pos
                };
            var ml = new List<int> {
                    1,
                    1,
                    1
                };

            //return zip(goal, phil, ml).ToList();
            ///return goal.ZipThree(phil, ml, (a, b, c) => new { a, b, c });
            //var zipped_goal_phil = goal.Zip(phil, (a, b) => new { a, b});
            //var zipped_goal_phil_ml = zipped_goal_phil.Zip(ml, (a, b) => new { a, b });
            //return zipped_goal_phil_ml;

            /*
            var zipped_goal_phil_ml = goal.Zip(phil, (a, b) => new
            {
                goal = a,
                phil = b,
            })
            .Zip(ml, (a, b) => new
            {
                //use the properties from a & b to construct your desired result
                interp_dists = a.goal,
                mode = a.phil,
                ml = b,
            });

            return zipped_goal_phil_ml;
            */

            /*
            var goal_phil_ml_List = new List<(List<List<double>> , List<double>, List<int>)>();
            var goal_phil_ml_ = (goal, phil, ml);
            goal_phil_ml_List.Add(goal_phil_ml_);
            return goal_phil_ml_List;
            */

            var goal_phil_ml_List = new List<(List<double>, double, int)>();
            for(int i=0; i < phil.Count; i++)
            {
                var goal_phil_ml_ = (goal[i], phil[i], ml[i]);
                //Debug.Log(goal[i][0] + " " + goal[i][1] + " " + goal[i][2] + " " + phil[i] + " " + ml[i]);
                goal_phil_ml_List.Add(goal_phil_ml_);
            }
            return goal_phil_ml_List;

        }

    }
}