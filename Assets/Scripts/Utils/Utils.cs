using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

namespace utils
{
    public class Utils
    {

        public static string ListToString<T>(List<T> list)
        {
            string result = "";
            for (int i = 0; i < list.Count; i++)
            {
                result += list[i] + "   ";
            }
            return result;
        }

        // --------------------------------------------------------------------------------------

        public static (double,double) substractTuples((double, double) lhs, (double, double) rhs)
        {
            return (lhs.Item1 - rhs.Item1, lhs.Item2 - rhs.Item2);
        }
        public static (double, double) substractTuples(List<double> lhs, (double, double) rhs)
        {
            return (lhs[0] - rhs.Item1, lhs[1] - rhs.Item2);
        }
        public static (double, double) substractTuples((double, double) lhs, List<double> rhs)
        {
            return (lhs.Item1 - rhs[0], lhs.Item2 - rhs[1]);
        }
        public static (double, double) substractTuples(List<double> lhs, List<double> rhs)
        {
            return (lhs[0] - rhs[0], lhs[1] - rhs[1]);
        }

        // --------------------------------------------------------------------------------------

        //public static (double,double) _transform(double x, double y, double w, double l, double theta, int id)
        public static List<double> _transform(double x, double y, double w, double l, double theta, int id)
        {
            double x_ = 0, y_ = 0;

            
            if (id == 1)
            {
                x_ = x + w * Math.Cos(theta) - l * Math.Sin(theta);
                y_ = y + w * Math.Sin(theta) + l * Math.Cos(theta);
            }
            if (id == 2)
            {
                x_ = x + w * Math.Cos(theta) + l * Math.Sin(theta);
                y_ = y + w * Math.Sin(theta) - l * Math.Cos(theta);
            }
            if (id == 3)
            {
                x_ = x - w * Math.Cos(theta) - l * Math.Sin(theta);
                y_ = y - w * Math.Sin(theta) + l * Math.Cos(theta);
            }
            if (id == 4)
            {
                x_ = x - w * Math.Cos(theta) + l * Math.Sin(theta);
                y_ = y - w * Math.Sin(theta) - l * Math.Cos(theta);
            }

            return new List<double> { x_, y_ };
        }

        public static double directional_theta((double, double) vec1, (double, double) vec2, int d)
        {
            // Calculate the directional theta change.

            var theta = Math.Atan2(vec2.Item2, vec2.Item1) - Math.Atan2(vec1.Item2, vec1.Item1);

            if (theta < 0 && d == 1)
            {
                theta += 2 * Math.PI;
            }
            else if (theta > 0 && d == -1)
            {
                theta -= 2 * Math.PI;
            }

            return theta;
        }

        // Distance of two points.
        public static double distance(List<double> pt1, List<double> pt2)
        {
            var d = Math.Sqrt(Math.Pow((pt1[0] - pt2[0]), 2) + Math.Pow((pt1[1] - pt2[1]), 2));

            return d;
        }

        // Check two points are same within a samll error.
        public static bool same_point(List<double> pt1, List<double> pt2, double h = 1e-2)
        {
            var d = distance(pt1, pt2);

            return d < h;
        }

        // Round theta to closest discretized value.
        public static double round_theta(double theta, List<double> thetas)
        {
            return Extensions.MinBy(thetas, x => Modulus(Math.Abs(x - theta), (2 * Math.PI)));
        }

        // Get all discretized theta values by unit value.
        public static List<double> get_discretized_thetas(double unit_theta)
        {
            var thetas = new List<double> { 0 };

            while (true)
            {
                var theta = thetas[thetas.Count-1] + unit_theta;
                if (theta > (2 * Math.PI - unit_theta))
                {
                    break;
                }

                thetas.Add(theta);
            }
            return thetas;
        }

        // --------------------------------------------------------------------------------------

        // Get a random number between 2 values.
        private static System.Random rand = new System.Random();
        public static double RandomNumber(double min, double max)
        {
            return rand.NextDouble() * (max - min) + min;
        }

        //  itertools.product C# equivalent
        public static List<Tuple<T, T>> Product<T>(List<T> a, List<T> b)
            where T : struct
        {
            List<Tuple<T, T>> result = new List<Tuple<T, T>>();

            foreach (T t1 in a)
            {
                foreach (T t2 in b)
                    result.Add(Tuple.Create<T, T>(t1, t2));
            }

            return result;
        }

        // --------------------------------------------------------------------------------------

        // C#'s "%" operator returns the remainder!! Use this instead for Modulus!
        public static double Modulus(double x, double m)
        {
            double r = x % m;
            return r < 0 ? r + m : r;
        }

        // --------------------------------------------------------------------------------------

        public static bool NodesListContainsNode(List<astar.AstarNode> nodes, astar.AstarNode node)
        {
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i].cell_id[0] == node.cell_id[0] && nodes[i].cell_id[1] == node.cell_id[1])
                {
                    return true;
                }
            }
            return false;
        }

        public static (bool, double) CostOfNodeInNodesList(List<astar.AstarNode> nodes, astar.AstarNode node)
        {
            for (int i = 0; i < nodes.Count; i++)
            {
                if (nodes[i].cell_id[0] == node.cell_id[0] && nodes[i].cell_id[1] == node.cell_id[1])
                {
                    return (true, node.g);
                }
            }
            return (false, double.PositiveInfinity);
        }

    }
}
