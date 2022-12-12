using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using env;
using System.Linq;

namespace utils
{

    // intersection: cross of edge
    // overlapping: overlap of area (could be no cross of edge, i.e. a contains b)

    public class Intersection
    {

        // Euclidean distance between two points.
        public static double distance(List<double> p1, List<double> p2)
        {
            return Math.Sqrt(Math.Pow((p1[0] - p2[0]), 2) + Math.Pow((p1[1] - p2[1]), 2));
        }

        // Two objects are separated by a line.
        public static bool separated(List<List<double>> line, List<List<double>> obj1, List<List<double>> obj2)
        {
            
            var p1 = line[0];
            var p2 = line[1];
            var normal = new List<double> { p2[1] - p1[1], p1[0] - p2[0] };

            var amin = 0.0;
            var amax = 0.0;
            foreach (var v in obj1)
            {
                var projected = normal[0] * v[0] + normal[1] * v[1];
                if ((amin == 0.0) || (projected < amin))
                {
                    amin = projected;
                }

                if ((amax == 0.0) || (projected > amax))
                {
                    amax = projected;
                }
            }

            var bmin = 0.0;
            var bmax = 0.0;
            foreach (var v in obj2)
            {
                var projected = normal[0] * v[0] + normal[1] * v[1];
                if ((bmin == 0.0) || (projected < bmin))
                {
                    bmin = projected;
                }

                if ((bmax == 0.0) || (projected > bmax))
                {
                    bmax = projected;
                }
            }

            if ((amax < bmin) || (bmax < amin))
            {
                return true;
            }

            return false;
        }

        // Check line segment intersected with a rectangle.
        public static bool line_rectangle_intersected(List<List<double>> line, List<List<double>> rect)
        {

            for (int i = 0; i < rect.Count; i++)
            {

                var j = (i + 1) % rect.Count;

                var edge = new List<List<double>> { rect[i], rect[j] };

                if (!(separated(edge, edge, line) || separated(line, edge, line)));
                {
                    return true;
                }
            }

            return false;
        }

        // Check line segment intersected with a circle.
        // https://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm
        public static (bool, List<List<double>>) line_circle_intersected(List<List<double>> line, List<double> circle)
        {
            var nodes = new List<List<double>>();

            var q_r = (new List<double> { circle[0], circle[1] }, circle[2]);
            var q = q_r.Item1;
            var r = q_r.Item2;
            var p1 = line[0];
            var p2 = line[1];

            var v1 = new List<double>() { p2[0] - p1[0], p2[1] - p1[1] };
            var v2 = new List<double>() { p1[0] - q[0], p1[1] - q[1] };

            var a = Math.Pow(v1[0], 2) + Math.Pow(v1[1], 2);
            var b = 2 * (v1[0] * v2[0] + v1[1] * v2[1]);
            var c = (Math.Pow(p1[0], 2) + Math.Pow(p1[1], 2)) + (Math.Pow(q[0], 2) + Math.Pow(q[1], 2)) - 2 * (p1[0] * q[0] + p1[1] * q[1]) - Math.Pow(r, 2);

            var disc = Math.Pow(b, 2) - 4 * a * c;

            if (disc < 0)
            {
                return (false, nodes);
            }

            var t1 = (-b + Math.Sqrt(disc)) / (2 * a);
            var t2 = (-b - Math.Sqrt(disc)) / (2 * a);

            if (!(0 <= t1 && t1 <= 1 || 0 <= t2 && t2 <= 1))
            {
                return (false, nodes);
            }


            if (0 <= t1 && t1 <= 1)
            {
                nodes.Add( new List<double>() { p1[0] + t1 * v1[0], p1[1] + t1 * v1[1]});
            }

            if (0 <= t2 && t2 <= 1)
            {
                nodes.Add(new List<double>() { p1[0] + t2 * v1[0], p1[1] + t2 * v1[1] });
            }
    
            return (true, nodes);
        }

        // Check polygons overlapping.
        public static bool polygons_overlapping(List<List<double>> polya, List<List<double>> polyb)
        {
            var polygons = new List<List<List<double>>> { polya, polyb };

            foreach (var polygon in polygons)
            {
                for (int i=0; i <polygon.Count; i++)
                {
                    var j = (i + 1) % (polygon.Count);
                    var edge = new List<List<double>> { polygon[i], polygon[j] };

                    if (separated(edge, polya, polyb))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        // Check rectangle intersected with an arc.
        public static bool rectangle_arc_intersected(List<List<double>> rect, List<double> arc)
        {
            // arc: [x, y, r, start, end]

            for (int i=0; i < rect.Count; i++)
            {

                var j = (i + 1) % rect.Count;

                var edge = new List<List<double>> { rect[i], rect[j] };

                var arc_3 = new List<double>() { arc[0], arc[1], arc[2] };
                var result_nodes = line_circle_intersected(edge, arc_3);
                var result = result_nodes.Item1;
                var nodes = result_nodes.Item2;

                if (result)
                {
                    foreach(var node in nodes)
                    {
                        var v = new List<double>() { node[0] - arc[0], node[1] - arc[1] };
                        var theta = Utils.Modulus((Math.Atan2(v[1], v[0])) ,(2 * Math.PI));

                        if (arc[3] < arc[4] && arc[3] <= theta && theta <= arc[4])
                        {
                            return true;
                        }


                        if (arc[3] > arc[4] && !( arc[4] < theta && theta < arc[3]))
                        {
                            return true;
                        }
                    }
                }
            }
            
            return false;
        }

        // Check a rectangle inside a ringsector.
        public static bool rectangle_in_ringsector(List<List<double>> rect, List<double> rs)
        {
            // rs: [x, y, rmin, rmax, start, end]
            // if all center-vertex distance within [rmin, rmax] and theta within [start, end]

            foreach (var p in rect)
            {
                var v = new List<double>() { p[0] - rs[0], p[1] - rs[1] };
                var rs_2 = new List<double>() { rs[0], rs[1] };
                var d = distance(p, rs_2);

                if (d >= rs[3] || d <= rs[2])
                {
                    return false;
                }

                var theta = Utils.Modulus( (Math.Atan2(v[1], v[0])) , (2 * Math.PI));

                if (rs[4] < rs[5] && !(rs[4] < theta && theta < rs[5]))
                {
                    return false;
                }

                if (rs[4] > rs[5] && rs[5] <= theta && theta <= rs[4])
                {
                    return false;
                }
            }
            return true;
        }

        // Check rectangle intersected with a ringsector.
        public static bool rectangle_ringsector_intersected(List<List<double>> rect, List<double> rs, bool edge = true)
        {
            // rs: [x, y, rmin, rmax, sta, end]
            // if rect in ringsector
            // if rect intersected with any arc/edge

            var rs_2 = new List<double>() { rs[0], rs[1] };
            var rs_3 = new List<double>() { rs[0], rs[1], rs[2] };
            var rs_neg_2 = new List<double>();
            for (int i = rs.Count-2; i < rs.Count; i++)
            {
                rs_neg_2.Add(rs[i]);
            }
            var rs_neg_3 = new List<double>();
            for (int i = rs.Count - 3; i < rs.Count; i++)
            {
                rs_neg_3.Add(rs[i]);
            }

            rs_3.AddRange(rs_neg_2);
            var arc1List = rs_3;
            rs_2.AddRange(rs_neg_3);
            var arc2List = rs_3;

            if (rectangle_in_ringsector(rect, rs))
            {
                return true;
            }

            if (rectangle_arc_intersected(rect, arc1List))
            {
                return true;
            }

            if (rectangle_arc_intersected(rect, arc2List))
            {
                return true;
            }

            if (edge)
            {
                
                var p1 = new List<double>() { rs[0] + rs[2] * Math.Cos(rs[4]), rs[1] + rs[2] * Math.Sin(rs[4]) };
                var p2 = new List<double>() { rs[0] + rs[3] * Math.Cos(rs[4]), rs[1] + rs[3] * Math.Sin(rs[4]) };
                var p3 = new List<double>() { rs[0] + rs[2] * Math.Cos(rs[5]), rs[1] + rs[2] * Math.Sin(rs[5]) };
                var p4 = new List<double>() { rs[0] + rs[3] * Math.Cos(rs[5]), rs[1] + rs[3] * Math.Sin(rs[5]) };

                if (line_rectangle_intersected(new List<List<double>>() { p1, p2 }, rect))
                {
                    return true;
                }

                if (line_rectangle_intersected(new List<List<double>>() { p3, p4 }, rect))
                {
                    return true;
                }
            }
            
            return false;
        }


        // ------------------------------------------------------------------------------------------------------

        public static bool LineRectIntersection(List<double> lineStartPoint, List<double> lineEndPoint, List<double> rectangle)
        {
            var lineStartPoint_x = lineStartPoint[0];
            var lineStartPoint_y = lineStartPoint[1];
            var lineEndPoint_x = lineEndPoint[0];
            var lineEndPoint_y = lineEndPoint[1];
            var minXLinePoint = lineStartPoint_x <= lineEndPoint_x ? lineStartPoint : lineEndPoint;
            var maxXLinePoint = lineStartPoint_x <= lineEndPoint_x ? lineEndPoint : lineStartPoint;
            var minYLinePoint = lineStartPoint_y <= lineEndPoint_y ? lineStartPoint : lineEndPoint;
            var maxYLinePoint = lineStartPoint_y <= lineEndPoint_y ? lineEndPoint : lineStartPoint;

            var x = rectangle[0];
            var y = rectangle[1];
            var w = rectangle[2];
            var l = rectangle[3];
            double rectMaxX = x + w / 2;
            double rectMinX = x - w / 2;
            double rectMaxY = y + l / 2;
            double rectMinY = y - l / 2;

            if (minXLinePoint[0] <= rectMaxX && rectMaxX <= maxXLinePoint[0])
            {
                double m = (maxXLinePoint[1] - minXLinePoint[1]) / (maxXLinePoint[0] - minXLinePoint[0]);

                double intersectionY = ((rectMaxX - minXLinePoint[0]) * m) + minXLinePoint[1];

                if (minYLinePoint[1] <= intersectionY && intersectionY <= maxYLinePoint[1]
                    && rectMinY <= intersectionY && intersectionY <= rectMaxY)
                {
                    return true;
                }
            }

            if (minXLinePoint[0] <= rectMinX && rectMinX <= maxXLinePoint[0])
            {
                double m = (maxXLinePoint[1] - minXLinePoint[1]) / (maxXLinePoint[0] - minXLinePoint[0]);

                double intersectionY = ((rectMinX - minXLinePoint[0]) * m) + minXLinePoint[1];

                if (minYLinePoint[1] <= intersectionY && intersectionY <= maxYLinePoint[1]
                    && rectMinY <= intersectionY && intersectionY <= rectMaxY)
                {
                    return true;
                }
            }

            if (minYLinePoint[1] <= rectMaxY && rectMaxY <= maxYLinePoint[1])
            {
                double rm = (maxYLinePoint[0] - minYLinePoint[0]) / (maxYLinePoint[1] - minYLinePoint[1]);

                double intersectionX = ((rectMaxY - minYLinePoint[1]) * rm) + minYLinePoint[0];

                if (minXLinePoint[0] <= intersectionX && intersectionX <= maxXLinePoint[0]
                    && rectMinX <= intersectionX && intersectionX <= rectMaxX)
                {
                    return true;
                }
            }

            if (minYLinePoint[1] <= rectMinY && rectMinY <= maxYLinePoint[1])
            {
                double rm = (maxYLinePoint[0] - minYLinePoint[0]) / (maxYLinePoint[1] - minYLinePoint[1]);

                double intersectionX = ((rectMinY - minYLinePoint[1]) * rm) + minYLinePoint[0];

                if (minXLinePoint[0] <= intersectionX && intersectionX <= maxXLinePoint[0]
                    && rectMinX <= intersectionX && intersectionX <= rectMaxX)
                {
                    return true;
                }
            }

            return false;
        }

        // ------------------------------------------------------------------------------------------------------


    }
}
