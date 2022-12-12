using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using env;

namespace env
{
    public class Grid
    {

        private env.Environment env;
        public float cell_size;
        private int n;
        private int m;
        private double cell_dia;
        private int[,] grid;

        public Grid(env.Environment env, float cell_size = 0.5f)
        {
            this.env = env;
            this.cell_size = cell_size;

            this.n = (int)(this.env.lx / this.cell_size);
            this.m = (int)(this.env.ly / this.cell_size);

            this.cell_dia = Math.Sqrt(2 * Math.Pow(this.cell_size, 2));

            // *
            this.get_obstacle_occupancy();
        }

        private void get_obstacle_occupancy()
        {
            try
            {
                // Initialise Grid.
                grid = new int[m,n];

                for (int i = 0; i < n; i++)
                {
                    for (int j = 0; j < m; j++)
                    {
                        this.grid[i,j] = 0;
                    }
                }

                // Fill grid with obstacles.

                foreach (var ob in this.env.obs)
                {
                    var x1 = this.to_cell_id(new List<double> { (ob.x), (ob.y) })[0];
                    var y1 = this.to_cell_id(new List<double> { (ob.x), (ob.y) })[1];
                    var x2 = this.to_cell_id(new List<double> { (ob.x) + (ob.w), (ob.y) + (ob.h) })[0];
                    var y2 = this.to_cell_id(new List<double> { (ob.x) + (ob.w), (ob.y) + (ob.h) })[1];

                    if ((ob.x + ob.w) % this.cell_size == 0)
                    {
                        x2 -= 1;
                    }

                    if ((ob.y + ob.h) % this.cell_size == 0)
                    {
                        y2 -= 1;
                    }

                    for (int i = x1; i < x2 + 1; i++)
                    {
                        for (int j = y1; j < y2 + 1; j++)
                        {
                            this.grid[i,j] = 1;
                        }
                    }
                }
            }
            catch (Exception e) { Debug.Log(e); }
        }

        public List<int> to_cell_id(List<double> pt)
        {
            // Convert point into grid index.

            var x = Mathf.Min((int)(pt[0] / this.cell_size), this.n - 1);
            var y = Mathf.Min((int)(pt[1] / this.cell_size), this.m - 1);

            return new List<int> { x, y };
        }

        public List<List<int>> get_neighbors(List<int> cell_id)
        {
            // Get all the 4 adjacent cells.

            var x = cell_id[0];
            var y = cell_id[1];
            var nbs = new List<List<int>>();

            var motions = new List<(int, int)> { (1, 0), (-1, 0), (0, 1), (0, -1) };

            foreach (var p in motions)
            {
                if (0 <= x + p.Item1 && x + p.Item1 < this.n && 0 <= y + p.Item2 && y + p.Item2 < this.m)
                {
                    if (this.grid[x + p.Item1, y + p.Item2] == 0)
                    {
                        nbs.Add(new List<int> { x + p.Item1, y + p.Item2 });
                    }
                }
            }

            return nbs;
        }


    }
}