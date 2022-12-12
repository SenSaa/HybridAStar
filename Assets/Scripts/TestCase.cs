using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using env;

public class TestCase
{

    /// <summary>
    /// Rotation should be in radians.
    /// Also it should be using right-handed coordinate system.
    /// </summary>

    public List<double> start_pos, end_pos, start_pos2, end_pos2;
    public List<List<double>> obs;
    public List<List<double>> obs2;

    public TestCase()
    {
        this.start_pos = new List<double>() { 4.6, 2.4, 0 };
        this.end_pos = new List<double>() { 1, 4, Math.PI/2 };
        this.start_pos2 = new List<double>() { 6.5, 5.2, Math.PI };
        this.end_pos2 = new List<double>() { 9, 8, Math.PI / 2 };

        this.obs = new List<List<double>>()
        {
            new List<double> {2, 3, 6, 0.1 },
            new List<double> {2, 3, 0.1, 1.5},
            new List<double> {4.3, 0, 0.1, 1.8},
            new List<double> {6.5, 1.5, 0.1, 1.5},
            new List<double> {0, 6, 3.5, 0.1},
            new List<double> {5, 6, 5, 0.1}
        };
    }

}
