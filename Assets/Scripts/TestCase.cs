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
        /*
        double rotation = 0;
        double rotationToDegrees = rotation * Mathf.Rad2Deg;
        ////double transformed_rotation = -(rotationToDegrees) + 90;
        double transformed_rotation = -(rotationToDegrees);
        double start_rotation = transformed_rotation >= 180 ? 360 - transformed_rotation : transformed_rotation;
        start_rotation = transformed_rotation <= -180 ? 360 + transformed_rotation : transformed_rotation;
        double start_rotation_rad = start_rotation * Mathf.Deg2Rad;

        //start_rotation_rad = 90 * Mathf.Deg2Rad;
        Debug.Log("start_rotation_rad: " + start_rotation_rad);

        this.start_pos = new List<double>() { 4.6, 2.4, start_rotation_rad };
        */

        /////this.end_pos = new List<double>() { 9, 8, -Math.PI / 2 };
        this.end_pos = new List<double>() { 1, 4, Math.PI/2 };
        ///this.end_pos = new List<double>() { 200, 200, -Math.PI / 2 };
            /*
            rotation = -Math.PI / 2;
            rotationToDegrees = rotation * Mathf.Rad2Deg;
            ////transformed_rotation = -(rotationToDegrees) + 90;
            transformed_rotation = -(rotationToDegrees);
            double end_rotation = transformed_rotation >= 180 ? 360 - transformed_rotation : transformed_rotation;
            end_rotation = transformed_rotation <= -180 ? 360 + transformed_rotation : transformed_rotation;
            double end_rotation_rad = end_rotation * Mathf.Deg2Rad;

            //end_rotation_rad = (-(-180/2) + 90) * Mathf.Deg2Rad;
            Debug.Log("end_rotation_rad: " + end_rotation_rad);

            this.end_pos = new List<double>() { 9, 8, end_rotation_rad };
            */

            ///this.start_pos2 = new List<double>() { 4, 4, 0 };
            ///this.end_pos2 = new List<double>() { 4, 8, 1.2 * Math.PI };
            //this.start_pos2 = new List<double>() { 4, 5, 0 };
            //this.end_pos2 = new List<double>() { 8, 4, Math.PI };
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
        //this.obs = new List<List<double>>();
    }

}
