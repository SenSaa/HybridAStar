using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RenderObstacles
{
    public RenderObstacles(List<List<double>> obstacles)
    {
        foreach(var ob in obstacles)
        {
            GameObject rectObst = GameObject.CreatePrimitive(PrimitiveType.Cube);
            rectObst.transform.position = new Vector3((float) (ob[0] + (ob[2]/2)), 0, (float) (ob[1] + (ob[3]/2)));
            rectObst.transform.localScale = new Vector3((float)ob[2], 1, (float)ob[3]);
        }
    }
}
