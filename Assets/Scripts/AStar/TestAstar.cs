using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using astar;
using env;

public class TestAstar : MonoBehaviour
{
    void Start()
    {
        var tc = new TestCase();
        var env = new env.Environment(tc.obs);
        var grid = new env.Grid(env);

        Astar astar = new Astar(grid, tc.start_pos);
        Debug.Log("pos: " + tc.start_pos[0] + " , " + tc.start_pos[1] + " -> " + tc.end_pos[0] + " , " + tc.end_pos[1]);

        var cost = astar.search_path(tc.end_pos);
        var route = astar.route;

        Debug.Log("cost = " + cost);
        Debug.Log("route.Count = " + route.Count);
        for(int i=0; i < route.Count; i++)
        {
            List<int> pos = route[i];
            GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Cube);
            obj.transform.position = new Vector3(pos[0], 0, pos[1]);
            obj.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
        }
    }

    void Update()
    {
        
    }
}
