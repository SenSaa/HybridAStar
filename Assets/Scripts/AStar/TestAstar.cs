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

        //List<int> start_pos = new List<int> { (int)tc.start_pos[0], (int)tc.start_pos[1] };
        ////List<float> start_pos = new List<float> { (float)(tc.start_pos[0]), (float)(tc.start_pos[1]) };
        ////Astar astar = new Astar(grid, start_pos);
        Astar astar = new Astar(grid, tc.start_pos);
        ////List<float> end_pos = new List<float> { (float)(tc.end_pos[0]), (float)(tc.end_pos[1]) };
        ////Debug.Log("pos: " + start_pos[0] + " , " + start_pos[1] + " -> " + end_pos[0] + " , " + end_pos[1]);
        Debug.Log("pos: " + tc.start_pos[0] + " , " + tc.start_pos[1] + " -> " + tc.end_pos[0] + " , " + tc.end_pos[1]);

        ////var cost = astar.search_path(end_pos);
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
