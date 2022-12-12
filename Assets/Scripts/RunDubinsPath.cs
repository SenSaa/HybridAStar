using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using env;
using Dubins;
using utils;

public class RunDubinsPath : MonoBehaviour
{

    [SerializeField] private Transform Agent;

    void Start()
    {
        TestCase tc = new TestCase();

        Environment env = new Environment(tc.obs);

        SimpleCar car = new SimpleCar(env, tc.start_pos2, tc.end_pos2);

        DubinsPath dubins = new DubinsPath(car);

        List<DubinsPathParams> solutions = dubins.find_tangents(car.start_pos, car.end_pos);
        // |\/|
        /*
        Debug.Log("solutions_count -> " + solutions.Count);
        for (int i = 0; i < solutions.Count; i++)
        {
            Debug.Log("c1: " + solutions[i].c1[0] + " " + solutions[i].c1[1]);
            Debug.Log("c2: " + solutions[i].c2[0] + " " + solutions[i].c2[1]);
            Debug.Log("d: " + solutions[i].d[0] + " " + solutions[i].d[1]);
            Debug.Log("t1: " + solutions[i].t1[0] + " " + solutions[i].t1[1] + " " + solutions[i].t1[2]);
            Debug.Log("t2: " + solutions[i].t2[0] + " " + solutions[i].t2[1] + " " + solutions[i].t2[2]);
            Debug.Log("len: " + solutions[i].len);
        }
        */
        var route_cost_safe = dubins.best_tangent(solutions);
        // |\/|
        /*
        for (int i = 0; i < route_cost_safe.Item1.Count; i++)
        {
            string route_str = "";
            for (int j = 0; j < route_cost_safe.Item1.Count; j++)
            {
                route_str += route_cost_safe.Item1[i].Item1[j] + "  ";
            }
            Debug.Log(route_str + route_cost_safe.Item1[i].Item2 + " " + route_cost_safe.Item1[i].Item3);
        }
        */
        //Debug.Log("cost: " + route_cost_safe.Item2);

        if (!route_cost_safe.Item3)
        {
            Debug.Log("No valid dubins path!");
            return; // *** Uncomment!!!
        }

        var path_ = car.get_path(car.start_pos, route_cost_safe.Item1);
        Debug.Log(path_.Count);
        
        // * Limit path points!
        //path = path[::5] + [path[-1]]
        var path = new List<CarState>(); 
        for (int i=0; i < path_.Count; i += 5)
        {
            path.Add(path_[i]);
        }
        path.Add(path[path.Count -1]);

        var carl = new List<double>();
        for (int i=0; i < path.Count; i++)
        {
            carl.Add(path[i].model[0].Item2);
        }

        var end_state = car.get_car_state(car.end_pos);

        var tangents = new List<double>();
        foreach (var s in solutions)
        {
            var s_t1 = ( s.t1[0], s.t1[1] );
            var s_t2 = ( s.t2[0], s.t2[1] );
            tangents.Add(s_t1.Item1); tangents.Add(s_t1.Item2); tangents.Add(s_t2.Item1); tangents.Add(s_t2.Item2);
        }


        // -----------------------------

        
        for(int i=0; i < path.Count; i++)
        {
            GameObject pathPoint = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            pathPoint.transform.position = new Vector3((float)path[i].pos[0], 0, (float)path[i].pos[1]);
            pathPoint.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
        }
        
        StartCoroutine(CustomUpdateRoutine(path));
    }

    private IEnumerator CustomUpdateRoutine(List<CarState> pathSet)
    {
        for (int i=0; i < pathSet.Count; i++)
        {
            var pose = pathSet[i].pos;
            Agent.position = new Vector3((float)pose[0], 0, (float)pose[1]);
            var yawFromRadsToDegrees = pose[2] * Mathf.Rad2Deg;
            var transformedYaw = -(float)yawFromRadsToDegrees + 90;
            Agent.rotation = Quaternion.Euler(Agent.rotation.eulerAngles.x, transformedYaw, Agent.rotation.eulerAngles.z);
            yield return new WaitForSeconds(0.02f);
        }
    }

}
