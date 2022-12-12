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
        var route_cost_safe = dubins.best_tangent(solutions);

        if (!route_cost_safe.Item3)
        {
            Debug.Log("No valid dubins path!");
            return;
        }

        var path_ = car.get_path(car.start_pos, route_cost_safe.Item1);
        Debug.Log(path_.Count);
        
        // * Limit path points!
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
