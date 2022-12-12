using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EventsManager
{

    public delegate void Event<TEventArgs>(TEventArgs e);
    public static event Event<RRTPointsEventArg> RRTPointsEvent;

    public static void InvokeRRTPointEvent(RRTPointsEventArg e)
    {
        RRTPointsEvent.Invoke(e);
    }

    public class RRTPointsEventArg
    {
        public bool RRTPointEventFlag;
        public Vector3 RRTNodePosition;
        public List<Vector3> RRTNodePositions;
    }

}
