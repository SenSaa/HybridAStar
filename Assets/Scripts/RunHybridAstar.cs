using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using env;
using hybridAStar;
using System.Threading;
using System.Linq;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class RunHybridAstar : MonoBehaviour
{

    env.Environment env;
    private SimpleCar car;
    private env.Grid grid;
    private List<CarState> path;
    private List<HybridAStarNode> closedNodes;
    private List<CarState> branches;
    private Thread hybridAStarthread;
    private bool DrawPathFlag;
    [SerializeField] private Material PathMaterial;
    private List<List<double>> Obstacles;

    [SerializeField] private Vector3 StartPosition;
    [SerializeField] private Vector3 GoalPosition;
    [SerializeField] private InputField StartYaw;
    [SerializeField] private InputField GoalYaw;
    [SerializeField] private GameObject CursorPrefab;
    private GameObject Cursor;
    [SerializeField] private UnityEngine.Vector3 MouseWorldPos;
    [SerializeField] private Camera Camera;
    private enum Modes
    {
        _,
        SetStartMode,
        SetGoalMode
    }
    [SerializeField] private Modes Mode;

    private void Start()
    {
        Cursor = Instantiate(CursorPrefab);
        Mode = Modes._;

        var tc = new TestCase();
        env = new env.Environment(tc.obs);
        grid = new env.Grid(env);
        Obstacles = tc.obs;
        StartCoroutine(VisualiseObstacles());
    }

    private void Update()
    {
        OnMouseClick();
        UpdateMousePosition();
        UpdateCursor();

        if (path != null && !DrawPathFlag)
        {
            DrawPathFlag = true;
            renderPath();
            renderBranches();
        }
    }

    private void initHybridAStarInputs()
    {
        var start_pose = new List<double> { StartPosition.x, StartPosition.z, double.Parse(StartYaw.text) };
        var end_pose = new List<double> { GoalPosition.x, GoalPosition.z, double.Parse(GoalYaw.text) };
        car = new SimpleCar(env, start_pose, end_pose);
    }

    private void runHybridAStar()
    {
        bool reverse = true;
        var hastar = new HybridAStar(car, grid, reverse);

        var path_closed = hastar.search_path();
        path = path_closed.Item1;
        closedNodes = path_closed.Item2;

        Debug.Log("path_count = " + path.Count);
        Debug.Log("closedNodes_count = " + closedNodes.Count);
    }


    private void renderPath()
    {
        GameObject go = new GameObject("FinalPath");
        LineRenderer lineRenderer = go.AddComponent<LineRenderer>();
        lineRenderer.positionCount = path.Count;
        lineRenderer.widthMultiplier = 0.1f;
        Vector3 pos = Vector3.zero;
        for (int i = 0; i < path.Count; i++)
        {
            pos.Set((float)path[i].pos[0], 0, (float)path[i].pos[1]);
            lineRenderer.SetPosition(i, pos);
        }
        lineRenderer.material = PathMaterial;
    }

    private void renderPathPoints()
    {
        for (int i = 0; i < path.Count; i++)
        {
            List<double> pos = path[i].pos;
            GameObject obj = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            obj.transform.position = new Vector3((float)pos[0], 0, (float)pos[1]);
            obj.transform.localScale = new Vector3(0.25f, 0.25f, 0.25f);
        }
    }

    private void renderBranches()
    {
        foreach (var node in closedNodes)
        {
            List<Vector3> branchPos = new List<Vector3>();
            GameObject go = new GameObject("Branch");
            LineRenderer lineRenderer = go.AddComponent<LineRenderer>();
            foreach (var b in node.branches)
            {
                lineRenderer.positionCount = b.Count;
                lineRenderer.widthMultiplier = 0.01f;
                for (int i = 0; i < b.Count; i++)
                {
                    branchPos.Add(new Vector3((float)b[i].Item2[0], 0, (float)b[i].Item2[1]));
                }
            }
            lineRenderer.SetPositions(branchPos.ToArray());
        }
    }

    IEnumerator VisualiseObstacles()
    {
        yield return new WaitForSeconds(0);
        if (Obstacles != null)
        {
            RenderObstacles renderObstacles = new RenderObstacles(Obstacles);
        }
    }

    private void OnApplicationQuit()
    {
        if (hybridAStarthread != null) { hybridAStarthread.Suspend(); }
    }


    private void OnMouseClick()
    {
        if (Input.GetMouseButtonUp(0) && !EventSystem.current.IsPointerOverGameObject()) // Left click while ignoring scene object!
        {
            if (Mode == Modes.SetStartMode)
            {
                StartPosition = MouseWorldPos;
            }
            else if (Mode == Modes.SetGoalMode)
            {
                GoalPosition = MouseWorldPos;
            }
            if (Mode != Modes._) { Mode = Modes._; }
        }
    }

    private void UpdateMousePosition()
    {
        UnityEngine.Vector3 mousePos = new UnityEngine.Vector3(Input.mousePosition.x, Input.mousePosition.y, 0.1f);
        Ray ray = Camera.ScreenPointToRay(mousePos);
        if (Physics.Raycast(ray, out RaycastHit hit, 1100f))
        {
            MouseWorldPos = hit.point;
            DetermineHoveredObject(hit);
            return;
        }

        MouseWorldPos = Camera.ScreenToWorldPoint(mousePos);
    }

    private void DetermineHoveredObject(RaycastHit raycastHit)
    {
        Collider currentCollider = raycastHit.collider;
        GameObject currentObjectAtMousePos;

        if (currentCollider.attachedRigidbody != null)
        {
            currentObjectAtMousePos = currentCollider.attachedRigidbody.gameObject;
        }
        else
        {
            currentObjectAtMousePos = currentCollider.gameObject;
        }
    }

    private void UpdateCursor()
    {
        if (Cursor != null)
        {
            UnityEngine.Vector3 mousePos = new UnityEngine.Vector3(MouseWorldPos.x, 0.5f, MouseWorldPos.z);

            Cursor.transform.position = mousePos;

            if (Mode == Modes.SetStartMode)
            {
                Cursor.GetComponent<Light>().color = Color.green;
            }
            else if (Mode == Modes.SetGoalMode)
            {
                Cursor.GetComponent<Light>().color = Color.red;
            }
            else
            {
                Cursor.GetComponent<Light>().color = Color.gray;
            }
        }
    }


    public void OnSelectStartPosition()
    {
        Mode = Modes.SetStartMode;
    }

    public void OnSelectGoalPosition()
    {
        Mode = Modes.SetGoalMode;
    }

    public void OnRunSearch()
    {
        initHybridAStarInputs();
        hybridAStarthread = new Thread(runHybridAStar);
        hybridAStarthread.Start();

        DrawPathFlag = false;
        path = null;
    }

}
