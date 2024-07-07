using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NetTopologySuite.Geometries;
using PathPlanning;
using Map;

namespace Robot {
    public class RobotController : MonoBehaviour
    {
        public MapLoader mapLoader;
        public Transform targetIndicator;
        public float defaultSpeed = 5.0f;
        public SearchAlgorithm selectedAlgorithm = SearchAlgorithm.Astar_Traffic_Completed;

        private Vector3 currentTarget;
        private int pathIndex = 0;
        private bool isMoving = false;
        private List<Tuple<ConnectionPoint, float>> path = new List<Tuple<ConnectionPoint, float>>();
        private CellSpace cellSpace;
        private RoutePoint routePoint;
        private ConnectionPoint connectionPoint;
        private Layer layer;
        private Graph graph;
        private IndoorSpace indoorSpace;
        private float startTime;
        private float elapsedTime;
        private float currentSpeed;



        void Start()
        {
            indoorSpace = mapLoader.LoadJson();
            graph = mapLoader.GenerateRouteGraph(indoorSpace); 
            SetStart(mapLoader.startPoint);
        }

        void Update()
        {
            if (!isMoving)
            {
                SetNewTarget(mapLoader.endPoint);
            }
            else if (pathIndex < path.Count)
            {
                MoveTowardsNext();
            }
            else
            {
                isMoving = false;
            }
        }

        void SetStart(Vector3 start)
        {
            transform.position = start;
            Debug.Log("Start position set to: " + start);
        }

        void SetNewTarget(Vector3 end)
        {
            currentTarget = end;
            targetIndicator.position = currentTarget;
            RoutePoint startPosition = graph.GetRoutePointFromCoordinate(transform.position, true);
            RoutePoint endPosition = graph.GetRoutePointFromCoordinate(currentTarget, true);
            if (startPosition != null && endPosition != null)
            {
                path = PathPlanner.FindPath(selectedAlgorithm, graph, startPosition, endPosition, defaultSpeed);
                isMoving = true;
                pathIndex = 0;
                startTime = Time.time;
            }
            else
            {
                Debug.LogError("One of the points is invalid.");
            }
        }

        void MoveTowardsNext()
        {
            if (pathIndex < path.Count)
            {
                Vector3 targetPosition = graph.GetCoordinatesFromConnectionPoint(path[pathIndex].Item1);
                CellSpace targetCellSpace = this.indoorSpace.GetCellSpaceFromConnectionPoint(path[pathIndex].Item1, false);
                Layer targetLayer = graph.GetLayerFromConnectionPoint(path[pathIndex].Item1, false);
                float currentSpeed = path[pathIndex].Item2;

                transform.position = Vector3.MoveTowards(transform.position, targetPosition, Time.deltaTime * currentSpeed);
                if (Vector3.Distance(transform.position, targetPosition) < 0.1f)
                {
                    pathIndex++;

                    if (pathIndex == path.Count)
                    {
                        elapsedTime = Time.time - startTime;
                        Debug.Log("Path completed in: " + elapsedTime + " seconds.");
                        isMoving = false;
                    }
                }
            }
        }
    }
}
