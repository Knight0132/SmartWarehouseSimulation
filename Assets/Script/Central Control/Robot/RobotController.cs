using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NetTopologySuite.Geometries;
using PathPlanning;
using Map;

namespace CentralControl
{
    public class RobotController : MonoBehaviour
    {
        public MapLoader mapLoader;
        public Transform targetIndicator;
        public float defaultSpeed = 5.0f;
        public SearchAlgorithm selectedAlgorithm = SearchAlgorithm.Astar_Traffic_Completed;
        public int Id { get; private set; }

        // private Vector3 currentTarget;
        // private int pathIndex = 0;
        // private List<Tuple<ConnectionPoint, float>> path = new List<Tuple<ConnectionPoint, float>>();
        // private CellSpace cellSpace;
        // private RoutePoint routePoint;
        // private ConnectionPoint connectionPoint;
        // private Layer layer;
        // private float startTime;
        // private float elapsedTime;
        // private float currentSpeed;
        private bool isMoving = false;
        private Graph graph;
        private IndoorSpace indoorSpace;
        private Queue<Order> ordersQueue = new Queue<Order>();

        void Update()
        {
            if (!isMoving && ordersQueue.Count > 0)
            {
                StartNextOrder();
            }
        }

        public void InitializeRobot(int id)
        {
            this.Id = id;
            indoorSpace = mapLoader.LoadJson();
            graph = mapLoader.GenerateRouteGraph(indoorSpace); 
        }

        public void ReceiveOrder(Order order)
        {
            ordersQueue.Enqueue(order);
            if (!isMoving)
            {
                StartNextOrder();
            }
        }

        private void StartNextOrder()
        {
            if (ordersQueue.Count > 0)
            {
                Order order = ordersQueue.Dequeue();
                ExecuteOrder(order);
            }
        }

        private void ExecuteOrder(Order order)
        {
            Vector3 targetPosition = order.Destination;
            targetIndicator.position = targetPosition;
            isMoving = true;
            StartCoroutine(MoveToPosition(targetPosition, order.ExecutionTime));
        }

        IEnumerator MoveToPosition(Vector3 target, float executionTime)
        {
            while (Vector3.Distance(transform.position, target) > 0.1f)
            {
                transform.position = Vector3.MoveTowards(transform.position, target, defaultSpeed * Time.deltaTime);
                yield return null;
            }
            
            yield return new WaitForSeconds(executionTime);
            
            isMoving = false;
            Debug.Log("Order completed and executed for " + executionTime + " seconds at " + target);
        }

        public bool IsFree
        {
            get { return !isMoving && ordersQueue.Count == 0; }
        }

        public bool IsMoving
        {
            get { return isMoving; }
        }

        // public void AssignOrder(Order order)
        // {
        //     currentTarget = order.Coordinates;
        //     targetIndicator.position = currentTarget;
        //     RoutePoint startPosition = graph.GetRoutePointFromCoordinate(transform.position, true);
        //     RoutePoint endPosition = graph.GetRoutePointFromCoordinate(currentTarget, true);
        //     if (startPosition != null && endPosition != null)
        //     {
        //         path = PathPlanner.FindPath(selectedAlgorithm, graph, startPosition, endPosition, defaultSpeed);
        //         isMoving = true;
        //         pathIndex = 0;
        //         startTime = Time.time;
        //     }
        //     else
        //     {
        //         Debug.LogError("One of the points is invalid.");
        //     }
        // }

        // public void MoveTowardsNext()
        // {
        //     if (pathIndex < path.Count)
        //     {
        //         Vector3 targetPosition = graph.GetCoordinatesFromConnectionPoint(path[pathIndex].Item1);
        //         CellSpace targetCellSpace = this.indoorSpace.GetCellSpaceFromConnectionPoint(path[pathIndex].Item1, false);
        //         Layer targetLayer = graph.GetLayerFromConnectionPoint(path[pathIndex].Item1, false);
        //         float currentSpeed = path[pathIndex].Item2;

        //         transform.position = Vector3.MoveTowards(transform.position, targetPosition, Time.deltaTime * currentSpeed);
        //         if (Vector3.Distance(transform.position, targetPosition) < 0.1f)
        //         {
        //             pathIndex++;

        //             if (pathIndex == path.Count)
        //             {
        //                 elapsedTime = Time.time - startTime;
        //                 Debug.Log("Path completed in: " + elapsedTime + " seconds.");
        //                 isMoving = false;
        //             }
        //         }
        //     }
        // }
    }
}
