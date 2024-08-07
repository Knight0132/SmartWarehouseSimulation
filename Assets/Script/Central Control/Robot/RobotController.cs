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
        public int MaxOrder = 5;
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace {get; private set; }


        private bool isMoving = false;
        private Queue<Order> ordersQueue = new Queue<Order>();

        void Update()
        {
            if (!isMoving && ordersQueue.Count > 0)
            {
                StartNextOrder();
            }
        }

        public void InitializeRobot(int id, IndoorSpace indoorSpace, Graph graph)
        {
            this.Id = id;
            this.indoorSpace = indoorSpace;
            this.graph = graph; 

            isMoving = false;
            ordersQueue.Clear();
            
            if (ordersQueue.Count > 0 && !isMoving)
            {
                StartNextOrder();
            }
        }

        public void ReceiveOrder(Order order)
        {
            ordersQueue.Enqueue(order);
            Debug.Log($"Robot {Id} received order {order.Id}");
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
                Debug.Log($"Robot {Id} starts order {order.Id}");
                ExecuteOrder(order);
            }
        }

        private void ExecuteOrder(Order order)
        {
            PickingPoint pickingPoint = order.PickingPoint;
            CellSpace correspondingCellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
            Point point = (Point)correspondingCellSpace.Node;
            Vector3 targetPosition = new Vector3((float)point.X, 0, (float)point.Y);
            SetTargetIndicator(targetPosition);
            isMoving = true;
            Debug.Log($"Robot {Id} executing order {order.Id} to {targetPosition}");

            // Customized path planning
            RoutePoint startPoint = graph.GetRoutePointFromCoordinate(transform.position, true);
            RoutePoint endPoint = graph.GetRoutePointFromCoordinate(targetPosition, true);
            
            if (startPoint != null && endPoint != null)
            {
                List<Tuple<ConnectionPoint, float>> path = PathPlanner.FindPath(SearchAlgorithm.Astar_Traffic_Completed, graph, startPoint, endPoint, defaultSpeed);
                if (path.Count > 0)
                {
                    StartCoroutine(MoveAlongPath(path, order.ExecutionTime, targetPosition));
                }
                else
                {
                    Debug.Log("Path is empty");
                    isMoving = false;
                }
            }
            else
            {
                Debug.Log("Start or end point is null");
                isMoving = false;
            }
        
        }

        IEnumerator MoveAlongPath(List<Tuple<ConnectionPoint, float>> path, float executionTime, Vector3 finalTargetPosition)
        {
            foreach (var step in path)
            {
                Point targetPoint = (Point)step.Item1.Point;
                Vector3 target = new Vector3((float)targetPoint.X, 0, (float)targetPoint.Y);
                while (Vector3.Distance(transform.position, target) > 0.1f)
                {
                    transform.position = Vector3.MoveTowards(transform.position, target, step.Item2 * Time.deltaTime);
                    yield return null;
                }
            }

            yield return MoveToPosition(finalTargetPosition);
            
            Debug.Log($"Robot {Id} reached destination, waiting for {executionTime} seconds");
            yield return new WaitForSeconds(executionTime);
            
            isMoving = false;
            Debug.Log($"Order completed for {executionTime} seconds at destination");
        }

        IEnumerator MoveToPosition(Vector3 target)
        {
            Debug.Log($"Moving to final position {target}");
            while (Vector3.Distance(transform.position, target) > 0.1f)
            {
                transform.position = Vector3.MoveTowards(transform.position, target, defaultSpeed * Time.deltaTime);
                yield return null;
            }
            
            isMoving = false;
        }

        public void SetTargetIndicator(Vector3 position)
        {
            if (targetIndicator != null)
            {
                targetIndicator.position = position;
            }
        }

        public bool IsFree
        {
            get
            {
                bool free = !isMoving && ordersQueue.Count < MaxOrder;
                Debug.Log($"Robot {Id} IsFree check: {free} (isMoving: {isMoving}, ordersQueue.Count: {ordersQueue.Count}, MaxOrder: {MaxOrder})");
                return free;
            }
        }

        public bool IsMoving
        {
            get { return isMoving; }
        }
    }
}
