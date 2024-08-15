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
        public SearchAlgorithm selectedAlgorithm = SearchAlgorithm.Astar_Traffic_Completed;
        public float defaultSpeed = 5.0f;
        public int MaxOrder = 20;
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace {get; private set; }


        private bool isMoving = false;
        private bool isPicking = false;
        private Queue<Order> ordersQueue = new Queue<Order>();
        public readonly object orderLock = new object();

        public void InitializeRobot(int id, IndoorSpace indoorSpace, Graph graph)
        {
            this.Id = id;
            this.indoorSpace = indoorSpace;
            this.graph = graph;

            isMoving = false;
            isPicking = false;
            ordersQueue.Clear();
        }

        public void ReceiveOrder(Order order)
        {
            lock (orderLock)
            {
                if (order.AssignedRobotId == null || order.AssignedRobotId == this.Id)
                {
                    if (!ordersQueue.Contains(order))
                    {
                        ordersQueue.Enqueue(order);
                        Debug.Log($"Robot {Id} received order {order.Id}");
                        if (!isMoving && !isPicking)
                        {
                            StartNextOrder();
                        }
                    }
                    else
                    {
                        Debug.LogError($"Order {order.Id} is already in the queue of Robot {Id}");
                    }
                }
                else
                {
                    Debug.LogError($"Order {order.Id} is already assigned to another robot {order.AssignedRobotId}");
                }
            }
        }

        private void StartNextOrder()
        {
            lock (orderLock)
            {
                if (ordersQueue.Count > 0)
                {
                    Order order = ordersQueue.Dequeue();
                    Debug.Log($"Robot {Id} starts order {order.Id}");
                    ExecuteOrder(order);
                }
            }
        }

        private void ExecuteOrder(Order order)
        {
            PickingPoint pickingPoint = order.PickingPoint;
            CellSpace correspondingCellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);

            if (correspondingCellSpace == null)
            {
                Debug.LogError($"Corresponding CellSpace for PickingPoint {pickingPoint.Id} is null.");
                return;
            }
            
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
                List<Tuple<ConnectionPoint, float>> path = PathPlanner.FindPath(selectedAlgorithm, graph, startPoint, endPoint, defaultSpeed);
                if (path.Count > 0)
                {
                    StartCoroutine(MoveAlongPath(path, order.ExecutionTime, targetPosition));
                }
                else
                {
                    Debug.Log("Path is empty");
                    isMoving = false;
                    UpdateRobotStatus();
                }
            }
            else
            {
                Debug.Log("Start or end point is null");
                isMoving = false;
                UpdateRobotStatus();
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
            
            isMoving = false;
            isPicking = true;
            
            yield return new WaitForSeconds(executionTime);
            
            UpdateRobotStatus();
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
        }

        public void SetTargetIndicator(Vector3 position)
        {
            if (targetIndicator != null)
            {
                targetIndicator.position = position;
            }
        }

        
        public int GetRobotOrdersQueueCount
        {
            get { return ordersQueue.Count; }
        }

        public List<string> listOrdersQueue
        {
            get
            {
                List<string> list = new List<string>();
                foreach (Order order in ordersQueue)
                {
                    list.Add(order.Id);
                }
                return list;
            }
        }

        public bool IsMoving
        {
            get { return isMoving; }
        }

        public bool IsPicking
        {
            get { return isPicking; }
        }

        public bool IsAvailable
        {
            get { return ordersQueue.Count < MaxOrder; }
        }

        public bool IsOnTask
        {
            get { return isMoving || isPicking; }
        }

        public bool IsFree
        {
            get { return !isMoving && !isPicking; }
        }

        private void UpdateRobotStatus()
        {
            lock (orderLock)
                {
                isMoving = false;
                isPicking = false;
                Debug.Log($"Robot {Id} status updated to free");

                if (ordersQueue.Count > 0)
                {
                    StartNextOrder();
                }
            }
        }
    }
}
