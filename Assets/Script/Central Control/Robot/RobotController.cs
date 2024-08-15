using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Threading;
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
        public IndoorSpace indoorSpace { get; private set; }

        private bool isMoving = false;
        private bool isPicking = false;
        private ConcurrentQueue<Order> ordersQueue = new ConcurrentQueue<Order>();
        private int isProcessingOrder = 0;

        public void InitializeRobot(int id, IndoorSpace indoorSpace, Graph graph)
        {
            Id = id;
            this.indoorSpace = indoorSpace;
            this.graph = graph;

            isMoving = false;
            isPicking = false;
            ordersQueue = new ConcurrentQueue<Order>();
        }

        public void ReceiveOrder(Order order)
        {
            if (order == null)
            {
                Debug.LogError($"Robot {Id}: Attempted to receive a null order");
                return;
            }

            if (order.AssignedRobotId == null || order.AssignedRobotId == Id)
            {
                order.AssignedRobotId = Id;
                ordersQueue.Enqueue(order);
                Debug.Log($"Robot {Id} received order {order.Id}");
                
                if (Interlocked.CompareExchange(ref isProcessingOrder, 1, 0) == 0)
                {
                    StartCoroutine(ProcessOrders());
                }
            }
            else
            {
                Debug.LogError($"Order {order.Id} is already assigned to another robot {order.AssignedRobotId}");
            }
        }

        private IEnumerator ProcessOrders()
        {
            while (ordersQueue.TryDequeue(out Order order))
            {
                Debug.Log($"Robot {Id} starts processing order {order.Id}");
                yield return StartCoroutine(ExecuteOrderCoroutine(order));
            }

            Debug.Log($"Robot {Id} has no more orders to execute");
            CentralController.NotifyRobotFree(this);
            Interlocked.Exchange(ref isProcessingOrder, 0);
        }

        private IEnumerator ExecuteOrderCoroutine(Order order)
        {
            if (!ValidateOrder(order))
            {
                yield break;
            }

            Vector3 targetPosition = GetTargetPosition(order.PickingPoint);
            SetTargetIndicator(targetPosition);
            isMoving = true;
            Debug.Log($"Robot {Id} executing order {order.Id} to {targetPosition}");

            List<Tuple<ConnectionPoint, float>> path = PlanPath(targetPosition);
            if (path != null && path.Count > 0)
            {
                yield return StartCoroutine(MoveAlongPath(path, order.ExecutionTime, targetPosition));
            }
            else
            {
                FailOrder(order);
            }
        }

        private bool ValidateOrder(Order order)
        {
            if (order == null || order.PickingPoint == null)
            {
                Debug.LogError($"Robot {Id}: Invalid order or picking point");
                return false;
            }

            CellSpace correspondingCellSpace = indoorSpace.GetCellSpaceFromId(order.PickingPoint.Id);
            if (correspondingCellSpace == null || correspondingCellSpace.Node == null)
            {
                Debug.LogError($"Robot {Id}: Invalid cell space for order {order.Id}");
                return false;
            }

            return true;
        }

        private Vector3 GetTargetPosition(PickingPoint pickingPoint)
        {
            CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
            Point point = (Point)cellSpace.Node;
            return new Vector3((float)point.X, 0, (float)point.Y);
        }

        private List<Tuple<ConnectionPoint, float>> PlanPath(Vector3 targetPosition)
        {
            RoutePoint startPoint = graph.GetRoutePointFromCoordinate(transform.position, true);
            RoutePoint endPoint = graph.GetRoutePointFromCoordinate(targetPosition, true);

            if (startPoint == null || endPoint == null)
            {
                Debug.LogError($"Robot {Id}: Invalid start or end point for path planning");
                return null;
            }

            return PathPlanner.FindPath(selectedAlgorithm, graph, startPoint, endPoint, defaultSpeed);
        }

        private void FailOrder(Order order)
        {
            Debug.LogError($"Robot {Id}: Failed to execute order {order.Id}");
            isMoving = false;
            UpdateRobotStatus();
        }

        private IEnumerator MoveAlongPath(List<Tuple<ConnectionPoint, float>> path, float executionTime, Vector3 finalTargetPosition)
        {
            foreach (var step in path)
            {
                yield return StartCoroutine(MoveToPosition(GetPointVector3(step.Item1.Point), step.Item2));
            }

            yield return StartCoroutine(MoveToPosition(finalTargetPosition, defaultSpeed));
            
            Debug.Log($"Robot {Id} reached destination, executing order for {executionTime} seconds");
            
            isMoving = false;
            isPicking = true;
            
            yield return new WaitForSeconds(executionTime);
            
            UpdateRobotStatus();
            Debug.Log($"Robot {Id} completed order");
        }

        private IEnumerator MoveToPosition(Vector3 target, float speed)
        {
            while (Vector3.Distance(transform.position, target) > 0.1f)
            {
                transform.position = Vector3.MoveTowards(transform.position, target, speed * Time.deltaTime);
                yield return null;
            }
        }

        private Vector3 GetPointVector3(Geometry point)
        {
            return new Vector3((float)((Point)point).X, 0, (float)((Point)point).Y);
        }

        public void SetTargetIndicator(Vector3 position)
        {
            if (targetIndicator != null)
            {
                targetIndicator.position = position;
            }
        }

        public int GetRobotOrdersQueueCount => ordersQueue.Count;

        public List<string> listOrdersQueue
        {
            get
            {
                return new List<string>(ordersQueue.Select(order => order.Id));
            }
        }

        public bool IsMoving => isMoving;
        public bool IsPicking => isPicking;
        public bool IsAvailable => ordersQueue.Count < MaxOrder;
        public bool IsOnTask => isMoving || isPicking;
        public bool IsFree => !isMoving && !isPicking;

        private void UpdateRobotStatus()
        {
            bool wasBusy = isMoving || isPicking;
            isMoving = false;
            isPicking = false;

            if (wasBusy)
            {
                Debug.Log($"Robot {Id} status updated to free");
                
                if (ordersQueue.Count > 0)
                {
                    StartCoroutine(ProcessOrders());
                }
                else
                {
                    CentralController.NotifyRobotFree(this);
                }
            }
        }
    }
}