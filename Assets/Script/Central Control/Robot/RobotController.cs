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
        public SearchAlgorithm selectedAlgorithm = SearchAlgorithm.Astar_Basic;
        public SearchMethod selectedSearchMethod = SearchMethod.Manhattan_Distance;
        public float defaultSpeed = 5.0f;
        public float maxAcceleration = 2.0f;
        public float maxDeceleration = 3.0f;
        public int MaxOrder = 20;
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace { get; private set; }

        private bool isMoving = false;
        private bool isPicking = false;
        private ConcurrentQueue<Order> ordersQueue = new ConcurrentQueue<Order>();
        private int isProcessingOrder = 0;
        private bool isProcessingOrdersRunning = false;
        private Coroutine statusCheckCoroutine;

        private void Start()
        {
            statusCheckCoroutine = StartCoroutine(PeriodicStatusCheck());
        }

        private IEnumerator PeriodicStatusCheck()
        {
            while (true)
            {
                yield return new WaitForSeconds(10f);
                if (!isMoving && !isPicking && ordersQueue.Count > 0 && !isProcessingOrdersRunning)
                {
                    Debug.LogWarning($"Robot {Id} is idle but has orders. Restarting order processing.");
                    StartCoroutine(ProcessOrders());
                }
                Debug.Log($"Robot {Id} status - Moving: {isMoving}, Picking: {isPicking}, Orders: {ordersQueue.Count}, Processing: {isProcessingOrdersRunning}");
            }
        }


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
                if (ordersQueue.Any(o => o.Id == order.Id))
                {
                    Debug.LogWarning($"Robot {Id}: Order {order.Id} is already in the queue. Skipping.");
                    return;
                }

                order.AssignedRobotId = Id;
                ordersQueue.Enqueue(order);
                Debug.Log($"Robot {Id} received order {order.Id}");
                
                if (Interlocked.CompareExchange(ref isProcessingOrder, 1, 0) == 0 && !isProcessingOrdersRunning)
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
            isProcessingOrdersRunning = true;
            while (ordersQueue.TryDequeue(out Order order))
            {
                Debug.Log($"Robot {Id} starts processing order {order.Id}");
                yield return StartCoroutine(ExecuteOrderCoroutine(order));
            }

            Debug.Log($"Robot {Id} has no more orders to execute");
            CentralController.NotifyRobotFree(this);
            Interlocked.Exchange(ref isProcessingOrder, 0);
            isProcessingOrdersRunning = false;
        }

        private IEnumerator ExecuteOrderCoroutine(Order order)
        {
            float startTime = Time.time;
            float timeout = 45f;

            if (!ValidateOrder(order))
            {
                Debug.LogError($"Robot {Id}: Order {order.Id} validation failed.");
                FailOrder(order);
                yield break;
            }
            
            Vector3 targetPosition = GetTargetPosition(order.PickingPoint);
            SetTargetIndicator(targetPosition);
            isMoving = true;
            Debug.Log($"Robot {Id} executing order {order.Id} to {targetPosition}");

            List<ConnectionPoint> path = new List<ConnectionPoint>();
            List<float> speeds = new List<float>();
            (path, speeds) = PlanPath(targetPosition);
            if (path != null && path.Count > 0)
            {
                yield return StartCoroutine(MoveAlongPath(path, speeds, order.ExecutionTime, targetPosition));
            }
            else
            {
                FailOrder(order);
            }

            while (isMoving && Time.time - startTime < timeout)
            {
                yield return null;
            }

            if (Time.time - startTime >= timeout)
            {
                Debug.LogError($"Robot {Id}: Order {order.Id} execution timed out");
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

        private (List<ConnectionPoint> path, List<float> speeds) PlanPath(Vector3 targetPosition)
        {
            RoutePoint startPoint = graph.GetRoutePointFromCoordinate(transform.position, true);
            RoutePoint endPoint = graph.GetRoutePointFromCoordinate(targetPosition, false);

            Debug.Log($"Start RoutePoint: {startPoint?.ConnectionPoint.Id}, End RoutePoint: {endPoint?.ConnectionPoint.Id}");

            if (startPoint == null || endPoint == null)
            {
                Debug.LogError($"Robot {Id}: Invalid start or end point for path planning. Start: {startPoint}, End: {endPoint}");
                return (null, null);
            }

            PathChecker pathChecker = new PathChecker(graph);
            bool pathExists = pathChecker.ValidatePath(startPoint.ConnectionPoint.Id, endPoint.ConnectionPoint.Id);

            if (pathExists)
            {
                Debug.Log("Path exists between the given points.");
            }
            else
            {
                Debug.LogWarning("No path found between the given points.");
            }
            
            PathPlanner pathPlanner = new PathPlanner(graph, selectedAlgorithm, selectedSearchMethod, maxAcceleration, maxDeceleration);
            var (path, speeds) = pathPlanner.FindPath(startPoint, endPoint, defaultSpeed);
            
            if (path == null || path.Count == 0)
            {
                Debug.LogError($"Robot {Id}: Failed to find path from {startPoint.ConnectionPoint.Id} to {endPoint.ConnectionPoint.Id}");
            }
            else
            {
                Debug.Log($"Robot {Id}: Path found with {path.Count} steps");
            }

            return (path, speeds);
        }

        private void FailOrder(Order order)
        {
            Debug.LogError($"Robot {Id}: Failed to execute order {order.Id}");
            isMoving = false;
            UpdateRobotStatus();
        }

        private IEnumerator MoveAlongPath(List<ConnectionPoint> path, List<float>speeds, float executionTime, Vector3 finalTargetPosition)
        {
            for (int index = 0; index < path.ToArray().Length; index++)
            {
                if (!isMoving)
                {
                    Debug.LogWarning($"Robot {Id}: Movement interrupted");
                    yield break;
                }

                ConnectionPoint point = path[index];
                float speed = speeds[index];

                yield return StartCoroutine(MoveToPosition(GetPointVector3(point.Point), speed));
            }

            yield return StartCoroutine(MoveToPosition(finalTargetPosition, defaultSpeed));
            
            Debug.Log($"Robot {Id} reached destination, executing order for {executionTime} seconds");
            
            isMoving = false;
            Debug.Log($"Robot {Id} finished moving, starting picking operation");
            isPicking = true;

            yield return new WaitForSeconds(executionTime);
            Debug.Log($"Robot {Id} finished picking operation");

            UpdateRobotStatus();
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
        public bool IsProcessingOrders => isProcessingOrdersRunning;

        private void UpdateRobotStatus()
        {
            bool wasBusy = isMoving || isPicking;
            isMoving = false;
            isPicking = false;

            Debug.Log($"Robot {Id} status updated. Was busy: {wasBusy}, Orders remaining: {ordersQueue.Count}");

            if (ordersQueue.Count > 0)
            {
                if (!isProcessingOrdersRunning)
                {
                    Debug.Log($"Robot {Id} starting to process orders after status update.");
                    StartCoroutine(ProcessOrders());
                }
                else
                {
                    Debug.Log($"Robot {Id} is already processing orders.");
                }
            }
            else
            {
                Debug.Log($"Robot {Id} is now free and has no orders.");
                CentralController.NotifyRobotFree(this);
            }
        }

        public string GetDiagnosticInfo()
        {
            return $"Robot {Id}: Position: {transform.position}, " +
                $"IsMoving: {isMoving}, IsPicking: {isPicking}, " +
                $"IsProcessingOrders: {isProcessingOrdersRunning}, " +
                $"OrderQueueCount: {ordersQueue.Count}";
        }
        
        public void ResetRobot()
        {
            StopAllCoroutines();
            isMoving = false;
            isPicking = false;
            isProcessingOrdersRunning = false;
            Interlocked.Exchange(ref isProcessingOrder, 0);
            
            while (ordersQueue.TryDequeue(out _)) { }
            
            Debug.Log($"Robot {Id} has been reset.");
            StartCoroutine(ProcessOrders());
        }

        private void OnDisable()
        {
            StopAllCoroutines();
        }
    }
}