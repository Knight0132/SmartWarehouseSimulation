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
using CentralControl.OrderAssignment;

namespace CentralControl.RobotControl
{
    public enum RobotState
    {
        Moving, 
        Picking
    }
    
    public class RobotController : MonoBehaviour
    {
        public Transform targetIndicator;
        public SearchAlgorithm selectedAlgorithm = SearchAlgorithm.Astar_Basic;
        public SearchMethod selectedSearchMethod = SearchMethod.Manhattan_Distance;
        public float defaultSpeed = 5.0f;
        public float maxAcceleration = 2.0f;
        public float maxDeceleration = 3.0f;
        public float radius = 0.5f;
        public int MaxOrder = 20;
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace { get; private set; }
        public MapGrid mapGrid { get; private set; }
        public bool lastReportedAvailability { get; set; }
        public bool lastReportedFreeStatus { get; set; }
        public int lastReportedOrderCount { get; set; } = -1;

        private HashSet<RobotState> currentStates = new HashSet<RobotState>();
        private HashSet<RobotState> lastReportedState = new HashSet<RobotState>();
        private ConcurrentQueue<Order> ordersQueue = new ConcurrentQueue<Order>();
        private int isProcessingOrder = 0;
        private bool isProcessingOrdersRunning = false;
        private Coroutine statusCheckCoroutine;

        public bool IsMoving => currentStates.Contains(RobotState.Moving);
        public bool IsPicking => currentStates.Contains(RobotState.Picking);
        public bool IsFree => currentStates.Count == 0;
        public bool IsOnTask => !IsFree;
        public bool IsAvailable => ordersQueue.Count < MaxOrder;
        public bool IsProcessingOrders => isProcessingOrdersRunning;
        public int GetRobotOrdersQueueCount => ordersQueue.Count;

        public List<string> listOrdersQueue => new List<string>(ordersQueue.Select(order => order.Id));

        // private float criticalDistance = 1.0f;
        // private Vector3 currentDestination;
        // private bool isAvoidingObstacle = false;
        // private Coroutine currentMovementCoroutine;
        // private const float StopAndReplanDelay = 0.5f;

        private void Start()
        {
            statusCheckCoroutine = StartCoroutine(PeriodicStatusCheck());
        }

        // private void Update()
        // {
        //     CheckSurroundings();
        // }

        private IEnumerator PeriodicStatusCheck()
        {
            while (true)
            {
                yield return new WaitForSeconds(10f);
                if (IsFree && ordersQueue.Count > 0 && !isProcessingOrdersRunning)
                {
                    Debug.LogWarning($"Robot {Id} is idle but has orders. Restarting order processing.");
                    StartCoroutine(ProcessOrders());
                }
                else
                {
                    if (lastReportedState != currentStates || lastReportedOrderCount != ordersQueue.Count)
                    {
                        Debug.Log($"Robot {Id} status - Moving: {IsMoving}, Picking: {IsPicking}, Orders: {ordersQueue.Count}, Processing: {isProcessingOrdersRunning}");
                        lastReportedState = new HashSet<RobotState>(currentStates);
                        lastReportedOrderCount = ordersQueue.Count;
                    }
                }
            }
        }

        public void InitializeRobot(int id, IndoorSpace indoorSpace, Graph graph, MapGrid mapGrid)
        {
            Id = id;
            this.indoorSpace = indoorSpace;
            this.graph = graph;
            this.mapGrid = mapGrid;

            currentStates.Clear();
            ordersQueue = new ConcurrentQueue<Order>();
        }

        private void SetState(RobotState state, bool value)
        {
            if (value)
            {
                currentStates.Add(state);
            }
            else
            {
                currentStates.Remove(state);
            }
        }

        public void ReceiveOrder(Order order)
        {
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
            SetState(RobotState.Moving, true);

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

            if (Time.time - startTime >= timeout)
            {
                FailOrder(order);
            }
            UpdateRobotStatus(); 
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
            return new Vector3((float)point.X, mapGrid.height, (float)point.Y);
        }

        // path planning module
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

            /* Test if path exists between the given points

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
            */
            
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
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, false);
            UpdateRobotStatus();
        }

        // moving module
        private IEnumerator MoveAlongPath(List<ConnectionPoint> path, List<float>speeds, float executionTime, Vector3 finalTargetPosition)
        {
            SetState(RobotState.Moving, true);
            SetState(RobotState.Picking, false);
            
            for (int index = 0; index < path.ToArray().Length; index++)
            {
                if (!IsMoving)
                {
                    Debug.LogWarning($"Robot {Id}: Movement interrupted");
                    yield break;
                }

                ConnectionPoint point = path[index];
                float speed = speeds[index];

                if (index < path.Count - 1)
                {
                    yield return StartCoroutine(MoveToPosition(GetPointVector3(point.Point), speed, applySlowdown: false));
                }
                else
                {
                    yield return StartCoroutine(MoveToPosition(GetPointVector3(point.Point), speed, applySlowdown: true));
                }
            }

            yield return StartCoroutine(MoveToPosition(finalTargetPosition, defaultSpeed, applySlowdown: true));
            
            Debug.Log($"Robot {Id} reached destination, executing order for {executionTime} seconds");
            
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, true);
            Debug.Log($"Robot {Id} finished moving, starting picking operation");

            yield return new WaitForSeconds(executionTime);
            Debug.Log($"Robot {Id} finished picking operation");

            UpdateRobotStatus();
        }

        private IEnumerator MoveToPosition(Vector3 target, float maxSpeed, bool applySlowdown = true)
        {
            float slowdownDistance = applySlowdown ? 2.0f : 0.1f;
            float minSpeed = applySlowdown ? 0.5f : maxSpeed;

            while (Vector3.Distance(transform.position, target) > 0.1f)
            {
                // if (isAvoidingObstacle)
                // {
                //     yield return null;
                //     continue;
                // }
                
                float distanceToTarget = Vector3.Distance(transform.position, target);
                float currentSpeed;

                if (distanceToTarget < slowdownDistance)
                {
                    float t = distanceToTarget / slowdownDistance;
                    currentSpeed = Mathf.Lerp(minSpeed, maxSpeed, t);
                }
                else
                {
                    currentSpeed = maxSpeed;
                }

                transform.position = Vector3.MoveTowards(transform.position, target, currentSpeed * Time.deltaTime);
                yield return null;
            }
        }

        private Vector3 GetPointVector3(Geometry point)
        {
            return new Vector3((float)((Point)point).X, mapGrid.height, (float)((Point)point).Y);
        }

        public void SetTargetIndicator(Vector3 position)
        {
            if (targetIndicator != null)
            {
                targetIndicator.position = position;
            }
        }

        // // replanning module
        // private IEnumerator ReplanPath()
        // {
        //     if (ordersQueue.Count > 0)
        //     {
        //         if (ordersQueue.TryPeek(out Order currentOrder))
        //         {
        //             Vector3 targetPosition = GetTargetPosition(currentOrder.PickingPoint);

        //             List<ConnectionPoint> newPath = new List<ConnectionPoint>();
        //             List<float> newSpeeds = new List<float>();
        //             (newPath, newSpeeds) = PlanPath(targetPosition);

        //             if (newPath != null && newPath.Count > 0)
        //             {
        //                 currentMovementCoroutine = StartCoroutine(MoveAlongPath(newPath, newSpeeds, currentOrder.ExecutionTime, targetPosition));
        //             }
        //             else
        //             {
        //                 Debug.LogError($"Robot {Id}: Failed to replan path");
        //             }
        //         }
        //         else
        //         {
        //             Debug.LogError($"Robot {Id}: Failed to peek at the current order");
        //         }
        //     }
        //     else
        //     {
        //         Debug.Log($"Robot {Id}: No orders to replan for.");
        //     }
        //     yield return null;
        // }

        // // collision detection & avoidance module
        // private void CheckSurroundings()
        // {
        //     Collider[] colliders = Physics.OverlapSphere(transform.position, radius);
        //     foreach (var collider in colliders)
        //     {
        //         if (collider.gameObject != gameObject)
        //         {
        //             HandleSurroundings(collider.gameObject);
        //         }
        //     }
        // }

        // private void HandleSurroundings(GameObject surroundingObject)
        // {
        //     float distance = Vector3.Distance(transform.position, surroundingObject.transform.position);
        //     if (distance < criticalDistance && isMoving && !isAvoidingObstacle)
        //     {
        //         isAvoidingObstacle = true;
        //         StartCoroutine(StopAndAvoidObstacle(surroundingObject));
        //     }
        // }

        // private Vector3 CalculateAvoidanceDirection(GameObject obstacle)
        // {
        //     Vector3 awayFromObstacle = transform.position - obstacle.transform.position;
        //     return Vector3.ProjectOnPlane(awayFromObstacle, Vector3.up).normalized;
        // }

        // private IEnumerator StopAndAvoidObstacle(GameObject obstacle)
        // {
        //     StopMovement();
        //     yield return new WaitForSeconds(StopAndReplanDelay);

        //     Vector3 avoidanceDirection = CalculateAvoidanceDirection(obstacle);
        //     Vector3 avoidanceTarget = transform.position + avoidanceDirection * radius;

        //     yield return StartCoroutine(MoveToPosition(avoidanceTarget, defaultSpeed, false));

        //     isAvoidingObstacle = false;
        //     StartCoroutine(ReplanPath());
        // }

        // private IEnumerator StopAndReplan()
        // {
        //     StopMovement();
        //     yield return new WaitForSeconds(StopAndReplanDelay);
        //     StartCoroutine(ReplanPath());
        // }

        // private void StopMovement()
        // {
        //     if (currentMovementCoroutine != null)
        //     {
        //         StopCoroutine(currentMovementCoroutine);
        //         currentMovementCoroutine = null;
        //     }
        //     isMoving = false;
        //     Rigidbody rb = GetComponent<Rigidbody>();
        //     if (rb != null)
        //     {
        //         rb.velocity = Vector3.zero;
        //         rb.angularVelocity = Vector3.zero;
        //     }
        //     Debug.Log($"Robot {Id} stopped moving for replanning.");
        // }

        // private void OnCollisionEnter(Collision collision)
        // {
        //     HandleCollision(collision.gameObject);
        // }

        // private void OnTriggerEnter(Collider other)
        // {
        //     HandleTrigger(other.gameObject);
        // }

        // private void HandleCollision(GameObject collisionObject)
        // {
        //     Debug.Log($"Robot {Id} collided with {collisionObject.name}");
        //     // replanning
        //     StartCoroutine(StopAndReplan());
        // }

        // private void HandleTrigger(GameObject triggeringObject)
        // {
        //     Debug.Log($"Robot {Id} triggered by {triggeringObject.name}");
        // }

        // status check module
        private void UpdateRobotStatus()
        {
            bool previousFreeStatus = IsFree;
            
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, false);

            if (previousFreeStatus != IsFree || lastReportedOrderCount != ordersQueue.Count)
            {
                Debug.Log($"Robot {Id} status updated. Is free: {IsFree}, Orders remaining: {ordersQueue.Count}");
                lastReportedOrderCount = ordersQueue.Count;
            }

            if (ordersQueue.Count > 0)
            {
                if (!isProcessingOrdersRunning)
                {
                    StartCoroutine(ProcessOrders());
                }
            }
            else if (!previousFreeStatus)
            {
                Debug.Log($"Robot {Id} is now free and has no orders.");
                CentralController.NotifyRobotFree(this);
            }
        }
        
        public void ResetRobot()
        {
            StopAllCoroutines();
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, false);
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