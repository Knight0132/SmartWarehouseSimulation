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
        public float detectionRadius = 0.5f;
        public float detectionAngle = 60.0f;
        public float defaultSpeed = 5.0f;
        public float maxAcceleration = 2.0f;
        public float maxDeceleration = 3.0f;
        public int MaxOrder = 20;
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace { get; private set; }
        public MapGrid mapGrid { get; private set; }
        public bool lastReportedAvailability { get; set; }
        public bool lastReportedFreeStatus { get; set; }
        public int lastReportedOrderCount { get; set; } = -1;

        private DynamicOccupancyLayer personalOccupancyLayer;
        private DynamicOccupancyLayer globalOccupancyLayer;
        private Vector3 currentPosition;
        private const float TIME_CORRECTION_THRESHOLD = 0.5f;

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
        
        private bool isInitialized = false;
        private bool isAvoidingObstacle = false;
        private Coroutine currentMovementCoroutine;
        private const float StopAndReplanDelay = 0.5f;


        private void Start()
        {
            statusCheckCoroutine = StartCoroutine(PeriodicStatusCheck());
        }

        private void Update()
        {
            if (!isInitialized) return;

            UpdatePersonalLayer();
            CheckSurroundings();
        }

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

        public void InitializeRobot(int id, IndoorSpace indoorSpace, Graph graph, MapGrid mapGrid, DynamicOccupancyLayer globalOccupancyLayer)
        {
            Id = id;
            this.indoorSpace = indoorSpace;
            this.graph = graph;
            this.mapGrid = mapGrid;
            this.globalOccupancyLayer = globalOccupancyLayer;

            currentStates.Clear();
            ordersQueue = new ConcurrentQueue<Order>();

            currentPosition = transform.position;

            this.personalOccupancyLayer = new DynamicOccupancyLayer(
                indoorSpace,
                graph,
                mapGrid,
                globalOccupancyLayer.GetcellSpaceCentroids(), 
                Time.time,
                globalOccupancyLayer.GetWindowDuration(), 
                globalOccupancyLayer.GetTimeStep()
                );

            isInitialized = true;
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
            List<float> times = new List<float>();
            List<float> speeds = new List<float>();
            bool[,,] timeSpaceMatrix = new bool[0, 0, 0];

            (path, times, speeds, timeSpaceMatrix) = PlanPath(targetPosition);
            if (path != null && path.Count > 0)
            {
                personalOccupancyLayer.MergeTimeSpaceMatrix(timeSpaceMatrix);
                yield return StartCoroutine(MoveAlongPath(path, times, speeds, order.ExecutionTime, targetPosition));
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
        private (List<ConnectionPoint> path, List<float> times, List<float> speeds, bool[,,] timeSpaceMatrix) PlanPath(Vector3 targetPosition)
        {
            RoutePoint startPoint = graph.GetRoutePointFromCoordinate(transform.position, true);
            RoutePoint endPoint = graph.GetRoutePointFromCoordinate(targetPosition, false);

            Debug.Log($"Start RoutePoint: {startPoint?.ConnectionPoint.Id}, End RoutePoint: {endPoint?.ConnectionPoint.Id}");

            if (startPoint == null || endPoint == null)
            {
                Debug.LogError($"Robot {Id}: Invalid start or end point for path planning. Start: {startPoint}, End: {endPoint}");
                return (null, null, null, null);
            }
            
            PathPlanner pathPlanner = new PathPlanner(graph, selectedAlgorithm, selectedSearchMethod, maxAcceleration, maxDeceleration);
            var (path, times, speeds, timeSpaceMatrix) = pathPlanner.FindPathWithDynamicObstacles(
                this.personalOccupancyLayer, startPoint, endPoint, Time.time, defaultSpeed);
            
            if (path == null || path.Count == 0)
            {
                Debug.LogError($"Robot {Id}: Failed to find path with dynamic obstacles");
            }
            else
            {
                Debug.Log($"Robot {Id}: Path found with {path.Count} steps considering dynamic obstacles");
            }

            return (path, times, speeds, timeSpaceMatrix);
        }

        private void FailOrder(Order order)
        {
            Debug.LogError($"Robot {Id}: Failed to execute order {order.Id}");
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, false);
            UpdateRobotStatus();
        }

        // moving module
        private IEnumerator MoveAlongPath(List<ConnectionPoint> path, List<float> times, List<float>speeds, float executionTime, Vector3 finalTargetPosition)
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

                ConnectionPoint connectionPoint = path[index];
                float targetTime = times[index];
                float speed = speeds[index];
                Vector3 targetPosition = GetPointVector3(connectionPoint.Point);

                yield return StartCoroutine(CorrectTimeAndWait(connectionPoint, targetTime));

                if (index < path.Count - 1)
                {
                    yield return StartCoroutine(MoveToPosition(targetPosition, speed, applySlowdown: false));
                }
                else
                {
                    yield return StartCoroutine(MoveToPosition(targetPosition, speed, applySlowdown: true));
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
            Rigidbody rb = GetComponent<Rigidbody>();
            float slowdownDistance = applySlowdown ? 2.0f : 0.1f;
            float minSpeed = applySlowdown ? 0.5f : maxSpeed;

            while (Vector3.Distance(transform.position, target) > 0.1f)
            {
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

                Vector3 direction = (target - transform.position).normalized;
                rb.velocity = direction * currentSpeed;

                yield return null;
            }
            rb.velocity = Vector3.zero;
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

        // replanning module
        private IEnumerator ReplanPath(int retryCount = 0, int maxRetries = 5)
        {
            if (ordersQueue.Count > 0)
            {
                if (ordersQueue.TryPeek(out Order currentOrder))
                {
                    Vector3 targetPosition = GetTargetPosition(currentOrder.PickingPoint);

                    RoutePoint startPoint = graph.GetRoutePointFromCoordinate(transform.position, true);
                    RoutePoint endPoint = graph.GetRoutePointFromCoordinate(targetPosition, false);

                    var (newPath, newTimes, newSpeeds, newTimeSpaceMatrix) = PlanPath(targetPosition);

                    if (newPath != null && newPath.Count > 0)
                    {
                        currentMovementCoroutine = StartCoroutine(MoveAlongPath(newPath, newTimes, newSpeeds, currentOrder.ExecutionTime, targetPosition));
                        isAvoidingObstacle = false;
                    }
                    else
                    {
                        if (retryCount < maxRetries)
                        {
                            Debug.LogWarning($"Robot {Id}: Replan failed, waiting and retrying... (Attempt {retryCount + 1}/{maxRetries})");
                            float waitTime = Mathf.Pow(2, retryCount);
                            yield return new WaitForSeconds(waitTime);
                            retryCount++;
                            yield return StartCoroutine(ReplanPath(retryCount, maxRetries));
                        }
                        else
                        {
                            FailOrder(currentOrder);
                        }
                        Debug.LogError($"Robot {Id}: Failed to replan path");
                    }
                }
                else
                {
                    Debug.LogError($"Robot {Id}: Failed to peek at the current order");
                }
            }
            else
            {
                Debug.Log($"Robot {Id}: No orders to replan for.");
            }
            yield return null;
        }


        // collision detection & avoidance module
        private void CheckSurroundings()
        {
            LayerMask groundLayerMask = LayerMask.GetMask("GroundLayer");
            Collider[] colliders = Physics.OverlapSphere(transform.position, detectionRadius);
            foreach (var collider in colliders)
            {
                if (collider.gameObject.layer == LayerMask.NameToLayer("Ground"))
                {
                    continue;
                }

                if (collider.gameObject != gameObject && collider.CompareTag("Robot"))
                {
                    Vector3 otherPosition = collider.transform.position;
                    Vector3 directionToOther = otherPosition - transform.position;
                    float distance = directionToOther.magnitude;

                    if (IsPotentialCollision(directionToOther))
                    {
                        if (!isAvoidingObstacle)
                        {
                            isAvoidingObstacle = true;
                            StartCoroutine(StopAndReplan());
                        }
                        break;
                    }
                }
            }
        }

        private bool IsPotentialCollision(Vector3 directionToOther)
        {
            Vector3 forward = transform.forward;
            float angle = Vector3.Angle(forward, directionToOther);
            float distance = directionToOther.magnitude;
            if (angle < detectionAngle && distance < detectionRadius)
            {
                return true;
            }
            return false;
        }

        private IEnumerator StopAndReplan()
        {
            StopMovement();
            yield return new WaitForSeconds(StopAndReplanDelay);
            StartCoroutine(ReplanPath());
            isAvoidingObstacle = false;
        }

        private void StopMovement()
        {
            if (currentMovementCoroutine != null)
            {
                StopCoroutine(currentMovementCoroutine);
                currentMovementCoroutine = null;
            }
            
            Rigidbody rb = GetComponent<Rigidbody>();
            if (rb != null)
            {
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
            Debug.Log($"Robot {Id} stopped moving for replanning.");
        }

        private void OnCollisionEnter(Collision collision)
        {
            HandleCollision(collision.gameObject);
        }

        private void OnTriggerEnter(Collider other)
        {
            HandleTrigger(other.gameObject);
        }

        private void HandleCollision(GameObject collisionObject)
        {
            if (collisionObject.layer == LayerMask.NameToLayer("CollisionLayer"))
            {
                Debug.Log($"Robot {Id} collided with another robot.");
                HandleCollisionWithRobot(collisionObject);
            }
            else
            {
                Debug.Log($"Robot {Id} collided with {collisionObject.name}");
                HandleCollisionWithObject(collisionObject);
            }
        }

        private void HandleCollisionWithRobot(GameObject otherRobot)
        {
            if (!isAvoidingObstacle)
            {
                isAvoidingObstacle = true;
                StartCoroutine(StopAndReplan());
            }
        }

        private void HandleCollisionWithObject(GameObject otherObject)
        {
            if (!isAvoidingObstacle)
            {
                isAvoidingObstacle = true;
                StartCoroutine(StopAndReplan());
            }
        }

        private void HandleTrigger(GameObject triggeringObject)
        {
            Debug.Log($"Robot {Id} triggered by {triggeringObject.name}");
        }

        // dynamic occupancy layer module
        private void UpdatePersonalLayer()
        {
            // Two function: Update real-time personal layer and update the latest global layer information
            Vector3 newPosition = transform.position;
            if (newPosition != currentPosition)
            {
                currentPosition = newPosition;
                UpdatePersonalLayerFromGlobal();
            }

            CellSpace currentCellSpace = indoorSpace.GetCellSpaceFromCoordinates(currentPosition);
            personalOccupancyLayer.SetOccupancy(currentCellSpace, Time.time, true);
        }

        private void UpdatePersonalLayerFromGlobal()
        {
            if (personalOccupancyLayer == null)
            {
                Debug.LogError($"Robot {Id}: Personal occupancy layer is null.");
                return;
            }

            if (globalOccupancyLayer == null)
            {
                Debug.LogError($"Robot {Id}: Global occupancy layer is null.");
                return;
            }

            bool[,,] globalMatrix = globalOccupancyLayer.GetTimeSpaceMatrix();
            if (globalMatrix == null)
            {
                Debug.LogError($"Robot {Id}: Global time space matrix is null.");
                return;
            }

            try
            {
                personalOccupancyLayer.MergeTimeSpaceMatrix(globalMatrix);
            }
            catch (System.Exception e)
            {
                Debug.LogError($"Robot {Id}: Error merging time space matrix: {e.Message}");
            }
        }
        private IEnumerator CorrectTimeAndWait(ConnectionPoint connectionPoint, float targetTime)
        {
            while (Time.time < targetTime - TIME_CORRECTION_THRESHOLD)
            {
                float timeDifference = Time.time - targetTime;
                if (Mathf.Abs(timeDifference) > TIME_CORRECTION_THRESHOLD)
                {
                    UpdateTimeSpaceMatrix(connectionPoint, Time.time);
                }
                yield return null;
            }
        }

        private void UpdateTimeSpaceMatrix(ConnectionPoint connectionPoint, float newTime)
        {
            personalOccupancyLayer.UpdateTimeForIndex(connectionPoint, newTime);
        }

        public DynamicOccupancyLayer GetPersonalOccupancyLayer()
        {
            return personalOccupancyLayer;
        }

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