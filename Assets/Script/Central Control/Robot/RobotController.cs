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
        public float baseDetectionRadius = 0.7f;
        public float emergencyStopDistance = 0.2f;
        public float defaultSpeed = 5.0f;
        public float maxAcceleration = 2.0f;
        public float maxDeceleration = 5.0f;
        public int MaxOrder = 20;
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace { get; private set; }
        public MapGrid mapGrid { get; private set; }
        public bool lastReportedAvailability { get; set; }
        public bool lastReportedFreeStatus { get; set; }
        public int lastReportedOrderCount { get; set; } = -1;

        public LineRenderer pathLineRenderer;
        public float lineWidth = 0.1f;
        public Color pathColor = Color.yellow;

        private DynamicOccupancyLayer personalOccupancyLayer;
        private DynamicOccupancyLayer globalOccupancyLayer;
        private Vector3 currentPosition;
        private const float TIME_CORRECTION_THRESHOLD = 0.5f;

        private HashSet<RobotState> currentStates = new HashSet<RobotState>();
        private HashSet<RobotState> lastReportedState = new HashSet<RobotState>();
        private ConcurrentQueue<Order> ordersQueue = new ConcurrentQueue<Order>();
        private int isProcessingOrder = 0;
        private bool isProcessingOrdersRunning = false;
        private Order currentOrder;
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
        private DWAPlanner dwaPlanner;
        private float dwaScoreThreshold = 0.3f;
        private IntersectionAreaManager intersectionManager;
        private bool isDWAEnabled = false;
        private const float INTERSECTION_ACTIVATION_DISTANCE = 3.0f;
        private const int LOOK_AHEAD_POINTS = 3;
        private bool isEmergencyStop = false;
        private float detectionRadius;
        private Coroutine currentMovementCoroutine;
        
        private float lastReplanTime = 0f;
        private float replanCooldown = 2f;

        public float pathDeviationDistanceThreshold = 1.0f;
        public float pathCheckInterval = 0.5f;
        private static readonly float maxPathDeviationDistance = Mathf.Sqrt(5f) / 2f;
        private List<ConnectionPoint> currentPath;
        private List<PathSegment> currentMovement;
        private int currentPathIndex;
        private bool isPathValid = false;

        public struct PathSegment
        {
            public ConnectionPoint Point;
            public float Time;
            public float Speed;

            public PathSegment(ConnectionPoint point, float time, float speed)
            {
                Point = point;
                Time = time;
                Speed = speed;
            }
        }


        private void Start()
        {
            statusCheckCoroutine = StartCoroutine(PeriodicStatusCheck());
            StartCoroutine(CheckPathDeviation());
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

        # region initialization module
        public void InitializeRobot(
            int id, 
            IndoorSpace indoorSpace, 
            Graph graph, 
            MapGrid mapGrid, 
            DynamicOccupancyLayer globalOccupancyLayer, 
            IntersectionAreaManager intersectionManager)
        {
            Id = id;
            this.indoorSpace = indoorSpace;
            this.graph = graph;
            this.mapGrid = mapGrid;
            this.globalOccupancyLayer = globalOccupancyLayer;
            this.intersectionManager = intersectionManager;

            currentStates.Clear();
            ordersQueue = new ConcurrentQueue<Order>();

            currentPosition = transform.position;
            currentPath = new List<ConnectionPoint>();
            currentMovement = new List<PathSegment>();

            this.personalOccupancyLayer = new DynamicOccupancyLayer(
                indoorSpace,
                graph,
                mapGrid,
                globalOccupancyLayer.GetcellSpaceCentroids(), 
                Time.time,
                globalOccupancyLayer.GetWindowDuration(), 
                globalOccupancyLayer.GetTimeStep()
                );

            InitializeDWAPlanner();

            isInitialized = true;
        }

        private void InitializeDWAPlanner()
        {
            dwaPlanner = new DWAPlanner
            {
                MaxSpeed = defaultSpeed,
                MinSpeed = 0.5f,
                MaxRotSpeed = 0.5f,
                MaxAccel = maxAcceleration,
                VelocityResolution = 0.1f,
                RotationResolution = 0.1f,
                PredictionTime = 1.0f,
                HeadingWeight = 0.25f,
                DistanceWeight = 0.2f,
                VelocityWeight = 0.3f, 
                ObstacleWeight = 0.25f,
                DWAScoreThreshold = dwaScoreThreshold, 
                ObstacleLayer = ~(LayerMask.GetMask("GroundLayer") | LayerMask.GetMask("BoundaryWall"))
            };
        }
        # endregion

        # region order processing module
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
                currentOrder = order;
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
                currentMovementCoroutine = StartCoroutine(MoveAlongPath(path, times, speeds, order.ExecutionTime, targetPosition));
                yield return currentMovementCoroutine;
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

        private void FailOrder(Order order)
        {
            Debug.LogError($"Robot {Id}: Failed to execute order {order.Id}");
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, false);
            UpdateRobotStatus();
        }
        # endregion

        # region path planning module
        private Vector3 GetTargetPosition(PickingPoint pickingPoint)
        {
            CellSpace cellSpace = indoorSpace.GetCellSpaceFromId(pickingPoint.Id);
            Point point = (Point)cellSpace.Node;
            return new Vector3((float)point.X, mapGrid.height, (float)point.Y);
        }

        private (List<ConnectionPoint> path, List<float> times, List<float> speeds, bool[,,] timeSpaceMatrix) PlanPath(Vector3 targetPosition)
        {
            currentPath.Clear();
            currentMovement.Clear();
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
                this.personalOccupancyLayer, this.globalOccupancyLayer, startPoint, endPoint, Time.time, defaultSpeed);
            
            if (path == null || path.Count == 0)
            {
                Debug.LogWarning($"Robot {Id}: Failed to find path with dynamic obstacles");
            }
            else
            {
                UpdateCurrentMovementInfo(path, times, speeds);
                List<string> pathId = new List<string>();
                foreach (var connectionPoint in path)
                {
                    pathId.Add(connectionPoint.Id);
                    currentPath.Add(connectionPoint);
                }

                Debug.Log($"Robot {Id}: Path found with {path.Count} steps: {string.Join(" -> ", pathId)}");
            }

            return (path, times, speeds, timeSpaceMatrix);
        }
        # endregion

        # region moving module
        private IEnumerator MoveAlongPath(List<ConnectionPoint> path, List<float> times, List<float>speeds, float executionTime, Vector3 finalTargetPosition)
        {
            if (path == null || path.Count == 0)
            {
                StartCoroutine(ReplanPath());
            }
            
            VisualizePath(path);
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

                UpdateDWAMaxSpeed(speed);

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

            UpdateDWAMaxSpeed(defaultSpeed);
            
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
                var (optimalVelocity, needReplan) = dwaPlanner.CalculateVelocity(transform.position, rb.velocity, target);
                // Debug.Log($"Robot {Id}: Optimal velocity: {optimalVelocity}");

                float distanceToTarget = Vector3.Distance(transform.position, target);

                if (distanceToTarget < slowdownDistance)
                {
                    float t = distanceToTarget / slowdownDistance;
                    optimalVelocity = Vector3.Lerp(optimalVelocity.normalized * minSpeed, optimalVelocity, t);
                }
                else
                {
                    rb.velocity = optimalVelocity;
                }

                yield return null;
            }
            rb.velocity = Vector3.zero;
        }

        private IEnumerator MoveToNearestPointThenFollowPath(List<ConnectionPoint> remainingPath, List<float> remainingTimes, List<float> remainingSpeeds)
        {
            Vector3 nearestPoint = GetPointVector3(remainingPath[0].Point);
            yield return StartCoroutine(MoveToPosition(nearestPoint, remainingSpeeds[0], true));

            yield return StartCoroutine(MoveAlongPath(remainingPath, remainingTimes, remainingSpeeds, currentOrder.ExecutionTime, GetTargetPosition(currentOrder.PickingPoint)));
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
        # endregion

        # region replanning module
        private IEnumerator ReplanPath(int retryCount = 0, int maxRetries = 5)
        {
            isPathValid = false;
            ClearPathVisualization();
            lastReplanTime = Time.time;

            Vector3 targetPosition = GetTargetPosition(currentOrder.PickingPoint);

            while (retryCount < maxRetries)
            {
                RoutePoint startPoint = graph.GetRoutePointFromCoordinate(transform.position, true);
                RoutePoint endPoint = graph.GetRoutePointFromCoordinate(targetPosition, false);

                if (startPoint == null || endPoint == null)
                {
                    Debug.LogWarning($"Robot {Id}: Invalid start or end point for path planning. Start: {startPoint}, End: {endPoint}. Retry {retryCount + 1}/{maxRetries}");
                    retryCount++;
                    yield return new WaitForSeconds(Mathf.Pow(2, retryCount));
                    continue;
                }

                var (newPath, newTimes, newSpeeds, newTimeSpaceMatrix) = PlanPath(targetPosition);

                if (newPath != null && newPath.Count > 0)
                {
                    isPathValid = true;
                    currentPathIndex = 0;
                    currentPath = newPath;
                    personalOccupancyLayer.MergeTimeSpaceMatrix(newTimeSpaceMatrix);
                    currentMovementCoroutine = StartCoroutine(MoveAlongPath(newPath, newTimes, newSpeeds, currentOrder.ExecutionTime, targetPosition));
                    yield break;
                }
                else
                {
                    Debug.LogWarning($"Robot {Id}: Path planning failed on attempt {retryCount + 1}/{maxRetries}");
                    retryCount++;
                    if (retryCount < maxRetries)
                    {
                        float waitTime = Mathf.Pow(2, retryCount);
                        Debug.Log($"Robot {Id}: Waiting {waitTime} seconds before retry...");
                        yield return new WaitForSeconds(waitTime);
                    }
                }
            }

            if (retryCount >= maxRetries)
            {
                Debug.LogError($"Robot {Id}: All path planning attempts failed after {maxRetries} retries");
                FailOrder(currentOrder);
            }
        }
        # endregion

        # region dwaplanner module
        private void CheckIntersectionAndControlDWA()
        {
            // check if the robot is approaching an intersection entry point or has reached an intersection exit point,
            // and enable/disable DWA accordingly

            if (currentPath == null || currentPath.Count <= currentPathIndex)
                return;

            // check if upcoming path points contain an intersection entry point
            bool shouldActivateDWA = CheckUpcomingIntersectionEntry();
            // check if current point is an intersection exit point
            bool shouldDeactivateDWA = CheckCurrentIntersectionExit();

            if (shouldActivateDWA && !isDWAEnabled)
            {
                EnableDWA();
            }
            else if (shouldDeactivateDWA && isDWAEnabled)
            {
                DisableDWA();
            }
        }

        private bool CheckUpcomingIntersectionEntry()
        {
            // check if the robot is approaching an intersection entry point
            
            for (int i = currentPathIndex; i < Mathf.Min(currentPathIndex + LOOK_AHEAD_POINTS, currentPath.Count); i++)
            {
                ConnectionPoint point = currentPath[i];
                foreach (var intersection in intersectionManager.GetAllIntersections().Values)
                {
                    if (intersection.EntryPoints.Any(ep => ep.Id == point.Id))
                    {
                        // calculate distance to entry point
                        Vector3 entryPosition = GetPointVector3(point.Point);
                        float distance = Vector3.Distance(transform.position, entryPosition);
                        
                        if (distance <= INTERSECTION_ACTIVATION_DISTANCE)
                        {
                            Debug.Log($"Robot {Id}: Approaching intersection entry point, distance: {distance}");
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        private bool CheckCurrentIntersectionExit()
        {
            // check if the robot has reached an intersection exit point
            
            if (currentPathIndex >= currentPath.Count)
                return false;

            ConnectionPoint currentPoint = currentPath[currentPathIndex];
            foreach (var intersection in intersectionManager.GetAllIntersections().Values)
            {
                if (intersection.ExitPoints.Any(ep => ep.Id == currentPoint.Id))
                {
                    Debug.Log($"Robot {Id}: Reached intersection exit point");
                    return true;
                }
            }
            return false;
        }

        private void EnableDWA()
        {
            // enable DWA and adjust the weights for better obstacle avoidance
            
            isDWAEnabled = true;
            dwaPlanner.HeadingWeight = 0.25f;
            dwaPlanner.DistanceWeight = 0.2f;
            dwaPlanner.VelocityWeight = 0.3f;
            dwaPlanner.ObstacleWeight = 0.25f;
            Debug.Log($"Robot {Id}: DWA enabled");
        }

        private void DisableDWA()
        {
            // disable DWA and restore the original weights
            
            isDWAEnabled = false;
            dwaPlanner.HeadingWeight = 0.4f;
            dwaPlanner.DistanceWeight = 0.3f;
            dwaPlanner.VelocityWeight = 0.2f;
            dwaPlanner.ObstacleWeight = 0.1f;
            Debug.Log($"Robot {Id}: DWA disabled");
        }
        # endregion

        # region collision detection & avoidance module
        private float CalculateStoppingDistance()
        {
            Rigidbody rb = GetComponent<Rigidbody>();
            float currentSpeed = rb.velocity.magnitude;
            
            float brakingDistance = (currentSpeed * currentSpeed) / (2 * maxDeceleration);
            
            return brakingDistance;
        }

        private void UpdateDetectionRadius()
        {
            detectionRadius = Mathf.Max(baseDetectionRadius, 0.7f * CalculateStoppingDistance());
        }

        private void CheckSurroundings()
        {
            CheckIntersectionAndControlDWA();

            if (!isDWAEnabled)
            {
                UpdateDetectionRadius();
                Vector3 robotVelocity = GetComponent<Rigidbody>().velocity;

                LayerMask detectionMask = ~(LayerMask.GetMask("GroundLayer") | LayerMask.GetMask("BoundaryWall"));
                Collider[] colliders = Physics.OverlapSphere(transform.position, detectionRadius, detectionMask);

                bool needEmergencyStop = false;

                foreach (var collider in colliders)
                {
                    if (collider.gameObject == gameObject) continue;

                    Vector3 directionToOther = collider.transform.position - transform.position;
                    float distance = directionToOther.magnitude;

                    if (distance < emergencyStopDistance)
                    {
                        needEmergencyStop = true;
                        break;
                    }
                }

                if (needEmergencyStop && !isEmergencyStop)
                {
                    StartCoroutine(EmergencyStop());
                }
            }

            else
            {
                Vector3 robotVelocity = GetComponent<Rigidbody>().velocity;
                Vector3 targetPosition = GetPointVector3(currentPath[currentPathIndex].Point);

                var (optimalVelocity, needReplan) = dwaPlanner.CalculateVelocity(
                    transform.position,
                    robotVelocity,
                    targetPosition
                );

                GetComponent<Rigidbody>().velocity = optimalVelocity;

                if (needReplan && Time.time - lastReplanTime > replanCooldown)
                {
                    StartCoroutine(ReplanPath());
                }
            }
        }

        private IEnumerator EmergencyStop()
        {
            isEmergencyStop = true;
            Debug.LogWarning($"Robot {Id} performing emergency stop!");

            Rigidbody rb = GetComponent<Rigidbody>();
            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            yield return new WaitForSeconds(1.0f);

            yield return StartCoroutine(ReplanPath());

            isEmergencyStop = false;
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
            Debug.Log($"Robot {Id} collided with {collisionObject.name}.");
            StartCoroutine(EmergencyStop());
        }

        private void HandleTrigger(GameObject triggeringObject)
        {
            Debug.Log($"Robot {Id} triggered by {triggeringObject.name}");
        }
        # endregion

        # region dynamic occupancy layer module
        private void UpdatePersonalLayer()
        {
            // Two function: Update real-time personal layer and update the latest global layer information
            Vector3 newPosition = transform.position;
            if (newPosition != currentPosition)
            {
                currentPosition = newPosition;
            }

            CellSpace currentCellSpace = indoorSpace.GetCellSpaceFromCoordinates(currentPosition);
            if (currentCellSpace != null)
            {
                personalOccupancyLayer.SetOccupancy(currentCellSpace, Time.time, true);
            }
            else
            {
                Debug.LogError($"Robot {Id}: Current position is not within any cell space.");
            }
        }

        public void UpdateGlobalLayer(DynamicOccupancyLayer globalLayer)
        {
            if (globalOccupancyLayer == null)
            {
                Debug.LogError($"Robot {Id}: Global occupancy layer is null.");
                return;
            }
            this.globalOccupancyLayer = globalLayer;
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
        # endregion

        # region path detection module
        private IEnumerator CheckPathDeviation()
        {
            while (true)
            {
                yield return new WaitForSeconds(pathCheckInterval);

                if (isPathValid && currentPath != null && currentPath.Count > currentPathIndex)
                {
                    Vector3 targetPosition = GetPointVector3(currentPath[currentPathIndex].Point);
                    float deviationDistance = Vector3.Distance(transform.position, targetPosition);

                    if (deviationDistance > maxPathDeviationDistance)
                    {
                        HandlePathDeviation(deviationDistance);
                    }
                }
            }
        }

        private void HandlePathDeviation(float deviationDistance)
        {
            Debug.LogWarning($"Robot {Id} has deviated from the path. Deviation distance: {deviationDistance}. Attempting to recover.");

            int nearestPointIndex = FindNearestPathPoint();

            if (nearestPointIndex != -1)
            {
                if (deviationDistance <= pathDeviationDistanceThreshold)
                {
                    currentPathIndex = nearestPointIndex;
                    List<PathSegment> remainingMovement = currentMovement.GetRange(nearestPointIndex, currentPath.Count - nearestPointIndex);
                    List<ConnectionPoint> remainingPath = new List<ConnectionPoint>();
                    List<float> remainingTimes = new List<float>();
                    List<float> remainingSpeeds = new List<float>();

                    foreach (PathSegment segment in remainingMovement)
                    {
                        remainingPath.Add(segment.Point);
                        remainingTimes.Add(segment.Time);
                        remainingSpeeds.Add(segment.Speed);
                    }
                    
                    if (currentMovementCoroutine != null)
                    {
                        StopCoroutine(currentMovementCoroutine);
                    }

                    currentMovementCoroutine = StartCoroutine(MoveToNearestPointThenFollowPath(remainingPath, remainingTimes, remainingSpeeds));
                    
                    Debug.Log($"Robot {Id} found a nearest path point. Resuming from index {currentPathIndex}");
                }
                else
                {
                    Debug.LogWarning($"Robot {Id} deviation too large. Initiating replan.");
                    StartCoroutine(ReplanPath());
                }
            }
            else
            {
                Debug.LogWarning($"Robot {Id} unable to find nearest path point. Initiating replan.");
                StartCoroutine(ReplanPath());
            }
        }

        private int FindNearestPathPoint()
        {
            float minDistance = float.MaxValue;
            int nearestIndex = -1;

            for (int i = currentPathIndex; i < currentPath.Count; i++)
            {
                float distance = Vector3.Distance(transform.position, GetPointVector3(currentPath[i].Point));
                if (distance < minDistance && distance <= pathDeviationDistanceThreshold)
                {
                    minDistance = distance;
                    nearestIndex = i;
                }
            }

            return nearestIndex;
        }
        # endregion
        
        # region status check module
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

        private void UpdateRobotStatus()
        {
            bool previousFreeStatus = IsFree;
            
            SetState(RobotState.Moving, false);
            SetState(RobotState.Picking, false);
            currentMovementCoroutine = null;

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

        private void UpdateCurrentMovementInfo(List<ConnectionPoint> path, List<float> times, List<float> speeds)
        {
            for (int i = 0; i < path.Count; i++)
            {
                currentMovement.Add(new PathSegment(path[i], times[i], speeds[i]));
            }
        }

        private void UpdateDWAMaxSpeed(float newMaxSpeed)
        {
            dwaPlanner.MaxSpeed = newMaxSpeed;
            // Debug.Log($"Robot {Id}: Updated DWA max speed to {newMaxSpeed}");
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
        # endregion

        # region visualization module
        private void OnDrawGizmos()
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(transform.position, detectionRadius);

            Gizmos.color = new Color(1, 1, 0, 0.1f); 
            Gizmos.DrawSphere(transform.position, detectionRadius);

            UnityEditor.Handles.Label(transform.position + Vector3.up * 2, $"Detection Radius: {detectionRadius}");

            Rigidbody rb = GetComponent<Rigidbody>();
            if (rb != null)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(transform.position, transform.position + rb.velocity * 0.5f);
                Gizmos.color = Color.red;
                Gizmos.DrawRay(transform.position, (GetTargetPosition(currentOrder.PickingPoint) - transform.position).normalized);
            }
        }

        private void VisualizePath(List<ConnectionPoint> path)
        {
            if (pathLineRenderer == null)
            {
                pathLineRenderer = gameObject.AddComponent<LineRenderer>();
            }

            pathLineRenderer.positionCount = path.Count;
            pathLineRenderer.startWidth = lineWidth;
            pathLineRenderer.endWidth = lineWidth;
            pathLineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            pathLineRenderer.startColor = pathColor;
            pathLineRenderer.endColor = pathColor;

            Vector3[] positions = new Vector3[path.Count];
            for (int i = 0; i < path.Count; i++)
            {
                Point point = (Point)path[i].Point;
                positions[i] = new Vector3((float)point.X, 0.1f, (float)point.Y);
            }

            pathLineRenderer.SetPositions(positions);
        }

        private void ClearPathVisualization()
        {
            if (pathLineRenderer != null)
            {
                pathLineRenderer.positionCount = 0;
            }
        }
        # endregion
    }
}