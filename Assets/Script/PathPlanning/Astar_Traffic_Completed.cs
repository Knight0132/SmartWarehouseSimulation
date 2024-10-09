using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using Map;
using System.Diagnostics;
using NetTopologySuite.Geometries;
using PointLocation = Map.Graph.PointLocation;

namespace PathPlanning
{
    public class Astar_Traffic
    {
        public static (List<ConnectionPoint> path, List<float> times, List<float> speeds, bool[,,] timeSpaceMatrix) AstarAlgorithm(
            Graph graph, 
            DynamicOccupancyLayer personalOccupancyLayer,
            RoutePoint startPosition, 
            RoutePoint endPosition, 
            float startTime, 
            float defaultSpeed,
            float maxAcceleration,
            float maxDeceleration,
            SearchMethod searchMethod)
        {
            if (graph == null)
            {
                UnityEngine.Debug.LogError("Graph is null");
                return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), new bool[0, 0, 0]);
            }

            if (startPosition == null || endPosition == null)
            {
                UnityEngine.Debug.LogError("Start position or end position is null");
                return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), new bool[0, 0, 0]);
            }

            if (startPosition.ConnectionPoint == null || endPosition.ConnectionPoint == null)
            {
                UnityEngine.Debug.LogError("Start or end ConnectionPoint is null");
                return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), new bool[0, 0, 0]);
            }

            UnityEngine.Debug.Log($"Starting A* algorithm from {startPosition.ConnectionPoint.Id} to {endPosition.ConnectionPoint.Id}");

            var stopwatch = new Stopwatch();
            stopwatch.Start();
            float lastSpeed = defaultSpeed;
            Dictionary<RoutePoint, float> gScore = new Dictionary<RoutePoint, float>();
            Dictionary<RoutePoint, float> fScore = new Dictionary<RoutePoint, float>();
            Dictionary<RoutePoint, Tuple<RoutePoint, float, float>> previous = new Dictionary<RoutePoint, Tuple<RoutePoint, float, float>>();
            Dictionary<RoutePoint, float> timeAtNode = new Dictionary<RoutePoint, float>();
            var openSet = new PriorityQueue<RoutePoint, float>();
            
            foreach (var node in graph.RoutePoints)
            {
                gScore[node] = float.PositiveInfinity;
                fScore[node] = float.PositiveInfinity;
                timeAtNode[node] = startTime;
            }

            gScore[startPosition] = 0;
            fScore[startPosition] = HeuristicCostEstimate(personalOccupancyLayer, startPosition, endPosition, defaultSpeed, searchMethod);
            openSet.Enqueue(startPosition, fScore[startPosition]);
            timeAtNode[startPosition] = startTime;

            while (openSet.TryDequeue(out RoutePoint current, out float currentPriority))
            {
                if (current == null || current.ConnectionPoint == null)
                {
                    UnityEngine.Debug.LogError("Encountered null RoutePoint or ConnectionPoint");
                    continue;
                }

                if (current == endPosition)
                {
                    stopwatch.Stop();
                    UnityEngine.Debug.Log($"A* Algorithm Execution Time: {stopwatch.ElapsedMilliseconds}ms");
                    var (path, times, speeds) = ReconstructPath(previous, current, defaultSpeed);

                    bool[,,] timeSpaceMatrix = GenerateTimeSpaceMatrix(graph, personalOccupancyLayer, path, times);
                    return (path, times, speeds, timeSpaceMatrix);
                }

                foreach (var neighborConnection in current.Children)
                {
                    RoutePoint neighbor = graph.GetRoutePointFormConnectionPoint(neighborConnection);
                    if (neighbor == null) continue;
                    
                    Layer layerOfNeighbor = graph.GetLayerFromConnectionPoint(neighborConnection, false);
                    float currentSpeed = ModifySpeed(graph, current.ConnectionPoint, neighborConnection, defaultSpeed, lastSpeed, 
                                                    layerOfNeighbor, Time.deltaTime, maxAcceleration, maxDeceleration);
                    float distance = (float)current.ConnectionPoint.Point.Distance(neighborConnection.Point);
                    float timeToReachNeighbor = distance / currentSpeed;
                    float arrivalTimeAtNeighbor = timeAtNode[current] + timeToReachNeighbor;

                    bool isOccupied = personalOccupancyLayer.IsOccupied(neighborConnection, arrivalTimeAtNeighbor);
                    float occupancyPenalty = isOccupied ? float.PositiveInfinity : 0f;

                    float tentativeGScore = gScore[current] + (distance / currentSpeed) + occupancyPenalty;

                    if (tentativeGScore < gScore[neighbor])
                    {
                        previous[neighbor] = new Tuple<RoutePoint, float, float>(current, currentSpeed, arrivalTimeAtNeighbor);
                        gScore[neighbor] = tentativeGScore;
                        timeAtNode[neighbor] = arrivalTimeAtNeighbor;
                        float f = gScore[neighbor] + HeuristicCostEstimate(personalOccupancyLayer, neighbor, endPosition, currentSpeed, searchMethod);
                        fScore[neighbor] = f;
                        openSet.Enqueue(neighbor, f);
                        lastSpeed = currentSpeed;
                    }
                }
            }

            UnityEngine.Debug.LogWarning($"No path found from {startPosition.ConnectionPoint.Id} to {endPosition.ConnectionPoint.Id}");
            return (new List<ConnectionPoint>(), new List<float>(), new List<float>(), new bool[0, 0, 0]);
        }

        private static (List<ConnectionPoint> path, List<float> times, List<float> speeds) ReconstructPath(
            Dictionary<RoutePoint, Tuple<RoutePoint, float, float>> cameFrom, 
            RoutePoint current, 
            float initialSpeed)
        {
            var path = new List<ConnectionPoint>();
            var times = new List<float>();
            var speeds = new List<float>();
            float speed = initialSpeed;

            while (current != null)
            {
                if (current.ConnectionPoint == null)
                {
                    UnityEngine.Debug.LogError($"Encountered RoutePoint with null ConnectionPoint while reconstructing path");
                    break;
                }

                path.Add(current.ConnectionPoint);
                speeds.Add(speed);

                if (!cameFrom.TryGetValue(current, out var previousInfo))
                {
                    break;
                }

                times.Add(previousInfo.Item3);
                speed = previousInfo.Item2;
                current = previousInfo.Item1;
            }

            path.Reverse();
            times.Reverse();
            speeds.Reverse();

            if (times.Count < path.Count)
            {
                float lastTime = times.Count > 0 ? times[times.Count - 1] : 0;
                while (times.Count < path.Count)
                {
                    lastTime += 0.1f;
                    times.Add(lastTime);
                }
            }

            return (path, times, speeds);
        }

        private static bool[,,] GenerateTimeSpaceMatrix(Graph graph, DynamicOccupancyLayer personalOccupancyLayer, List<ConnectionPoint> path, List<float> times)
        {
            int width = (int)personalOccupancyLayer.Width;
            int length = (int)personalOccupancyLayer.Length;
            int timeSteps = personalOccupancyLayer.TimeSteps;
            bool[,,] matrix = new bool[width, length, timeSteps];

            for (int i = 0; i < path.Count; i++)
            {
                ConnectionPoint connectionPoint = path[i];
                float time = times[i];

                if (time >= personalOccupancyLayer.GetStartTime() && time <= personalOccupancyLayer.GetEndTime())
                {
                    personalOccupancyLayer.SetOccupancy(connectionPoint, time, true);
                }
            }

            return matrix;
        }

        private static float HeuristicCostEstimate(DynamicOccupancyLayer personalOccupancyLayer, RoutePoint a, RoutePoint b, float speed, SearchMethod searchMethod)
        {
            var p1 = (Point)a.ConnectionPoint.Point;
            var p2 = (Point)b.ConnectionPoint.Point;
            float distance;
            
            switch (searchMethod)
            {
                case SearchMethod.Euclidean_Distance:
                    distance = (float)Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
                    break;
                case SearchMethod.Manhattan_Distance:
                    distance = (float)(Math.Abs(p1.X - p2.X) + Math.Abs(p1.Y - p2.Y));
                    break;
                default:
                    UnityEngine.Debug.LogWarning($"Unknown search method: {searchMethod}. Using Euclidean distance.");
                    distance = (float)Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
                    break;
            }
            
            float baseHeuristic = distance / speed;
            
            return baseHeuristic;
        }

        private static float ModifySpeed(Graph graph, ConnectionPoint current, ConnectionPoint neighbor, 
            float defaultSpeed, float lastSpeed, Layer layerOfNeighbor, float deltaTime, 
            float maxAcceleration, float maxDeceleration)
        {
            float speedLimit = layerOfNeighbor.SpeedLimit();
            float targetSpeed = defaultSpeed;

            PointLocation currentLocation = graph.GetPointLocation(current);
            PointLocation neighborLocation = graph.GetPointLocation(neighbor);

            switch (currentLocation)
            {
                case PointLocation.InAisle when neighborLocation == PointLocation.ApproachingIntersection:
                    targetSpeed = speedLimit * 0.8f;
                    break;
                case PointLocation.ApproachingIntersection:
                case PointLocation.InIntersection:
                    targetSpeed = speedLimit;
                    break;
                case PointLocation.OutIntersection when neighborLocation == PointLocation.InAisle:
                    targetSpeed = Mathf.Min(defaultSpeed, speedLimit);
                    break;
                default:
                    targetSpeed = Mathf.Min(defaultSpeed, speedLimit);
                    break;
            }

            float speedDifference = targetSpeed - lastSpeed;
            float acceleration = speedDifference / deltaTime;
            
            acceleration = Mathf.Clamp(acceleration, -maxDeceleration, maxAcceleration);
            
            float newSpeed = lastSpeed + acceleration * deltaTime;

            newSpeed = Mathf.Clamp(newSpeed, speedLimit * 0.5f, speedLimit);

            float randomFactor = UnityEngine.Random.Range(0.98f, 1.02f);
            newSpeed *= randomFactor;

            return newSpeed;
        }
    }
}