using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using Map;
using System.Diagnostics;
using NetTopologySuite.Geometries;

namespace PathPlanning
{
    public class Astar_Traffic_Completed
    {
        public RoutePoint startPosition;
        public RoutePoint endPosition;
        public Graph graph;
        public List<Tuple<ConnectionPoint, float>> path = new List<Tuple<ConnectionPoint, float>>();

        public static List<Tuple<ConnectionPoint, float>> AstarAlgorithm_TC(Graph graph, RoutePoint startPosition, RoutePoint endPosition, float defaultSpeed)
        {
            if (startPosition == null || endPosition == null)
            {
                UnityEngine.Debug.LogError("Start position or end position is null");
                return new List<Tuple<ConnectionPoint, float>>();
            }
            UnityEngine.Debug.Log($"Starting A* algorithm from {startPosition.ConnectionPoint.Id} to {endPosition.ConnectionPoint.Id}");

            var stopwatch = new Stopwatch();
            stopwatch.Start();
            float lastSpeed = 0f;
            Dictionary<RoutePoint, float> gScore = new Dictionary<RoutePoint, float>();
            Dictionary<RoutePoint, float> fScore = new Dictionary<RoutePoint, float>();
            Dictionary<RoutePoint, Tuple<RoutePoint, float>> previous = new Dictionary<RoutePoint, Tuple<RoutePoint, float>>();
            var openSet = new PriorityQueue<RoutePoint, float>();
            
            foreach (var node in graph.RoutePoints)
            {
                gScore[node] = float.PositiveInfinity;
                fScore[node] = float.PositiveInfinity;
            }

            gScore[startPosition] = 0;
            fScore[startPosition] = HeuristicCostEstimate(startPosition, endPosition, defaultSpeed);
            openSet.Enqueue(startPosition, fScore[startPosition]);

            while (openSet.TryDequeue(out RoutePoint current, out float currentPriority))
            {
                if (current == endPosition)
                {
                    stopwatch.Stop();
                    UnityEngine.Debug.Log($"A* Algorithm Execution Time: {stopwatch.ElapsedMilliseconds}ms");
                    return ReconstructPath(previous, current, defaultSpeed);
                }

                foreach (var neighborConnection in current.Children)
                {
                    RoutePoint neighbor = graph.GetRoutePointFormConnectionPoint(neighborConnection);
                    if (neighbor == null) continue;

                    Layer layerOfNeighbor = graph.GetLayerFromConnectionPoint(neighborConnection, false);
                    float currentSpeed = ModifySpeed(graph, current.ConnectionPoint, neighborConnection, defaultSpeed, lastSpeed, layerOfNeighbor);
                    float distance = (float)current.ConnectionPoint.Point.Distance(neighborConnection.Point);
                    float tentativeGScore = gScore[current] + (distance / currentSpeed);

                    if (tentativeGScore < gScore[neighbor])
                    {
                        previous[neighbor] = new Tuple<RoutePoint, float>(current, currentSpeed);
                        gScore[neighbor] = tentativeGScore;
                        float f = gScore[neighbor] + HeuristicCostEstimate(neighbor, endPosition, currentSpeed);
                        fScore[neighbor] = f;
                        openSet.Enqueue(neighbor, f);
                        lastSpeed = currentSpeed;
                    }
                }
            }

            UnityEngine.Debug.LogWarning($"No path found from {startPosition.ConnectionPoint.Id} to {endPosition.ConnectionPoint.Id}");
            return new List<Tuple<ConnectionPoint, float>>();
        }

        private static RoutePoint FindLowestFScore(List<RoutePoint> openSet, Dictionary<RoutePoint, float> f_score)
        {
            RoutePoint lowest = openSet[0];
            foreach (RoutePoint node in openSet)
            {
                if (f_score[node] < f_score[lowest])
                {
                    lowest = node;
                }
            }
            return lowest;
        }

        private static List<Tuple<ConnectionPoint, float>> ReconstructPath(Dictionary<RoutePoint, Tuple<RoutePoint, float>> cameFrom, RoutePoint current, float initialSpeed)
        {
            var path = new List<Tuple<ConnectionPoint, float>>();
            float speed = initialSpeed;

            while (cameFrom.ContainsKey(current))
            {
                path.Add(new Tuple<ConnectionPoint, float>(current.ConnectionPoint, speed));
                var previousInfo = cameFrom[current];
                speed = previousInfo.Item2;
                current = previousInfo.Item1;
            }

            path.Add(new Tuple<ConnectionPoint, float>(current.ConnectionPoint, initialSpeed));
            path.Reverse();

            return path;
        }

        private static float FScore(float gscore, float hscore)
        {
            return gscore + hscore;
        }

        private static float HeuristicCostEstimate(RoutePoint a, RoutePoint b, float speed)
        {
            var p1 = (Point)a.ConnectionPoint.Point;
            var p2 = (Point)b.ConnectionPoint.Point;
            float distance = (float)Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
            return distance / speed;
        }

        private static float ModifySpeed(Graph graph, ConnectionPoint current, ConnectionPoint neighbor, float defaultSpeed, float lastSpeed, Layer layerOfNeighbor)
        {
            float currentSpeed = 0f;
            if (graph.InAisle(current) && graph.InAisle(neighbor))
            {
                return currentSpeed = Mathf.Min(defaultSpeed, layerOfNeighbor.SpeedLimit());
            }
            else if (graph.InAisle(current) && graph.NearIntersection(neighbor))
            {
                float deaccelerationFactor = layerOfNeighbor.DeaccelerationFactor();
                return currentSpeed *= deaccelerationFactor;
            }
            else if (graph.NearIntersection(current) || (graph.InIntersection(current) && graph.OutIntersection(neighbor)))
            {
                return currentSpeed = lastSpeed;
            }
            else if (graph.OutIntersection(current) && graph.InAisle(neighbor))
            {
                return currentSpeed = Mathf.Min(defaultSpeed, layerOfNeighbor.SpeedLimit());
            }
            else if (graph.OutIntersection(current) && graph.NearIntersection(neighbor))
            {
                currentSpeed = Mathf.Min(defaultSpeed, layerOfNeighbor.SpeedLimit());
                float deaccelerationFactor = layerOfNeighbor.DeaccelerationFactor();
                return currentSpeed *= deaccelerationFactor;
            }
            else
            {
                return currentSpeed = defaultSpeed;
            }
        }
        private static void LogPreviousDictionary(Dictionary<RoutePoint, Tuple<RoutePoint, float>> previous)
        {
            if (previous == null || previous.Count == 0)
            {
                UnityEngine.Debug.Log("Previous dictionary is empty or null");
                return;
            }

            System.Text.StringBuilder sb = new System.Text.StringBuilder();
            sb.AppendLine("Previous Dictionary Contents:");

            foreach (var kvp in previous)
            {
                string currentId = kvp.Key.ConnectionPoint?.Id ?? "null";
                string previousId = kvp.Value.Item1?.ConnectionPoint?.Id ?? "null";
                float speed = kvp.Value.Item2;

                sb.AppendLine($"Current: {currentId} <- Previous: {previousId}, Speed: {speed:F2}");
            }

            sb.AppendLine($"Total entries: {previous.Count}");

            UnityEngine.Debug.Log(sb.ToString());
        }
    }
}
