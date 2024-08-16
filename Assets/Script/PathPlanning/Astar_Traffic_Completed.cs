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

        public static List<Tuple<ConnectionPoint, float>> AstarAlgorithm_TC(Graph graph, RoutePoint startPosition, RoutePoint endPosition, float speed)
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
            Dictionary<RoutePoint, float> g_score = new Dictionary<RoutePoint, float>();
            Dictionary<RoutePoint, Tuple<RoutePoint, float>> previous = new Dictionary<RoutePoint, Tuple<RoutePoint, float>>();
            Dictionary<RoutePoint, float> h_score = new Dictionary<RoutePoint, float>();
            Dictionary<RoutePoint, float> f_score = new Dictionary<RoutePoint, float>();
            List<RoutePoint> openSet = new List<RoutePoint>();

            foreach (RoutePoint node in graph.RoutePoints)
            {
                g_score[node] = Mathf.Infinity;
                previous[node] = new Tuple<RoutePoint, float>(null, 0f);
                h_score[node] = hScore(node, endPosition, speed);
                f_score[node] = Mathf.Infinity;
                openSet.Add(node);
            }

            g_score[startPosition] = 0;
            f_score[startPosition] = hScore(startPosition, endPosition, speed);

            while (openSet.Count != 0)
            {
                RoutePoint current = FindLowestFScore(openSet, f_score);

                if (current == endPosition)
                {
                    stopwatch.Stop();
                    UnityEngine.Debug.Log("A* Algorithm Execution Time: " + stopwatch.ElapsedMilliseconds + "ms");
                    LogPreviousDictionary(previous);
                    return GetPath(previous, startPosition, endPosition, speed);
                }
                openSet.Remove(current);

                foreach (ConnectionPoint neighbor in current.Children)
                {
                    bool sequence = false;
                    RoutePoint neighborNode = graph.GetRoutePointFormConnectionPoint(neighbor);

                    Layer layerOfNeighbor = graph.GetLayerFromConnectionPoint(neighbor, sequence);

                    float currentSpeed = speed;
                    float distance = (float)current.ConnectionPoint.Point.Distance(neighbor.Point);
                    
                    currentSpeed = ModifySpeed(graph, current.ConnectionPoint, neighbor, speed, currentSpeed, lastSpeed, layerOfNeighbor);

                    float timeCost = distance / currentSpeed;
                    float tentative_gscore = g_score[current] + timeCost;

                    if (tentative_gscore < g_score[neighborNode])
                    {
                        if (current == null)
                        {
                            UnityEngine.Debug.LogError($"Attempting to set null as previous for {neighborNode.ConnectionPoint.Id}");
                        }
                        else
                        {
                            previous[neighborNode] = new Tuple<RoutePoint, float>(current, currentSpeed);
                            g_score[neighborNode] = tentative_gscore;
                            f_score[neighborNode] = g_score[neighborNode] + h_score[neighborNode];
                            
                            if (!openSet.Contains(neighborNode))
                            {
                                openSet.Add(neighborNode);
                            }
                            lastSpeed = currentSpeed;
                        }
                    }
                }
            }

            UnityEngine.Debug.Log("No path found");
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

        private static List<Tuple<ConnectionPoint, float>> GetPath(Dictionary<RoutePoint, Tuple<RoutePoint, float>> previous, RoutePoint startPosition, RoutePoint endPosition, float speed)
        {
            List<Tuple<ConnectionPoint, float>> path = new List<Tuple<ConnectionPoint, float>>();
            HashSet<RoutePoint> visitedPoints = new HashSet<RoutePoint>();

            RoutePoint current = endPosition;
            float currentSpeed = 0f;
            while (current != null && current != startPosition)
            {
                if (visitedPoints.Contains(current))
                {
                    UnityEngine.Debug.LogError($"Cycle detected in path at RoutePoint: {current.ConnectionPoint.Id}");
                    break;
                }
                visitedPoints.Add(current);

                if (!previous.ContainsKey(current))
                {
                    UnityEngine.Debug.LogError($"Path is incomplete. Missing key in previous dictionary: {current.ConnectionPoint.Id}");
                    break;
                }

                path.Add(new Tuple<ConnectionPoint, float>(current.ConnectionPoint, currentSpeed));
                var prevTuple = previous[current];
                current = prevTuple.Item1;
                currentSpeed = prevTuple.Item2;

                // if (current == null)
                // {
                //     UnityEngine.Debug.LogError($"Unexpected null RoutePoint in path. Last valid point: {path[path.Count - 1].Item1.Id}");
                //     break;
                // }
            }

            if (current == startPosition)
            {
                path.Add(new Tuple<ConnectionPoint, float>(startPosition.ConnectionPoint, speed));
                path.Reverse();
                UnityEngine.Debug.Log($"Path found with {path.Count} points");
                return path;
            }
            else
            {
                UnityEngine.Debug.LogError($"Failed to construct complete path. Path size: {path.Count}, Last point: {(path.Count > 0 ? path[path.Count - 1].Item1.Id : "N/A")}");
                return new List<Tuple<ConnectionPoint, float>>();
            }
        }

        private static float fScore(float gscore, float hscore)
        {
            return gscore + hscore;
        }

        private static float hScore(RoutePoint currentPosition, RoutePoint endPosition, float speed)
        {
            Point current = (Point)currentPosition.ConnectionPoint.Point;
            Point end = (Point)endPosition.ConnectionPoint.Point;

            float distance = (float)Math.Sqrt(Math.Pow(current.X - end.X, 2) + Math.Pow(current.Y - end.Y, 2));

            float timeCost = distance / speed;

            return timeCost;
        }

        private static float ModifySpeed(Graph graph, ConnectionPoint current, ConnectionPoint neighbor, float defaultSpeed, float currentSpeed, float lastSpeed, Layer layerOfNeighbor)
        {
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
