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
                RoutePoint current = openSet[0];
                foreach (RoutePoint node in openSet)
                {
                    if (f_score[node] < f_score[current])
                    {
                        current = node;
                    }
                }

                if (current == endPosition)
                {
                    stopwatch.Stop();
                    UnityEngine.Debug.Log("A* Algorithm Execution Time: " + stopwatch.ElapsedMilliseconds + "ms");
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

            UnityEngine.Debug.Log("No path found");
            return new List<Tuple<ConnectionPoint, float>>();
        }

        private static List<Tuple<ConnectionPoint, float>> GetPath(Dictionary<RoutePoint, Tuple<RoutePoint, float>> previous, RoutePoint startPosition, RoutePoint endPosition, float speed)
        {
            List<Tuple<ConnectionPoint, float>> path = new List<Tuple<ConnectionPoint, float>>();

            RoutePoint current = endPosition;
            float currentSpeed = 0f;
            while (current != null && current != startPosition)
            {
                path.Add(new Tuple<ConnectionPoint, float>(current.ConnectionPoint, currentSpeed));
                current = previous[current].Item1;
                currentSpeed = previous[current].Item2;
            }

            path.Add(new Tuple<ConnectionPoint, float>(startPosition.ConnectionPoint, speed));
            path.Reverse();
            path.RemoveAt(path.Count - 1);
            
            List<string> pathIds = new List<string>();
            List<float> speeds = new List<float>();

            foreach (Tuple<ConnectionPoint, float> connectionPoint in path)
            {
                pathIds.Add(connectionPoint.Item1.Id);
                speeds.Add(connectionPoint.Item2);
            }
            UnityEngine.Debug.Log("Path: " + string.Join(" -> ", pathIds.ToArray()));
            UnityEngine.Debug.Log("Speeds: " + string.Join(" -> ", speeds.ToArray()));

            return path;
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
    }
}
