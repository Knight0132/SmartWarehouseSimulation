using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Map;

namespace PathPlanning
{
    public enum SearchAlgorithm
    {
        BFS, 
        Astar_Basic,
        Astar_Traffic
    }

    public enum SearchMethod
    {
        Euclidean_Distance,
        Manhattan_Distance,
        NotApplicable
    }

    public class PathPlanner
    {
        private Graph graph;
        private SearchMethod searchMethod;
        private SearchAlgorithm searchAlgorithm;
        private float maxAcceleration;
        private float maxDeceleration;
        public RoutePoint startPoint;
        public RoutePoint endPoint;
        public float speed;
        
        public PathPlanner(Graph graph, SearchAlgorithm algorithm, SearchMethod method, float maxAcceleration, float maxDeceleration)
        {
            this.graph = graph;
            this.searchAlgorithm = algorithm;
            this.searchMethod = method;
            this.maxAcceleration = maxAcceleration;
            this.maxDeceleration = maxDeceleration;
        }

        public (List<ConnectionPoint> path, List<float> speeds) FindPath(
            RoutePoint startPoint, 
            RoutePoint endPoint, 
            float speed)
        {
            Debug.Log($"PathPlanner.FindPath called with algorithm: {this.searchAlgorithm}");
            switch (this.searchAlgorithm)
            {
                case SearchAlgorithm.Astar_Traffic:
                    return Astar_Traffic.AstarAlgorithm(graph, startPoint, endPoint, speed, this.maxAcceleration, this.maxDeceleration, this.searchMethod);
                case SearchAlgorithm.Astar_Basic:
                    return Astar_Basic.AstarAlgorithm(this.graph, startPoint, endPoint, speed, this.searchMethod);
                case SearchAlgorithm.BFS:
                    return BFS.BFSAlgorithm(this.graph, startPoint, endPoint, speed, this.searchMethod);
                default:
                    return (null, null);
            }
        }
    }
}
