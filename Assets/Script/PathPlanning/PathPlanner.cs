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
        Astar_Traffic_Completed
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
        public RoutePoint startPoint;
        public RoutePoint endPoint;
        public float speed;
        
        public PathPlanner(Graph graph, SearchAlgorithm algorithm, SearchMethod method)
        {
            this.graph = graph;
            this.searchAlgorithm = algorithm;
            this.searchMethod = method;
        }

        public (List<ConnectionPoint> path, List<float> speeds) FindPath(
            RoutePoint startPoint, 
            RoutePoint endPoint, 
            float speed)
        {
            Debug.Log($"PathPlanner.FindPath called with algorithm: {this.searchAlgorithm}");
            switch (this.searchAlgorithm)
            {
                // case SearchAlgorithm.Astar_Traffic_Completed:
                //     return Astar_Traffic_Completed.AstarAlgorithm_TC(graph, startPoint, endPoint, speed);
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
