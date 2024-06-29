using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Map;

namespace PathPlanning
{
    public enum SearchAlgorithm
    {
        Astar_Basic_Distance,
        Astar_Basic_Time,
        Astar_Traffic_Completed
    }

    public class PathPlanner
    {
        public Graph graph;
        public RoutePoint startPoint;
        public RoutePoint endPoint;
        public float speed;
        
        public static List<Tuple<ConnectionPoint, float>> FindPath(SearchAlgorithm searchAlgorithm, Graph graph, RoutePoint startPoint, RoutePoint endPoint, float speed)
        {
            switch (searchAlgorithm)
            {
                case SearchAlgorithm.Astar_Traffic_Completed:
                    return Astar_Traffic_Completed.AstarAlgorithm_TC(graph, startPoint, endPoint, speed);
                case SearchAlgorithm.Astar_Basic_Distance:
                    return Astar_Basic_Distance.AstarAlgorithm_Basic_Distance(graph, startPoint, endPoint, speed);
                case SearchAlgorithm.Astar_Basic_Time:
                    return Astar_Basic_Time.AstarAlgorithm_Basic_Time(graph, startPoint, endPoint, speed);
                default:
                    return null;
            }
        }
    }
}
