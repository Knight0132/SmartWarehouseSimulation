using UnityEngine;
using UnityEngine.UI;
using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using PathPlanning;

namespace Map {
    public class MapLoader : MonoBehaviour
    {
        public TextAsset jsonFile;
        public IndoorSpace indoorSpace;
        public MapVisualizer mapVisualizer;

        [Range(0, 100)]
        public int cameraHeight;
        [Range(0, 100)]
        public float cameraPositionX;
        [Range(0, 100)]
        public float cameraPositionY;
        [Range(0, 100)]
        public float width;
        [Range(0, 100)]
        public float length;
        public Vector3 startPoint;
        public Vector3 endPoint;

        private Graph graph;
        private SearchAlgorithm selectedAlgorithm = SearchAlgorithm.Astar_Traffic_Completed;
        private float speed;

        private void Start()
        {
            indoorSpace = LoadJson();
            Debug.Log(indoorSpace.ToJson());

            if (indoorSpace == null)
            {
                throw new ArgumentNullException(nameof(indoorSpace), "IndoorSpace must not be null.");
            }
            if (indoorSpace.CellSpaces == null)
            {
                throw new ArgumentNullException(nameof(indoorSpace.CellSpaces), "CellSpaces must not be null.");
            }

            Camera.main.transform.position = new Vector3(cameraPositionX, cameraHeight, cameraPositionY);
            Camera.main.transform.rotation = Quaternion.Euler(90, 0, 0);
            Camera.main.orthographic = true;
            Camera.main.orthographicSize = cameraHeight * 0.5f;

            mapVisualizer.VisualizeMap(indoorSpace);
            this.graph = GenerateRouteGraph(indoorSpace);
        }

        public Vector3 GetRandomNavigablePosition(IndoorSpace indoorSpace)
        {
            bool validPositionFound = false;
            int attempts = 0;
            Vector3 randomPosition = Vector3.zero;

            while (!validPositionFound && attempts < 100)
            {
                randomPosition = new Vector3(UnityEngine.Random.Range(0f, length), 0f, UnityEngine.Random.Range(0f, width));
                CellSpace cellSpace = indoorSpace.GetCellSpaceFromCoordinates(randomPosition);

                if (cellSpace != null && (bool)cellSpace.Properties["navigable"])
                {
                    validPositionFound = true;
                    Debug.Log($"Valid position found at: {randomPosition}");
                }
                else
                {
                    Debug.Log($"Attempt {attempts}: Position {randomPosition} is not navigable.");
                }
                attempts++;
            }
            
            if (validPositionFound)
            {
                Debug.Log("Position set to: " + randomPosition);
                return randomPosition;
            }
            else
            {
                Debug.Log("Failed to find a navigable position after 100 attempts.");
                return Vector3.zero;
            }
        }

        public List<Tuple<ConnectionPoint, float>> CalculatePath(Vector3 start, Vector3 end)
        {
            RoutePoint startPoint = graph.GetRoutePointFromCoordinate(start, true);
            RoutePoint endPoint = graph.GetRoutePointFromCoordinate(end, true);

            if (startPoint != null && endPoint != null)
            {
                return PathPlanner.FindPath(selectedAlgorithm, graph, startPoint, endPoint, speed);
            }
            return new List<Tuple<ConnectionPoint, float>>();
        }

        public IndoorSpace LoadJson()
        {
            JsonSerializerSettings settings = new JsonSerializerSettings
            {
                Converters = new List<JsonConverter> { new GeometryConverter() }
            };

            IndoorSpace indoorSpace = null;

            try
            {
                indoorSpace = JsonConvert.DeserializeObject<IndoorSpace>(jsonFile.text, settings);
                Debug.Log("JSON Deserialized successfully");
            }
            catch (Exception ex)
            {
                Debug.LogError("Error during JSON deserialization: " + ex.Message);
            }

            return indoorSpace;
        }

        public Graph GenerateRouteGraph(IndoorSpace indoorSpace)
        {
            Graph graph = new Graph(indoorSpace);
            Hypergraph hypergraph = indoorSpace.GetHypergraph();

            foreach (var connectionPoint in hypergraph.HyperNodes)
            {
                RoutePoint routePoint = new RoutePoint();
                routePoint.AddConnectionPoint(connectionPoint);
                
                foreach (var rLineGroup in hypergraph.HyperEdges)
                {
                    if (rLineGroup.ConnectionPoints.Contains(connectionPoint.Id))
                    {
                        foreach (var innerEdge in rLineGroup.InnerEdges)
                        {
                            if (innerEdge[0] == connectionPoint.Id)
                            {
                                routePoint.AddChild(indoorSpace.GetConnectionPointFromId(innerEdge[1]));
                            }
                            else if (innerEdge[1] == connectionPoint.Id)
                            {
                                routePoint.AddParent(indoorSpace.GetConnectionPointFromId(innerEdge[0]));
                            }
                        }
                    }
                }

                graph.AddNode(routePoint);
            }
            Debug.Log("Route Graph Generated");
            return graph;
        }
    }
}
