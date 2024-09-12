using UnityEngine;
using UnityEngine.UI;
using System;
using System.IO;
using System.Collections.Generic;
using System.Threading.Tasks;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using NetTopologySuite.IO;
using PathPlanning;
using CentralControl;

namespace Map {
    public class MapLoader : MonoBehaviour
    {
        public TextAsset jsonFile;
        public IndoorSpace indoorSpace;
        public MapVisualizer mapVisualizer;

        public float height = 0.0f;

        [Range(0, 100)]
        public float width;
        [Range(0, 100)]
        public float length;

        private Graph graph;
        private MapGrid mapGrid;
        private float speed;

        private async void Start()
        {
            indoorSpace = await LoadJsonAsync();

            if (indoorSpace == null)
            {
                throw new ArgumentNullException(nameof(indoorSpace), "IndoorSpace must not be null.");
            }
            
            if (indoorSpace.CellSpaces == null)
            {
                throw new ArgumentNullException(nameof(indoorSpace.CellSpaces), "CellSpaces must not be null.");
            }

            mapVisualizer.VisualizeMap(indoorSpace);
            this.graph = GenerateRouteGraph(indoorSpace);
            this.mapGrid = SetMapGrid(width, height, length);

            EventManager.Instance.TriggerMapLoaded();
        }

        public async Task<IndoorSpace> LoadJsonAsync()
        {
            string jsonText = jsonFile.text;
            return await Task.Run(() =>
            {
                JsonSerializerSettings settings = new JsonSerializerSettings
                {
                    Converters = new List<JsonConverter> { new GeometryConverter() }
                };

                var indoorSpace = JsonConvert.DeserializeObject<IndoorSpace>(jsonText, settings);
                if (indoorSpace != null)
                {
                    indoorSpace.LoadProperties();
                }
                Debug.Log("JSON Deserialized successfully");
                return indoorSpace;
            });
        }

        public Graph GenerateRouteGraph(IndoorSpace indoorSpace)
        {
            if (indoorSpace == null)
            {
                throw new ArgumentNullException(nameof(indoorSpace), "IndoorSpace must not be null.");
            }

            Graph graph = new Graph(indoorSpace);
            Hypergraph hypergraph = indoorSpace.GetHypergraph();

            if (hypergraph == null || hypergraph.HyperNodes == null || hypergraph.HyperEdges == null)
            {
                throw new InvalidOperationException("Hypergraph or its components must not be null.");
            }

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

        public MapGrid SetMapGrid(float width, float height, float length)
        {
            MapGrid mapGrid = new MapGrid(width, height, length);
            return mapGrid;
        }

        public Graph GetGraph()
        {
            return graph;
        }

        public MapGrid GetMapGrid()
        {
            return mapGrid;
        }
    }
}
