using System.Collections;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using System.Collections.Generic;
using System.Linq;

namespace Map {
    public class IndoorSpace
    {
        private Dictionary<string, object> _properties = new Dictionary<string, object>();
        private List<CellSpace> _cellSpaces = new List<CellSpace>();
        private List<CellBoundary> _cellBoundaries = new List<CellBoundary>();
        private List<Layer> _layers = new List<Layer>();
        private List<Container> _containers = new List<Container>();
        [JsonProperty("properties")]

        public Dictionary<string, object> Properties { get => _properties; set => _properties = value; }
        [JsonProperty("CellSpaces")]
        public List<CellSpace> CellSpaces { get => _cellSpaces; }
        [JsonProperty("CellBoundaries")]
        public List<CellBoundary> CellBoundaries { get => _cellBoundaries; }
        [JsonProperty("Layers")]
        public List<Layer> Layers { get => _layers; }
        [JsonProperty("Containers")]
        public List<Container> Containers { get => _containers; }

        public void AddCellSpace(CellSpace cellSpace)
        {
            if (_cellSpaces.Any(c => c.Id == cellSpace.Id))
                throw new System.Exception("CellSpace id already exists");
            _cellSpaces.Add(cellSpace);
        }

        public void AddCellBoundary(CellBoundary cellBoundary)
        {
            if (_cellBoundaries.Any(c => c.Id == cellBoundary.Id))
                throw new System.Exception("CellBoundary id already exists");
            if (!_cellSpaces.Any(c => c.Id == cellBoundary.Source) || !_cellSpaces.Any(c => c.Id == cellBoundary.Target))
                throw new System.Exception("Source and/or target cell do not exist");
            _cellBoundaries.Add(cellBoundary);
        }

        public void AddLayer(Layer layer)
        {
            _layers.Add(layer);
        }

        public void AddContainer(Container container)
        {
            _containers.Add(container);
        }

        public CellSpace GetCellFromId(string id)
        {
            foreach (var cellSpace in _cellSpaces)
            {
                if (cellSpace.Id == id)
                {
                    // Debug.Log(cellSpace.ToJson());
                    return cellSpace;
                }
            }
            Debug.Log("CellSpace not found");
            return null;
        }

        public CellSpace GetCellSpaceFromConnectionPoint(ConnectionPoint connectionPoint, bool sequence = true)
        {
            if (sequence)
            {
                foreach (var cellBoundary in _cellBoundaries)
                {
                    if (cellBoundary.Id == connectionPoint.Id)
                    {
                        return GetCellFromId(cellBoundary.Source);
                    }
                }
            }
            else
            {
                foreach (var cellBoundary in _cellBoundaries)
                {
                    if (cellBoundary.Id == connectionPoint.Id)
                    {
                        return GetCellFromId(cellBoundary.Target);
                    }
                }
            }
            Debug.Log("CellSpace not found");
            return null;
        }

        public CellSpace GetCellSpaceFromCoordinates(Vector3 position)
        {
            foreach (var cellSpace in _cellSpaces)
            {
                Polygon polygon = (Polygon)cellSpace.Space;
                if (polygon.Contains(new Point(position.x, position.z)))
                {
                    return cellSpace;
                }
            }
            Debug.Log("CellSpace not found");
            return null;
        }

        public CellBoundary GetCellBoundaryFromId(string id)
        {
            foreach (var cellBoundary in _cellBoundaries)
            {
                if (cellBoundary.Id == id)
                {
                    Debug.Log(cellBoundary.ToJson());
                    return cellBoundary;
                }
            }
            Debug.Log("CellBoundary not found");
            return null;
        }

        public ConnectionPoint GetConnectionPointFromId(string id)
        {
            foreach (var cellBoundary in _cellBoundaries)
            {
                if (cellBoundary.Id == id)
                {
                    return cellBoundary.ConvertToConnectionPoint();
                }
            }
            Debug.Log(id + "ConnectionPoint not found");
            return null;
        }

        public ConnectionPoint GetConnectionPointFromCellSpace(string cellSpaceId, bool sequence = true)
        {
            foreach (var cellBoundary in _cellBoundaries)
            {
                // Debug.Log(sequence);
                if (sequence)
                {
                    if (cellBoundary.Source == cellSpaceId)
                    {
                        // Debug.Log("ConnectionPoint matching: " + cellBoundary.Source + " " + cellSpaceId);
                        // Debug.Log(cellBoundary.ConvertToConnectionPoint().ToJson());
                        return cellBoundary.ConvertToConnectionPoint();
                    }
                }
                else
                {
                    if (cellBoundary.Target == cellSpaceId)
                    {
                        // Debug.Log("ConnectionPoint matching: " + cellBoundary.Target + " " + cellSpaceId);
                        return cellBoundary.ConvertToConnectionPoint();
                    }
                }
            }
            Debug.Log(cellSpaceId);
            Debug.Log("ConnectionPoint not found");
            return null;
        }

        public RLineGroup GetRlineGroupFromCellSpaceId(string id)
        {
            foreach (var container in _containers)
            {
                if (container.CellSpaceId == id)
                {
                    return container.ConvertToRLineGroup();
                }
            }
            Debug.Log("RLineGroup not found");
            return null;
        }

        public Layer GetLayerFromCellSpaceId(string id)
        {
            foreach (var layer in _layers)
            {
                if (layer.CellSpaces.Contains(id))
                {
                    return layer;
                }
            }
            Debug.Log("Layer not found");
            return null;
        }

        public Hypergraph GetHypergraph()
        {
            Hypergraph hypergraph = new Hypergraph();
            foreach (var cellBoundary in _cellBoundaries)
            {
                ConnectionPoint connectionPoint = cellBoundary.ConvertToConnectionPoint();
                hypergraph.AddHyperNode(connectionPoint);
            }
            foreach (var container in _containers)
            {
                RLineGroup rLineGroup = container.ConvertToRLineGroup();
                hypergraph.AddHyperEdge(rLineGroup);
            }
            return hypergraph;
        }

        public string ToJson()
        {
            var settings = new JsonSerializerSettings
            {
                Formatting = Formatting.Indented,
                Converters = new List<JsonConverter> { new GeometryConverter() }
            };
            return JsonConvert.SerializeObject(this, settings);
        }

        public static IndoorSpace FromJson(string json)
        {
            var settings = new JsonSerializerSettings
            {
                Converters = new List<JsonConverter> { new GeometryConverter() }
            };
            var indoorSpace = JsonConvert.DeserializeObject<IndoorSpace>(json, settings);
            return indoorSpace;
        }
    }
}
