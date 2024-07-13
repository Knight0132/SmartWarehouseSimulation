using System;
using System.Collections;
using System.Linq;
using UnityEngine;
using Newtonsoft.Json;
using NetTopologySuite.Geometries;
using System.Collections.Generic;

namespace Map {
    public class CellSpace
    {
        [JsonProperty("id")]
        public string Id { get; private set; }
        public Dictionary<string, object> Properties { get; private set; }
        public Geometry Space { get; private set; }
        public Geometry Node { get; private set; }


        public CellSpace(string cellId, Dictionary<string, object> properties, Geometry space, Geometry node)
        {
            this.Id = cellId;
            this.Properties = properties;
            this.Space = space;
            this.Node = node;
        }

        public bool IsBusinesspoint()
        {
            if (Properties.TryGetValue("functions", out object functionsObj) && functionsObj is List<object> functions)
            {
                foreach (var functionObj in functions)
                {
                    if (functionObj is Dictionary<string, object> function && function.TryGetValue("type", out object typeObj) && typeObj as string == "shelf")
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        public bool IsPickingPoint()
        {
            if (Properties.TryGetValue("functions", out object functionsObj) && functionsObj is List<object> functions)
            {
                foreach (var functionObj in functions)
                {
                    if (functionObj is Dictionary<string, object> function && function.TryGetValue("type", out object typeObj) && typeObj is string typeString)
                    {
                        Console.WriteLine($"Debug: typeString before trim = '{typeString}'"); // 打印调试信息
                        typeString = typeString.Trim(); // 修剪字符串
                        Console.WriteLine($"Debug: typeString after trim = '{typeString}'"); // 打印调试信息
        
                        if (string.Equals(typeString, "picking point", StringComparison.OrdinalIgnoreCase))
                        {
                            Console.WriteLine("Debug: Match found"); // 打印匹配信息
                            return true;
                        }
                    }
                }
            }
            Console.WriteLine("Debug: No match found"); // 如果没有找到匹配项，打印此信息
            return false;
        }

        public bool IsNavigable()
        {
            if (Properties.TryGetValue("navigable", out object propertiesObj) && propertiesObj is bool navigable && navigable)
            {
                return true;
            }
            return false;
        }

        public List<string> GetBoundings()
        {
            if (Properties.TryGetValue("functions", out object functionsObj) && functionsObj is List<object> functions)
            {
                foreach (var functionObj in functions)
                {
                    if (functionObj is Dictionary<string, object> function && function.TryGetValue("type", out object typeObj) && typeObj as string == "picking point")
                    {
                        if (function.TryGetValue("features", out object featuresObj) && featuresObj is Dictionary<string, object> features)
                        {
                            if (features.TryGetValue("boundings", out object boundingObj) && boundingObj is List<object> boundings)
                            {
                                return boundings.Cast<string>().ToList();
                            }
                        }
                    }
                }
            }
            return null;
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

        public static CellSpace FromJson(string json)
        {
            var settings = new JsonSerializerSettings
            {
                Converters = new List<JsonConverter> { new GeometryConverter() }
            };
            var cellSpace = JsonConvert.DeserializeObject<CellSpace>(json, settings);
            return cellSpace;
        }
    }
}

