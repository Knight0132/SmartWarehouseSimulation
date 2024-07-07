using System;
using UnityEngine;

namespace CentralControl 
{
    public class Order
    {
        public string Id { get; private set; }
        public Vector3 Coordinates { get; private set; }
        public DateTime CreationTime { get; private set; }
        public DateTime Priority => CreationTime;  // 优先级 -> 创建时间

        public Order(string id, Vector3 coordinates)
        {
            Id = id;
            Coordinates = coordinates;
            CreationTime = DateTime.Now;
        }
    }
}
