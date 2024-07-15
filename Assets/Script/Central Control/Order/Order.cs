using System;
using UnityEngine;
using Map;

namespace CentralControl 
{
    public class Order
    {
        public string Id { get; private set; }
        public Vector3 Destination { get; private set; }
        public PickingPoint PickingPoint { get; private set; }
        public DateTime CreationTime { get; private set; }
        public DateTime Priority => CreationTime;  // 优先级 -> 创建时间
        public float ExecutionTime { get; private set; }

        public Order(string id, Vector3 destination, PickingPoint pickingPoint, float executionTime)
        {
            this.Id = id;
            this.Destination = destination;
            this.PickingPoint = pickingPoint;
            this.CreationTime = DateTime.Now;
            this.ExecutionTime = executionTime;
        }
    }
}
