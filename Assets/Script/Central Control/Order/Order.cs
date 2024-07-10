using System;
using UnityEngine;

namespace CentralControl 
{
    public class Order
    {
        public string Id { get; private set; }
        public Vector3 Destination { get; private set; }
        public DateTime CreationTime { get; private set; }
        public DateTime Priority => CreationTime;  // 优先级 -> 创建时间
        public float ExecutionTime { get; private set; }

        public Order(string id, Vector3 destination, float executionTime)
        {
            Id = id;
            Destination = destination;
            CreationTime = DateTime.Now;
            ExecutionTime = executionTime;
        }
    }
}
