using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Order 
{
    public class Order
    {
        public int OrderId { get; private set; } 

        // 订单的真实位置（货架的位置）
        public float X { get; private set; } 
        public float Z { get; private set; }

        public Order(int orderId, float x, float z)
        {
            OrderId = orderId;
            TargetLocation = new Vector3(x, 0, z);
        }
    }

    public class OrderModule
    {
        private Queue<Order> orderQueue;

        public OrderModule()
        {
            orderQueue = new Queue<Order>();
        }

        public void GenerateOrder(int orderId, Vector2 targetLocation)
        {
            Order newOrder = new Order(orderId, targetLocation);
            orderQueue.Enqueue(newOrder);
        }

        public Order GetNextOrder()
        {
            return orderQueue.Dequeue();
        }
    }
}
