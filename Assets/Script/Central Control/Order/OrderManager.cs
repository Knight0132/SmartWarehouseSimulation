using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace CentralControl
{
    public class OrderManager: MonoBehaviour
    {
        private List<Order> orders = new List<Order>();

        public void AddOrder(string id, Vector3 coordinates, float executionTime)
        {
            Order newOrder = new Order(id, coordinates, executionTime);
            orders.Add(newOrder);
            Debug.Log($"Added new order: {id} at {coordinates}");
        }

        public List<Order> GetAllOrders()
        {
            return orders.OrderBy(o => o.Priority).ToList();
        }

        public Order GetOrderById(string id)
        {
            return orders.FirstOrDefault(o => o.Id == id);
        }

        public Order GetNextOrder()
        {
            var order = orders.OrderBy(o => o.Priority).FirstOrDefault();
            if (order != null)
            {
                orders.Remove(order);
            }
            return order;
        }
    }
}