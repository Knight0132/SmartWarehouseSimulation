using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace CentralControl
{
    public class OrderManager
    {
        private List<Order> orders = new List<Order>();

        public void AddOrder(string id, Vector3 coordinates)
        {
            Order newOrder = new Order(id, coordinates);
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
    }
}