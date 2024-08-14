using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;

namespace CentralControl
{
    public class OrderManager: MonoBehaviour
    {
        private List<Order> orders = new List<Order>();

        public void AddOrder(string id, Vector3 coordinates, PickingPoint pickingPoint, float executionTime)
        {
            Order newOrder = new Order(id, coordinates, pickingPoint, executionTime);

            if (newOrder == null)
            {
                throw new System.ArgumentNullException(nameof(newOrder), "Order cannot be null.");
            }

            orders.Add(newOrder);
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