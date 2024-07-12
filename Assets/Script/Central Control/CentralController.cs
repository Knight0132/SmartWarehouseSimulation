using System.Collections.Generic;
using System.Collections;
using System.Linq;
using NetTopologySuite.Geometries;
using UnityEngine;
using Map;

namespace CentralControl
{
    public class CentralController: MonoBehaviour
    {
        public OrderManager orderManager;
        public RobotManager robotManager;
        public IndoorSpace indoorSpaceProvider;
        public MapLoader mapLoader;
        public float width, length;
        public Graph graph;
        public float interval = 5f;
        public int orderCount = 10;

        public void Start()
        {
            indoorSpaceProvider = mapLoader.LoadJson();
            width = mapLoader.width;
            length = mapLoader.length;
            graph = mapLoader.GenerateRouteGraph(indoorSpaceProvider); 
            StartCoroutine(StartGeneratingOrders());
        }

        IEnumerator StartGeneratingOrders()
        {
            while (true)
            {
                GenerateRandomOrders();
                DispatchOrders();
                yield return new WaitForSeconds(interval);
            }
        }

        private void GenerateRandomOrders()
        {
            int attempts = 0;
            int maxAttempts = orderCount * 100;
            int i = 0;
            while (i < orderCount && attempts < maxAttempts)
            {
                string id = System.Guid.NewGuid().ToString();
                int randomIndex = (int)Random.Range(0, indoorSpaceProvider.BusinessPoints.Count - 1);
                
                Debug.Log($"Generated randomIndex: {randomIndex} / {indoorSpaceProvider.BusinessPoints.Count}");
                CellSpace cellSpace = indoorSpaceProvider.BusinessPoints[randomIndex];
                float executionTime = Random.Range(0f, 30f);
                Debug.Log($"Generated order: {id} in {cellSpace.Id} lasting {executionTime} seconds");
            }
        }

        private void DispatchOrders()
        {
            foreach (var order in orderManager.GetAllOrders())
            {
                RobotController closestRobot = robotManager.GetClosestFreeRobot(order.Destination);
                if (closestRobot != null && closestRobot.IsFree)
                {
                    closestRobot.ReceiveOrder(order);
                    Debug.Log($"Order {order.Id} assigned to robot {closestRobot.Id}");
                }
            }
        }
    }
}

