using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Threading.Tasks;
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

        async void Start()
        {
            await InitializeCentralControllerAsync();
            StartCoroutine(StartGeneratingOrders());
        }

        private async Task InitializeCentralControllerAsync()
        {
            indoorSpaceProvider = await mapLoader.LoadJsonAsync();
            if (indoorSpaceProvider == null)
            {
                Debug.LogError("Failed to load indoor space data.");
                return;
            }

            width = mapLoader.width;
            length = mapLoader.length;
            graph = await Task.Run(() => mapLoader.GenerateRouteGraph(indoorSpaceProvider));
            if (graph == null)
            {
                throw new System.ArgumentNullException(nameof(graph), "Graph must not be null.");
            }
            Debug.Log("Central controller initialized.");
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
            int i = 0;
            while (i < orderCount)
            {
                string id = System.Guid.NewGuid().ToString();
                int randomIndex = (int)Random.Range(0, indoorSpaceProvider.BusinessPoints.Count - 1);
                CellSpace cellSpace = indoorSpaceProvider.BusinessPoints[randomIndex];
                Vector3 position = new Vector3((float)cellSpace.Space.Centroid.X, 0, (float)cellSpace.Space.Centroid.Y);
                float executionTime = Random.Range(0f, 30f);
                PickingPoint pickingPointCellSpace = indoorSpaceProvider.GetPickingPointFromBusinessPoint(cellSpace);
                orderManager.AddOrder(id, position, pickingPointCellSpace, executionTime);
                i++;
                Debug.Log($"Generating order {id} at {cellSpace.Id}");
            }
        }

        private void DispatchOrders()
        {
            var orders = orderManager.GetAllOrders();
            Debug.Log($"Dispatching {orders.Count} orders.");
            foreach (var order in orders)
            {
                RobotController closestRobot = robotManager.GetClosestFreeRobot(order.Destination);
                if (closestRobot != null && closestRobot.IsFree)
                {
                    closestRobot.ReceiveOrder(order);
                    Debug.Log($"Order {order.Id} assigned to robot {closestRobot.Id}");
                }
                else
                {
                    Debug.Log($"No available robots for order {order.Id}");
                }
            }
        }
    }
}

