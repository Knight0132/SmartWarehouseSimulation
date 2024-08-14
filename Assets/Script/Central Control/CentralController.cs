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
        public float correctionFactor = 0.1f;

        private bool generatingOrders;

        async void Start()
        {
            await InitializeCentralControllerAsync();
            await InitializeRobotsAsync();
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

        private async Task InitializeRobotsAsync()
        {
            await robotManager.InitializeRobotsAsync();
        }

        IEnumerator StartGeneratingOrders()
        {
            generatingOrders = true;
            while (generatingOrders)
            {
                Debug.Log("Starting to generate orders...");
                try
                {
                    GenerateRandomOrders();
                }
                catch (System.Exception ex)
                {
                    Debug.LogError($"Error generating orders: {ex.Message}");
                }
                Debug.Log("Orders generated, starting to dispatch orders...");
                try
                {
                    DispatchOrders();
                }
                catch (System.Exception ex)
                {
                    Debug.LogError($"Error dispatching orders: {ex.Message}");
                }
                Debug.Log("Orders dispatched, waiting for next interval...");
                yield return new WaitForSeconds(interval);
            }
        }

        private void GenerateRandomOrders()
        {
            int i = 0;
            while (i < orderCount)
            {
                try
                {
                    string id = System.Guid.NewGuid().ToString();
                    int randomIndex = (int)Random.Range(0, indoorSpaceProvider.BusinessPoints.Count - 1);
                    CellSpace cellSpace = indoorSpaceProvider.BusinessPoints[randomIndex];
                    Vector3 position = new Vector3((float)cellSpace.Space.Centroid.X, 0, (float)cellSpace.Space.Centroid.Y);
                    float executionTime = Random.Range(0f * correctionFactor, 30f * correctionFactor);
                    PickingPoint pickingPointCellSpace = indoorSpaceProvider.GetPickingPointFromBusinessPoint(cellSpace);
                    orderManager.AddOrder(id, position, pickingPointCellSpace, executionTime);
                    i++;
                    Debug.Log($"Generated order {id} at {cellSpace.Id} with execution time {executionTime}");
                }
                catch (System.Exception ex)
                {
                    Debug.LogError($"Error generating order: {ex.Message}");
                }
            }
        }

        private void DispatchOrders()
        {
            var orders = orderManager.GetAllOrders();
            Debug.Log($"Dispatching {orders.Count} orders.");
            foreach (var order in orders)
            {
                bool orderAssigned = false;
                while (!orderAssigned)
                {
                    try
                    {
                        Debug.Log($"Processing order {order?.Id}");

                        if (order.Destination == null)
                        {
                            Debug.LogError($"Order {order.Id} has null destination.");
                            break;
                        }

                        Debug.Log($"Trying to find the closest robot for order {order.Id} with destination {order.Destination}");
                        RobotController closestRobot = robotManager.GetClosestAvailableRobot(order.Destination);
                        Debug.Log($"Closest robot for order {order.Id} found: {closestRobot?.Id}");

                        //寻找最近的机器人（两个策略——1.最近的available机器人；2.附近没有available机器人，选择最近的free机器人）
                        //将来考虑使用Kmeans聚类来找available机器人
                        if (closestRobot == null)
                        {
                            robotManager.CheckRobotStatus();
                            Debug.LogError($"No available robot for order {order.Id}. Trying to find closest free robot");
                            
                            closestRobot = robotManager.GetClosestFreeRobot(order.Destination);
                            Debug.Log($"Closest free robot for order {order.Id} found: {closestRobot?.Id}");
                        }

                        if (closestRobot == null)
                        {
                            Debug.LogError($"No available or free robot for order {order.Id}.");
                            continue;
                        }
                        if (closestRobot.IsAvailable)
                        {
                            Debug.Log($"Assigning order {order.Id} to robot {closestRobot.Id}");
                            closestRobot.ReceiveOrder(order);
                            Debug.Log($"Order {order.Id} assigned to robot {closestRobot.Id}");
                            orderAssigned = true;
                        }
                        else if (closestRobot.IsFree)
                        {
                            Debug.Log($"Assigning order {order.Id} to free robot {closestRobot.Id}");
                            closestRobot.ReceiveOrder(order);
                            Debug.Log($"Order {order.Id} assigned to free robot {closestRobot.Id}");
                            orderAssigned = true;
                        }
                        else
                        {
                            Debug.Log($"Robot {closestRobot.Id} is not available or free.");
                            continue;
                        }
                    }
                    catch (System.Exception ex)
                    {
                        Debug.LogError($"Error dispatching order {order.Id}: {ex.Message}");
                        break;
                    }
                }
            }
        }
    }
}

